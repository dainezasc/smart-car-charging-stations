#!/usr/bin/python
import RPi.GPIO as GPIO
import time, argparse, sys, termios, socket, re

parser = argparse.ArgumentParser()
parser.add_argument('command', type=str, default='testdrive', help='The function to execute')
parser.add_argument('-pwm', type=int, default=50, help='Motor PWM frequency, in Hz')
parser.add_argument('-Ku', type=float, default=400, help='Ultimate gain for stable oscillations')
parser.add_argument('-Tu', type=float, default=425, help='Period for stable oscillations')
parser.add_argument('-offset', type=float, default=0.1, help='Offset for the PID controller')
parser.add_argument('-speed', type=int, default=100, help='Max duty cycle for the motors')
parser.add_argument('-data', type=str, default="Hello, world!", help='Data to send to station')
parser.add_argument('-TCP_IP', type=str, nargs='+', help='The IP address array to bind to for TCP')
parser.add_argument('-TCP_PORT', type=int, default=5005, help='The socket to bind to for TCP')
args = parser.parse_args()

def changeMotorDC(LeftDC, RightDC):
    '''
    Sets the duty cycles of the motors according to retrieved args.
    If the arg exceeds 100 or -100, the duty cycle is reset to 100.
    '''
    try:        
        if(LeftDC > 0):
            GPIO.setup(LMC_1, GPIO.HIGH)
            GPIO.setup(LMC_2, GPIO.LOW)
            if(LeftDC > 100):
                LM_PWM.start(100)
            else:
                LM_PWM.start(LeftDC)
        else:
            GPIO.setup(LMC_1, GPIO.LOW)
            GPIO.setup(LMC_2, GPIO.HIGH)
            if(LeftDC < -100):
                LM_PWM.start(100)
            else:
                LM_PWM.ChangeDutyCycle(LeftDC * (-1))
        if(RightDC > 0):
            GPIO.setup(RMC_1, GPIO.HIGH)
            GPIO.setup(RMC_2, GPIO.LOW)
            if(RightDC > 100):
                RM_PWM.start(100)
            else:
                RM_PWM.ChangeDutyCycle(RightDC)
        else:
            GPIO.setup(RMC_1, GPIO.LOW)
            GPIO.setup(RMC_2, GPIO.HIGH)
            if(RightDC < -100):
                RM_PWM.start(100)
            else:
                RM_PWM.start(RightDC *(-1))
    except KeyboardInterrupt:
        return ("KeyboardInterrupt triggered")

def stopMotors():
    '''Stops PWM of all motors'''
    changeMotorDC(0, 0)
    LM_PWM.stop()
    RM_PWM.stop()

def testdrive():
    '''Simple def to test motor control'''
    try:
        print("Going forward in 8 secs")
        time.sleep(8)
        changeMotorDC(99, 99)
        print("Going left in 1 sec")
        time.sleep(1)
        changeMotorDC(-99, 50)
        print("Going right in 1 sec")
        time.sleep(1)
        changeMotorDC(50, -99)
        print("Going backwards in 1 sec")
        time.sleep(1)
        changeMotorDC(-99, -99)
        print("Stopping in 1 sec")
        time.sleep(1)
        stopMotors()
        return True
    except KeyboardInterrupt:
        return ("KeyboardInterrupt triggered")

def CheckOptoSensors():
    '''
    Creates a list that gathers the TCRT5000 sensor outputs and stores them as strings.
    Returns output of left, center and right line sensor respectively.
    '''
    sensor = [GPIO.input(LSData), GPIO.input(CSData), GPIO.input(RSData)]
    value = str(sensor[0]) + str(sensor[1]) + str(sensor[2])
    return value

def GetPIDError():
    '''
    Sets the error for the PID controller algorithm,
    based on the sensor value array.
    '''
    value = CheckOptoSensors()
    if(value == '000'):
        error = 0
    elif(value == '001'):
        error = 15
    elif(value == '010'):
        error = 0
    elif(value == '011'):
        error = 5
    elif(value == '100'):
        error = -15
    elif(value == '101'):
        error = 0
    elif(value == '110'):
        error = -5
    elif(value == '111'):
        error = 0   
    return error

def __gen_ch_getter(echo):
    def __fun():
        fd = sys.stdin.fileno()
        oldattr = termios.tcgetattr(fd)
        newattr = oldattr[:]
        try:
            if echo:
                lflag = ~(termios.ICANON | termios.ECHOCTL)
            else:
                lflag = ~(termios.ICANON | termios.ECHO)
            newattr[3] &= lflag
            termios.tcsetattr(fd, termios.TCSADRAIN, newattr)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, oldattr)
        return ch
    return __fun
getch = __gen_ch_getter(False)
getche = __gen_ch_getter(True)

def manualdrive(speed):
    '''Enables car control using manual input'''
    print("W: accelerate, A/D: steer, S: reverse")
    print("Press C or Ctrl-C to exit")
    try:
        while (True):
            char = getch()
            if(char == "w"):
                changeMotorDC(speed, speed)
            if(char == "s"):
                changeMotorDC(-speed, -speed) 
            if(char == "a"):
                changeMotorDC(-speed, speed)
            if(char == "d"):
                changeMotorDC(speed, -speed) 
            if(char == "c"):
                return
            char = ""
    except KeyboardInterrupt:
        return ("KeyboardInterrupt triggered")

def PIDInit(Ku,Tu):
    '''
    Initializes the PID controller algorithm by calculating Kp, Ki and Kd values
    from inputs Ku (ultimate gain) and Tu (oscillation period).

    By default, Ku is 400 and Tu is 425. They can be changed by specifying them
    as additional parameters when running the script.

    The calculations are based on the Ziegler-Nichols method of PID tuning,
    specifically the "no overshoot" settings taken from Microstar Labs:
    http://www.mstarlabs.com/control/znrule.html
    '''
    int_min = -20
    int_max = 20
    integ = 0
    deriv = 0
    lasterror = 0

    Kp = 0.2 * Ku
    Ki = (0.4 * Ku) / Tu
    Kd = (Ku * Tu) / 15
    
    print("Ku={} \nTu={} \nKp={} \nKi={} \nKd={}".format(Ku,Tu,Kp,Ki,Kd))
    return Kp,Ki,Kd,int_min,int_max,integ,deriv,lasterror

def PIDControl(Kp,Ki,Kd,int_min,int_max,lastinteg,deriv,lasterror,speed,offset):
    '''
    The PID controller algorithm.
    Retrieves the error value from the sensors, and changes motor duty cycles
    based on the error and retrieved Kp, Ki and Kd values.
    To function as intended, it needs to be executed periodically and as fast as
    possible.
    '''
    error = GetPIDError() - offset
    integral = lastinteg + error
    if(integral > int_max):
        integral = int_max
    elif(integral < int_min):
        integral = int_min

    deriv = (error - lasterror)

    control = Kp * error + Ki * integral + Kd * deriv
    changeMotorDC(speed + control, speed - control)

    return integral, error

def PIDTest(Ku,Tu,speed,offset):
    '''
    Test def for the PID controller.
    Calls PIDInit, then periodically calls PIDControl until KeyboardInterrupt
    is triggered.
    '''
    Kp,Ki,Kd,int_min,int_max,lastinteg,deriv,lasterror = PIDInit(Ku,Tu)
    try:
        while(True):
            integ, error = PIDControl(Kp,Ki,Kd,int_min,int_max,lastinteg,deriv,lasterror,speed,offset)
            lastinteg = integ
            lasterror = error
    except KeyboardInterrupt:
        return ("KeyboardInterrupt triggered")

def TCP_TestSend(message,TCP_PORT,TCP_IP):
    '''
    Def for testing TCP connections with the stations.

    Input:
    message = The data you wish to send to the server
    TCP_IP = The IP of the server in the LAN
    TCP_PORT = The socket port that the server is listening to

    Output:
    First 20 bytes of message, since the server just echoes it back
    After echoed data is received, the connection is closed.
    '''
    BUFFER_SIZE = 1024
    IsConnected = False
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    for entry in range(len(TCP_IP)):
        try:
            print("Connecting to:",TCP_IP[entry]) 
            s.connect((TCP_IP[entry], TCP_PORT))
            IsConnected = True
            print("Connection successful")
            break
        except:
            pass
    if (IsConnected == False):
        print("No servers are responding")
        return False
    s.sendall(message.encode('utf-8'))
    data = s.recv(BUFFER_SIZE)
    s.close()
    return ("received data:", data)

def AdjustLine(speed,direction):
    try:
        value = CheckOptoSensors()
        if(value == '111'):
            pass
        else: return None
        if (direction == "left"):
            while(value != '001'):
                changeMotorDC(-speed, speed)
                time.sleep(0.01)
                value = CheckOptoSensors()
            stopMotors()
            return True
        elif (direction == "right"):
            while(value != '100'):
                changeMotorDC(speed, -speed)
                time.sleep(0.01)
                value = CheckOptoSensors()
            stopMotors()
            return True
        else: return False
    except KeyboardInterrupt:
        return ("KeyboardInterrupt triggered")
    
def ChargeCar_Line(TCP_PORT,Ku,Tu,speed,offset,TCP_IP):
    '''
    1. Attempts to connect to specified TCP_IP(s) and port.
    2. Sends charge request to the stations connected.
    3. When first reply is received, checks received station ID.
    4. If ID = 1, adjust to left side, if ID = 2, adjust to right side. Lame, we know.
    5. Enable PID controlled movement, don't stop until station tells us to.
    6. If stop command received, stop car. We should be charging now.
    '''
    BUFFER_SIZE = 1024
    IsConnected = False
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    for entry in range(len(TCP_IP)):
        try:
            print("Connecting to:",TCP_IP[entry]) 
            s.connect((TCP_IP[entry], TCP_PORT))
            IsConnected = True
            print("Connection successful")
            break
        except:
            pass
    if (IsConnected == False):
        print("No servers are responding")
        return False
    data = ""
    while (True):
        s.sendall("ChargeRequest".encode('utf-8'))
        data = s.recv(BUFFER_SIZE)
        if (data != ""):
            break
    try:
        StrData = data.decode('utf-8').strip('\x00')
        print("Received:",StrData)
        StationID = StrData[StrData.find("#")+1]
    except:
        print("Invalid data:", data)
        return False
    if (StationID == '1'):
        print("Adjusting to the left!")
        AdjustLine(speed * 0.5,"left")
        time.sleep(0.5)
    elif (StationID == '2'):
        print("Adjusting to the right!")
        AdjustLine(speed * 0.5,"right")
        time.sleep(0.5)
    else:
        print("This ID is not supported:",StationID)
        return False
    s.setblocking(0)
    stopdata = ""
    Kp,Ki,Kd,int_min,int_max,lastinteg,deriv,lasterror = PIDInit(Ku,Tu)
    while(stopdata != b"STOP"):
        integ, error = PIDControl(Kp,Ki,Kd,int_min,int_max,lastinteg,deriv,lasterror,speed,offset)
        lastinteg = integ
        lasterror = error
        try:
            stopdata = s.recv(BUFFER_SIZE)
        except:
            pass
    stopMotors()
    s.setblocking(1)
    s.close()
    print("Arrived at destination")
    return True

class CommandList:
    '''
    Python KV dictionary containing list of available commands to be passed as additional args.
    The dictionary is used similar to switch-case in C or C++, since Python doesn't have
    a native switch-case solution.
    It is more efficient than if..elif..elif..else, and supports addition of new commands easily,
    even at run-time.
    '''
    def argtodef(self, argument):
        cmdlist = {
            'testdrive': testdrive,
            'testsense': CheckOptoSensors,
            'adjustline': lambda: AdjustLine(args.speed, "right"),
            'manual': lambda: manualdrive(args.speed),
            'testtcp': lambda: TCP_TestSend(args.data,args.TCP_PORT,args.TCP_IP),
            'testpid': lambda: PIDTest(args.Ku,args.Tu,args.speed,args.offset),
            'line': lambda: ChargeCar_Line(args.TCP_PORT,args.Ku,args.Tu,args.speed,args.offset,args.TCP_IP),
        }
        func = cmdlist.get(argument, lambda: print("Invalid command: {}".format(argument)))
        result_list = func()
        return result_list

def argumentparse(command):
    '''
    This def takes the parsed args and runs them through the KV dictionary above.
    Afterwards, cleans the GPIO and returns a result, if there is one.
    '''
    try:
        cmdlist = CommandList()
        result = cmdlist.argtodef(command)
        GPIO.cleanup()
        return result
    except KeyboardInterrupt:
        print("KeyboardInterrupt")
        GPIO.cleanup()
        return False
    except:
        raise

if (__name__ == "__main__"):
    '''
    Setting up the robot car. You can change the pin numbers according to your own setup.
    '''
    GPIO.setmode(GPIO.BCM)	# Setting GPIO setmode to BCM
    LMC_1 = 0	# Front Left Motor Control 1
    LMC_2 = 5	# Front Left Motor Control 2
    LMEn = 27	# Front Left Motor Enable
    RMC_1 = 6	# Front Right Motor Control 1
    RMC_2 = 13	# Front Right Motor Control 2
    RMEn = 22	# Front Right Motor Enable

    LSData = 26	# Left Sensor Data input
    CSData = 12	# Center Sensor Data input
    RSData = 16	# Right Sensor Data input

    CS_VCC = 4	# VCC output for Center Sensor (left and right sensors use dedicated 3.3V outputs)
    output_list = [LMC_1,LMC_2,LMEn,RMC_1,RMC_2,RMEn]
    input_list = [LSData,CSData,RSData]

    GPIO.setup(output_list, GPIO.OUT) # Setting up output pins in the Pi
    GPIO.setup(input_list, GPIO.IN)	# Setting up input pins in the Pi
    GPIO.setup(CS_VCC, GPIO.OUT, initial=GPIO.HIGH)	# Since the Pi only has 2 dedicated 3.3V outputs, we add an additional 3.3V supply from a GPIO pin

    LM_PWM = GPIO.PWM(LMEn, args.pwm)
    RM_PWM = GPIO.PWM(RMEn, args.pwm)
    LM_PWM.start(0)
    RM_PWM.start(0)

    result = argumentparse(args.command)
    print (result)
