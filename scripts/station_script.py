#!/usr/bin/python
import RPi.GPIO as GPIO
import time, argparse, socket

parser = argparse.ArgumentParser()
parser.add_argument('command', type=str, default='testtcp', help='The function to execute')
parser.add_argument('-TCP_PORT', type=int, default=5005, help='The socket to bind to for TCP')
parser.add_argument('-StationID', type=int, default=1, help='The ID of the station passed to car')

args = parser.parse_args()

def TCP_Testserve(TCP_PORT):
    '''
    Def for testing TCP server setups for the stations.
    Default port for listening is 5005, but can be changed with TCP_PORT arg.
    Must be started BEFORE attempting to connect with clients.
    Once it receives data, echoes it back to client, then closes the socket.
    '''
    BUFFER_SIZE = 20
    
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(('0.0.0.0', TCP_PORT))
    print("Listening...")
    s.listen(1)
    
    conn, addr = s.accept()
    print ('Connection address:', addr)
    while (True):
        data = conn.recv(BUFFER_SIZE)
        if (not data): break
        print ("received data from client: ", data)
        conn.send(data)
    conn.close()
    return True

def DisableLED():
    try:
        GPIO.output(LED_Red, GPIO.LOW)
        GPIO.output(LED_Yellow, GPIO.LOW)
        GPIO.output(LED_Green, GPIO.LOW)
        return True
    except:
        raise

def EnableLED(color):
    try:
        DisableLED()
        GPIO.output(color, GPIO.HIGH)
        return True
    except:
        raise

def TestLED():
    try:
        while(True):
            EnableLED(LED_Red)
            time.sleep(1)
            EnableLED(LED_Yellow)
            time.sleep(1)
            EnableLED(LED_Green)
            time.sleep(1)
    except KeyboardInterrupt:
        print("KeyboardInterrupt triggered")
        return None
    except:
        raise

def ToggleCoil(state):
    if (state):
        GPIO.output(MosfetPin, GPIO.LOW)
        return True
    else:
        GPIO.output(MosfetPin, GPIO.HIGH)
        return True

def TestCoil():
    try:
        while(True):
            print("Enabling coil for 10 secs")
            ToggleCoil(True)
            time.sleep(10)
            print("Disabling coil for 10 secs")
            ToggleCoil(False)
            time.sleep(10)
    except KeyboardInterrupt:
        print("KeyboardInterrupt triggered")
        return None
    except:
        raise

def CheckPressurePad():
    if (GPIO.input(PSInput) == GPIO.LOW):
        return True
    else:
        return False

def EnableStation(TCP_PORT,StationID):
    '''
    1. Check status by checking pressure pad value.
    if there is pressure, wait for 1sec, then try again. No precise timing needed here.
    2. If its free, Start a TCP server and listen. Keep checking the pressure pad.
    3. If request received, establish connection and reply w/ station ID.
    4. Check pressure pad value again.
    if no pressure, try again. The more frequent the checks the better.
    5. If there is pressure, send TCP stop command to car, then change status.
    6. Loop back to step 1.
    '''
    print("My ID is :",StationID)
    try:
        s = None
        while (True):
            EnableLED(LED_Red)
            while(not CheckPressurePad()):
                time.sleep(0.1)
            BUFFER_SIZE = 20
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind(('0.0.0.0', TCP_PORT))
            s.listen(1)
            EnableLED(LED_Green)
            ToggleCoil(False)
            print("Station is free. Listening for hungry robot cars...")
            IsConnected = False
            while (CheckPressurePad()):
                try:
                    s.setblocking(0)
                    conn, addr = s.accept()
                    IsConnected = True
                    break
                except:
                    pass
            if (not IsConnected):
                print("Station is no longer free. Disabling TCP server...")
                s.setblocking(1)
                s.close()
                continue
            print ('Request from IP:', addr)
            data = conn.recv(BUFFER_SIZE)
            if (not data): break
            print ("Received message: ", data)
            if (data == b"ChargeRequest"):
                conn.send(("FREE:#{}".format(StationID)).encode('utf-8'))
                print("Response sent")
                EnableLED(LED_Yellow)
            else:
                conn.send("InvalidCmd".encode('utf-8'))
                conn.close()
                continue
            while (True):
                if (not CheckPressurePad()):
                    print("The car has arrived!")
                    break
            conn.send("STOP".encode('utf-8'))
            ToggleCoil(True)
            conn.close()
            s.close()
            print("Charging...")
    except OSError:
        print("OSError: Address already in use")
    except KeyboardInterrupt:
        print("KeyboardInterrupt triggered")
        if (s != None):
            s.close()
        return None
    except:
        raise

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
            'testpressure': lambda: CheckPressurePad(),
            'testled': lambda: TestLED(),
            'testcoil': lambda: TestCoil(),
            'testtcp': lambda: TCP_Testserve(args.TCP_PORT),
            'enable': lambda: EnableStation(args.TCP_PORT,args.StationID),
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
    Setting up the station. You can change the pin numbers according to your own setup.
    '''
    GPIO.setmode(GPIO.BCM)	# Setting GPIO setmode to BCM
    LED_Red = 5	        # Red LED
    LED_Yellow = 6	# Yellow LED
    LED_Green = 13	# Green LED
    MosfetPin = 12      # MOSFET control pin for coil toggle
    PSInput = 26	# Pressure Sensor input

    output_list = [LED_Red,LED_Yellow,LED_Green,MosfetPin]

    GPIO.setup(output_list, GPIO.OUT) # Setting up output pins in the Pi
    GPIO.output(MosfetPin, True)
    GPIO.setup(PSInput, GPIO.IN)	# Setting up input pins in the Pi

    result = argumentparse(args.command)
    print (result)
