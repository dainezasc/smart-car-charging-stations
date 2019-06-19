import lcddriver
import spidev

# Load the LCD driver
display = lcddriver.lcd()

# Open up SPI bus for MCP3008
spi = spidev.SpiDev()
spi.open(0,0)
spi.max_speed_hz=1000000

# Initialize what sensor is where on the MCP3009
currentChannel = 0


def getReading(channel):
    # First pull the raw data from the MCP3008
    rawData = spi.xfer([1, (8 + channel) << 4, 0])
    # Process the raw data into something we understand
    processedData = ((rawData[1]&3) << 8) + rawData[2]
    return processedData

# Convert bit values to voltage
def convertVoltage(bitValue, decimalPlaces=2):
    voltage = (bitValue * 5) / float(1023)
    voltage = round(voltage, decimalPlaces)
    return voltage

# Convert bit values to voltage
def convertCurrent(bitValue, decimalPlaces=2):                          #Offset Value 
    current = (convertVoltage(bitValue, decimalPlaces) - 2.5) / 0.066 #- 0.0364
    current = round(current, decimalPlaces)
    return current

try:
    while True:
        currentBit = getReading(currentChannel)
        currentVoltage = convertVoltage(currentBit)
        currentCurrent = convertCurrent(currentBit)
        
        # Converting results to str for display in LCD
        padcurrentVoltage = str(currentVoltage).ljust(4,'0')
        padcurrentCurrent = str(currentCurrent).ljust(4,'0')
    
        # Print all the things in Shell
        print("BitValue = {}  ;  Voltage = {}V  ;  Current = {}A".format(currentBit, currentVoltage, currentCurrent))
                
        # Print all the things on LCD
        display.lcd_display_string("Current Measurement", 1)                  # Write line of text to first line of display
        display.lcd_display_string("BitValue = {}".format(currentBit), 2)     # Write line of text to second line of display
        display.lcd_display_string("Voltage = {}V".format(padcurrentVoltage), 3) # Write line of text to 3rd line of display
        display.lcd_display_string("Current = {}A".format(padcurrentCurrent), 4)                 # Write line of text to 4th line of display

# When you press ctrl+c, you exit the program and cleanup the LCD
except KeyboardInterrupt:
    print("Cleaning up!")
    display.lcd_clear()
    