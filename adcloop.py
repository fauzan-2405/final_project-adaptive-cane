import board
import busio
import time
from digitalio import DigitalInOut
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn

# create the spi bus
spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)

# create the cs (chip select) using GPIO22
cs = DigitalInOut(board.D22)  # Use GPIO22 for CS

# create the mcp object
mcp = MCP.MCP3008(spi, cs)

# create an analog input channel on pin 0
chan = AnalogIn(mcp, MCP.P0)

vref = 5

try:
    while True:
        print("Raw ADC Value: ", chan.value)
        voltage = (chan.value/1023.0)*vref
        print("Calculated Voltage: {:.2f} V".format(voltage))
        time.sleep(1)  # Delay for 1 second

except KeyboardInterrupt:
    print("Program stopped by user.")
