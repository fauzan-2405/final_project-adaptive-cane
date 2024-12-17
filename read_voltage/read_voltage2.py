import spidev
import time

# Create an SPI object
spi = spidev.SpiDev()
spi.open(0, 0)  # Open /dev/spidev0.0
spi.max_speed_hz = 1350000  # Set the speed (optional)

def read_channel(channel):
    # Ensure the channel is between 0 and 7
    if channel < 0 or channel > 7:
        raise ValueError("Channel must be between 0 and 7")

    # MCP3008 has a specific way to read channels
    # Start bit (1), Single-ended bit (1), and channel number (3 bits)
    command = 0b11 << 6 | channel

    # Send the command and read the response
    response = spi.xfer2([command, 0])  # Send the command and get the response
    adc_value = ((response[0] & 0x0F) << 8) | response[1]  # Combine the two bytes

    return adc_value

try:
    while True:
        # Change the channel number here (0 to 7)
        channel = 0  # Change this to the desired channel (0-7)
        adc_value = read_channel(channel)
        voltage = adc_value * (3.3 / 1023)  # Convert ADC value to voltage
        print(f"Channel {channel}: ADC Value: {adc_value}, Voltage: {voltage:.2f} V")
        time.sleep(1)

except KeyboardInterrupt:
    spi.close()
