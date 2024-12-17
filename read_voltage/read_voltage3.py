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

# Assumption: uses 2A of current constantly
def voltage_class(voltage):
    # Define voltage ranges and corresponding battery status messages
    voltage_ranges = [
        (3.95, "Battery between 90% and 100%"),
        (3.90, "Battery between 80% and 90%"),
        (3.78, "Battery between 70% and 80%"),
        (3.70, "Battery between 60% and 70%"),
        (3.60, "Battery between 50% and 60%"),
        (3.50, "Battery between 40% and 50%"),
        (3.40, "Battery between 30% and 40%"),
        (3.325, "Battery between 20% and 30%"),
        (3.10, "Battery between 10% and 20%"),
        (0, "Battery below 10%")  # Assuming voltage can't go below 0
    ]

    # Find the appropriate message based on the voltage
    for threshold, message in voltage_ranges:
        if voltage >= threshold:
            print(message)
            return  # Exit the function after printing the message

    print("Invalid voltage")  # Optional: Handle case for invalid voltage inputs

try:
    while True:
        # Change the channel number here (0 to 7)
        channel = 0  # Change this to the desired channel (0-7)
        adc_value = read_channel(channel)
        voltage_reads = adc_value * (3.3 / 1023)  # Convert ADC value to voltage
        
        # For some reason this code always get half the voltage so it's multiplied by 2, then multiplied by 2 again because of using voltage divider
        voltage = voltage_reads * 2 * 2
        print(f"Channel {channel}: ADC Value: {adc_value}, Voltage: {voltage:.2f} V")
        voltage_class(voltage)
        time.sleep(1)

except KeyboardInterrupt:
    spi.close()