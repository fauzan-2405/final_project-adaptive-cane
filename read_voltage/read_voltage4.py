import spidev
import time
import logging

# Configure logging
logging.basicConfig(level=logging.INFO)

# Constants for ADC configuration
ADC_MAX_VALUE = 1023
REFERENCE_VOLTAGE = 3.3
VOLTAGE_DIVIDER_FACTOR = 4  # Adjust based on your voltage divider configuration

class MCP3008:
    def __init__(self, bus=0, device=0, max_speed_hz=1350000):
        """Initialize the MCP3008 ADC."""
        self.spi = spidev.SpiDev()
        self.spi.open(bus, device)
        self.spi.max_speed_hz = max_speed_hz

    def read_channel(self, channel):
        """Read the ADC value from the specified channel.

        Parameters:
        channel (int): The channel number (0-7).

        Returns:
        int: The ADC value or None if an error occurs.
        """
        if channel < 0 or channel > 7:
            raise ValueError("Channel must be between 0 and 7")

        command = 0b11 << 6 | channel

        try:
            response = self.spi.xfer2([command, 0])
        except Exception as e:
            logging.error(f"Failed to read from channel {channel}: {e}")
            return None  # Handle error gracefully

        adc_value = ((response[0] & 0x0F) << 8) | response[1]
        return adc_value

    def close(self):
        """Close the SPI connection."""
        self.spi.close()

def voltage_class(voltage):
    """Classify the battery status based on voltage.

    Parameters:
    voltage (float): The voltage to classify.

    Returns:
    str: The battery status message.
    """
    voltage_ranges = {
        4.046: "Battery between 90% and 100%",
        3.981: "Battery between 80% and 90%",
        3.894: "Battery between 70% and 80%",
        3.822: "Battery between 60% and 70%",
        3.747: "Battery between 50% and 60%",
        3.658: "Battery between 40% and 50%",
        3.558: "Battery between 30% and 40%",
        3.467: "Battery between 20% and 30%",
        3.333: "Battery between 10% and 20%",
        0: "Battery below 10%"
    }

    for threshold in sorted(voltage_ranges.keys(), reverse=True):
        if voltage >= threshold:
            return voltage_ranges[threshold]

    return "Invalid voltage"  # Handle case for invalid voltage inputs

def main():
    adc = MCP3008()  # Create an instance of the MCP3008 class

    try:
        while True:
            channel = 0  # Change this to the desired channel (0-7)
            adc_value = adc.read_channel(channel)

            if adc_value is not None:  # Check if the read was successful
                voltage_reads = adc_value * (REFERENCE_VOLTAGE / ADC_MAX_VALUE)
                voltage = voltage_reads * VOLTAGE_DIVIDER_FACTOR
                status_message = voltage_class(voltage)
                logging.info(f"Channel {channel}: ADC Value: {adc_value}, Voltage: {voltage:.2f} V - {status_message}")
            time.sleep(1)

    except KeyboardInterrupt:
        logging.info("Program terminated by user.")
    finally:
        adc.close()  # Ensure SPI connection is closed

if __name__ == "__main__":
    main()
