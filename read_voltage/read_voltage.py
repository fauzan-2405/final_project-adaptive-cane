import spidev
import time

class MCP3008:
    def __init__(self, bus=0, device=0):
        self.spi = spidev.SpiDev()
        self.spi.open(bus, device)
        self.spi.max_speed_hz = 1350000

    def read_channel(self, channel):
        if channel < 0 or channel > 7:
            raise ValueError("Channel must be between 0 and 7")

        # MCP3008 uses 3 bits for the control byte
        command = 0b00000001
        command <<= 4
        command |= (channel & 0b00000111)
        command <<= 1  # Single-ended mode

        # Send the command and read the response
        response = self.spi.xfer2([command, 0])
        adc_value = ((response[0] & 0b00000011) << 8) | response[1]
        return adc_value

    def close(self):
        self.spi.close()

def adc_value_to_voltage(adc_value):
    return (adc_value / 1023.0) * 3.3  # Convert ADC value to voltage

if __name__ == "__main__":
    mcp = MCP3008()

    try:
        print("Starting voltage measurement...")
        while True:
            adc_value = mcp.read_channel(1)  # Read from channel 1
            voltage = adc_value_to_voltage(adc_value)
            print(f"ADC Value: {adc_value}, Voltage: {voltage:.2f} V")
            time.sleep(1)  # Wait for 1 second before the next measurement
    except KeyboardInterrupt:
        print("Measurement stopped by user.")
    finally:
        mcp.close()
        print("SPI connection closed.")
