import spidev
import time
import logging
import os  # Import os untuk memutar file audio

# Konfigurasi logging
logging.basicConfig(level=logging.INFO)

# Konstanta untuk konfigurasi ADC
ADC_MAX_VALUE = 1023
REFERENCE_VOLTAGE = 3.3
VOLTAGE_DIVIDER_FACTOR = 4  # Sesuaikan berdasarkan konfigurasi pembagi tegangan
ERROR_VALUE = 0.28

class MCP3008:
    def __init__(self, bus=0, device=0, max_speed_hz=1350000):
        """Inisialisasi ADC MCP3008."""
        self.spi = spidev.SpiDev()
        self.spi.open(bus, device)
        self.spi.max_speed_hz = max_speed_hz

    def read_channel(self, channel):
        """Membaca nilai ADC dari saluran yang ditentukan.

        Parameter:
        channel (int): Nomor saluran (0-7).

        Mengembalikan:
        int: Nilai ADC atau None jika terjadi kesalahan.
        """
        if channel < 0 or channel > 7:
            raise ValueError("Saluran harus antara 0 dan 7")

        command = 0b11 << 6 | channel

        try:
            response = self.spi.xfer2([command, 0])
        except Exception as e:
            logging.error(f"Gagal membaca dari saluran {channel}: {e}")
            return None  # Tangani kesalahan dengan baik

        adc_value = ((response[0] & 0x0F) << 8) | response[1]
        return adc_value

    def close(self):
        """Menutup koneksi SPI."""
        self.spi.close()

def voltage_class(voltage):
    """Mengklasifikasikan status baterai berdasarkan tegangan dan mengembalikan file audio yang sesuai.

    Parameter:
    voltage (float): Tegangan untuk diklasifikasikan.

    Mengembalikan:
    str: File audio yang akan diputar berdasarkan status baterai.
    """
    voltage_ranges = {
        4.046: "90persen.m4a",  # 90% - 100%
        3.981: "80persen.m4a",  # 80% - 90%
        3.894: "70persen.m4a",  # 70% - 80%
        3.822: "60persen.m4a",  # 60% - 70%
        3.747: "50persen.m4a",  # 50% - 60%
        3.658: "40persen.m4a",  # 40% - 50%
        3.558: "30persen.m4a",  # 30% - 40%
        3.467: "20persen.m4a",  # 20% - 30%
        3.333: "10persen.m4a",  # 10% - 20%
        0: "0persen.m4a"        # Di bawah 10%
    }

    for threshold in sorted(voltage_ranges.keys(), reverse=True):
        if voltage >= threshold:
            return voltage_ranges[threshold]

    return None  # Tangani kasus untuk input tegangan yang tidak valid

def main():
    adc = MCP3008()  # Membuat instance dari kelas MCP3008
    last_audio_file = None  # Melacak file audio terakhir yang diputar

    try:
        while True:
            channel = 0  # Ubah ini ke saluran yang diinginkan (0-7)
            adc_value = adc.read_channel(channel)

            if adc_value is not None:  # Periksa apakah pembacaan berhasil
                voltage_reads = adc_value * (REFERENCE_VOLTAGE / ADC_MAX_VALUE)
                voltage = voltage_reads * VOLTAGE_DIVIDER_FACTOR + ERROR_VALUE
                status_message = voltage_class(voltage)

                if status_message and status_message != last_audio_file:
                    os.system(f'mpv {status_message}')  # Memutar file audio
                    last_audio_file = status_message  # Memperbarui file audio terakhir yang diputar

                logging.info(f"Saluran {channel}: Nilai ADC: {adc_value}, Tegangan: {voltage:.2f} V - {status_message}")
            time.sleep(1)

    except KeyboardInterrupt:
        logging.info("Program dihentikan oleh pengguna.")
    finally:
        adc.close()  # Pastikan koneksi SPI ditutup

if __name__ == "__main__":
    main()
