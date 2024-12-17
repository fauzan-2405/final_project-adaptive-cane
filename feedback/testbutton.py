import RPi.GPIO as GPIO
import time

# Konfigurasi GPIO
BUTTON_PIN = 18  # Pastikan ini adalah pin yang terhubung dengan push button
GPIO.setmode(GPIO.BCM)
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # Menggunakan pull-up internal

# Callback untuk mendeteksi penekanan tombol
def button_callback(channel):
    print("Tombol ditekan!")

# Mengatur event detection untuk push button
GPIO.add_event_detect(BUTTON_PIN, GPIO.FALLING, callback=button_callback, bouncetime=300)

try:
    print("Menunggu penekanan tombol...")
    while True:
        # Loop ini hanya untuk mempertahankan program tetap berjalan
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Pengujian dihentikan oleh pengguna")

finally:
    GPIO.cleanup()
