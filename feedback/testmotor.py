import RPi.GPIO as GPIO
import time

# Setup
motor_pin = 13  # Pin GPIO yang digunakan untuk motor getar

# Gunakan penomoran pin GPIO (BCM mode)
GPIO.setmode(GPIO.BCM)

# Set pin sebagai OUTPUT
GPIO.setup(motor_pin, GPIO.OUT)

try:
    # Nyalakan motor selama 5 detik
    print("Menyalakan motor...")
    GPIO.output(motor_pin, GPIO.HIGH)
    time.sleep(5)
    
    # Matikan motor
    print("Mematikan motor...")
    GPIO.output(motor_pin, GPIO.LOW)

except KeyboardInterrupt:
    print("Program dihentikan oleh pengguna")

finally:
    # Bersihkan konfigurasi GPIO
    GPIO.cleanup()
