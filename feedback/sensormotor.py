import time
import board
import busio
import RPi.GPIO as GPIO
from adafruit_vl53l1x import VL53L1X

# Setup I2C untuk sensor jarak VL53L1X
i2c = busio.I2C(board.SCL, board.SDA)
vl53 = VL53L1X(i2c)

# Set timing budget untuk sensor (semakin kecil, semakin cepat sensor memberikan data)
vl53.timing_budget = 100  # dalam ms
vl53.start_ranging()

# Setup GPIO dan PWM untuk vibration motor
motor_pin = 13  # Pin GPIO yang digunakan untuk motor getar
GPIO.setmode(GPIO.BCM)
GPIO.setup(motor_pin, GPIO.OUT)

# Setup PWM untuk motor dengan frekuensi 1000 Hz
motor_pwm = GPIO.PWM(motor_pin, 1000)
motor_pwm.start(0)  # Mulai PWM dengan duty cycle 0 (motor mati)

# Definisikan jarak maksimum dan ambang batas getaran
MAX_RANGE = 4000  # Maksimal jarak deteksi 4 meter
DISTANCE_THRESHOLD = 1000  # Batas jarak 1 meter (1000 mm)

def set_motor_speed(distance):
    """
    Mengatur kecepatan getaran motor berdasarkan jarak objek.
    Jarak lebih dekat menghasilkan duty cycle yang lebih tinggi.
    """
    if distance < DISTANCE_THRESHOLD:
        # Normalisasi jarak ke duty cycle (0-100)
        # Semakin dekat, duty cycle semakin tinggi
        duty_cycle = 100 * (1 - (distance / DISTANCE_THRESHOLD))
        motor_pwm.ChangeDutyCycle(duty_cycle)
    else:
        motor_pwm.ChangeDutyCycle(0)  # Matikan motor jika objek lebih jauh dari threshold

try:
    while True:
        if vl53.data_ready:
            distance = vl53.distance
            if distance is not None:
                print(f"Distance: {distance} mm")
                
                # Atur kecepatan motor getar berdasarkan jarak yang terdeteksi
                set_motor_speed(distance)
            else:
                print(f"Distance: Greater than {MAX_RANGE} mm")
                motor_pwm.ChangeDutyCycle(0)  # Matikan motor jika tidak ada objek terdeteksi

            vl53.clear_interrupt()  # Bersihkan interrupt sensor
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Program dihentikan oleh pengguna")

finally:
    # Bersihkan konfigurasi GPIO dan hentikan PWM
    motor_pwm.stop()
    GPIO.cleanup()
    vl53.stop_ranging()
