import time
import RPi.GPIO as GPIO

# Setup GPIO untuk sensor ultrasonik
TRIG = 4  # GPIO4 sebagai Trigger
ECHO = 17  # GPIO17 sebagai Echo
GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

# Setup GPIO dan PWM untuk motor getar
motor_pin = 13  # Pin GPIO yang digunakan untuk motor getar
GPIO.setup(motor_pin, GPIO.OUT)

# Setup PWM untuk motor dengan frekuensi 1000 Hz
motor_pwm = GPIO.PWM(motor_pin, 1000)
motor_pwm.start(0)  # Mulai PWM dengan duty cycle 0 (motor mati)

# Definisikan jarak maksimum dan ambang batas getaran
DISTANCE_THRESHOLD = 100  # Ambang batas jarak dalam cm

def distance():
    """
    Mengukur jarak menggunakan sensor ultrasonik
    """
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    StartTime = time.time()
    StopTime = time.time()

    while GPIO.input(ECHO) == 0:
        StartTime = time.time()

    while GPIO.input(ECHO) == 1:
        StopTime = time.time()

    # Hitung jarak berdasarkan waktu
    TimeElapsed = StopTime - StartTime
    return (TimeElapsed * 34300) / 2

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
        dist = distance()
        if dist is not None:
            print(f"Jarak: {dist:.1f} cm")
            # Atur kecepatan motor getar berdasarkan jarak yang terdeteksi
            set_motor_speed(dist)
        else:
            print("Tidak ada objek terdeteksi")
            motor_pwm.ChangeDutyCycle(0)  # Matikan motor jika tidak ada objek terdeteksi
        
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Program dihentikan oleh pengguna")

finally:
    # Bersihkan konfigurasi GPIO dan hentikan PWM
    motor_pwm.stop()
    GPIO.cleanup()
