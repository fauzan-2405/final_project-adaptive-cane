import RPi.GPIO as GPIO
import time

# Definisikan GPIO mode (BCM atau BOARD)
GPIO.setmode(GPIO.BCM)

# Definisikan pin Trig dan Echo
TRIG = 4  # GPIO4 sebagai Trigger
ECHO = 17  # GPIO17 sebagai Echo

# Set up GPIO sebagai output (Trig) dan input (Echo)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

def distance():
    # Set Trigger to HIGH
    GPIO.output(TRIG, True)
    # Set Trigger after 0.01ms to LOW
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    # Hitung waktu mulai dan waktu berhenti
    StartTime = time.time()
    StopTime = time.time()

    # Simpan waktu mulai
    while GPIO.input(ECHO) == 0:
        StartTime = time.time()

    # Simpan waktu tiba di echo
    while GPIO.input(ECHO) == 1:
        StopTime = time.time()

    # Hitung selisih waktu antara Start dan Stop
    TimeElapsed = StopTime - StartTime
    # Kecepatan suara = 34300 cm/s
    distance = (TimeElapsed * 34300) / 2

    return distance

try:
    while True:
        dist = distance()
        print("Jarak: %.1f cm" % dist)
        time.sleep(1)

except KeyboardInterrupt:
    print("Pengukuran dihentikan oleh pengguna")
    GPIO.cleanup()
