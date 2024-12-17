import time
import RPi.GPIO as GPIO
import numpy as np
import pygame

# Setup GPIO untuk sensor ultrasonik
TRIG = 4   # GPIO4 sebagai Trigger
ECHO = 17  # GPIO17 sebagai Echo
BUTTON_PIN = 18  # GPIO18 untuk button mode feedback
GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # Setup button dengan internal pull-up

# Setup GPIO dan PWM untuk motor getar
motor_pin = 13  # Pin GPIO yang digunakan untuk motor getar
GPIO.setup(motor_pin, GPIO.OUT)
motor_pwm = GPIO.PWM(motor_pin, 1000)
motor_pwm.start(0)  # Mulai PWM dengan duty cycle 0 (motor mati)

# Setup pygame untuk audio feedback
pygame.mixer.init(frequency=44100)

# Konfigurasi jarak
DISTANCE_THRESHOLD = 100  # Batas jarak untuk aktifkan feedback (100 cm)
feedback_mode = "vibrate"  # Mode default adalah getar

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
    return (TimeElapsed * 34300) / 2  # Hasil dalam cm

def set_motor_speed(distance):
    """
    Mengatur kecepatan getaran motor berdasarkan jarak objek.
    """
    if distance < DISTANCE_THRESHOLD:
        duty_cycle = 100 * (1 - (distance / DISTANCE_THRESHOLD))
        motor_pwm.ChangeDutyCycle(duty_cycle)
    else:
        motor_pwm.ChangeDutyCycle(0)

def play_beep(frequency, duration):
    """
    Menghasilkan suara beep pada earphone Bluetooth.
    """
    sample_rate = 44100
    t = np.linspace(0, duration, int(sample_rate * duration), False)
    wave = np.sin(frequency * 2 * np.pi * t)
    sound = np.array(wave * 32767, dtype=np.int16)
    stereo_sound = np.column_stack((sound, sound))
    beep_sound = pygame.sndarray.make_sound(stereo_sound)
    beep_sound.play()
    time.sleep(duration)
    beep_sound.stop()

def button_callback(channel):
    """
    Mengubah mode feedback antara getar dan audio saat button ditekan.
    """
    global feedback_mode
    feedback_mode = "audio" if feedback_mode == "vibrate" else "vibrate"
    if feedback_mode == "audio":
         motor_pwm.ChangeDutyCycle(0)

    print(f"Mode feedback diubah menjadi: {feedback_mode}")

# Event detection untuk button
GPIO.add_event_detect(BUTTON_PIN, GPIO.FALLING, callback=button_callback, bouncetime=300)

try:
    while True:
        dist = distance()
        if dist < DISTANCE_THRESHOLD:
            if feedback_mode == "vibrate":
                set_motor_speed(dist)
            elif feedback_mode == "audio":
                frequency = 500 + (100 - dist) * 5  # Meningkatkan frekuensi jika jarak lebih dekat
                duration = 0.1
                play_beep(frequency, duration)
        else:
            motor_pwm.ChangeDutyCycle(0)  # Matikan motor jika jarak lebih dari threshold

        time.sleep(0.1)  # Interval pengukuran

except KeyboardInterrupt:
    print("Program dihentikan oleh pengguna")

finally:
    motor_pwm.stop()
    GPIO.cleanup()
