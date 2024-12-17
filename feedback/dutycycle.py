import RPi.GPIO as GPIO
import time

# Setup
motor_pin = 13  # Pin GPIO yang digunakan untuk motor getar

# Gunakan penomoran pin GPIO (BCM mode)
GPIO.setmode(GPIO.BCM)

# Set pin sebagai OUTPUT
GPIO.setup(motor_pin, GPIO.OUT)

# Setup PWM untuk motor dengan frekuensi 1000 Hz
motor_pwm = GPIO.PWM(motor_pin, 1000)
motor_pwm.start(0)  # Mulai PWM dengan duty cycle 0 (motor mati)

try:
    motor_pwm.ChangeDutyCycle(10)
    print("Duty Cycle 10")
    time.sleep(5)
    
    motor_pwm.ChangeDutyCycle(20)
    print("Duty Cycle 20")
    time.sleep(5)

    motor_pwm.ChangeDutyCycle(30)
    print("Duty Cycle 30")
    time.sleep(5)
    
    motor_pwm.ChangeDutyCycle(40)
    print("Duty Cycle 40")
    time.sleep(5)

    motor_pwm.ChangeDutyCycle(50)
    print("Duty Cycle 50")
    time.sleep(5)
    
    motor_pwm.ChangeDutyCycle(60)
    print("Duty Cycle 60")
    time.sleep(5)

    motor_pwm.ChangeDutyCycle(70)
    print("Duty Cycle 70")
    time.sleep(5)
    
    motor_pwm.ChangeDutyCycle(80)
    print("Duty Cycle 80")
    time.sleep(5)

    motor_pwm.ChangeDutyCycle(90)
    print("Duty Cycle 90")
    time.sleep(5)
    
    motor_pwm.ChangeDutyCycle(100)
    print("Duty Cycle 100")
    time.sleep(5)

except KeyboardInterrupt:
    print("Program dihentikan oleh pengguna")

finally:
    # Bersihkan konfigurasi GPIO
    GPIO.cleanup()
