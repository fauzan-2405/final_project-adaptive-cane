import os
import time
import numpy as np
import cv2
import RPi.GPIO as GPIO
from tflite_runtime.interpreter import Interpreter
import pygame
import spidev
import logging

# Konfigurasi logging
logging.basicConfig(level=logging.INFO)

# Konstanta untuk konfigurasi ADC
ADC_MAX_VALUE = 1023
REFERENCE_VOLTAGE = 3.3
VOLTAGE_DIVIDER_FACTOR = 4  # Sesuaikan berdasarkan konfigurasi pembagi tegangan
ERROR_VALUE = 0.28

# Setup GPIO
BUTTON_ULTRASONIC_PIN = 18  # Tombol untuk mode ultrasonik
BUTTON_DETECTION_PIN = 12    # Tombol untuk mode deteksi objek
BUTTON_INSTRUCTION_PIN = 24   # Tombol untuk instruksi
GPIO.setmode(GPIO.BCM)
GPIO.setup(BUTTON_ULTRASONIC_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(BUTTON_DETECTION_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(BUTTON_INSTRUCTION_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Setup motor getar
motor_pin = 13
GPIO.setup(motor_pin, GPIO.OUT)
motor_pwm = GPIO.PWM(motor_pin, 1000)
motor_pwm.start(0)

# Setup pygame untuk audio feedback
pygame.mixer.init(frequency=44100)

# Setup sensor ultrasonik
TRIG = 4
ECHO = 17
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

# Konfigurasi jarak
DISTANCE_THRESHOLD = 100  # Batas jarak untuk aktifkan feedback (216 cm)
mode = 'ultrasonic'
prev_mode = None
feedback_mode = "vibrate"  # Mode default adalah getar
instruction_mode = "voltage"  # Default mode untuk pembacaan tegangan

# Variabel untuk mengelola audio instruksi
instruction_audio = "voltage"
audio_playing = False
audio_initial_playing = True  # Flag untuk audio awal

class MCP3008:
    def __init__(self, bus=0, device=0, max_speed_hz=1350000):
        """Inisialisasi ADC MCP3008."""
        self.spi = spidev.SpiDev()
        self.spi.open(bus, device)
        self.spi.max_speed_hz = max_speed_hz

    def read_channel(self, channel):
        """Membaca nilai ADC dari saluran yang ditentukan."""
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
    """Mengklasifikasikan status baterai berdasarkan tegangan dan mengembalikan file audio yang sesuai."""
    voltage_ranges = {
        4.046: "90persen.mp3",  # 90% - 100%
        3.981: "80persen.mp3",  # 80% - 90%
        3.894: "70persen.mp3",  # 70% - 80%
        3.822: "60persen.mp3",  # 60% - 70%
        3.747: "50persen.mp3",  # 50% - 60%
        3.658: "40persen.mp3", # 40% - 50%
        3.558: "30persen.mp3",  # 30% - 40%
        3.467: "20persen.mp3",  # 20% - 30%
        3.333: "10persen.mp3",  # 10% - 20%
        0: "0persen.mp3"        # Di bawah 10%
    }

    for threshold in sorted(voltage_ranges.keys(), reverse=True):
        if voltage >= threshold:
            return voltage_ranges[threshold]

    return None  # Tangani kasus untuk input tegangan yang tidak valid

def play_initial_audio():
    global audio_initial_playing
    play_audio('tongkat_hidup.mp3')  # Memutar audio awal
    audio_initial_playing = True  # Set flag ke True

def play_audio(file):
    stop_audio()  # Hentikan audio yang sedang diputar
    try:
        pygame.mixer.music.load(file)
        pygame.mixer.music.play()
        global audio_playing
        audio_playing = True  # Set flag audio_playing ke True
    except Exception as e:
        print(f"Error loading audio file {file}: {e}")

def stop_audio():
    global audio_playing
    if audio_playing:
        pygame.mixer.music.stop()
        audio_playing = False

def distance():
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    StartTime = time.time()
    StopTime = time.time()

    while GPIO.input(ECHO) == 0:
        StartTime = time.time()

    while GPIO.input(ECHO) == 1:
        StopTime = time.time()

    TimeElapsed = StopTime - StartTime
    return (TimeElapsed * 34300) / 2  # Hasil dalam cm

def set_motor_speed(distance):
    if distance < DISTANCE_THRESHOLD:
        duty_cycle = 100 * (1 - (distance / DISTANCE_THRESHOLD))
        motor_pwm.ChangeDutyCycle(duty_cycle)
    else:
        motor_pwm.ChangeDutyCycle(0)

def play_beep(frequency, duration):
    """Menghasilkan suara beep pada earphone Bluetooth."""
    sample_rate = 44100
    t = np.linspace(0, duration, int(sample_rate * duration), False)
    wave = np.sin(frequency * 2 * np.pi * t) * 0.06  # Mengurangi amplitudo menjadi 20%

    # Fade in dan fade out
    fade_in = np.linspace(0, 1, int(sample_rate * duration * 0.1))  # 10% dari durasi untuk fade in
    fade_out = np.linspace(1, 0, int(sample_rate * duration * 0.1))  # 10% dari durasi untuk fade out
    wave[:len(fade_in)] *= fade_in
    wave[-len(fade_out):] *= fade_out

    sound = np.array(wave * 32767, dtype=np.int16)
    stereo_sound = np.column_stack((sound, sound))
    beep_sound = pygame.sndarray.make_sound(stereo_sound)
    beep_sound.play()
    time.sleep(duration)
    beep_sound.stop()

def run_detection(image):
    model_dir = "custom_model_lite/"
    graph_name = "detect.tflite"
    min_conf_threshold = 0.7

    interpreter = Interpreter(model_path=os.path.join(model_dir, graph_name))
    interpreter.allocate_tensors()

    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()
    height = input_details[0]['shape'][1]
    width = input_details[0]['shape'][2]

    image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    image_resized = cv2.resize(image_rgb, (width, height))
    input_data = np.expand_dims(image_resized, axis=0)
    input_data = np.clip(input_data, 0, 255).astype(np.uint8)

    interpreter.set_tensor(input_details[0]['index'], input_data)
    interpreter.invoke()

    boxes = interpreter.get_tensor(output_details[1]['index'])[0]
    classes = interpreter.get_tensor(output_details[3]['index'])[0]
    scores = interpreter.get_tensor(output_details[0]['index'])[0]

    detections = []
    for i in range(len(scores)):
        if scores[i] > min_conf_threshold:
            ymin, xmin, ymax, xmax = boxes[i]
            detections.append((int(classes[i]), scores[i], xmin, ymin, xmax, ymax))

    return detections

def capture_image():
    image_name = "image_0.jpg"
    capture_command = f"libcamera-still --width 320 --height 240 --nopreview --immediate -o {image_name}"
    os.system(capture_command)
    return cv2.imread(image_name)

def button_ultrasonic_callback(channel):
    global mode, feedback_mode
    stop_audio()  # Hentikan audio jika sebelumnya sedang diputar
    if mode == 'ultrasonic':
        feedback_mode = "audio" if feedback_mode == "vibrate" else "vibrate"
        if feedback_mode == "audio":
            motor_pwm.ChangeDutyCycle(0)  # Matikan motor saat beralih ke audio
        print(f"Mode feedback diubah menjadi: {feedback_mode}")
    else:
        mode = 'ultrasonic'
        feedback_mode = "vibrate"  # Reset ke mode getar
        print("Beralih ke mode sensor ultrasonik.")

def button_detection_callback(channel):
    global mode
    stop_audio()  # Hentikan audio jika sebelumnya sedang diputar
    mode = 'detection'
    print("Beralih ke mode deteksi objek")

def button_instruction_callback(channel):
    global mode, instruction_mode, instruction_audio, audio_playing
    stop_audio()  # Hentikan audio jika sebelumnya sedang diputar
    if mode == 'instruction':
        # Ganti mode instruksi
        instruction_mode = "technical" if instruction_mode == "voltage" else "voltage"
        print(f"Mode instruksi diubah menjadi: {instruction_mode}")

        # Tentukan audio yang akan diputar berdasarkan mode
        if instruction_mode == "voltage":
            instruction_audio = voltage_class(adc.read_channel(0) * (REFERENCE_VOLTAGE / ADC_MAX_VALUE) * VOLTAGE_DIVIDER_FACTOR)
        else:
            instruction_audio = "ada_objek.mp3"  # Ganti dengan file audio yang sesuai untuk mode technical

        # Putar audio baru
        if instruction_audio:
            play_audio(instruction_audio)  # Ganti os.system dengan play_audio
            audio_playing = True  # Set flag audio_playing ke True
            print("Instruction Audio = ", instruction_audio)

    else:
        mode = 'instruction'
        instruction_mode = "voltage"  
        print("Beralih ke mode instruksi.") 

def play_audio_for_detections(detections):
    audio_files = {
        0: "ada_got.mp3",        # Kelas 0: got
        1: "ada_tanjakan.mp3",   # Kelas 1: tanjakan
        2: "ada_upstair.mp3",    # Kelas 2: upstair
        3: "ada_downstair.mp3"   # Kelas 3: downstair
    }
    played_files = set()
    for detection in detections:
        class_id = detection[0]
        if class_id in audio_files and audio_files[class_id] not in played_files:
            play_audio(audio_files[class_id])  # Ganti os.system dengan play_audio
            played_files.add(audio_files[class_id])

def main():
    global mode, adc
    adc = MCP3008()  # Inisialisasi ADC di sini
    play_initial_audio() 
    print("Tekan tombol untuk mengubah mode...")

    GPIO.add_event_detect(BUTTON_ULTRASONIC_PIN, GPIO.FALLING, callback=button_ultrasonic_callback, bouncetime=300)
    GPIO.add_event_detect(BUTTON_DETECTION_PIN, GPIO.FALLING, callback=button_detection_callback, bouncetime=300)
    GPIO.add_event_detect(BUTTON_INSTRUCTION_PIN, GPIO.FALLING, callback=button_instruction_callback, bouncetime=300)

    while True:
        if mode == 'ultrasonic':
            distance_value = distance()
            if distance_value < DISTANCE_THRESHOLD:
                if feedback_mode == "vibrate":
                    set_motor_speed(distance_value)
                elif feedback_mode == "audio":
                    frequency = 500 + (100 - distance_value) * 5  # Meningkatkan frekuensi jika jarak lebih dekat
                    duration = 0.2
                    play_beep(frequency, duration)  # Memutar beep sebagai feedback audio
            else:
                motor_pwm.ChangeDutyCycle(0)  # Matikan motor jika jarak lebih dari threshold

        elif mode == 'detection':
            motor_pwm.ChangeDutyCycle(0)  # Matikan motor jika jarak lebih dari threshold
            if GPIO.input(BUTTON_DETECTION_PIN) == GPIO.LOW:
                image = capture_image()
                detections = run_detection(image)
                if detections:
                    play_audio_for_detections(detections)
                else:
                    play_audio('takada_objek.mp3')  # Ganti os.system dengan play_audio
                time.sleep(0.5)  # Delay untuk menghindari pengambilan gambar berulang
        
        elif mode == 'instruction':
            motor_pwm.ChangeDutyCycle(0)
            if instruction_mode == "voltage":
                adc_value = adc.read_channel(0)  # Pembacaan ADC dilakukan di sini
                voltage_reads = adc_value * (REFERENCE_VOLTAGE / ADC_MAX_VALUE)
                voltage = voltage_reads * VOLTAGE_DIVIDER_FACTOR + ERROR_VALUE
                status_message = voltage_class(voltage)

                if status_message:
                    play_audio(status_message)  # Ganti os.system dengan play_audio
                    audio_playing = True  # Set flag audio_playing ke True
                    print("Instruction Audio = ", status_message)
                    time.sleep(8)  # Delay untuk menghindari pengulangan audio yang terlalu cepat
                        
            elif instruction_mode == "technical":
                play_audio('ada_objek.mp3')  # Ganti os.system dengan play_audio
                audio_playing = True  # Set flag audio_playing ke True
                print("Instruction Audio = ", instruction_audio)
                time.sleep(8)  # Delay untuk menghindari pengulangan audio yang terlalu cepat
        
if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Program dihentikan.")
    finally:
        motor_pwm.stop()
        GPIO.cleanup()
        adc.close()  # Pastikan koneksi ADC ditutup saat program selesai
        pygame.mixer.quit()  # Hentikan pygame mixer
