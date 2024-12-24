import threading
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
feedback_mode = "vibrate"  # Mode default adalah getar
submode_instruction = "voltage"  # Default mode untuk pembacaan tegangan

# Variabel untuk mengelola audio instruksi
instruction_audio = "voltage"
audio_playing = False
#audio_initial_playing = True  # Flag untuk audio awal

lock = threading.Lock()

def play_initial_audio():
    #global audio_initial_playing
    play_audio('tongkat_hidup.mp3')  # Memutar audio awal
    #audio_initial_playing = True  # Set flag ke True

def play_audio(file):
    """Putar audio dengan perlindungan akses multi-threading."""
    global audio_playing
    stop_audio()
    try:
        pygame.mixer.music.load(file)
        pygame.mixer.music.play()
        audio_playing = True
    except Exception as e:
        print(f"Error loading audio file {file}: {e}")

def stop_audio():
    """Hentikan audio dengan perlindungan akses multi-threading."""
    global audio_playing
    if audio_playing:
        pygame.mixer.music.stop()
        audio_playing = False

#------------------------------------------- Instruction ---------------------------------
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
        response = self.spi.xfer2([command, 0])
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

def instruction_mode():
    """Thread untuk menjalankan logika mode instruction."""
    global mode, submode_instruction, adc
    while True:
        motor_pwm.ChangeDutyCycle(0)
        if mode == 'instruction':
            if submode_instruction == 'voltage':
                print("Mode Instruction: Submode Voltage aktif")
                # Logika membaca kapasitas baterai
                adc_value = adc.read_channel(0)  # Pembacaan ADC dilakukan di sini
                voltage_reads = adc_value * (REFERENCE_VOLTAGE / ADC_MAX_VALUE)
                voltage = voltage_reads * VOLTAGE_DIVIDER_FACTOR + ERROR_VALUE
                audio_file = voltage_class(voltage)

                if audio_file:
                    with lock:
                        play_audio(audio_file)
                        audio_playing = True  # Set flag audio_playing ke True

                # Delay berbasis waktu
                start_time = time.time()
                while time.time() - start_time < 8:
                    if mode != 'instruction':  # Respons cepat terhadap perubahan mode
                        break
                    time.sleep(0.1)

            elif submode_instruction == 'technical':
                print("Mode Instruction: Submode Technical aktif")
                with lock:
                    # Memutar audio teknis secara berulang
                    play_audio('ada_objek.mp3')
                    audio_playing = True  # Set flag audio_playing ke True

                # Delay berbasis waktu
                start_time = time.time()
                while time.time() - start_time < 8:
                    if mode != 'instruction':  # Respons cepat terhadap perubahan mode
                        break
                    time.sleep(0.1)
        time.sleep(0.1)

def button_instruction_callback(channel):
    """Callback untuk mengganti submode pada mode instruction."""
    global mode, submode_instruction
    with lock:
        if mode == 'instruction':
            # Toggle submode
            submode_instruction = 'technical' if submode_instruction == 'voltage' else 'voltage'
            print(f"Submode Instruction berubah menjadi: {submode_instruction}")
        else:
            mode = 'instruction'
            submode_instruction = 'voltage'  # Reset ke submode default
            print("Mode berubah menjadi: Instruction")

#------------------------------------------- Ultrasonic ---------------------------------
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
    """
    Mengatur kecepatan getaran motor berdasarkan jarak objek.
    Jarak lebih dekat menghasilkan duty cycle yang lebih tinggi.
    """
    if 0 < distance <= 20 :
        motor_pwm.ChangeDutyCycle(100)
    elif  20 < distance <= 40:
        motor_pwm.ChangeDutyCycle(90)
    elif  40 < distance <= 60:
        motor_pwm.ChangeDutyCycle(80)
    elif  60 < distance <= 80:
        motor_pwm.ChangeDutyCycle(70)
    elif  80 < distance <= 100:
        motor_pwm.ChangeDutyCycle(60)
    elif  100 < distance <= 120:
        motor_pwm.ChangeDutyCycle(50)
    elif  120 < distance <= 140:
        motor_pwm.ChangeDutyCycle(40)
    elif  140 < distance <= 160:
        motor_pwm.ChangeDutyCycle(30)
    elif  160 < distance <= 180:
        motor_pwm.ChangeDutyCycle(20)
    elif distance > 180 :
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

def ultrasonic_mode():
    """Thread untuk menjalankan logika mode ultrasonik."""
    global mode, feedback_mode
    while True:
        with lock:
            if mode != 'ultrasonic':
                break    
            print("Mode Ultrasonik sedang aktif")
            distance_value = distance()

            if distance_value < DISTANCE_THRESHOLD:
                if feedback_mode == 'vibrate':
                    print(f"Jarak: {distance_value} cm - Submode Getar aktif")
                    set_motor_speed(distance_value)  # Aktifkan motor getar sesuai jarak
                elif feedback_mode == 'audio':
                    print(f"Jarak: {distance_value} cm - Submode Audio aktif")
                    frequency = 500 + (DISTANCE_THRESHOLD - distance_value) * 5
                    duration = 0.2
                    play_beep(frequency, duration)  # Aktifkan audio beep
            else:
                print("Jarak lebih dari threshold, motor dimatikan")
                motor_pwm.ChangeDutyCycle(0)  # Matikan motor jika jarak lebih dari threshold

        time.sleep(0.1)

# BUTTON ULTRASONIC CALLBACK
def button_ultrasonic_callback(channel):
    """Callback untuk mengganti submode pada mode ultrasonik."""
    global mode, feedback_mode
    stop_audio()
    with lock:
        if mode == 'ultrasonic':
            # Toggle submode
            feedback_mode = 'audio' if feedback_mode == 'vibrate' else 'vibrate'
            if feedback_mode == "audio":
                motor_pwm.ChangeDutyCycle(0)  # Matikan motor saat beralih ke audio
            print(f"Submode Ultrasonik berubah menjadi: {feedback_mode}")
        else:
            mode = 'ultrasonic'
            feedback_mode = 'vibrate'  # Reset ke submode default
            print("Mode berubah menjadi: Ultrasonik")

#------------------------------------------- Detection ---------------------------------
class ObjectDetector:
    def __init__(self, model_path, min_conf_threshold=0.7):
        self.interpreter = Interpreter(model_path=model_path)
        self.interpreter.allocate_tensors()
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()
        self.height = self.input_details[0]['shape'][1]
        self.width = self.input_details[0]['shape'][2]
        self.min_conf_threshold = min_conf_threshold

    def preprocess_image(self, image):
        """Ubah gambar menjadi format yang sesuai untuk model."""
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image_resized = cv2.resize(image_rgb, (self.width, self.height))
        return np.expand_dims(image_resized, axis=0).astype(np.uint8)

    def run_detection(self, image):
        input_data = self.preprocess_image(image)
        self.interpreter.set_tensor(self.input_details[0]['index'], input_data)
        self.interpreter.invoke()

        boxes = self.interpreter.get_tensor(self.output_details[1]['index'])[0]
        classes = self.interpreter.get_tensor(self.output_details[3]['index'])[0]
        scores = self.interpreter.get_tensor(self.output_details[0]['index'])[0]

        # Gunakan list comprehension untuk membuat daftar deteksi
        detections = [
            (int(classes[i]), scores[i], *boxes[i])
            for i in range(len(scores)) if scores[i] > self.min_conf_threshold
        ]
        return detections

def capture_image():
    image_name = "image_0.jpg"
    capture_command = f"libcamera-still --width 320 --height 240 --nopreview --immediate -o {image_name}"
    os.system(capture_command)
    image = cv2.imread(image_name)
    if image is None:
        print("Error: Gambar tidak berhasil diambil.")
        return None
    return image

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

def detection_mode():
    """Thread untuk mode deteksi objek."""
    global mode
    while True:
        with lock:
            if mode != 'detection':
                break
        print("Mode Detection sedang aktif")
        image = capture_image()
        if image is None:
            print("Error: Gambar tidak tersedia, pendeteksian diabaikan.")
            continue
        if detector is None:
            print("Error: Detektor tidak diinisialisasi.")
            break

        detections = detector.run_detection(image)
        if not detections:
            print("Tidak ada objek yang terdeteksi.")
            play_audio('takada_objek.mp3')
        else:
            print(f"Deteksi ditemukan: {detections}")
            play_audio_for_detections(detections)

        # Delay berbasis waktu untuk respons cepat
        start_time = time.time()
        while time.time() - start_time < 0.5:
            with lock:
                if mode != 'detection':
                    break
            time.sleep(0.1)
        time.sleep(0.1)

# BUTTON DETECTION
def button_detection_callback(channel):
    global mode
    with lock:
        stop_audio()  # Hentikan audio jika sebelumnya sedang diputar
        mode = 'detection'
        print("Beralih ke mode deteksi objek")


#------------------------------------------- Main ---------------------------------

def main():
    global mode, adc, detector
    GPIO.add_event_detect(BUTTON_ULTRASONIC_PIN, GPIO.FALLING, callback=button_ultrasonic_callback, bouncetime=300)
    GPIO.add_event_detect(BUTTON_DETECTION_PIN, GPIO.FALLING, callback=button_detection_callback, bouncetime=300)
    GPIO.add_event_detect(BUTTON_INSTRUCTION_PIN, GPIO.FALLING, callback=button_instruction_callback, bouncetime=300)

    # Inisialisasi class
    adc = MCP3008()  # Inisialisasi ADC di sini
    print("adc selese, detector?")
    detector = ObjectDetector(model_path="custom_model_lite/detect.tflite", min_conf_threshold=0.7) # Inisialisasi model deteksi objek
    print("detector selese")

    play_audio('tongkat_hidup.mp3')
    print("Tekan tombol untuk mengubah mode...")

    threads = [
        threading.Thread(target=ultrasonic_mode, daemon=True),
        threading.Thread(target=detection_mode, daemon=True),
        threading.Thread(target=instruction_mode, daemon=True),
    ]
    for thread in threads:
        thread.start()
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        GPIO.cleanup()
        adc.close()
        pygame.mixer.quit()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Program dihentikan.")
    finally:
        motor_pwm.stop()
        GPIO.cleanup()
        adc.close()
        pygame.mixer.quit()