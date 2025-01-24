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
ERROR_VALUE = 0.25

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
DISTANCE_THRESHOLD = 216  # Batas jarak untuk aktifkan feedback (216 cm)
mode = 'ultrasonic'
feedback_mode = "vibrate"  # Mode default adalah getar
submode_instruction = "voltage"  # Default mode untuk pembacaan tegangan

# Variabel untuk mengelola audio instruksi
instruction_audio = "voltage"
audio_playing = False
#audio_initial_playing = True  # Flag untuk audio awal

#lock = threading.Lock()

def play_audio(file):
    """Putar audio dengan perlindungan akses multi-threading."""
    global audio_playing
    stop_audio()
    try:
        pygame.mixer.music.set_volume(1.0)
        pygame.mixer.music.load(file)
        pygame.mixer.music.play()
        audio_playing = True
    except Exception as e:
        print(f"Error loading audio file {file}: {e}")

def stop_audio():
    """Hentikan audio dengan perlindungan akses multi-threading."""
    global audio_playing
    if audio_playing and pygame.mixer.music.get_busy():
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
        if not instruction_event.is_set():  # Jika event tidak aktif, thread berhenti
            print("Thread instruction_mode berhenti.")
            break

        print("Mode instruction_mode sedang aktif")
        motor_pwm.ChangeDutyCycle(0)
        # Delay berbasis waktu
        start_time = time.time()
        while time.time() - start_time < 0.1:
            if mode != 'instruction':  # Respons cepat terhadap perubahan mode
                break
            time.sleep(0.1)

        if mode == 'instruction':
            if submode_instruction == 'voltage':
                print("Mode Instruction: Submode Voltage aktif")
                # Logika membaca kapasitas baterai
                adc_value = adc.read_channel(0)  # Pembacaan ADC dilakukan di sini
                voltage_reads = adc_value * (REFERENCE_VOLTAGE / ADC_MAX_VALUE)
                voltage = voltage_reads * VOLTAGE_DIVIDER_FACTOR + ERROR_VALUE
                audio_file = voltage_class(voltage)
                logging.info(f"Saluran 0: Nilai ADC: {adc_value}, Tegangan: {voltage: .2f} V - {audio_file}")

                if audio_file:
                    play_audio(audio_file)
                    audio_playing = True  # Set flag audio_playing ke True


            elif submode_instruction == 'technical':
                print("Mode Instruction: Submode Technical aktif")
                # Memutar audio teknis secara berulang
                play_audio('instruction.mp3')
                audio_playing = True  # Set flag audio_playing ke True

        while pygame.mixer.music.get_busy():  # Tunggu sampai audio selesai
            if mode != 'instruction' or submode_instruction not in ['voltage', 'technical']:
                print("Mode atau submode berubah, audio dihentikan.")
                pygame.mixer.music.stop()  # Hentikan jika ada perubahan mode/submode
                break
            time.sleep(0.1)
        time.sleep(0.1)

def button_instruction_callback(channel):
    """Callback untuk mengganti submode pada mode instruction."""
    global mode, submode_instruction
    print("Tombol Instruction ditekan")
    detection_event.clear()
    instruction_event.set() # Hidupkan event instruction
    ultrasonic_event.clear()

    #mode = 'instruction'
    stop_audio()
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
    if distance > 216 or distance <= 0:
        duty_cycle = 0
    else:
        duty_cycle = 100 - (distance / 216) * 95

    motor_pwm.ChangeDutyCycle(duty_cycle)

def play_beep(frequency, duration, count, pause):
    """Memutar beep dengan frekuensi tertentu sebanyak beberapa kali."""
    sample_rate = 44100
    t = np.linspace(0, duration, int(sample_rate * duration), False)
    wave = np.sin(frequency * 2 * np.pi * t) * 0.1
    sound = np.array(wave * 32767, dtype=np.int16)
    stereo_sound = np.column_stack((sound, sound))
    beep_sound = pygame.sndarray.make_sound(stereo_sound)
    beep_sound.set_volume(1.0)

    for _ in range(count):
        beep_sound.play()
        time.sleep(duration)
        beep_sound.stop()
        time.sleep(pause)

def ultrasonic_mode():
    """Thread untuk menjalankan logika mode ultrasonik."""
    global mode, feedback_mode
    print("Thread ultrasonic_mode dimulai.")
    while True:
        if not ultrasonic_event.is_set():
            print("Thread ultrasonic_mode berhenti.")
            stop_audio()
            break

        print("Mode Ultrasonik sedang aktif")
        distance_value = distance()

        if distance_value < DISTANCE_THRESHOLD:
            if feedback_mode == 'vibrate':
                print(f"Jarak: {distance_value} cm - Submode Getar aktif")
                set_motor_speed(distance_value)
            elif feedback_mode == 'audio':
                print(f"Jarak: {distance_value} cm - Submode Audio aktif")
                if 0 < distance_value <= 72:
                    beep_count = 1
                    play_beep(1000, 0.1, beep_count, 0.01)
                elif 72 < distance_value <= 144:
                    beep_count = 1
                    play_beep(1000, 0.1, beep_count, 0.5)
                elif 144 < distance_value <= 216:
                    beep_count = 1
                    play_beep(1000, 0.1, beep_count, 1)
        else:
            print("Jarak lebih dari threshold, motor dimatikan")
            motor_pwm.ChangeDutyCycle(0)

        time.sleep(0.1)

# BUTTON ULTRASONIC CALLBACK
def button_ultrasonic_callback(channel):
    """Callback untuk mengganti submode pada mode ultrasonik."""
    global mode, feedback_mode
    print("Tombol Ultrasonik ditekan")
    detection_event.clear()
    instruction_event.clear()
    ultrasonic_event.set()  # Hidupkan event ultrasonik

    #mode = 'ultrasonic'
    stop_audio()
    if mode != 'ultrasonic':
        mode = 'ultrasonic'
        if feedback_mode != 'vibrate':
            feedback_mode = 'vibrate'  # Reset ke submode default
        print("Mode berubah menjadi: Ultrasonik dengan submode vibrate")
    else:
        # Toggle submode
        feedback_mode = 'audio' if feedback_mode == 'vibrate' else 'vibrate'
        if feedback_mode == "audio":
            motor_pwm.ChangeDutyCycle(0)  # Matikan motor saat beralih ke audio
        print(f"Submode Ultrasonik berubah menjadi: {feedback_mode}")

#------------------------------------------- Detection ---------------------------------
class ObjectDetector:
    def __init__(self, model_path, min_conf_threshold=0.4):
        self.interpreter = Interpreter(model_path=model_path)
        self.interpreter.allocate_tensors()
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()
        self.height = self.input_details[0]['shape'][1]
        self.width = self.input_details[0]['shape'][2]
        self.min_conf_threshold = min_conf_threshold

        # Validasi format model
        self.validate_model_format()

    def validate_model_format(self):
        """Cetak informasi tentang format input dan output model."""
        print(f"Input tensor type: {self.input_details[0]['dtype']}")
        print(f"Output tensor type: {self.output_details[0]['dtype']}")
        print(f"Input quantization: {self.input_details[0]['quantization']}")
        print(f"Output quantization: {self.output_details[0]['quantization']}")

    def preprocess_image(self, image):
        """Ubah gambar menjadi format yang sesuai untuk model."""
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image_resized = cv2.resize(image_rgb, (self.width, self.height))

        if self.input_details[0]['dtype'] == np.float32:
            # Normalisasi untuk model FP32
            image_normalized = image_resized / 255.0
            return np.expand_dims(image_normalized, axis=0).astype(np.float32)
        elif self.input_details[0]['dtype'] == np.uint8:
            # Tidak perlu normalisasi untuk model quantized
            return np.expand_dims(image_resized, axis=0).astype(np.uint8)
        else:
            raise ValueError("Tipe data input model tidak didukung.")

    def run_detection(self, image):
        input_data = self.preprocess_image(image)
        self.interpreter.set_tensor(self.input_details[0]['index'], input_data)
        self.interpreter.invoke()

        # Ambil hasil tensor output
        boxes = self.interpreter.get_tensor(self.output_details[1]['index'])[0]
        classes = self.interpreter.get_tensor(self.output_details[3]['index'])[0]
        scores = self.interpreter.get_tensor(self.output_details[0]['index'])[0]

        # Lakukan dequantization jika output bertipe uint8
        if self.output_details[0]['dtype'] == np.uint8:
            scale, zero_point = self.output_details[0]['quantization']
            scores = (scores - zero_point) * scale

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
        if not detection_event.is_set():  # Jika event tidak aktif, thread berhenti
            print("Thread detection_event berhenti.")
            break

        print("Mode Detection sedang aktif")
        motor_pwm.ChangeDutyCycle(0)
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
        while time.time() - start_time < 1:
            if mode != 'detection':
                break
            time.sleep(0.1)
        time.sleep(1)

# BUTTON DETECTION
def button_detection_callback(channel):
    global mode
    print("Tombol Detection ditekan")
    detection_event.set() # Hidupkan event detection
    instruction_event.clear()
    ultrasonic_event.clear()

    mode = 'detection'
    stop_audio()  # Hentikan audio jika sebelumnya sedang diputar
    print("Beralih ke mode deteksi objek")


#------------------------------------------- Self-scan ---------------------------------
def self_scan():
    """Fungsi untuk memeriksa konektivitas komponen sebelum program dijalankan."""
    issues = []

    # Periksa sensor ultrasonik
    try:
        GPIO.output(TRIG, True)
        time.sleep(0.00001)
        GPIO.output(TRIG, False)
        start_time = time.time()
        while GPIO.input(ECHO) == 0:
            start_time = time.time()
        while GPIO.input(ECHO) == 1:
            pass
        duration = time.time() - start_time
        distance = (duration * 34300) / 2
        if distance <= 0 or distance > 400:  # Asumsi jarak maksimal sensor 400 cm
            raise ValueError("Sensor ultrasonik tidak merespons.")
    except Exception as e:
        logging.error(f"Ultrasonik Error: {e}")
        issues.append("takada_ultrasonik.mp3")

    # Periksa modul getar
    try:
        motor_pwm.ChangeDutyCycle(50)  # Tes setengah duty cycle
        time.sleep(0.1)
        motor_pwm.ChangeDutyCycle(0)  # Matikan kembali
    except Exception as e:
        logging.error(f"Modul Getar Error: {e}")
        issues.append("takada_getar.mp3")

    # Periksa mini kamera
    try:
        image = capture_image()
        if image is None:
            raise ValueError("Mini kamera tidak merespons.")
    except Exception as e:
        logging.error(f"Mini Kamera Error: {e}")
        issues.append("takada_kamera.mp3")

    # Jika ada masalah, putar file audio masing-masing
    if issues:
        for issue in issues:
            play_audio(issue)
            while pygame.mixer.music.get_busy():
                time.sleep(0.1)
        return False

    # Semua komponen berfungsi
    play_audio("komponen_aman.mp3")
    while pygame.mixer.music.get_busy():
        time.sleep(0.1)

#------------------------------------------- Main ---------------------------------

# Event flags untuk masing-masing mode
ultrasonic_event = threading.Event()
detection_event = threading.Event()
instruction_event = threading.Event()

def main():
    global mode, adc, detector
    play_audio('mulai_scan.mp3')
    self_scan()
    GPIO.add_event_detect(BUTTON_ULTRASONIC_PIN, GPIO.FALLING, callback=button_ultrasonic_callback, bouncetime=300)
    GPIO.add_event_detect(BUTTON_DETECTION_PIN, GPIO.FALLING, callback=button_detection_callback, bouncetime=300)
    GPIO.add_event_detect(BUTTON_INSTRUCTION_PIN, GPIO.FALLING, callback=button_instruction_callback, bouncetime=300)

    # Inisialisasi class
    adc = MCP3008()
    detector = ObjectDetector(model_path="detect.tflite", min_conf_threshold=0.4)

    play_audio('tongkat_hidup.mp3')
    print("Tekan tombol untuk mengubah mode...")

    # Hidupkan mode awal (ultrasonik sebagai default)
    #time.sleep(0.5)  # Pastikan semua thread sudah berjalan

    detection_event.clear()
    instruction_event.clear()
    ultrasonic_event.set()

    try:
        ultrasonic_thread   = threading.Thread(target=ultrasonic_mode, daemon=True)
        detection_thread    = threading.Thread(target=detection_mode, daemon=True)
        instruction_thread  = threading.Thread(target=instruction_mode, daemon=True)
        active_thread = None  # Variabel untuk melacak thread yang aktif
        while True:
            if ultrasonic_event.is_set():
                if active_thread != ultrasonic_thread:
                    print("Memulai thread ultrasonic_mode.")
                    if active_thread is not None and active_thread.is_alive():
                        print("Menghentikan thread sebelumnya.")
                        active_thread.join(0.1)  # Berikan waktu untuk berhenti
                    ultrasonic_thread = threading.Thread(target=ultrasonic_mode, daemon=True)
                    ultrasonic_thread.start()
                    active_thread = ultrasonic_thread

            elif detection_event.is_set():
                if active_thread != detection_thread:
                    print("Memulai thread detection_mode.")
                    if active_thread is not None and active_thread.is_alive():
                        print("Menghentikan thread sebelumnya.")
                        active_thread.join(0.1)
                    detection_thread = threading.Thread(target=detection_mode, daemon=True)
                    detection_thread.start()
                    active_thread = detection_thread

            elif instruction_event.is_set():
                if active_thread != instruction_thread:
                    print("Memulai thread instruction_mode.")
                    if active_thread is not None and active_thread.is_alive():
                        print("Menghentikan thread sebelumnya.")
                        active_thread.join(0.1)
                    instruction_thread = threading.Thread(target=instruction_mode, daemon=True)
                    instruction_thread.start()
                    active_thread = instruction_thread

            time.sleep(0.1)  # Loop utama tetap berjalan
    except KeyboardInterrupt:
        print("Program dihentikan.")
    finally:
        motor_pwm.stop()
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