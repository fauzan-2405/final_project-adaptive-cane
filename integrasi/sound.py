import pygame
import RPi.GPIO as GPIO

# Setup GPIO dan Pygame
GPIO.setmode(GPIO.BCM)
BUTTON_INSTRUCTION_PIN = 24
GPIO.setup(BUTTON_INSTRUCTION_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

pygame.mixer.init()

# Variabel untuk menyimpan status audio
current_audio = '0persen.mp3'

# Fungsi untuk memutar audio
def play_audio(file):
    pygame.mixer.music.load(file)
    pygame.mixer.music.play()

# Fungsi untuk menghentikan audio
def stop_audio():
    pygame.mixer.music.stop()

# Callback untuk tombol
def button_instruction_callback(channel):
    global current_audio
    if pygame.mixer.music.get_busy():  # Jika audio sedang diputar
        stop_audio()  # Hentikan audio
        # Ganti audio
        if current_audio == '0persen.mp3':
            current_audio = '90persen.mp3'
        else:
            current_audio = '0persen.mp3'
    play_audio(current_audio)  # Putar audio baru

# Tambahkan deteksi tombol
GPIO.add_event_detect(BUTTON_INSTRUCTION_PIN, GPIO.FALLING, callback=button_instruction_callback, bouncetime=300)

try:
    while True:
        pass  # Loop utama
except KeyboardInterrupt:
    pass
finally:
    GPIO.cleanup()
