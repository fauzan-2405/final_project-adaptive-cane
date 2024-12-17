#!/bin/bash

# Loop melalui semua file .m4a di direktori saat ini
for file in *.m4a; do
    # Mengambil nama file tanpa ekstensi
    filename="${file%.m4a}"
    # Mengonversi file ke mp3
    ffmpeg -i "$file" -acodec mp3 "${filename}.mp3"
    echo "Converted $file to ${filename}.mp3"
done
