import busio
import time
from adafruit_vl53l1x import VL53L1X
import board

# Setup I2C untuk sensor jarak VL53L1X
i2c = busio.I2C(board.SCL, board.SDA)
vl53 = VL53L1X(i2c)

# Set distance mode ke long untuk rentang jarak yang lebih jauh
vl53.distance_mode = 2
vl53.timing_budget = 500  # Set timing budget tetap

# Mulai pengukuran
vl53.start_ranging()

try:
    while True:
        if vl53.data_ready:
            distance = vl53.distance  # Dapatkan jarak dalam cm

            if distance is not None:
                print(f"Distance = {distance} cm")
            else:
                print("Distance: Greater than sensor range")

            vl53.clear_interrupt()  # Bersihkan interrupt sensor

        time.sleep(0.1)

except KeyboardInterrupt:
    print("Program dihentikan oleh pengguna")

finally:
    # Hentikan pengukuran jarak
    vl53.stop_ranging()
