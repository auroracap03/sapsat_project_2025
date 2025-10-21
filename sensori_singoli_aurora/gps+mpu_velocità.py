from machine import I2C, Pin, UART
from imu import MPU6050
import time
import math

# I2C MPU6050
i2c = I2C(0, sda=Pin(20), scl=Pin(21), freq=400000)
mpu = MPU6050(i2c)

# UART GPS
gps = UART(1, baudrate=9600, tx=Pin(4), rx=Pin(5))

# Offset e variabili
az_offset = 0  # da calcolare con una calibrazione come già fai
vz_mpu = 0
altitude_prev = None
vz_gps = None

alpha = 0.7  # prevalenza alla stima GPS (più stabile a lungo termine)
last_time = time.ticks_ms()

while True:
    # === LETTURA GPS ALTITUDINE ===
    altitude = None
    if gps.any():
        line = gps.readline()
        if line:
            try:
                line = line.decode('utf-8')
                if line.startswith('$GPGGA'):
                    parts = line.strip().split(',')
                    if parts[9]:  # campo altitudine (in metri)
                        altitude = float(parts[9])
                        print(f"Altitudine GPS: {altitude:.2f} m")
            except:
                pass

    # === LETTURA MPU6050 e integrazione ===
    az = mpu.accel.z * 9.8 - az_offset
    az_corr = az - 9.81  # compensazione gravità

    current_time = time.ticks_ms()
    dt = (current_time - last_time) / 1000
    last_time = current_time

    vz_mpu += az_corr * dt
    print(f"Vel. verticale da MPU: {vz_mpu:.2f} m/s")

    # === Calcolo derivata GPS (vz_gps) ===
    if altitude is not None and altitude_prev is not None:
        vz_gps = (altitude - altitude_prev) / dt
        print(f"Vel. verticale da GPS: {vz_gps:.2f} m/s")
    altitude_prev = altitude

    # === Fusione ===
    if vz_gps is not None:
        vz_fused = alpha * vz_gps + (1 - alpha) * vz_mpu
        print(f"Vel. verticale FUSA: {vz_fused:.2f} m/s")
    else:
        print(f"Vel. verticale stimata (solo MPU): {vz_mpu:.2f} m/s")

    print("-" * 40)
    time.sleep(0.5)
