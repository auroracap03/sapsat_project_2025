import math
import time
from imu import MPU6050
from machine import I2C, Pin

# Inizializzazione della comunicazione I2C e del sensore MPU6050
i2c = I2C(0, sda=Pin(20), scl=Pin(21), freq=400000)
mpu = MPU6050(i2c)

def calibra_mpu(mpu, num_samples=200):
    gx_offset, gy_offset, gz_offset = 0, 0, 0
    ax_offset, ay_offset, az_offset = 0, 0, 0

    for _ in range(num_samples):
        gx_offset += mpu.gyro.x
        gy_offset += mpu.gyro.y
        gz_offset += mpu.gyro.z
        ax_offset += mpu.accel.x * 9.8
        ay_offset += mpu.accel.y * 9.8
        az_offset += mpu.accel.z * 9.8
        time.sleep(0.01)

    gx_offset /= num_samples
    gy_offset /= num_samples
    gz_offset /= num_samples
    ax_offset /= num_samples
    ay_offset /= num_samples
    az_offset = (az_offset / num_samples) - 9.81  # Correzione gravitazionale

    return gx_offset, gy_offset, gz_offset, ax_offset, ay_offset, az_offset

def filtro_complementare(roll_gyro, pitch_gyro, roll_acc, pitch_acc, alpha=0.85):
    roll = alpha * roll_gyro + (1 - alpha) * roll_acc
    pitch = alpha * pitch_gyro + (1 - alpha) * pitch_acc
    return roll, pitch

# Calibrazione iniziale
gx_offset, gy_offset, gz_offset, ax_offset, ay_offset, az_offset = calibra_mpu(mpu)

# Tempo iniziale per il calcolo dell'integrazione del giroscopio
last_time = time.ticks_ms()

# Inizializzazione angoli di Roll e Pitch basati sull'accelerometro
ax = (mpu.accel.x * 9.8) - ax_offset
ay = (mpu.accel.y * 9.8) - ay_offset
az = (mpu.accel.z * 9.8) - az_offset
roll_acc = math.degrees(math.atan2(ay, az))
pitch_acc = math.degrees(math.atan2(-ax, math.sqrt(ay**2 + az**2)))

# Inizializzazione degli angoli per il giroscopio
roll_gyro = roll_acc
pitch_gyro = pitch_acc

# Coefficiente per il filtro complementare
alpha = 0.85

while True:
    # Lettura dei valori grezzi
    ax = (mpu.accel.x * 9.8) - ax_offset
    ay = (mpu.accel.y * 9.8) - ay_offset
    az = (mpu.accel.z * 9.8) - az_offset
    gx = mpu.gyro.x - gx_offset
    gy = mpu.gyro.y - gy_offset
    gz = mpu.gyro.z - gz_offset

    # Calcolo del tempo trascorso
    current_time = time.ticks_ms()
    dt = (current_time - last_time) / 1000
    last_time = current_time

    # Integrazione giroscopio
    roll_gyro += gx * dt
    pitch_gyro += gy * dt

    # Calcolo angoli accelerometro
    roll_acc = math.degrees(math.atan2(ay, az))
    pitch_acc = math.degrees(math.atan2(-ax, math.sqrt(ay**2 + az**2)))

    # Filtro complementare
    roll, pitch = filtro_complementare(roll_gyro, pitch_gyro, roll_acc, pitch_acc, alpha)

    # Stampa dei dati
    print("-" * 50)
    print(f"Dati grezzi ACC -> ax: {ax:.2f}, ay: {ay:.2f}, az: {az:.2f}")
    print(f"Dati grezzi GIR-> gx: {gx:.2f}, gy: {gy:.2f}, gz: {gz:.2f}")
    print(f"Giros -> Roll: {roll_gyro:.2f}°, Pitch: {pitch_gyro:.2f}°")
    print(f"Accel -> Roll: {roll_acc:.2f}°, Pitch: {pitch_acc:.2f}°")
    print(f"Comb -> Roll: {roll:.2f}°, Pitch: {pitch:.2f}°")

    # Ritardo per stabilizzare le letture
    time.sleep(1)
