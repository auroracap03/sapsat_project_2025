from config import *
import sdcard  # Libreria sd
import uos
import os
import time
from ds3231 import DS3231  # Libreria rtc
import bme280 
from machine import Pin, I2C, UART, ADC
from imu import MPU6050
import math

# Variabili globali necessarie
SEA_LEVEL_PRESSURE = 1013.25  # hPa
reference_pressure = SEA_LEVEL_PRESSURE  # Inizializzazione temporanea
last_altitude = 0
last_az = 0
vel_mpu = 0
alpha = 0.90
accel_factor = 0.8
last_roll_acc = 0
last_pitch_acc = 0

def init_sensori(i2c, bme, ds, buzzer, adc, mpu, gps_serial, servo, line, uart):
    global reference_pressure, last_altitude, last_az, vel_mpu, alpha, accel_factor, last_roll_acc, last_pitch_acc
    
    # Inizializza RTC
    ds.set_time()  # Imposta l'ora attuale
    print("Inizializzazione componenti")

    # Assign chip select (CS) pin (and start it high)
    cs = machine.Pin(9, machine.Pin.OUT)

    header = "data,ora,pressure,relative_altitude,absolute_altitude,temp,humi,line,ax,ay,az,gx,gy,roll_gyro,pitch_gyro,roll_acc,pitch_acc,roll,pitch,velocita,gps"

    # Creazione e pulizia file Flash
    log_flash = "data_log_flash.csv"
    with open(log_flash, "w") as f2:
        f2.write(header+"\n")
    print("Dati Flash cancellati e file inizializzato")

    # Analogico
    adc = ADC(26)
    print("Analogico inizializzato")

    # Inizializza I2C 
    i2c = I2C(0, scl=Pin(13), sda=Pin(12))
    print("I2C inizializzato")

    # IMU
    mpu = MPU6050(i2c,device_addr=1)
    print("MPU inizializzato")

    # DS3231 RTC
    ds = DS3231(i2c)
    # Set orario
    ds.set_time()
    print("RTC inizializzato")

    # BME
    try:
        bme = bme280.BME280(i2c=i2c)
        print("BME inizializzato")
    except Exception as e:
        print(f"Errore BME: {e}")

    # UART radio
    uart = UART(0, baudrate=9600, tx=Pin(0), rx=Pin(1))
    print("Radio inizializzata")

    # UART GPS
    gps_serial = machine.UART(1, baudrate=9600, tx=4, rx=5)
    line=str(0)
    print("GPS inizializzato")

    #SERVO
    servo = machine.PWM(machine.Pin(18))  
    servo.freq(50)
    servo_write(servo, 0)
    print("SERVO inizializzato")

    # Buzzer
    buzzer = Pin(19, Pin.OUT)
    buzzer.value(0)
    print("Buzz inizializzato")
    
    # Calibrazione IMU
    # Numero di campioni per la calibrazione
    num_samples = 100

    # Variabili per gli offset di calibrazione
    gx_offset, gy_offset, gz_offset = 0, 0, 0
    ax_offset, ay_offset, az_offset = 0, 0, 0 

    # Raccolta dati per la calibrazione (mediante la media su num_samples campioni)
    for _ in range(num_samples):
        gx_offset += mpu.gyro.x
        gy_offset += mpu.gyro.y
        gz_offset += mpu.gyro.z
        ax_offset += mpu.accel.x * 9.8  # Conversione in m/s²
        ay_offset += mpu.accel.y * 9.8
        az_offset += mpu.accel.z * 9.8
        time.sleep(0.01)

    # Calcolo degli offset medi
    gx_offset /= num_samples
    gy_offset /= num_samples
    gz_offset /= num_samples
    ax_offset /= num_samples
    ay_offset /= num_samples
    az_offset = (az_offset / num_samples) - 9.81  # Sottraiamo la gravità terrestre

    # Stampa degli offset calcolati
    print(f"Offset giroscopio -> gx: {gx_offset:.2f}, gy: {gy_offset:.2f}, gz: {gz_offset:.2f}")
    print(f"Offset accelerometro -> ax: {ax_offset:.2f}, ay: {ay_offset:.2f}, az: {az_offset:.2f}")

    # Tempo iniziale per il calcolo dell'integrazione del giroscopio
    last_time = time.ticks_ms()

    # Inizializzazione angoli di Roll e Pitch basati sull'accelerometro
    ax = (mpu.accel.x * 9.8) - ax_offset
    ay = (mpu.accel.y * 9.8) - ay_offset
    az = (mpu.accel.z * 9.8) - az_offset

    # Calcolo degli angoli iniziali usando le formule dell'accelerometro
    roll_acc = math.atan2(ay, az) * (180 / math.pi)
    pitch_acc = math.atan2(-ax, math.sqrt(ay**2 + az**2)) * (180 / math.pi)

    # Inizializzazione degli angoli per il giroscopio
    roll_gyro = roll_acc
    pitch_gyro = pitch_acc

    # Inizializza variabili globali
    last_az = az
    vel_mpu = 0
    last_roll_acc = roll_acc
    last_pitch_acc = pitch_acc
    
    # Inizializza la pressione di riferimento per l'altitudine relativa
    try:
        reference_pressure = bme.get_data()['pressure'] / 100  # Conversione da Pascal a hPa
        last_altitude = get_relative_altitude(bme)
    except Exception as e:
        print(f"Errore inizializzazione pressione: {e}")
        reference_pressure = SEA_LEVEL_PRESSURE
        last_altitude = 0
    
    print("Calibrazione IMU eseguita")

    return gx_offset, gy_offset, gz_offset, ax_offset, ay_offset, az_offset, last_time, roll_gyro, pitch_gyro, ds, bme

    
#FUNZIONI SERVO
def interval_mapping(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def servo_write(pin, angle):
    pulse_width = interval_mapping(angle, 0, 180, 0.5, 2.5)
    duty = int(interval_mapping(pulse_width, 0, 20, 0, 65535))
    pin.duty_u16(duty)
    
# Funzione per approssimare i valori
def approx(value, decimals=2):
    """
    Approssima un valore al numero di cifre decimali specificato
    Args:
        value: valore da approssimare
        decimals: numero di cifre decimali (default: 2)
    Returns:
        valore approssimato
    """
    try:
        if value is None:
            return None
        return round(float(value), decimals)
    except (ValueError, TypeError):
        return value

#Filtro Velocità MPU e BME
def fuse_velocity(vel_bme, vel_mpu, weight_bme=0.8):
    weight_mpu = 1.0 - weight_bme
    return vel_bme * weight_bme + vel_mpu * weight_mpu

def get_absolute_altitude(bme):
    try:
        pressure = bme.get_data()['pressure']
        pressure = pressure/100  #Conversione da Pascal a hPa
        altitude = 44330 * (1 - (pressure / SEA_LEVEL_PRESSURE) ** (1/5.255))
        return altitude
    except Exception as e:
        print(f"Errore calcolo altitudine assoluta: {e}")
        return 0

def get_relative_altitude(bme):
    global reference_pressure
    try:
        pressure = bme.get_data()['pressure']
        pressure = pressure/100  #Conversione da Pascal a hPa
        altitude = 44330 * (1 - (pressure / reference_pressure) ** (1/5.255))
        return altitude
    except Exception as e:
        print(f"Errore calcolo altitudine relativa: {e}")
        return 0

def lettura_sensori(gx_offset, gy_offset, gz_offset, ax_offset, ay_offset, az_offset, last_time, roll_gyro, pitch_gyro, ds, bme, mpu, adc, gps_serial):
    global reference_pressure, last_altitude, last_az, vel_mpu, alpha, accel_factor, last_roll_acc, last_pitch_acc
        
    # Print the current date in the format: month/day/year
    try:
        data = f"{ds.get_time()[2]}/{ds.get_time()[1]}/{ds.get_time()[0]}"
        ora = f"{ds.get_time()[3]}:{ds.get_time()[4]}:{ds.get_time()[5]}"
    except Exception as e:
        print(f"Errore lettura data/ora: {e}")
        data = "00/00/0000"
        ora = "00:00:00"
      
    # Lettura BME280
    try:
        pressure = approx(bme.get_data()['pressure'], 2)
        temp = approx(bme.get_data()['temperature'], 2)
        humi = approx(bme.get_data()['humidity'], 2)
        absolute_altitude = approx(get_absolute_altitude(bme), 2)
        relative_altitude = approx(get_relative_altitude(bme), 2)
    except Exception as e:
        print(f"Errore BME280: {e}")
        pressure = 0
        temp = 0
        humi = 0
        absolute_altitude = 0
        relative_altitude = 0
    
    # Calcolo velocità BME
    vel_bme = approx((relative_altitude - last_altitude), 0)
    last_altitude = relative_altitude
    
    # Lettura potenziometro
    try:
        raw = adc.read_u16()  # 16 bit (0–65535), anche se ADC è a 12 bit
        voltage = (raw / 65535) * 3.3  # tensione al pin
        perc = approx(max(0, min(100, ((voltage - 2.9) / 0.4) * 100)), 1)
    except Exception as e:
        print(f"Errore ADC: {e}")
        perc = 0
    
    # Lettura IMU
    try:
        # Lettura dei valori grezzi dell'accelerometro (corretti con l'offset)
        ax = approx((mpu.accel.x * 9.8) - ax_offset, 1)
        ay = approx((mpu.accel.y * 9.8) - ay_offset, 1)
        az = approx((mpu.accel.z * 9.8) - az_offset, 1)
        
        # Calcolo velocità MPU
        vel_mpu += 0.5*(az+last_az)/9.81 
        vel_mpu = approx(vel_mpu, 3)
        last_az = az

        # Lettura dei valori grezzi del giroscopio (corretti con l'offset)
        gx = approx(mpu.gyro.x - gx_offset, 3)
        gy = approx(mpu.gyro.y - gy_offset, 3)
        gz = approx(mpu.gyro.z - gz_offset, 3)
    except Exception as e:
        print(f"Errore IMU: {e}")
        ax = ay = az = gx = gy = gz = 0
        vel_mpu = 0

    # Calcolo velocità fusa
    vel = approx(fuse_velocity(vel_bme, vel_mpu, weight_bme=1), 0)
        
    # Lettura GPS
    try:
        if gps_serial.any():
            line = gps_serial.readline()  # Legge una riga completa dalla comunicazione seriale
            if line:
                line = line.decode('utf-8')  # Decodifica i dati ricevuti da byte a stringa
                line = line.strip()
            else:
                line = ""
        else:
            line = ""
    except Exception as e:
        print(f"Errore GPS: {e}")
        line = ""

    # Calcoli angolari
    try:
        # Calcolo del tempo trascorso dall'ultima iterazione
        current_time = time.ticks_ms()
        dt = (current_time - last_time) / 1000  # Conversione da millisecondi a secondi
        last_time = current_time

        # Integrazione per ottenere gli angoli del giroscopio
        roll_gyro += gx * dt
        pitch_gyro += gy * dt
        roll_gyro = approx(roll_gyro, 2)
        pitch_gyro = approx(pitch_gyro, 2)

        # Calcolo angoli accelerometro con filtro passa-basso
        if az != 0:  # Evita divisione per zero
            roll_acc = (1 - accel_factor) * last_roll_acc + accel_factor * math.atan2(ay, az) * (180 / math.pi)
            pitch_acc = (1 - accel_factor) * last_pitch_acc + accel_factor * math.atan2(-ax, math.sqrt(ay**2 + az**2)) * (180 / math.pi)
        else:
            roll_acc = last_roll_acc
            pitch_acc = last_pitch_acc
            
        roll_acc = approx(roll_acc, 2)
        pitch_acc = approx(pitch_acc, 2)
        last_roll_acc, last_pitch_acc = roll_acc, pitch_acc

        # Filtro complementare per combinare giroscopio e accelerometro
        roll = approx(alpha * roll_gyro + (1 - alpha) * roll_acc, 2)
        pitch = approx(alpha * pitch_gyro + (1 - alpha) * pitch_acc, 2)
    except Exception as e:
        print(f"Errore calcoli angolari: {e}")
        roll_acc = pitch_acc = roll = pitch = 0
    
    return data, ora, pressure, relative_altitude, absolute_altitude, temp, humi, line, ax, ay, az, gx, gy, roll_gyro, pitch_gyro, roll_acc, pitch_acc, roll, pitch, vel