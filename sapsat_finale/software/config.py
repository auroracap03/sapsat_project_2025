#In questo file inizializziamo tutti i pin dei sensori

from machine import Pin, I2C, ADC
import time
import sdcard #libreria sd
import os 
from ds3231 import DS3231  #libreria rtc
import bme280
from imu import MPU6050
import machine
from machine import Pin, I2C, UART, ADC
import time
import bme280  # Assicurati che il file bme280.py sia salvato sulla scheda
from ds3231 import *
import math
from imu import MPU6050
import uos
import os

def configure():
    try:
        #Inizializza I2C
        i2c = I2C(0, scl=Pin(13), sda=Pin(12))
        #Inizializza bme
        bme = bme280.BME280(i2c=i2c)
        #Inizializza rtc
        ds = DS3231(i2c)
        #Inizializzazione buzzer
        buzzer = Pin(19, Pin.OUT)
        buzzer.value(0)
        #Inizializzazione potenziometro (ingresso analogico)
        adc = ADC(26)
        #  inizializza mpu6050 (acc,gir)
        mpu = MPU6050(i2c,device_addr=1)
        #  inizializza neo6m (gps)
        gps_serial = machine.UART(1, baudrate=9600, tx=4, rx=5)
        #  inizializza sg80 (servo)
        servo = machine.PWM(machine.Pin(16))
        # UART radio
        uart = UART(0, baudrate=9600, tx=Pin(0), rx=Pin(1))
        line=str(0)
        #header
        header = "data,ora,pressure,relative_altitude,absolute_altitude,temp,humi,line,ax,ay,az,gx,gy,roll_gyro,pitch_gyro,roll_acc,pitch_acc,roll,pitch"
        
        # Creazione e pulizia file Flash
        log_flash = "data_log_flash.csv"
        with open(log_flash, "w") as f2:
            f2.write(header+"\n")
        print("Dati Flash cancellati e file inizializzato")
        
        return i2c, bme, ds, buzzer, adc, mpu, gps_serial, servo, line, uart
    except Exception as e:
        print(f"Errore: {e}")

