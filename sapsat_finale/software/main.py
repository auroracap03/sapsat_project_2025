from lettura_sensori_fixed4 import *
from config import * 
from state_machine_fixed3 import *
import time
import machine
from machine import Pin, I2C, UART, ADC
import bme280  # Assicurati che il file bme280.py sia salvato sulla scheda
from ds3231 import *
import math
from imu import MPU6050
import sdcard
import uos
import os

def main():
    try:
        config_tuple = configure()
        if config_tuple is None:
            print("Errore nella configurazione.")
            return
        i2c, bme, ds, buzzer, adc, mpu, gps_serial, servo, line, uart = config_tuple

    except Exception as e:
        print(f"Errore configurazione: {e}")
        return  # Esce dalla funzione

        

    stateMachine = StateMachine("data_log_flash.csv", i2c, bme, ds, buzzer, adc, mpu, gps_serial, servo, line, uart)

    while True:
        try:
            stateMachine.transizione()
            stateMachine.mostra_stato()
        except Exception as e:
            print(f"Errore state_machine: {e}")
        time.sleep(1)
        
if __name__ == "__main__":
    main()