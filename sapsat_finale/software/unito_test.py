from machine import Pin, I2C, UART, ADC
import time
import bme280  # Assicurati che il file bme280.py sia salvato sulla scheda
from ds3231 import *
import math
from imu import MPU6050
import sdcard
import uos
import os

print("Inizializzazione componenti")

'''
# Assign chip select (CS) pin (and start it high)
cs = machine.Pin(9, machine.Pin.OUT) #9

# Intialize SPI peripheral (start with 1 MHz)
spi = machine.SPI(1, #1
                  baudrate=1000000,
                  polarity=0,
                  phase=0,
                  bits=8,
                  firstbit=machine.SPI.MSB,
                  sck=machine.Pin(10), #verde 10
                  mosi=machine.Pin(11), #11
                  miso=machine.Pin(8)) #8

# Initialize SD card
sd = sdcard.SDCard(spi, cs)

# Mount filesystem
vfs = uos.VfsFat(sd)
uos.mount(vfs, "/sd")
'''
# Creazione e pulizia file SD


header = "data,ora,pressure,relative_altitude,absolute_altitude,temp,humi,line,ax,ay,az,gx,gy,roll_gyro,pitch_gyro,roll_acc,pitch_acc,roll,pitch,velocita,gps"
'''
log_sd = "/sd/data_log.csv"
with open(log_sd, "w") as f:
    f.write(header+"\n")

print("Dati SD cancellati e file inizializzato")
'''

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
velocity_mpu=0

# DS3231 RTC
ds = DS3231(i2c)
# Set orario
ds.set_time()
print("RTC inizializzato")


# BME
bme = bme280.BME280(i2c=i2c)
print("BME inizializzato")



Pin(10, Pin.OUT).value(0)
Pin(10, Pin.OUT).value(0)
# UART radio
uart = UART(0, baudrate=9600, tx=Pin(0), rx=Pin(1))
print("Radio inizializzata")

# UART GPS
gps_serial = machine.UART(1, baudrate=9600, tx=4, rx=5)
line=str(0)
print("GPS inizializzato")

#FUNZIONI SERVO
def interval_mapping(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
def servo_write(pin, angle):
    pulse_width = interval_mapping(angle, 0, 180, 0.5, 2.5)
    duty = int(interval_mapping(pulse_width, 0, 20, 0, 65535))
    pin.duty_u16(duty)

#SERVO
servo = machine.PWM(machine.Pin(18))  
servo.freq(50)
servo_write(servo, 0)
print("SERVO inizializzato")

# Buzzer
buzzer = Pin(19, Pin.OUT)
buzzer.value(1)
print("Buzz inizializzato")

# Pressione standard slm
SEA_LEVEL_PRESSURE = 1013.25  # hPa

# Pressione di riferimento
try:
    pressure = bme.get_data()['pressure']
except Exception as e:
    print(e)
    
reference_pressure = pressure/100  # Converti da Pa a hPa
print("Calibrazione pressione di riferimento")

def get_absolute_altitude():
    try:
        pressure = bme.get_data()['pressure']
    except Exception as e:
        print(e)
    pressure = pressure/100  # Converti da Pascal a hPa
    altitude = 44330 * (1 - (pressure / SEA_LEVEL_PRESSURE) ** (1/5.255))
    return altitude

def get_relative_altitude():
    try:
        pressure = bme.get_data()['pressure']
    except Exception as e:
        print(e)
    pressure = pressure/100  # Converti da Pascal a hPa
    altitude = 44330 * (1 - (pressure / reference_pressure) ** (1/5.255))
    return altitude

last_altitude=get_relative_altitude()

#Altre funzioni
print("Definizione funzioni")

#Filtro Velocità MPU e BME
def fuse_velocity(vel_bme, vel_mpu, weight_bme=0.5):
    weight_mpu = 1.0 - weight_bme
    return vel_bme * weight_bme + vel_mpu * weight_mpu

# Calibrazione magnetometro
# Numero di campioni per la calibrazione
num_samples = 100

# Variabili per gli offset di calibrazione
gx_offset, gy_offset, gz_offset = 0, 0, 0
ax_offset, ay_offset, az_offset = 0, 0, 0 

# Raccolta dati per la calibrazione (mediante la media su num_samples campioni)
for _ in range(num_samples):
    try:
        gx_offset += mpu.gyro.x
        gy_offset += mpu.gyro.y
        gz_offset += mpu.gyro.z
        ax_offset += mpu.accel.x * 9.8  # Conversione in m/s²
        ay_offset += mpu.accel.y * 9.8
        az_offset += mpu.accel.z * 9.8
    except Exception as e:
        print(e)
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
try:
    ax = (mpu.accel.x * 9.8) - ax_offset
    ay = (mpu.accel.y * 9.8) - ay_offset
    az = (mpu.accel.z * 9.8) - az_offset
except Exception as e:
        print(e)

# Calcolo degli angoli iniziali usando le formule dell'accelerometro
roll_acc = math.atan2(ay, az) * (180 / math.pi)
pitch_acc = math.atan2(-ax, math.sqrt(ay**2 + az**2)) * (180 / math.pi)

# Inizializzazione degli angoli per il giroscopio
roll_gyro = roll_acc
pitch_gyro = pitch_acc

# Coefficiente per il filtro complementare
alpha = 0.90  # Più vicino a 1 -> prevalenza giroscopio, più vicino a 0 -> prevalenza accelerometro

# Filtro passa-basso per l'accelerometro
accel_factor = 0.8  # Valori più alti danno maggiore influenza ai dati più recenti
last_roll_acc = roll_acc
last_pitch_acc = pitch_acc

last_az=az
vel_mpu=0
print("Calibrazione magnetometro eseguita")

#Salvataggio su Flash
with open("data_log_flash.csv", "a") as log_flash:
        
    # Loop di lettura ciclica
    while True:
        try:
            print('TRUE')
            
            # Print the current date in the format: month/day/year
            try:
                data=ds.get_time()[2], ds.get_time()[1],ds.get_time()[0]
                print("Date={}/{}/{}" .format(*data))
            except Exception as e:
                print(e)

            # Print the current time in the format: hours:minutes:seconds
            try:
                ora=ds.get_time()[3], ds.get_time()[4],ds.get_time()[5]
                print( "Time={}:{}:{}" .format(*ora) )
            except Exception as e:
                print(e)

            absolute_altitude = get_absolute_altitude()
            relative_altitude = get_relative_altitude()
            
            try:
                temp=bme.get_data()['temperature']
                humi=bme.get_data()['humidity']
            except Exception as e:
                print(e)

            
            print("Pressione Pa:", pressure)
            print(f"Altitudine Assoluta: {absolute_altitude:.2f} m")
            print(f"Altitudine Relativa: {relative_altitude:.2f} m")
            
            vel_bme=(relative_altitude - last_altitude)
            last_altitude = relative_altitude
            print("Velocità BME:", vel_bme)
            
            print("Temperatura °C:",temp)
            print("Umidità:",humi)
            
            try:
                raw = adc.read_u16()  # 16 bit (0–65535), anche se ADC è a 12 bit
                voltage = (raw / 65535) * 3.3  # tensione al pin
                perc = max(0, min(100, ((voltage - 2.9) / 0.4) * 100))
                print("V. analogico:",raw,"Tensione:", voltage,"Percentuale batteria:", int(perc))
            except Exception as e:
                print(e)


            if relative_altitude <= 0:
                buzzer.value(1)
                
            else:
                buzzer.value(0) #Aggiunto per spegnere il buzzer quando non è sotto 0.
            print("Buzzer:", buzzer.value())
            
            vel_mpu+=0.5*(az+last_az)/9.81 #cacca
            last_az=az
            print("Velocità MPU:",vel_mpu)
            
            vel= fuse_velocity(vel_bme, vel_mpu, weight_bme=0.7)
            print("Velocità fusa:", vel)
            
            #MOVIMENTO SERVO
            if relative_altitude <=200 and vel_bme<=0:
                servo_write(servo, 0)
                time.sleep_ms(500)  
                servo_write(servo, 90)
                time.sleep_ms(500)  
                servo_write(servo, 180)
                time.sleep_ms(500)              
                
            if gps_serial.any():
                line = gps_serial.readline()  # Legge una riga completa dalla comunicazione seriale
                if line:
                    line = line.decode('utf-8')  # Decodifica i dati ricevuti da byte a stringa
                    print("Coord. GPS:",line.strip())  # Stampa la riga ricevuta, rimuovendo eventuali spazi o caratteri di nuova linea
                
            # Lettura dei valori grezzi dell'accelerometro (corretti con l'offset)
            try:
                ax = (mpu.accel.x * 9.8) - ax_offset
                ay = (mpu.accel.y * 9.8) - ay_offset
                az = (mpu.accel.z * 9.8) - az_offset
            except Exception as e:
                print(e)


            # Lettura dei valori grezzi del giroscopio (corretti con l'offset)
            try:
                gx = mpu.gyro.x - gx_offset
                gy = mpu.gyro.y - gy_offset
                gz = mpu.gyro.z - gz_offset
            except Exception as e:
                print(e)


            # Calcolo del tempo trascorso dall'ultima iterazione
            try:
                current_time = time.ticks_ms()
                dt = (current_time - last_time) / 1000  # Conversione da millisecondi a secondi
                last_time = current_time
            except Exception as e:
                print(e)


            # Integrazione per ottenere gli angoli del giroscopio
            roll_gyro += gx * dt
            pitch_gyro += gy * dt

            # Calcolo angoli accelerometro con filtro passa-basso
            roll_acc = (1 - accel_factor) * last_roll_acc + accel_factor * math.atan2(ay, az) * (180 / math.pi)
            pitch_acc = (1 - accel_factor) * last_pitch_acc + accel_factor * math.atan2(-ax, math.sqrt(ay**2 + az**2)) * (180 / math.pi)
            last_roll_acc, last_pitch_acc = roll_acc, pitch_acc

            # Filtro complementare per combinare giroscopio e accelerometro
            roll = alpha * roll_gyro + (1 - alpha) * roll_acc
            pitch = alpha * pitch_gyro + (1 - alpha) * pitch_acc
            
            # Stampa dei dati
            print("Accelerometro e Giroscopio:")
            print(f"Dati grezzi ACC -> ax: {ax:.2f}, ay: {ay:.2f}, az: {az:.2f}")
            print(f"Dati grezzi GIR-> gx: {gx:.2f}, gy: {gy:.2f}, gz: {gz:.2f}")
            print(f"Giros -> Roll: {roll_gyro:.2f}°, Pitch: {pitch_gyro:.2f}°")
            print(f"Accel -> Roll: {roll_acc:.2f}°, Pitch: {pitch_acc:.2f}°")
            print(f"Comb -> Roll: {roll:.2f}°, Pitch: {pitch:.2f}°")
            
            #dati_live=f"{data},{ora},{pressure},{relative_altitude},{absolute_altitude},{temp},{humi},{line},{ax},{ay},{az},{gx},{gy},{roll_gyro},{pitch_gyro},{roll_acc},{pitch_acc},{roll},{pitch}\n"
            dati_live=f"{data},{ora},{pressure},{relative_altitude},{absolute_altitude},{temp},{humi},{ax},{ay},{az},{gx},{gy},{roll_gyro},{pitch_gyro},{roll_acc},{pitch_acc},{roll},{pitch},{vel_bme},{line}"
              
            uart.write(dati_live)
            print("Invio radio eseguito")
            '''            
            # Salvataggio su SD
            with open("/sd/data_log.csv", "a") as log_sd:
                log_sd.write(dati_live)
            print("Salvataggio su SD eseguito")
            '''           
            log_flash.write(dati_live)
            print("Salvataggio su Flash eseguito")
            
            print("------------------------")
            time.sleep(1)
        except Exception as e:
            print(e)
    print('FALSE')





