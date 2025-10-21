from machine import UART, Pin
import time
import sdcard
import uos
import os

led = Pin(25, Pin.OUT)
# Intialize SPI peripheral (start with 1 MHz)
spi = machine.SPI(1,
                  baudrate=1000000,
                  polarity=0,
                  phase=0,
                  bits=8,
                  firstbit=machine.SPI.MSB,
                  sck=machine.Pin(10),
                  mosi=machine.Pin(11),
                  miso=machine.Pin(8))

# Assign chip select (CS) pin (and start it high)
cs = machine.Pin(9, machine.Pin.OUT)

# M0=0, M1=0 per modalità normale
#Pin(10, Pin.OUT).value(0)
#Pin(11, Pin.OUT).value(0)

# UART1 su GP8 (TX) e GP9 (RX)
uart = UART(0, baudrate=9600, tx=Pin(0), rx=Pin(1))

#Initialize SD card
sd = sdcard.SDCard(spi, cs)

# Mount filesystem
vfs = uos.VfsFat(sd)
uos.mount(vfs, "/sd")

# Creazione e pulizia file SD
header = "data,ora,pressure,relative_altitude,absolute_altitude,temp,humi,ax,ay,az,gx,gy,roll_gyro,pitch_gyro,roll_acc,pitch_acc,roll,pitch,vel,line"
log_sd = "/sd/data_log.csv"

with open(log_sd, "w") as f:
    f.write(header+"\n")
    print("Dati SD cancellati e file inizializzato")

time.sleep(1)
msg=str(0)
while True:
    time.sleep(1)
    if uart.any():
        msg = uart.readline()
        if msg:
            print("Ricevuto:", msg.decode().strip())
            led.toggle()
    
    #sd pezzotto
    #Salvataggio su SD
    with open("/sd/data_log.csv", "a") as log_sd:
        log_sd.write(msg)
        print("Salvataggio su SD eseguito")

     
