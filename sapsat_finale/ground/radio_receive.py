from machine import UART, Pin
import time
import uos
import os
import sdcard


led = Pin(25, Pin.OUT)
led.value(0)
# M0=0, M1=0 per modalità normale
Pin(10, Pin.OUT).value(0)
Pin(11, Pin.OUT).value(0)

# UART1 su GP8 (TX) e GP9 (RX)
uart = UART(0, baudrate=9600, tx=Pin(0), rx=Pin(1))
str=""


# Assign chip select (CS) pin (and start it high)
cs = machine.Pin(9, machine.Pin.OUT)

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

# Initialize SD card
sd = sdcard.SDCard(spi, cs)

# Mount filesystem
vfs = uos.VfsFat(sd)
uos.mount(vfs, "/sd")

# Creazione e pulizia file SD


header = "stato,data,ora, pressure,relative_altitude,absolute_altitude,temp,humi,ax,ay,az,gx,gy,roll_gyro,pitch_gyro,roll_acc,pitch_acc,roll,pitch,velocita,gps"
log_sd = "/sd/data_log.csv"
with open(log_sd, "w") as f:
    f.write(header+"\n")

print("Dati SD cancellati e file inizializzato")

print("INIZIO")
while True:
    if uart.any():
        msg = uart.readline()
        if msg:
            str+=msg.decode().strip()
            led.toggle()
            if msg.decode()=="\n":
                print("INIZIO RICEZIONE")
                print(str)
                print("_______________")
                with open(log_sd,"a")as f:
                    f.write(str+"\n")
                str=""