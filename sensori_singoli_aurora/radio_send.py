from machine import UART, Pin
import time

uart = UART(0, baudrate=9600, tx=Pin(0), rx=Pin(1))
m0 = Pin(10, Pin.OUT)
m1 = Pin(11, Pin.OUT)

# Set to Normal mode
m0.value(0)
m1.value(0)

time.sleep(1)

while True:
    uart.write("Hello from Sender Pico!\n")
    print("Sent message.")
    time.sleep(2)
