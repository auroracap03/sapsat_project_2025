import machine  # Importa il modulo per l'uso dell'hardware della Raspberry Pi Pico
from time import sleep  # Importa la funzione sleep per gestire le pause nel loop

# Definisce la comunicazione UART (seriale) per il modulo GPS
# UART1 con baud rate di 9600, pin TX collegato al GPIO 4, pin RX al GPIO 5
gps_serial = machine.UART(1, baudrate=9600, tx=4, rx=5)

while True:
    # Controlla se ci sono dati disponibili nella seriale
    if gps_serial.any():
        line = gps_serial.readline()  # Legge una riga completa dalla comunicazione seriale
        if line:
            line = line.decode('utf-8')  # Decodifica i dati ricevuti da byte a stringa
            print(line.strip())  # Stampa la riga ricevuta, rimuovendo eventuali spazi o caratteri di nuova linea

    sleep(1)  # Aspetta 500 ms prima di controllare di nuovo
