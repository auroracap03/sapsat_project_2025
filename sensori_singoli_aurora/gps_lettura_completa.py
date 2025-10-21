import machine  # Importa il modulo per l'uso dell'hardware della Raspberry Pi Pico
from time import sleep  # Importa la funzione sleep per gestire le pause nel loop

# Definisce la comunicazione UART (seriale) per il modulo GPS
# UART1 con baud rate di 9600, pin TX collegato al GPIO 4, pin RX al GPIO 5
gps_serial = machine.UART(1, baudrate=9600, tx=4, rx=5)

altitude_prev = None  # Variabile per l'altitudine precedente

while True:
    # Controlla se ci sono dati disponibili nella seriale
    if gps_serial.any():
        line = gps_serial.readline()  # Legge una riga completa dalla comunicazione seriale
        if line:
            line = line.decode('utf-8')  # Decodifica i dati ricevuti da byte a stringa
            print(line.strip())  # Stampa la riga ricevuta, rimuovendo eventuali spazi o caratteri di nuova linea

            # Estrazione altitudine e velocità verticale
            if line.startswith('$GPGGA'):
                parts = line.strip().split(',')
                if len(parts) > 9 and parts[9]:  # Verifica che l'altitudine sia presente
                    try:
                        # Verifica se l'altitudine è un numero valido
                        altitude = float(parts[9])  # Prova a convertire l'altitudine in float
                        print(f"Altitudine GPS: {altitude:.2f} m")
                        
                        # Calcolo velocità verticale (derivata dell'altitudine)
                        if altitude_prev is not None:
                            vz_gps = (altitude - altitude_prev)  # Velocità verticale in m/s
                            print(f"Velocità verticale GPS: {vz_gps:.2f} m/s")
                        altitude_prev = altitude  # Aggiorna l'altitudine precedente
                    except ValueError:
                        print(f"Errore: dati di altitudine non validi - {parts[9]}")
                else:
                    print("Altitudine non disponibile.")
            elif line.startswith('$GPRMC') or line.startswith('$GPVTG'):
                print("Messaggio GPS di stato o velocità (senza dati di altitudine).")

    sleep(1)  # Aspetta 1 secondo prima di controllare di nuovo
