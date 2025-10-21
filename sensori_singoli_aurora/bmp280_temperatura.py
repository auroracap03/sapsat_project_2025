from machine import I2C, Pin  # Importa le classi I2C e Pin per la comunicazione con i dispositivi I2C
import bmp280  # Importa la libreria per il sensore BMP280
import time  # Importa la libreria per la gestione del tempo

# Inizializza la comunicazione I2C sulla periferica 0
# SDA collegato al pin 20, SCL al pin 21, con frequenza di 100 kHz
i2c = I2C(0, sda=Pin(20), scl=Pin(21), freq=100000)

# Configura il sensore BMP280 usando l'interfaccia I2C appena creata
bmp = bmp280.BMP280(i2c)

# Imposta il livello di oversampling per ottenere misure più precise
bmp.oversample(bmp280.BMP280_OS_HIGH)

# Ciclo infinito per acquisire e stampare i dati periodicamente
while True:
    # Configura il sensore per la modalità di monitoraggio meteo
    bmp.use_case(bmp280.BMP280_CASE_WEATHER)

    # Legge e stampa la temperatura in gradi Celsius
    print("tempC: {}".format(bmp.temperature))

    # Legge e stampa la pressione atmosferica in Pascal
    print("pressure: {}Pa".format(bmp.pressure))

    # Aspetta 1 secondo prima di effettuare una nuova lettura
    time.sleep_ms(1000)
