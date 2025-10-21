import machine  # Importa il modulo per l'uso dell'hardware della Raspberry Pi Pico
import time  # Importa il modulo per la gestione dei ritardi temporali

# Inizializza il controllo PWM sul pin 16 per pilotare un servo motore
servo = machine.PWM(machine.Pin(18))  
servo.freq(50)  # Imposta la frequenza PWM a 50Hz, tipica per i servo motori


def interval_mapping(x, in_min, in_max, out_min, out_max):
    
    #Mappa un valore da un intervallo a un altro.
    #Utile per convertire gli angoli del servo (0-180°) in un'adeguata larghezza di impulso (in ms).
    
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def servo_write(pin, angle):
    
    #Muove il servo a un angolo specifico.
    #L'angolo è convertito in un duty cycle PWM appropriato.
    
    # Mappa l'angolo (0-180°) nella corrispondente larghezza di impulso (0.5-2.5 ms)
    pulse_width = interval_mapping(angle, 0, 180, 0.5, 2.5)

    # Converte la larghezza di impulso in un valore di duty cycle per il PWM (0-65535)
    duty = int(interval_mapping(pulse_width, 0, 20, 0, 65535))

    # Imposta il duty cycle PWM per il servo
    pin.duty_u16(duty)


# Loop principale per muovere il servo avanti e indietro
while True:
    # Muove il servo da 0° a 180°
    
    servo_write(servo, 0)
    time.sleep_ms(2000)  
    servo_write(servo, 90)
    time.sleep_ms(2000)  
    servo_write(servo, 180)
    time.sleep_ms(2000)  
    servo_write(servo, 360)
        

