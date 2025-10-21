import machine          
import time             


servo = machine.PWM(machine.Pin(16))  # Imposta il pin GP16 come uscita PWM per controllare il servo motore
servo.freq(50)                        # Imposta la frequenza del segnale PWM a 50 Hz, standard per i servo

def interval_mapping(x, in_min, in_max, out_min, out_max):
    # Mappa un valore x da un intervallo [in_min, in_max] a uno [out_min, out_max]
    # Utile per convertire angoli in millisecondi o valori PWM
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def servo_write(pin, angle):
    # Funzione per scrivere un angolo (in gradi) al servo motore
    
    #serve prima trasformare l’angolo in millisecondi di impulso,
    #poi trasformare i millisecondi nel valore PWM a 16 bit, perchè
    #i servo motori non capiscono “angoli”, ma capiscono duty cycle PWM

    #quindi bisogna tradurre un angolo umano in un segnale elettrico reale
    #comprensibile per il servo, usando proporzioni tra intervalli numerici.
    
    pulse_width = interval_mapping(angle, 0, 180, 0.5, 2.5)
    #traduzione della linea precedente:
    # "Se l'angolo è 0°, allora l’impulso dev'essere 0.5 ms.
    # Se l'angolo è 180°, allora dev'essere 2.5 ms.
    # E per gli angoli intermedi fammi una proporzione lineare."

    #trasformare la durata dell’impulso (in millisecondi) in un valore digitale PWM da 0 a 65535.
    duty = int(interval_mapping(pulse_width, 0, 20, 0, 65535))

    # Imposta il duty cycle PWM sul pin, comandando fisicamente il movimento del servo
    pin.duty_u16(duty)


def read_altitude():
    # inserire la lettura reale del sensore
    return 200  #valore di test per la logica

def read_velocity():
    # calcolare velocità verticale del razzo (in m/s)
    # come? derivare l’altezza nel tempo oppure leggere da un accelerometro filtrato
    return -5.3  # Valore negativo = il razzo sta scendendo (velocità verticale negativa)

parachute_released = False   # Flag che indica se il paracadute è già stato rilasciato (serve per evitarlo due volte)

while True:
    altitude = read_altitude()     # Legge l’altitudine attuale dal sensore
    velocity = read_velocity()     # Legge la velocità verticale attuale dal sensore

    # se: 1) il paracadute non è ancora stato rilasciato
    #     2) siamo a quota <= 200 metri
    #     3) il razzo sta scendendo (velocità negativa)
    if not parachute_released and altitude <= 200 and velocity < 0:
        print("Quota di 200m raggiunta e discesa rilevata. Sgancio paracadute.")
        servo_write(servo, 180)     # Ruota il servo a 180°, che dovrebbe sganciare il paracadute
        parachute_released = True   # Imposta il flag per evitare sganci successivi

    time.sleep(0.1)  # Aspetta 100 millisecondi prima di ricontrollare i sensori (frequenza di aggiornamento = 10 Hz)
