from lettura_sensori_fixed4 import *
import time
from machine import Pin, I2C, UART, ADC

'''
def dict_to_str(data: dict):
    str_data = ""
    for key in data:
        value = data[key]
        if value is None:
            value = ""
        str_data += str(value) + ","
    return str_data.rstrip(",") + "\n"  # Rimuove l'ultima virgola e aggiunge newline
    

# Questa funzione deve ricevere un dizionario 'data'
def write_data(filename: str, data: dict):
    try:
        with open(filename, "a") as f:
            f.write(dict_to_str(data))
    except Exception as e:
        print(f"Errore nel salvataggio dati: {e}")
'''

def write_csv_line(filename: str, data: dict, campi: list):
    try:
        # Crea una stringa CSV rispettando l'ordine dei campi
        riga = ','.join(str(data.get(campo, "")) for campo in campi) + '\n'
        
        # Scrive su file in modalità append
        with open(filename, "a") as f:
            f.write(riga)
    except Exception as e:
        print(f"Errore nel salvataggio dati: {e}")

#FUNZIONI SERVO
def interval_mapping(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
def servo_write(pin, angle):
    pulse_width = interval_mapping(angle, 0, 180, 0.5, 2.5)
    duty = int(interval_mapping(pulse_width, 0, 20, 0, 65535))
    pin.duty_u16(duty)


class StateMachine:
    def __init__(self, filename: str, i2c, bme, ds, buzzer, adc, mpu, gps_serial, servo, line, uart):
        self.filename = filename
        self.stato = "initialize"
        self.i2c = i2c
        self.bme = bme
        self.ds = ds
        self.buzzer = buzzer
        self.adc = adc
        self.mpu = mpu
        self.gps_serial = gps_serial
        self.servo = servo
        self.line = line
        self.uart = uart
        self.counter = 0
        
        # Inizializza i sensori e ottieni tutti i parametri necessari
        try:
            init_result = init_sensori(i2c, bme, ds, buzzer, adc, mpu, gps_serial, servo, line, uart)
            (self.gx_offset, self.gy_offset, self.gz_offset, self.ax_offset, self.ay_offset, 
             self.az_offset, self.last_time, self.roll_gyro, self.pitch_gyro, self.ds, self.bme) = init_result
            
            self.data = {}
            self.update_data()  # Prima lettura sensori
        except Exception as e:
            print(f"Errore inizializzazione state machine: {e}")
            self.data = {}
        
    def clock(self, massimo=2):
        if self.counter == massimo:
            self.counter = 0
            return True
        return False
    
    
        
    def muovi_servo(self, angle=90):
        self.servo_write(angle)
        time.sleep(0.2)
        self.servo_write(angle)
    
    def send_data(self):
        campi = [
            "stato", "data", "ora", "pressure", "relative_altitude", "absolute_altitude",
            "temp", "humi", "line", "ax", "ay", "az", "gx", "gy",
            "roll_gyro", "pitch_gyro", "roll_acc", "pitch_acc",
            "roll", "pitch", "velocita", "gps"
        ]
        # Inserisce lo stato attuale nel dizionario dati
        self.data["stato"] = self.stato

        # Scrive su file
        write_csv_line(self.filename, self.data, campi)

        # Converte in stringa CSV ordinata e la invia via UART
        stringa_dati = ','.join(str(self.data.get(k, '')) for k in campi)
        
        self.uart.write(stringa_dati + '\n')
        
        

        
    def update_data(self):
        try:
            self.counter += 1
            valori = lettura_sensori(
                self.gx_offset, self.gy_offset, self.gz_offset, self.ax_offset, 
                self.ay_offset, self.az_offset, self.last_time, self.roll_gyro, 
                self.pitch_gyro, self.ds, self.bme, self.mpu, self.adc, self.gps_serial
            )
            
            chiavi = [
                "data", "ora", "pressure", "relative_altitude", "absolute_altitude", 
                "temperature", "humidity", "line", "ax", "ay", "az", "gx", "gy", 
                "roll_gyro", "pitch_gyro", "roll_acc", "pitch_acc", "roll", "pitch", "velocity"
            ]
            
            # Assicurati che ci siano abbastanza valori
            if len(valori) >= len(chiavi):
                nuovi_dati = dict(zip(chiavi, valori[:len(chiavi)]))
                
                # Aggiorna solo i valori che non sono None
                for k, v in nuovi_dati.items():
                    if v is not None:
                        self.data[k] = v

                self.send_data()
            else:
                print(f"Errore: ricevuti {len(valori)} valori, attesi {len(chiavi)}")

        except Exception as e:
            print(f"Errore durante lettura sensori: {e}")
    
    
    
    
    def transizione(self):
        stato_prec = self.stato
        

        try:
            if self.stato == "initialize":
                self.update_data()
                if self.clock(10):
                    self.stato = "launch_wait"

            elif self.stato == "launch_wait":
                self.update_data()
                # Calcolo dell'accelerazione totale
                ax = self.data.get("ax", 0)
                ay = self.data.get("ay", 0)
                az = self.data.get("az", 0)
                if ax is not None and ay is not None and az is not None:
                    acceleration = (ax**2 + ay**2 + az**2)**0.5
                    if acceleration > 20 and self.clock():
                        self.stato = "ascent"
                        self.counter = 0

            elif self.stato == "ascent":
                prec_alt = self.data.get("absolute_altitude", 0)
                self.update_data()
                new_alt = self.data.get("absolute_altitude", 0)
                vel = self.data.get("velocity", 0)
                if vel is not None and new_alt is not None and prec_alt is not None:
                    if vel < 0 and new_alt < prec_alt and self.clock():
                        self.stato = "cansat_descent"
                        self.counter = 0

            elif self.stato == "cansat_descent":
                self.update_data()
                rel_alt = self.data.get("relative_altitude", 9999)
                if rel_alt is not None and rel_alt < 200 and self.clock():
                    self.muovi_servo()
                    self.stato = "pc_deploy"
                    self.counter = 0

            elif self.stato == "pc_deploy":
                self.update_data()
                vel = self.data.get("velocity", 0)
                alt = self.data.get("relative_altitude", 9999)
                if vel is not None and alt is not None:
                    if vel < 0 and alt <= 5 and self.clock():
                        self.stato = "landed"
                        self.counter = 0
                        self.buzzer.value(1)

            if stato_prec != self.stato:
                print(f"Transizione: {stato_prec} ➜ {self.stato}")
                
        except Exception as e:
            print(f"Errore nella transizione: {e}")

    def mostra_stato(self):
        try:
            print(f"Stato attuale: {self.stato}")
            # Mostra alcuni dati per debug
            if self.data:
                alt = self.data.get("absolute_altitude", "N/A")
                vel = self.data.get("velocity", "N/A")
                temp = self.data.get("temperature", "N/A")
                ax = self.data.get("ax", "N/A")
                ay = self.data.get("ay", "N/A")
                az = self.data.get("az", "N/A")
                print(f"Alt: {alt}, Vel: {vel}, Temp: {temp}, ax: {ax}, ay: {ay}, az: {az}")
        except Exception as e:
            print(f"Errore mostra stato: {e}")