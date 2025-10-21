# 🚀 SapSat 2025 (Sapienza Space Team) 
### Small-Scale Reentry Payload Mission

## 🌟 Missione e Obiettivi

**SapSat** è un payload di rientro (**Reentry Probe**) progettato per riprodurre, su piccola scala, le fasi principali del ciclo di vita di una missione aerospaziale.  
L’obiettivo è dimostrare la capacità del sistema di eseguire un profilo di volo autonomo comprendente rilascio, frenata aerodinamica e recupero controllato.

### Obiettivi principali
1. **Separazione:** rilascio del payload all’apogeo, in seguito all’eiezione del motore del razzo.  
2. **Dispiegamento dello scudo termico:** apertura immediata dell’**heatshield** dopo la separazione.  
3. **Fase di frenata:** limitazione della velocità di discesa a un massimo di **20 m/s** mediante lo scudo termico utilizzato come **aerobrake**.  
4. **Rilascio del paracadute:** apertura a **200 m di altitudine**, riducendo la velocità a **5 ± 1 m/s**.  
5. **Acquisizione dati:** registrazione continua della telemetria su **SD card** durante ascesa e discesa.

---

## 💻 Software di Bordo e Ground Station

Il contributo software si concentra su due aree principali:
- lo **sviluppo della logica di volo** del payload SapSat;
- la realizzazione della **Ground Station (GS)** per il monitoraggio della missione e la ricezione della telemetria.

### 🧠 Logica di Volo

Il software di bordo, scritto in **MicroPython**, gestisce:
- la lettura dei sensori di bordo,
- l’elaborazione dei dati di altitudine e velocità,
- l’attivazione automatica dei meccanismi (scudo termico e paracadute).

#### 📊 Dati di Telemetria Raccolti

- Temperatura interna (**BME280**)  
- Voltaggio batteria (**Partitore resistivo + ADC**)  
- Altitudine (**BME280**)  
- Accelerazione (**MPU6050**)  
- Velocità (rate) (**Elaborazione software**)  
- Velocità angolare (**MPU6050**)   

Tutti i dati vengono memorizzati localmente su SD card e trasmessi alla Ground Station.

---

### 📡 Ground Station & Operazioni di Missione

La **Ground Station** è il centro operativo per la ricezione e il monitoraggio della telemetria durante il volo.

#### Funzionalità principali
- Ricezione dei pacchetti di dati via radio  
- Decodifica e visualizzazione dei parametri di missione  
- Invio di eventuali comandi correttivi o di stato  

#### Tecnologie utilizzate
- **Linguaggio:** MicroPython
- 
---

## ⚙️ Hardware Setup

Il sistema di bordo è costruito attorno a un **Raspberry Pi Pico**, connesso ai vari sensori e moduli di comunicazione.

### Componenti principali
- **Raspberry Pi Pico** (microcontrollore principale)  
- **BME280** – sensore di temperatura, pressione e altitudine  
- **MPU6050** – accelerometro e giroscopio a 6 DOF  
- **HMC5883L** – magnetometro a 3 assi (alla fine non implementato) 
- **Modulo SD Card** – registrazione dei dati di volo  
- **Modulo Radio (LoRa)** – trasmissione telemetria  
- **Servo motori** – attuazione scudo termico e paracadute  
- **Partitore resistivo** – misura della tensione batteria  
- **Batteria Li-Po 3.7V** – alimentazione del payload  

### Connessioni principali (I²C / SPI / GPIO)
- I²C Bus: **BME280**, **MPU6050**, **HMC5883L**  
- SPI Bus: **Modulo SD Card**  
- UART: **Modulo Radio**  
- GPIO: **Servo motori**, **Trigger separazione**

---

## 👤 Autori
**Aurora Caputo**, **Michele Capponi**, **Federico Capoferri**  
**CAP’S SOFTWARE Team**

---
