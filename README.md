# Questo programma è progettato per un dispositivo LilyGO ESP32 dotato di un modem SIM7000G, con l'obiettivo principale di monitorare e trasmettere dati di temperatura e altri parametri ambientali. Ecco una spiegazione dettagliata delle sue funzionalità:

1. Configurazione hardware:
   - Utilizza un sensore di temperatura DS18B20 collegato al pin 18.
   - Impiega un accelerometro ADXL345 per misurare le vibrazioni.
   - Usa un pin analogico (35) per monitorare la tensione della batteria.
   - Configura i pin per la comunicazione con il modem SIM7000G.

2. Connettività:
   - Il dispositivo si connette a una rete cellulare utilizzando il modem SIM7000G.
   - Usa l'APN "shared.tids.tim.it" per la connessione dati.
   - Si collega a un server MQTT (broker: "telemetry.tecnocons.com") per inviare i dati.
   - Invia anche dati a un server Traccar per il tracciamento della posizione.

3. Funzionalità GPS:
   - Attiva e utilizza il GPS integrato nel modem SIM7000G per ottenere la posizione.
   - Invia le coordinate GPS tramite MQTT e al server Traccar.

4. Monitoraggio della temperatura:
   - Legge la temperatura dal sensore DS18B20 e la invia tramite MQTT.

5. Monitoraggio della batteria:
   - Legge la tensione della batteria e la invia tramite MQTT.
   - Utilizza questa informazione per gestire i cicli di sleep del dispositivo.

6. Gestione del risparmio energetico:
   - Implementa un sistema di deep sleep per risparmiare energia.
   - La durata del sleep varia tra il giorno (10 minuti) e la notte (30 minuti).
   - Se la tensione della batteria scende sotto una certa soglia, entra in un ciclo di sleep più lungo.

7. Accelerometro:
   - Legge i dati dall'accelerometro ADXL345 per monitorare le vibrazioni.
   - Calcola valori medi e estremi (min/max) per ogni asse.

8. Logging locale:
   - Utilizza una connessione seriale (probabilmente collegata a un modulo OpenLog) per registrare localmente i dati.
   - Registra timestamp, velocità calcolata, letture di pressione, e dati dell'accelerometro.

9. MQTT:
   - Pubblica dati su vari topic MQTT, inclusi temperatura, tensione della batteria, posizione GPS, e velocità.
   - Si sottoscrive a topic per ricevere comandi, come l'accensione/spegnimento di un LED di test o il comando di stop.

10. Funzionalità aggiuntive:
    - Calcola una velocità basata su conteggi di impulsi (forse da un sensore esterno non dettagliato nel codice).
    - Legge valori da tre sensori di pressione analogici.

11. Gestione degli errori e riconnessione:
    - Implementa logiche per riavviare il modem e ritentare le connessioni in caso di errori.

In sintesi, questo programma trasforma l'ESP32 in un dispositivo IoT autonomo per il monitoraggio ambientale, con particolare focus sulla temperatura. Utilizza una connessione cellulare per trasmettere dati in tempo reale, gestisce efficacemente l'energia per operazioni a lungo termine, e fornisce funzionalità di logging locale e remoto. È progettato per essere robusto, con capacità di auto-recupero in caso di problemi di connessione, e flessibile nell'adattarsi a diverse condizioni operative e di alimentazione.

Ecco un elenco di istruzioni per il collaboratore che deve personalizzare questo firmware per dispositivi diversi (Railtemp01, 02, 03, ecc.):

1. Identificatore del dispositivo:
   - Modificare la variabile `myid` per ogni dispositivo:
     ```cpp
     String myid = "20240729railtemp01";
     ```
   - Cambiare "railtemp01" in "railtemp02", "railtemp03", ecc. per ogni dispositivo.

2. Topic MQTT:
   - Aggiornare tutti i topic MQTT per riflettere l'ID del dispositivo:
     ```cpp
     const char* topicTestLed = "railtemp01/TestLed";
     const char* topicStop = "railtemp01/Stop";
     const char* topicStatus = "railtemp01/Status";
     const char* topicBatteryStatus = "railtemp01/BatteryVoltage";
     const char* topicGPSlat = "railtemp01/lat";
     const char* topicGPSlon = "railtemp01/lon";
     const char* topicGPSspeed = "railtemp01/speed";
     const char* topicAmpMotore = "railtemp01/AmpMotore";
     const char* topicWatt = "railtemp01/Watt";
     const char* topicTemperature = "railtemp01/Temperature";
     ```
   - Sostituire "railtemp01" con "railtemp02", "railtemp03", ecc. per ogni dispositivo.

3. Connessione MQTT:
   - Modificare l'identificatore client nella funzione `mqttConnect()`:
     ```cpp
     boolean status = mqtt.connect("railtemp01", "tecnocons", "nonserve");
     ```
   - Cambiare "railtemp01" in "railtemp02", "railtemp03", ecc.

4. Configurazione hardware (se necessario):
   - Verificare e aggiornare i pin per i sensori se differiscono tra i dispositivi:
     ```cpp
     #define ONE_WIRE_BUS 18  // Pin per il DS18B20
     #define BATTERY_PIN 35  // Pin per la lettura della batteria
     ```

5. Intervalli di sleep:
   - Se necessario, personalizzare gli intervalli di sleep per dispositivi specifici:
     ```cpp
     uint64_t daySleepInterval = 600;  // 10 minuti in secondi
     uint64_t nightSleepInterval = 1800;  // 30 minuti in secondi
     ```

6. Configurazione APN:
   - Verificare che l'APN sia corretto per tutti i dispositivi:
     ```cpp
     const char apn[] = "shared.tids.tim.it";
     ```
   - Modificare se necessario per dispositivi in aree o con provider diversi.

7. Server e Broker:
   - Confermare che gli indirizzi del server e del broker MQTT siano corretti per tutti i dispositivi:
     ```cpp
     const char server[] = "track.tecnocons.com";
     const char* broker = "telemetry.tecnocons.com";
     ```

8. Calibrazione dei sensori:
   - Se necessario, aggiustare eventuali parametri di calibrazione specifici per ogni dispositivo, come la conversione della velocità:
     ```cpp
     float speed = frequency * 0.1; // Calibrazione con il GPS
     ```

9. Soglie della batteria:
   - Adattare le soglie della batteria se i dispositivi hanno batterie diverse:
     ```cpp
     if (batteryVoltage > 3.5) { // Condizione di risveglio se sopra 35%
     ```

10. Commenti e documentazione:
    - Aggiungere commenti nel codice per indicare la specifica versione del dispositivo:
      ```cpp
      // Configurazione per Railtemp02
      ```

11. Verifica finale:
    - Controllare tutto il codice per riferimenti specifici al dispositivo e aggiornare di conseguenza.
    - Assicurarsi che tutte le funzionalità specifiche del dispositivo siano correttamente configurate.

12. Compilazione e test:
    - Compilare il firmware personalizzato per ogni dispositivo.
    - Testare ogni versione del firmware per assicurarsi che funzioni correttamente con l'hardware specifico.

13. Versionamento:
    - Mantenere un registro delle versioni del firmware per ogni dispositivo, annotando le modifiche specifiche.

14. Backup:
    - Creare un backup del firmware originale e di ogni versione personalizzata.

Seguendo queste istruzioni, il collaboratore dovrebbe essere in grado di personalizzare efficacemente il firmware per ciascun dispositivo Railtemp, garantendo che ogni unità funzioni correttamente con il proprio identificatore unico e le eventuali configurazioni hardware specifiche.

---

## Funzionalità OTA (Over-The-Air Update) - v1.1.6

A partire dalla versione 1.1.6, il firmware supporta l'aggiornamento remoto via HTTPS attraverso la rete cellulare.

### Caratteristiche principali

- **Download HTTPS sicuro**: Utilizza la libreria SSLClient (mbedtls) per connessioni SSL/TLS
- **Supporto GitHub Releases**: Scarica firmware direttamente dalle release GitHub
- **Gestione redirect**: Segue automaticamente i redirect HTTP (301, 302, 307, 308)
- **Protezione batteria**: Rifiuta l'aggiornamento se la batteria è sotto 3.8V
- **Feedback MQTT**: Pubblica lo stato dell'aggiornamento in tempo reale
- **Checksum SHA256**: Verifica l'integrità del firmware (opzionale)

### Topic MQTT per OTA

| Topic | Direzione | Descrizione |
|-------|-----------|-------------|
| `{DEVICE_ID}/cmd/ota` | IN | Comando per avviare l'aggiornamento |
| `{DEVICE_ID}/cmd/reboot` | IN | Comando per riavviare il dispositivo |
| `{DEVICE_ID}/cmd/diagnostics` | IN | Richiesta diagnostica completa |
| `{DEVICE_ID}/ota/status` | OUT | Stato dell'aggiornamento OTA |
| `{DEVICE_ID}/diagnostics` | OUT | Dati diagnostici in formato JSON |

### Formato comando OTA

Pubblica sul topic `{DEVICE_ID}/cmd/ota` un JSON con questa struttura:

```json
{
  "url": "https://github.com/user/repo/releases/download/v1.2.0/firmware.bin",
  "checksum": "sha256:abc123..."
}
```

### Utilizzo con GitHub Actions

Il repository include una GitHub Action che automaticamente:
1. Compila il firmware quando viene creato un tag `v*`
2. Calcola il checksum SHA256
3. Crea una release con `firmware.bin` e `version.json`

**Per creare una nuova release:**

```bash
git tag v1.2.0
git push origin v1.2.0
```

**Per inviare il comando OTA:**

```bash
# Usando il CLI tool incluso
python tools/railtemp_cli.py ota v1.2.0

# Oppure manualmente via MQTT
mosquitto_pub -h telemetry.tecnocons.com -u tecnocons -P nonserve \
  -t "Railtemp03/cmd/ota" \
  -m '{"url":"https://github.com/fichetto/Rail_Temp/releases/download/v1.2.0/firmware.bin","checksum":"sha256:..."}'
```

### Stati OTA

Durante l'aggiornamento, il dispositivo pubblica su `{DEVICE_ID}/ota/status`:

| Stato | Descrizione |
|-------|-------------|
| `starting` | Aggiornamento avviato |
| `downloading` | Download in corso (con progress %) |
| `verifying` | Verifica integrità |
| `success` | Completato con successo |
| `failed` | Fallito (con messaggio errore) |
| `rejected` | Rifiutato (batteria bassa, già in corso) |

### Esempio risposta diagnostica

Pubblica qualsiasi messaggio su `{DEVICE_ID}/cmd/diagnostics` per ricevere:

```json
{
  "firmware_version": "1.1.6",
  "device_id": "Railtemp03",
  "uptime_ms": 123456,
  "free_heap": 180000,
  "battery_voltage": 4.12,
  "signal_quality": 25,
  "gps_fix": true,
  "gps_lat": 45.123456,
  "gps_lon": 9.123456,
  "temperature_c": 22.5,
  "mqtt_connected": true,
  "gprs_connected": true,
  "last_error": "none",
  "ota_in_progress": false
}
```

### Note tecniche

- **Libreria SSL**: govorox/SSLClient (wrapper mbedtls per ESP32)
- **Modem**: TinyGSM con TINY_GSM_MODEM_SIM7000 (connessione TCP standard)
- **Timeout download**: 60 secondi di inattività
- **Max redirect**: 5 redirect consecutivi
- **Chunk size**: 1024 bytes per lettura

### Limitazioni note

- Il firmware deve essere ospitato su server HTTPS accessibile
- Il download richiede una connessione GPRS stabile
- Su rete LTE-M/NB-IoT la velocità può essere limitata (~10-50 KB/s)
- Il dispositivo non è raggiungibile durante l'aggiornamento (~5-15 minuti per ~900KB)

