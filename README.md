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


