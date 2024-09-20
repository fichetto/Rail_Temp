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
