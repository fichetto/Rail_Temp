#include <Arduino.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// ===== CONFIGURAZIONE DISPOSITIVO - MODIFICARE SOLO QUESTE DUE RIGHE =====
const char* DEVICE_ID = "Railtemp03";              // ID del dispositivo
const char* APN = "shared.tids.tim.it";            // APN dell'operatore TIM
// ========================================================================

// Configurazione pin sensore temperatura
#define ONE_WIRE_BUS 18
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// Configurazione seriali
#define SerialMon Serial
#define SerialAT Serial1

// Configurazione modem GSM
#define TINY_GSM_MODEM_SIM7000
#define TINY_GSM_RX_BUFFER 1024
#define GSM_PIN ""

// Configurazione Deep Sleep
#define uS_TO_S_FACTOR      1000000ULL
#define TIME_TO_SLEEP_DAY   600         // 10 minuti di giorno
#define TIME_TO_SLEEP_NIGHT 1800        // 30 minuti di notte
#define TIME_TO_SLEEP_LOW_BATTERY 3600  // 1 ora con batteria scarica

// Pin definitions
#define UART_BAUD   115200
#define PIN_DTR     25
#define PIN_TX      27
#define PIN_RX      26
#define PWR_PIN     4
#define LED_PIN     12
#define BATTERY_PIN 35

// Intervalli di trasmissione
#define SEND_INTERVAL_MS    10000
#define MAX_SAMPLES_BEFORE_SLEEP 3

// Configurazione attesa GPS
#define GPS_FIX_TIMEOUT 300000          // 5 minuti massimo per GPS fix (era 90s)
#define GPS_CHECK_INTERVAL 5000         // Controlla GPS ogni 5 secondi

#include <TinyGsmClient.h>
#include <PubSubClient.h>

// Credenziali GPRS (solitamente vuote)
const char gprsUser[] = "";
const char gprsPass[] = "";

// MQTT Configuration
const char* broker = "telemetry.tecnocons.com";
char topicTemperature[50];
char topicBatteryStatus[50];
char topicGPSlat[50];
char topicGPSlon[50];
char topicSignalLevel[50];
char topicStatus[50];
char topicTestLed[50];
char topicInit[50];
char topicDateTime[50];

// Connection Manager per gestione tentativi con backoff
struct ConnectionManager {
    int attemptCount = 0;
    unsigned long lastAttemptTime = 0;
    unsigned long totalConnectionTime = 0;
    const unsigned long MAX_TOTAL_TIME = 600000;  // 10 minuti max
    const int MAX_ATTEMPTS = 8;
    const int delayTable[8] = {5, 10, 20, 30, 60, 120, 180, 300};
    
    unsigned long getNextDelay() {
        if (attemptCount >= MAX_ATTEMPTS) return 0;
        return delayTable[min(attemptCount, 7)] * 1000UL;
    }
    
    void reset() {
        attemptCount = 0;
        lastAttemptTime = 0;
        totalConnectionTime = 0;
    }
} connManager;

// Oggetti globali
TinyGsm modem(SerialAT);
TinyGsmClient client(modem);
PubSubClient mqtt(client);

// Variabili di stato
bool connectionOK = false;
int ledStatus = LOW;
uint32_t lastReconnectAttempt = 0;
int sentSampleCount = 0;
float batteryVoltage = 0;

// Variabili per gestione GPS fix
bool gpsFixObtained = false;
bool isDayTime = false;
bool timeKnown = false;
bool gpsCurrentlyEnabled = false;

// Cache GPS per riutilizzo ultimo fix valido
struct GPSCache {
    bool hasValidFix = false;
    float latitude = 0.0f;
    float longitude = 0.0f;
    String utcTime = "";
    unsigned long lastFixTime = 0;
    const unsigned long MAX_CACHE_AGE = 3600000;  // 1 ora validità cache

    bool isValid() {
        return hasValidFix && (millis() - lastFixTime < MAX_CACHE_AGE);
    }

    void update(float lat, float lon, String utc) {
        latitude = lat;
        longitude = lon;
        utcTime = utc;
        lastFixTime = millis();
        hasValidFix = true;
    }

    void clear() {
        hasValidFix = false;
        latitude = 0.0f;
        longitude = 0.0f;
        utcTime = "";
        lastFixTime = 0;
    }
} gpsCache;

// Dichiarazioni funzioni
void modemPowerOn();
void modemPowerOff();
void modemRestart();
void enableGPS();
void disableGPS();
bool waitForGPSFix();
float readBatteryVoltage();
void goToDeepSleep(uint64_t time_in_seconds);
void updateMqttTopics();
bool initializeModem();
bool connectToNetwork();
bool connectToGPRS();
void ConnectTask();
void SendMqttDataTask();
boolean mqttConnect();
void mqttCallback(char* topic, byte* payload, unsigned int len);
void logConnectionStats();
bool testGPSStandard();
void sendATCommand(String cmd, int timeout = 1000);

// Aggiorna i topic MQTT con l'ID del dispositivo
void updateMqttTopics() {
    snprintf(topicTemperature, sizeof(topicTemperature), "%s/Temperature", DEVICE_ID);
    snprintf(topicBatteryStatus, sizeof(topicBatteryStatus), "%s/BatteryVoltage", DEVICE_ID);
    snprintf(topicGPSlat, sizeof(topicGPSlat), "%s/lat", DEVICE_ID);
    snprintf(topicGPSlon, sizeof(topicGPSlon), "%s/lon", DEVICE_ID);
    snprintf(topicSignalLevel, sizeof(topicSignalLevel), "%s/SignalLevel", DEVICE_ID);
    snprintf(topicStatus, sizeof(topicStatus), "%s/Status", DEVICE_ID);
    snprintf(topicTestLed, sizeof(topicTestLed), "%s/TestLed", DEVICE_ID);
    snprintf(topicInit, sizeof(topicInit), "%s/init", DEVICE_ID);
    snprintf(topicDateTime, sizeof(topicDateTime), "%s/DateTime", DEVICE_ID);
}

// Legge tensione batteria
float readBatteryVoltage() {
    int analogValue = analogRead(BATTERY_PIN);
    float voltage = analogValue * (3.3 / 4095.0) * 2.16; // Partitore 1:2
    return voltage;
}

// Controllo alimentazione modem
void modemPowerOn() {
    pinMode(PWR_PIN, OUTPUT);
    digitalWrite(PWR_PIN, HIGH);
    delay(300);
    digitalWrite(PWR_PIN, LOW);
    delay(5000);
}

void modemPowerOff() {
    pinMode(PWR_PIN, OUTPUT);
    digitalWrite(PWR_PIN, LOW);
    delay(1200);
    digitalWrite(PWR_PIN, HIGH);
}

void modemRestart() {
    modemPowerOff();
    delay(1000);
    modemPowerOn();
}

// Utility per inviare comandi AT con log
void sendATCommand(String cmd, int timeout) {
    SerialMon.print(">>> AT");
    SerialMon.println(cmd);
    modem.sendAT(cmd);
    
    String response = "";
    int result = modem.waitResponse(timeout, response);
    
    if (response.length() > 0) {
        SerialMon.print("<<< ");
        SerialMon.println(response);
    }
    
    if (result == 1) {
        SerialMon.println("[OK]");
    } else if (result == 2) {
        SerialMon.println("[ERROR]");
    } else {
        SerialMon.println("[TIMEOUT]");
    }
}

// Gestione GPS con comandi CGNS corretti per SIM7000G
// NOTA: SIM7000G usa AT+CGNS*, NON AT+CGPS*
void enableGPS() {
    if (gpsCurrentlyEnabled) {
        SerialMon.println("GPS già abilitato, skip");
        return;
    }

    SerialMon.println("\n=== ENABLING GNSS ===");

    // 1. Spegni GNSS se attivo
    sendATCommand("+CGNSPWR=0", 2000);
    delay(1000);

    // 2. Accendi antenna GPS via GPIO4 (specifico LilyGO T-SIM7000G)
    sendATCommand("+SGPIO=0,4,1,1", 1000);
    delay(1000);

    // 3. Accendi GNSS
    sendATCommand("+CGNSPWR=1", 2000);
    delay(2000);

    // 4. Verifica stato GNSS
    modem.sendAT("+CGNSPWR?");
    String response = "";
    if (modem.waitResponse(2000L, response) == 1) {
        SerialMon.println("GNSS Power status: " + response);
    }

    gpsCurrentlyEnabled = true;
    SerialMon.println("=== GNSS ENABLED ===\n");
}

void disableGPS() {
    if (!gpsCurrentlyEnabled) {
        SerialMon.println("GPS già disabilitato, skip");
        return;
    }

    SerialMon.println("Disabilitazione GPS per risparmio energetico...");
    // Usa CGNS per SIM7000G
    modem.sendAT("+CGNSPWR=0");
    modem.waitResponse(2000L);
    modem.sendAT("+SGPIO=0,4,1,0");
    modem.waitResponse(2000L);
    gpsCurrentlyEnabled = false;
    SerialMon.println("GPS disabilitato");
}

// Test GPS con metodo standard migliorato
bool testGPSStandard() {
    SerialMon.println("\n--- Tentativo acquisizione GPS ---");

    // Verifica stato GNSS (usa CGNS per SIM7000G)
    modem.sendAT("+CGNSPWR?");
    String pwrStatus = "";
    if (modem.waitResponse(2000L, pwrStatus) == 1) {
        SerialMon.println("GNSS Power: " + pwrStatus);
    }
    
    // Loop di acquisizione
    unsigned long startTime = millis();
    int checkCount = 0;
    
    while (millis() - startTime < GPS_FIX_TIMEOUT) {
        if ((millis() - startTime) > checkCount * GPS_CHECK_INTERVAL) {
            checkCount++;

            // Controllo batteria ogni 30 secondi (ogni 6 check)
            if (checkCount % 6 == 0) {
                float currentBatt = readBatteryVoltage();
                if (currentBatt < 3.4 && currentBatt > 0.5) {
                    SerialMon.println("\n!!! BATTERIA CRITICA durante acquisizione GPS !!!");
                    SerialMon.printf("Tensione: %.2fV - Interrompo ricerca GPS\n", currentBatt);
                    return false;
                }
                SerialMon.printf("Batteria: %.2fV - OK\n", currentBatt);
            }

            // Usa CGNSINF che funziona meglio
            modem.sendAT("+CGNSINF");
            String cgnsinf = "";
            if (modem.waitResponse(3000L, cgnsinf) == 1) {
                cgnsinf.trim();
                SerialMon.println("CGNSINF: " + cgnsinf);
                
                // Se GNSS è OFF, riaccendilo
                if (cgnsinf.startsWith("+CGNSINF: 0")) {
                    SerialMon.println("GNSS OFF, riaccendo...");
                    enableGPS();
                    continue;
                }
                
                // Controlla se c'è un fix valido
                int pos = cgnsinf.indexOf("+CGNSINF: 1,");
                bool fixOk = (pos >= 0 && cgnsinf.charAt(pos + 12) != '0');
                
                if (fixOk) {
                    // Estrai coordinate
                    int start = pos, field = 0;
                    String utcStr, latStr, lonStr;
                    
                    while (field < 6 && start > -1) {
                        int next = cgnsinf.indexOf(',', start);
                        if (next == -1) break;
                        switch (field) {
                            case 2: utcStr = cgnsinf.substring(start, next); break;
                            case 3: latStr = cgnsinf.substring(start, next); break;
                            case 4: lonStr = cgnsinf.substring(start, next); break;
                        }
                        start = next + 1;
                        field++;
                    }
                    
                    float lat = latStr.toFloat();
                    float lon = lonStr.toFloat();

                    if (lat != 0.0f || lon != 0.0f) {
                        SerialMon.printf("GPS FIX: %s, %s @ %s UTC\n",
                                        latStr.c_str(), lonStr.c_str(), utcStr.c_str());

                        // Salva in cache
                        gpsCache.update(lat, lon, utcStr);
                        SerialMon.println("GPS fix salvato in cache");

                        // Determina se è giorno o notte dall'ora UTC
                        if (utcStr.length() >= 6) {
                            int hour = utcStr.substring(8, 10).toInt();
                            // Considera giorno dalle 6 alle 18 UTC
                            isDayTime = (hour >= 6 && hour < 18);
                            timeKnown = true;
                            SerialMon.printf("Ora UTC: %02d, Periodo: %s\n",
                                           hour, isDayTime ? "GIORNO" : "NOTTE");
                        }

                        // Mantieni GPS acceso per fix futuri (non spegnere)
                        SerialMon.println("GPS rimane acceso per fix futuri");

                        return true;
                    }
                }
            }
            
            // LED lampeggia durante attesa GPS
            digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        }
        
        delay(100);
    }
    
    SerialMon.println("GPS timeout - nessun fix");
    return false;
}

// Attesa GPS fix dopo connessione
bool waitForGPSFix() {
    SerialMon.println("\n=== WAITING FOR GPS FIX ===");

    // Controlla se c'è un fix valido in cache
    if (gpsCache.isValid()) {
        unsigned long cacheAge = (millis() - gpsCache.lastFixTime) / 1000;
        SerialMon.printf("Cache GPS valida! Età: %lu secondi\n", cacheAge);
        SerialMon.printf("Usando coordinate cached: %.6f, %.6f\n",
                        gpsCache.latitude, gpsCache.longitude);
        gpsFixObtained = true;

        // Non serve accendere GPS, usiamo cache
        return true;
    }

    SerialMon.println("Nessuna cache GPS valida, acquisizione nuovo fix...");

    // Abilita GPS se non già attivo
    enableGPS();

    // Prova ad ottenere fix
    bool fixObtained = testGPSStandard();

    if (fixObtained) {
        gpsFixObtained = true;
        SerialMon.println("GPS FIX ottenuto! GPS rimane acceso.");
    } else {
        SerialMon.println("GPS FIX non ottenuto dopo timeout - GPS rimane acceso");
        SerialMon.println("Continuo comunque, riprover\u00f2 durante trasmissione dati");
    }

    return fixObtained;
}

// Inizializzazione modem con gestione errori
bool initializeModem() {
    SerialMon.println("Initializing modem...");
    
    for (int i = 0; i < 3; i++) {
        if (modem.testAT()) {
            SerialMon.println("Modem ready");
            break;
        }
        if (i == 2) {
            SerialMon.println("No response, restarting");
            modemRestart();
            return false;
        }
        delay(1000);
    }
    
    SerialMon.printf("Setting APN: %s\n", APN);
    modem.sendAT("+CGDCONT=1,\"IP\",\"" + String(APN) + "\"");
    if (modem.waitResponse(10000L) != 1) {
        SerialMon.println("APN set failed");
        return false;
    }
    
    // Rete automatica 2G/LTE/CAT-M/NB
    modem.sendAT("+CNMP=2");
    modem.waitResponse(5000L);
    modem.sendAT("+CMNB=3");
    modem.waitResponse(5000L);
    
    return true;
}

// Connessione alla rete con timeout progressivi
bool connectToNetwork() {
    SerialMon.println("Connecting to network...");
    
    int baseTimeout = 30;
    int additionalTimeout = connManager.attemptCount * 10;
    int maxWaitTime = baseTimeout + additionalTimeout;
    
    SerialMon.printf("Network timeout: %d seconds\n", maxWaitTime);
    
    if (connManager.attemptCount > 1) {
        modem.sendAT("+COPS=0");
        modem.waitResponse(60000L);
        delay(5000);
    }
    
    unsigned long startTime = millis();
    while (millis() - startTime < (maxWaitTime * 1000UL)) {
        int signal = modem.getSignalQuality();
        
        if (signal != 99 && signal > 0) {
            SerialMon.printf("Network connected! Signal: %d\n", signal);
            SerialMon.println("Operator: " + modem.getOperator());
            return true;
        }
        
        if ((millis() - startTime) % 5000 < 100) {
            SerialMon.printf("Waiting... Signal: %d, Time: %lds/%ds\n", 
                           signal, (millis() - startTime) / 1000, maxWaitTime);
        }
        
        delay(1000);
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    }
    
    return false;
}

// Connessione GPRS con gestione errori
bool connectToGPRS() {
    SerialMon.println("Connecting to GPRS...");
    SerialMon.printf("Attempting GPRS connection with APN: %s\n", APN);
    
    if (modem.gprsConnect(APN, gprsUser, gprsPass)) {
        SerialMon.println("GPRS connected!");
        SerialMon.println("IP address: " + modem.getLocalIP());
        return true;
    }
    
    SerialMon.println("GPRS connection failed");
    return false;
}

// Gestione connessione con backoff progressivo
void ConnectTask() {
    if (connectionOK && modem.isGprsConnected()) {
        if (!modem.isNetworkConnected() || !modem.isGprsConnected()) {
            SerialMon.println("Connection lost");
            connectionOK = false;
            connManager.reset();
        }
        return;
    }
    
    if (connManager.totalConnectionTime == 0) {
        connManager.totalConnectionTime = millis();
    }
    
    if (millis() - connManager.totalConnectionTime > connManager.MAX_TOTAL_TIME) {
        SerialMon.println("Total connection time exceeded - deep sleep");
        goToDeepSleep(TIME_TO_SLEEP_LOW_BATTERY);
        return;
    }
    
    unsigned long currentTime = millis();
    unsigned long nextDelay = connManager.getNextDelay();
    
    if (connManager.attemptCount > 0 && 
        (currentTime - connManager.lastAttemptTime) < nextDelay) {
        return;
    }
    
    if (connManager.attemptCount >= connManager.MAX_ATTEMPTS) {
        SerialMon.println("Max attempts reached - deep sleep");
        goToDeepSleep(TIME_TO_SLEEP_LOW_BATTERY);
        return;
    }
    
    SerialMon.printf("\n=== Connection Attempt %d/%d ===\n", 
                     connManager.attemptCount + 1, connManager.MAX_ATTEMPTS);
    SerialMon.printf("Next delay: %lu seconds\n", nextDelay / 1000);
    
    connManager.lastAttemptTime = currentTime;
    connManager.attemptCount++;
    
    float battVoltage = readBatteryVoltage();
    if (battVoltage < 3.6 && battVoltage > 0.5) {
        SerialMon.println("Battery too low");
        goToDeepSleep(TIME_TO_SLEEP_LOW_BATTERY);
        return;
    }
    
    if (!initializeModem()) {
        return;
    }
    
    if (!connectToNetwork()) {
        return;
    }
    
    if (!connectToGPRS()) {
        return;
    }
    
    SerialMon.println("*** CONNECTION SUCCESSFUL! ***");
    connectionOK = true;
    connManager.reset();
    
    // Attendi GPS fix dopo connessione
    waitForGPSFix();
    
    mqttConnect();
}

// Connessione MQTT
boolean mqttConnect() {
    SerialMon.print("Connecting to MQTT broker...");
    
    mqtt.setKeepAlive(120);
    mqtt.setSocketTimeout(30);
    
    for (int i = 0; i < 3; i++) {
        if (mqtt.connect(DEVICE_ID, "tecnocons", "nonserve")) {
            SerialMon.println(" success");
            
            mqtt.subscribe(topicTestLed);
            mqtt.publish(topicInit, "Device started");
            
            return true;
        }
        
        SerialMon.printf(" fail (attempt %d/3)\n", i + 1);
        delay(5000 * (i + 1));
    }
    
    return false;
}

// Callback MQTT
void mqttCallback(char* topic, byte* payload, unsigned int len) {
    SerialMon.print("Message arrived [");
    SerialMon.print(topic);
    SerialMon.print("]: ");
    SerialMon.write(payload, len);
    SerialMon.println();

    if (String(topic) == String(topicTestLed)) {
        ledStatus = !ledStatus;
        digitalWrite(LED_PIN, ledStatus);
        mqtt.publish(topicStatus, ledStatus ? "LED ON" : "LED OFF");
    }
}

// Invio dati MQTT
void SendMqttDataTask() {
    batteryVoltage = readBatteryVoltage();
    
    if (!connectionOK) {
        return;
    }
    
    // Controllo per deep sleep dopo 3 invii
    if (sentSampleCount >= MAX_SAMPLES_BEFORE_SLEEP) {
        // Verifica se abbiamo mai ottenuto un fix GPS
        if (!gpsCache.hasValidFix) {
            // PROTEZIONE BATTERIA: se batteria troppo bassa, vai in sleep anche senza GPS
            if (batteryVoltage < 3.4 && batteryVoltage > 0.5) {
                SerialMon.println("ATTENZIONE: Batteria critica!");
                SerialMon.printf("Tensione: %.2fV - Vado in deep sleep per preservare batteria\n", batteryVoltage);

                // Lampeggio rosso (veloce) 10 volte per segnalare batteria critica
                for(int i = 0; i < 10; i++) {
                    digitalWrite(LED_PIN, HIGH);
                    delay(50);
                    digitalWrite(LED_PIN, LOW);
                    delay(50);
                }

                // Vai in deep sleep lungo per preservare batteria
                modemPowerOff();
                delay(5000);
                goToDeepSleep(TIME_TO_SLEEP_LOW_BATTERY);
                return;
            }

            SerialMon.println("ATTENZIONE: Nessun fix GPS mai ottenuto!");
            SerialMon.printf("Batteria: %.2fV - Continuo a cercare GPS...\n", batteryVoltage);
            sentSampleCount = 0;  // Reset counter per evitare loop

            // Lampeggio lento per indicare ricerca GPS
            for(int i = 0; i < 3; i++) {
                digitalWrite(LED_PIN, HIGH);
                delay(500);
                digitalWrite(LED_PIN, LOW);
                delay(500);
            }
            return;  // Non andare in deep sleep
        }

        // Lampeggio rapido 5 volte per indicare tentativo deep sleep
        for(int i = 0; i < 5; i++) {
            digitalWrite(LED_PIN, HIGH);
            delay(100);
            digitalWrite(LED_PIN, LOW);
            delay(100);
        }

        if (batteryVoltage > 0.5) {  // Soglia per distinguere USB da batteria
            SerialMon.println("Max samples sent - going to deep sleep");
            SerialMon.printf("Battery voltage: %.2fV\n", batteryVoltage);
            SerialMon.println("GPS fix presente in cache, OK per sleep");
            
            // Determina tempo di sleep in base all'ora
            uint64_t sleepTime;
            if (timeKnown && isDayTime) {
                sleepTime = TIME_TO_SLEEP_DAY;
                SerialMon.println("Sleep time: 600 seconds (10 minuti - GIORNO)");
            } else {
                sleepTime = TIME_TO_SLEEP_NIGHT;
                if (timeKnown) {
                    SerialMon.println("Sleep time: 1800 seconds (30 minuti - NOTTE)");
                } else {
                    SerialMon.println("Sleep time: 1800 seconds (30 minuti - ORA SCONOSCIUTA)");
                }
            }
            
            modemPowerOff();
            delay(5000);
            goToDeepSleep(sleepTime);
        } else {
            SerialMon.println("USB powered (voltage < 0.5V) - skip deep sleep");
            sentSampleCount = 0;
            
            digitalWrite(LED_PIN, HIGH);
            delay(1000);
            digitalWrite(LED_PIN, LOW);
        }
    }

    // Gestione riconnessione MQTT
    if (!mqtt.connected()) {
        uint32_t t = millis();
        if (t - lastReconnectAttempt > 10000L) {
            SerialMon.println("MQTT not connected");
            lastReconnectAttempt = t;
            
            if (!modem.isGprsConnected()) {
                SerialMon.println("GPRS lost");
                connectionOK = false;
                connManager.reset();
                return;
            }
            
            if (mqttConnect()) {
                lastReconnectAttempt = 0;
            }
        }
        return;
    }

    mqtt.loop();

    // Invio dati periodico
    static unsigned long lastSend = 0;
    if (millis() - lastSend >= SEND_INTERVAL_MS) {
        lastSend = millis();
        
        // Lettura e invio temperatura
        sensors.requestTemperatures();
        float temperatureC = sensors.getTempCByIndex(0);
        
        if (temperatureC != DEVICE_DISCONNECTED_C) {
            char tempStr[10];
            dtostrf(temperatureC, 4, 1, tempStr);
            mqtt.publish(topicTemperature, tempStr);
            SerialMon.print("Temperature sent: ");
            SerialMon.println(tempStr);
        }
        
        // Invio tensione batteria
        char battStr[10];
        dtostrf(batteryVoltage, 4, 2, battStr);
        mqtt.publish(topicBatteryStatus, battStr);
        SerialMon.print("Battery voltage sent: ");
        SerialMon.println(battStr);
        
        // Invio livello segnale
        int signal = modem.getSignalQuality();
        char signalStr[10];
        itoa(signal, signalStr, 10);
        mqtt.publish(topicSignalLevel, signalStr);
        SerialMon.print("Signal level sent: ");
        SerialMon.println(signalStr);
        
        // GPS DEBUG & PUBLISH
        SerialMon.println("\n--- GPS DEBUG ---");

        // Controlla se abbiamo cache valida
        if (gpsCache.isValid()) {
            unsigned long cacheAge = (millis() - gpsCache.lastFixTime) / 1000;
            SerialMon.printf("Usando GPS cache (età: %lu s)\n", cacheAge);

            // Pubblica dati dalla cache
            char latStr[16], lonStr[16];
            dtostrf(gpsCache.latitude, 10, 6, latStr);
            dtostrf(gpsCache.longitude, 10, 6, lonStr);

            mqtt.publish(topicGPSlat, latStr);
            mqtt.publish(topicGPSlon, lonStr);
            mqtt.publish(topicDateTime, gpsCache.utcTime.c_str());
            SerialMon.printf("GPS PUBLISHED (cache): %s, %s @ %s UTC\n",
                            latStr, lonStr, gpsCache.utcTime.c_str());

            // GPS sempre acceso, prova comunque a refreshare cache
            SerialMon.println("Tento refresh cache GPS...");

            modem.sendAT("+CGNSINF");
            String cgnsinf = "";
            if (modem.waitResponse(3000L, cgnsinf) == 1) {
                cgnsinf.trim();
                int pos = cgnsinf.indexOf("+CGNSINF: 1,");
                bool fixOk = (pos >= 0 && cgnsinf.charAt(pos + 12) != '0');
                if (fixOk) {
                    // Estrazione rapida campi per aggiornare cache
                    int start = pos, field = 0;
                    String utcStr, newLatStr, newLonStr;
                    while (field < 6 && start > -1) {
                        int next = cgnsinf.indexOf(',', start);
                        if (next == -1) break;
                        switch (field) {
                            case 2: utcStr = cgnsinf.substring(start, next); break;
                            case 3: newLatStr = cgnsinf.substring(start, next); break;
                            case 4: newLonStr = cgnsinf.substring(start, next); break;
                        }
                        start = next + 1;
                        field++;
                    }

                    float lat = newLatStr.toFloat();
                    float lon = newLonStr.toFloat();
                    if (lat != 0.0f || lon != 0.0f) {
                        gpsCache.update(lat, lon, utcStr);
                        SerialMon.println("Cache GPS aggiornata con nuovo fix");

                        // Aggiorna info giorno/notte
                        if (utcStr.length() >= 8) {
                            int hour = utcStr.substring(8, 10).toInt();
                            isDayTime = (hour >= 6 && hour < 18);
                            timeKnown = true;
                        }
                    }
                } else {
                    SerialMon.println("Nessun nuovo fix disponibile");
                }
            }
        } else {
            // Nessuna cache valida, prova ad ottenere fix
            SerialMon.println("Nessuna cache GPS, tentativo acquisizione...");
            modem.sendAT("+CGNSINF");
            String cgnsinf = "";
            if (modem.waitResponse(3000L, cgnsinf) == 1) {
                cgnsinf.trim();
                SerialMon.println("CGNSINF: " + cgnsinf);

                if (cgnsinf.startsWith("+CGNSINF: 0")) {
                    SerialMon.println("GNSS core is OFF, forcing ON...");
                    enableGPS();
                    return;
                }

                int pos = cgnsinf.indexOf("+CGNSINF: 1,");
                bool fixOk = (pos >= 0 && cgnsinf.charAt(pos + 12) != '0');
                if (fixOk) {
                    // Estrazione campi
                    int start = pos, field = 0;
                    String utcStr, latStr, lonStr;
                    while (field < 6 && start > -1) {
                        int next = cgnsinf.indexOf(',', start);
                        if (next == -1) break;
                        switch (field) {
                            case 2: utcStr = cgnsinf.substring(start, next); break;
                            case 3: latStr = cgnsinf.substring(start, next); break;
                            case 4: lonStr = cgnsinf.substring(start, next); break;
                        }
                        start = next + 1;
                        field++;
                    }

                    float lat = latStr.toFloat();
                    float lon = lonStr.toFloat();
                    if (lat != 0.0f || lon != 0.0f) {
                        // Salva in cache
                        gpsCache.update(lat, lon, utcStr);

                        mqtt.publish(topicGPSlat, latStr.c_str());
                        mqtt.publish(topicGPSlon, lonStr.c_str());
                        mqtt.publish(topicDateTime, utcStr.c_str());
                        SerialMon.printf("GPS PUBLISHED: %s, %s @ %s UTC\n",
                                        latStr.c_str(), lonStr.c_str(), utcStr.c_str());
                        gpsFixObtained = true;

                        // Aggiorna info giorno/notte con ora UTC
                        if (utcStr.length() >= 8) {
                            int hour = utcStr.substring(8, 10).toInt();
                            isDayTime = (hour >= 6 && hour < 18);
                            timeKnown = true;
                            SerialMon.printf("Ora UTC: %02d, Periodo: %s\n",
                                           hour, isDayTime ? "GIORNO" : "NOTTE");
                        }

                        // GPS rimane acceso
                        SerialMon.println("GPS fix ottenuto e salvato");
                    }
                } else {
                    SerialMon.println("No valid GPS fix in CGNSINF");
                }
            } else {
                // Nessuna risposta CGNSINF, assicurati che GPS sia acceso
                if (!gpsCurrentlyEnabled) {
                    SerialMon.println("GPS non risponde, tento riattivazione...");
                    enableGPS();
                }
            }
        }
        
        sentSampleCount++;
        SerialMon.printf("Samples sent: %d/%d\n", sentSampleCount, MAX_SAMPLES_BEFORE_SLEEP);
    }
}

// Deep sleep con calcolo tempo basato su ora del giorno
void goToDeepSleep(uint64_t seconds) {
    float v = readBatteryVoltage();
    if (v < 0.5) { 
        sentSampleCount = 0; 
        return; 
    }

    SerialMon.printf("Deep-sleep %llu s\n", seconds);
    
    disableGPS();
    modem.sendAT("AT+CPOWD=1");
    modem.poweroff();
    
    esp_sleep_enable_timer_wakeup(seconds * uS_TO_S_FACTOR);
    esp_deep_sleep_start();
}

// Log statistiche connessione
void logConnectionStats() {
    static unsigned long lastLog = 0;
    
    if (millis() - lastLog > 60000) {
        lastLog = millis();
        
        SerialMon.println("\n=== Connection Status ===");
        SerialMon.printf("Device ID: %s\n", DEVICE_ID);
        SerialMon.printf("APN: %s\n", APN);
        SerialMon.printf("Connected: %s\n", connectionOK ? "YES" : "NO");
        SerialMon.printf("Signal: %d\n", modem.getSignalQuality());
        SerialMon.printf("Battery: %.2fV\n", readBatteryVoltage());
        SerialMon.printf("Samples sent: %d/%d\n", sentSampleCount, MAX_SAMPLES_BEFORE_SLEEP);
        SerialMon.printf("Connection attempts: %d\n", connManager.attemptCount);
        SerialMon.printf("GPS fix obtained: %s\n", gpsFixObtained ? "YES" : "NO");
        SerialMon.printf("GPS enabled: %s\n", gpsCurrentlyEnabled ? "YES" : "NO");
        SerialMon.printf("GPS cache: %s", gpsCache.hasValidFix ? "VALID" : "INVALID");
        if (gpsCache.hasValidFix) {
            unsigned long age = (millis() - gpsCache.lastFixTime) / 1000;
            SerialMon.printf(" (age: %lu s)\n", age);
        } else {
            SerialMon.println();
        }
        SerialMon.printf("Time known: %s, Period: %s\n",
                        timeKnown ? "YES" : "NO",
                        timeKnown ? (isDayTime ? "GIORNO" : "NOTTE") : "N/A");
        SerialMon.println("========================\n");
    }
}

// Setup principale
void setup() {
    SerialMon.begin(115200);
    delay(100);
    
    // Accendo il modem PRIMA di aprire la UART
    modemPowerOn();
    delay(5000);
    SerialAT.begin(UART_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);
    
    SerialMon.println("\n\n=== RailTemp 3.0 – GPS Enhanced ===");
    SerialMon.printf("Device ID: %s  –  APN: %s\n", DEVICE_ID, APN);
    
    pinMode(LED_PIN, OUTPUT);
    pinMode(BATTERY_PIN, INPUT);
    
    for (int i = 0; i < 3; i++) { 
        digitalWrite(LED_PIN, HIGH); 
        delay(200); 
        digitalWrite(LED_PIN, LOW); 
        delay(200); 
    }
    
    updateMqttTopics();
    sensors.begin();
    
    WiFi.mode(WIFI_OFF); 
    WiFi.disconnect(true);
    
    sentSampleCount = 0;
    gpsFixObtained = false;
    timeKnown = false;
    isDayTime = false;
    gpsCurrentlyEnabled = false;
    gpsCache.clear();

    mqtt.setServer(broker, 1883);
    mqtt.setCallback(mqttCallback);
    
    SerialMon.println("Setup completed – starting main loop\n");
}

// Loop principale
void loop() {
    ConnectTask();
    SendMqttDataTask();
    logConnectionStats();
    
    // Indicazione LED dello stato
    static unsigned long lastBlink = 0;
    
    if (connectionOK && millis() - lastBlink > 10000) {
        lastBlink = millis();
        
        for(int i = 0; i < sentSampleCount; i++) {
            digitalWrite(LED_PIN, HIGH);
            delay(200);
            digitalWrite(LED_PIN, LOW);
            delay(200);
        }
    }
    
    delay(100);
}