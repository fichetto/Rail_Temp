#include <Arduino.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// ===== CONFIGURAZIONE DISPOSITIVO =====
const char* DEVICE_ID = "Railtemp03";              // ID del dispositivo
const char* APN = "shared.tids.tim.it";            // APN dell'operatore TIM
const char* FIRMWARE_VERSION = "1.1.6";            // Versione firmware (per OTA)
// =======================================

// Configurazione pin sensore temperatura
#define ONE_WIRE_BUS 18
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// Configurazione seriali
#define SerialMon Serial
#define SerialAT Serial1

// Configurazione modem GSM
#define TINY_GSM_MODEM_SIM7000  // Usa versione standard (SSL via SSLClient)
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

// Configurazione GPS Task asincrono
#define GPS_CHECK_INTERVAL 5000         // Controlla GPS ogni 5 secondi
#define GPS_TASK_STACK_SIZE 4096        // Stack size per task GPS
#define GPS_TASK_PRIORITY 1             // Priorità task GPS (bassa)

#include <TinyGsmClient.h>
#include <SSLClient.h>
#include <PubSubClient.h>
#include <Update.h>
#include <ArduinoJson.h>

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

// Nuovi topic per OTA e diagnostica
char topicCmdOta[50];           // Comando OTA (in ingresso)
char topicCmdReboot[50];        // Comando reboot (in ingresso)
char topicCmdDiagnostics[50];   // Richiesta diagnostica (in ingresso)
char topicOtaStatus[50];        // Stato OTA (in uscita)
char topicDiagnostics[50];      // Diagnostica JSON (in uscita)

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

// FreeRTOS Task e Mutex per GPS asincrono
TaskHandle_t gpsTaskHandle = NULL;
SemaphoreHandle_t gpsCacheMutex = NULL;
SemaphoreHandle_t modemMutex = NULL;
volatile bool gpsTaskRunning = false;
volatile bool gpsTaskShouldStop = false;
volatile bool mqttInProgress = false;  // Flag per far pausare GPS task durante MQTT

// Cache GPS per riutilizzo ultimo fix valido (thread-safe con mutex)
struct GPSCache {
    bool hasValidFix = false;
    float latitude = 0.0f;
    float longitude = 0.0f;
    char utcTime[32] = "";
    unsigned long lastFixTime = 0;
    const unsigned long MAX_CACHE_AGE = 3600000;  // 1 ora validità cache

    bool isValid() {
        return hasValidFix && (millis() - lastFixTime < MAX_CACHE_AGE);
    }

    void update(float lat, float lon, const char* utc) {
        latitude = lat;
        longitude = lon;
        strncpy(utcTime, utc, sizeof(utcTime) - 1);
        utcTime[sizeof(utcTime) - 1] = '\0';
        lastFixTime = millis();
        hasValidFix = true;
    }

    void clear() {
        hasValidFix = false;
        latitude = 0.0f;
        longitude = 0.0f;
        utcTime[0] = '\0';
        lastFixTime = 0;
    }
} gpsCache;

// Variabili OTA
volatile bool otaInProgress = false;
String otaUrl = "";
String otaChecksum = "";
int otaProgress = 0;
String lastError = "none";

// Dichiarazioni funzioni
void modemPowerOn();
void modemPowerOff();
void modemRestart();
void enableGPS();
void disableGPS();
void startGPSTask();
void stopGPSTask();
void gpsTask(void* parameter);
bool checkGPSFix();
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
void sendATCommand(String cmd, int timeout = 1000);
bool getGPSCacheSafe(float &lat, float &lon, char* utc, size_t utcSize);
void updateGPSCacheSafe(float lat, float lon, const char* utc);

// Nuove funzioni OTA e diagnostica
void sendDiagnostics();
void publishOtaStatus(const char* status, int progress = -1, const char* error = nullptr);
void handleOtaCommand(const char* payload, unsigned int len);
void performOtaUpdate();
bool httpGetToUpdate(const char* url, const char* expectedChecksum);

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

    // Topic OTA e diagnostica
    snprintf(topicCmdOta, sizeof(topicCmdOta), "%s/cmd/ota", DEVICE_ID);
    snprintf(topicCmdReboot, sizeof(topicCmdReboot), "%s/cmd/reboot", DEVICE_ID);
    snprintf(topicCmdDiagnostics, sizeof(topicCmdDiagnostics), "%s/cmd/diagnostics", DEVICE_ID);
    snprintf(topicOtaStatus, sizeof(topicOtaStatus), "%s/ota/status", DEVICE_ID);
    snprintf(topicDiagnostics, sizeof(topicDiagnostics), "%s/diagnostics", DEVICE_ID);
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

// ============== FUNZIONI GPS THREAD-SAFE ==============

// Accesso thread-safe alla cache GPS per lettura
bool getGPSCacheSafe(float &lat, float &lon, char* utc, size_t utcSize) {
    if (xSemaphoreTake(gpsCacheMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        bool valid = gpsCache.isValid();
        if (valid) {
            lat = gpsCache.latitude;
            lon = gpsCache.longitude;
            strncpy(utc, gpsCache.utcTime, utcSize - 1);
            utc[utcSize - 1] = '\0';
        }
        xSemaphoreGive(gpsCacheMutex);
        return valid;
    }
    return false;
}

// Accesso thread-safe alla cache GPS per scrittura
void updateGPSCacheSafe(float lat, float lon, const char* utc) {
    if (xSemaphoreTake(gpsCacheMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        gpsCache.update(lat, lon, utc);
        gpsFixObtained = true;

        // Aggiorna info giorno/notte
        if (strlen(utc) >= 10) {
            char hourStr[3] = {utc[8], utc[9], '\0'};
            int hour = atoi(hourStr);
            isDayTime = (hour >= 6 && hour < 18);
            timeKnown = true;
        }
        xSemaphoreGive(gpsCacheMutex);
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

    // Prendi il mutex modem per operazioni AT
    if (modemMutex != NULL && xSemaphoreTake(modemMutex, pdMS_TO_TICKS(5000)) == pdTRUE) {
        // 1. Spegni GNSS se attivo
        modem.sendAT("+CGNSPWR=0");
        modem.waitResponse(2000L);
        delay(1000);

        // 2. Accendi antenna GPS via GPIO4 (specifico LilyGO T-SIM7000G)
        modem.sendAT("+SGPIO=0,4,1,1");
        modem.waitResponse(1000L);
        delay(1000);

        // 3. Accendi GNSS
        modem.sendAT("+CGNSPWR=1");
        modem.waitResponse(2000L);
        delay(2000);

        // 4. Verifica stato GNSS
        modem.sendAT("+CGNSPWR?");
        String response = "";
        if (modem.waitResponse(2000L, response) == 1) {
            SerialMon.println("GNSS Power status: " + response);
        }

        xSemaphoreGive(modemMutex);
        gpsCurrentlyEnabled = true;
        SerialMon.println("=== GNSS ENABLED ===\n");
    } else {
        SerialMon.println("ERRORE: impossibile ottenere mutex modem per enableGPS");
    }
}

void disableGPS() {
    if (!gpsCurrentlyEnabled) {
        SerialMon.println("GPS già disabilitato, skip");
        return;
    }

    SerialMon.println("Disabilitazione GPS per risparmio energetico...");

    if (modemMutex != NULL && xSemaphoreTake(modemMutex, pdMS_TO_TICKS(5000)) == pdTRUE) {
        modem.sendAT("+CGNSPWR=0");
        modem.waitResponse(2000L);
        modem.sendAT("+SGPIO=0,4,1,0");
        modem.waitResponse(2000L);
        xSemaphoreGive(modemMutex);
        gpsCurrentlyEnabled = false;
        SerialMon.println("GPS disabilitato");
    }
}

// ============== TASK GPS ASINCRONO (FreeRTOS) ==============

// Task GPS che gira in background e cerca continuamente il fix
void gpsTask(void* parameter) {
    SerialMon.println("[GPS Task] Avviato");
    gpsTaskRunning = true;

    unsigned long lastCheck = 0;
    int checkCount = 0;

    while (!gpsTaskShouldStop) {
        // Se MQTT è in corso, aspetta senza usare il modem
        if (mqttInProgress) {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        // Controlla ogni GPS_CHECK_INTERVAL ms
        if (millis() - lastCheck >= GPS_CHECK_INTERVAL) {
            lastCheck = millis();
            checkCount++;

            // Controllo batteria ogni 30 secondi (ogni 6 check)
            if (checkCount % 6 == 0) {
                float currentBatt = readBatteryVoltage();
                if (currentBatt < 3.4 && currentBatt > 0.5) {
                    SerialMon.println("[GPS Task] Batteria critica, pausa ricerca");
                    vTaskDelay(pdMS_TO_TICKS(60000));  // Pausa 1 minuto
                    continue;
                }
            }

            // Ricontrolla flag MQTT prima di acquisire mutex
            if (mqttInProgress) {
                vTaskDelay(pdMS_TO_TICKS(100));
                continue;
            }

            // Prova ad ottenere mutex modem (timeout breve per non bloccare MQTT)
            if (modemMutex != NULL && xSemaphoreTake(modemMutex, pdMS_TO_TICKS(500)) == pdTRUE) {
                // Interroga GPS
                modem.sendAT("+CGNSINF");
                String cgnsinf = "";
                if (modem.waitResponse(3000L, cgnsinf) == 1) {
                    cgnsinf.trim();

                    // Se GNSS è OFF, riaccendilo
                    if (cgnsinf.startsWith("+CGNSINF: 0")) {
                        SerialMon.println("[GPS Task] GNSS OFF, riaccendo...");
                        xSemaphoreGive(modemMutex);
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
                            SerialMon.printf("[GPS Task] FIX: %.6f, %.6f @ %s\n",
                                            lat, lon, utcStr.c_str());

                            // Aggiorna cache in modo thread-safe
                            xSemaphoreGive(modemMutex);  // Rilascia modem prima
                            updateGPSCacheSafe(lat, lon, utcStr.c_str());
                            continue;  // Salta il rilascio mutex sotto
                        }
                    } else {
                        // Solo log occasionale per non spammare
                        if (checkCount % 12 == 0) {
                            SerialMon.println("[GPS Task] Cercando satelliti...");
                        }
                    }
                }
                xSemaphoreGive(modemMutex);
            }

            // Toggle LED durante ricerca
            digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        }

        // Yield per permettere ad altri task di eseguire
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    SerialMon.println("[GPS Task] Terminato");
    gpsTaskRunning = false;
    gpsTaskHandle = NULL;
    vTaskDelete(NULL);
}

// Avvia il task GPS in background
void startGPSTask() {
    if (gpsTaskHandle != NULL) {
        SerialMon.println("GPS Task già in esecuzione");
        return;
    }

    // Crea mutex se non esistono
    if (gpsCacheMutex == NULL) {
        gpsCacheMutex = xSemaphoreCreateMutex();
    }
    if (modemMutex == NULL) {
        modemMutex = xSemaphoreCreateMutex();
    }

    gpsTaskShouldStop = false;

    // Abilita GPS prima di avviare il task
    enableGPS();

    // Crea il task GPS su core 0 (il loop principale gira su core 1)
    xTaskCreatePinnedToCore(
        gpsTask,              // Funzione task
        "GPSTask",            // Nome
        GPS_TASK_STACK_SIZE,  // Stack size
        NULL,                 // Parametri
        GPS_TASK_PRIORITY,    // Priorità
        &gpsTaskHandle,       // Handle
        0                     // Core 0
    );

    SerialMon.println("GPS Task avviato in background");
}

// Ferma il task GPS
void stopGPSTask() {
    if (gpsTaskHandle == NULL) {
        return;
    }

    SerialMon.println("Fermando GPS Task...");
    gpsTaskShouldStop = true;

    // Attendi che il task termini (max 5 secondi)
    int timeout = 50;
    while (gpsTaskRunning && timeout > 0) {
        vTaskDelay(pdMS_TO_TICKS(100));
        timeout--;
    }

    if (gpsTaskRunning) {
        SerialMon.println("GPS Task non risponde, forzo terminazione");
        vTaskDelete(gpsTaskHandle);
        gpsTaskHandle = NULL;
        gpsTaskRunning = false;
    }
}

// Controlla se abbiamo un fix GPS (non bloccante)
bool checkGPSFix() {
    float lat, lon;
    char utc[32];
    return getGPSCacheSafe(lat, lon, utc, sizeof(utc));
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
        bool registered = modem.isNetworkConnected();

        // Verifica REGISTRAZIONE alla rete, non solo segnale
        if (registered && signal != 99 && signal > 0) {
            SerialMon.printf("Network registered! Signal: %d\n", signal);
            SerialMon.println("Operator: " + modem.getOperator());
            return true;
        }

        if ((millis() - startTime) % 5000 < 100) {
            SerialMon.printf("Waiting... Signal: %d, Registered: %s, Time: %lds/%ds\n",
                           signal, registered ? "YES" : "NO",
                           (millis() - startTime) / 1000, maxWaitTime);
        }

        delay(1000);
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    }

    SerialMon.println("Network registration timeout");
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
    // Se già connessi, verifica che la connessione sia ancora attiva
    if (connectionOK) {
        // Usa mutex per evitare conflitti con GPS task
        bool gprsOk = true;
        if (modemMutex != NULL && xSemaphoreTake(modemMutex, pdMS_TO_TICKS(500)) == pdTRUE) {
            gprsOk = modem.isGprsConnected();
            xSemaphoreGive(modemMutex);
        }
        // Se non riusciamo a prendere il mutex, assumiamo connessione OK (modem occupato)

        if (!gprsOk) {
            SerialMon.println("Connection lost - GPRS disconnected");
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
        float v = readBatteryVoltage();
        if (v < 0.5) {
            // USB powered - skip deep sleep e resetta timer per evitare loop
            SerialMon.println("USB powered - resetting connection timer");
            connManager.totalConnectionTime = 0;
            connManager.attemptCount = 0;
            return;
        }
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

    // Dopo 3 fallimenti consecutivi, fai un hard reset del modem via GPIO
    if (connManager.attemptCount > 0 && connManager.attemptCount % 3 == 0) {
        SerialMon.println("=== MODEM POWER CYCLE (3 fallimenti) ===");
        gpsCurrentlyEnabled = false;  // Reset flag GPS perché il modem verrà riavviato
        modemPowerOff();
        delay(3000);
        modemPowerOn();
        delay(5000);
        SerialMon.println("=== MODEM RIAVVIATO ===");
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

    // Avvia task GPS in background (non bloccante)
    startGPSTask();

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

            // Subscribe ai topic di comando
            mqtt.subscribe(topicTestLed);
            mqtt.subscribe(topicCmdOta);
            mqtt.subscribe(topicCmdReboot);
            mqtt.subscribe(topicCmdDiagnostics);

            // Pubblica messaggio init con versione firmware
            char initMsg[100];
            snprintf(initMsg, sizeof(initMsg), "Device started - FW %s", FIRMWARE_VERSION);
            mqtt.publish(topicInit, initMsg);

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

    String topicStr = String(topic);

    // Comando LED (esistente)
    if (topicStr == String(topicTestLed)) {
        ledStatus = !ledStatus;
        digitalWrite(LED_PIN, ledStatus);
        mqtt.publish(topicStatus, ledStatus ? "LED ON" : "LED OFF");
    }
    // Comando OTA
    else if (topicStr == String(topicCmdOta)) {
        handleOtaCommand((const char*)payload, len);
    }
    // Comando Reboot
    else if (topicStr == String(topicCmdReboot)) {
        SerialMon.println("Reboot command received!");
        mqtt.publish(topicStatus, "Rebooting...");
        delay(1000);
        ESP.restart();
    }
    // Comando Diagnostica
    else if (topicStr == String(topicCmdDiagnostics)) {
        SerialMon.println("Diagnostics request received");
        sendDiagnostics();
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
        // Se alimentato da USB (batteria < 0.5V), non andare mai in deep sleep
        if (batteryVoltage < 0.5) {
            SerialMon.println("USB powered (voltage < 0.5V) - skip deep sleep");
            sentSampleCount = 0;
            digitalWrite(LED_PIN, HIGH);
            delay(1000);
            digitalWrite(LED_PIN, LOW);
            return;
        }

        // Verifica se abbiamo mai ottenuto un fix GPS (thread-safe)
        if (!checkGPSFix()) {
            // Senza GPS: vai in deep sleep SOLO se batteria < 3.7V
            if (batteryVoltage < 3.7) {
                SerialMon.println("Nessun fix GPS + batteria bassa!");
                SerialMon.printf("Tensione: %.2fV (< 3.7V) - Deep sleep per preservare batteria\n", batteryVoltage);

                // Lampeggio veloce 10 volte per segnalare batteria bassa senza GPS
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

            // Batteria >= 3.7V: continua a cercare GPS
            SerialMon.println("Nessun fix GPS - continuo ricerca");
            SerialMon.printf("Batteria: %.2fV (>= 3.7V) - Cerco ancora GPS...\n", batteryVoltage);
            sentSampleCount = 0;  // Reset counter per continuare

            // Lampeggio lento per indicare ricerca GPS
            for(int i = 0; i < 3; i++) {
                digitalWrite(LED_PIN, HIGH);
                delay(500);
                digitalWrite(LED_PIN, LOW);
                delay(500);
            }
            return;  // Non andare in deep sleep
        }

        // GPS fix ottenuto e batteria > 0.5V: vai in deep sleep normale
        // Lampeggio rapido 5 volte per indicare deep sleep
        for(int i = 0; i < 5; i++) {
            digitalWrite(LED_PIN, HIGH);
            delay(100);
            digitalWrite(LED_PIN, LOW);
            delay(100);
        }

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
    }

    // Segnala al task GPS che MQTT sta usando il modem
    mqttInProgress = true;
    delay(100);  // Dai tempo al GPS task di finire operazioni in corso

    // Gestione riconnessione MQTT
    if (!mqtt.connected()) {
        uint32_t t = millis();
        if (t - lastReconnectAttempt > 10000L) {
            SerialMon.println("MQTT not connected");
            lastReconnectAttempt = t;

            // Verifica GPRS con mutex
            bool gprsOk = false;
            if (modemMutex != NULL && xSemaphoreTake(modemMutex, pdMS_TO_TICKS(2000)) == pdTRUE) {
                gprsOk = modem.isGprsConnected();
                xSemaphoreGive(modemMutex);
            }

            if (!gprsOk) {
                SerialMon.println("GPRS lost or mutex busy");
                connectionOK = false;
                connManager.reset();
                mqttInProgress = false;  // Rilascia flag
                return;
            }

            if (mqttConnect()) {
                lastReconnectAttempt = 0;
            }
        }
        mqttInProgress = false;  // Rilascia flag
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
        
        // Invio livello segnale (con mutex)
        int signal = 0;
        if (modemMutex != NULL && xSemaphoreTake(modemMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
            signal = modem.getSignalQuality();
            xSemaphoreGive(modemMutex);
        }
        char signalStr[10];
        itoa(signal, signalStr, 10);
        mqtt.publish(topicSignalLevel, signalStr);
        SerialMon.print("Signal level sent: ");
        SerialMon.println(signalStr);
        
        // GPS PUBLISH - leggi dalla cache thread-safe (il task GPS aggiorna in background)
        SerialMon.println("\n--- GPS STATUS ---");

        float gpsLat, gpsLon;
        char gpsUtc[32];

        if (getGPSCacheSafe(gpsLat, gpsLon, gpsUtc, sizeof(gpsUtc))) {
            // Cache GPS valida - pubblica
            char latStr[16], lonStr[16];
            dtostrf(gpsLat, 10, 6, latStr);
            dtostrf(gpsLon, 10, 6, lonStr);

            mqtt.publish(topicGPSlat, latStr);
            mqtt.publish(topicGPSlon, lonStr);
            mqtt.publish(topicDateTime, gpsUtc);
            SerialMon.printf("GPS PUBLISHED: %s, %s @ %s\n", latStr, lonStr, gpsUtc);
        } else {
            // Nessun fix GPS ancora - il task sta cercando in background
            SerialMon.println("GPS: in ricerca (task in background)");

            // Verifica che il task GPS sia attivo
            if (!gpsTaskRunning && gpsTaskHandle == NULL) {
                SerialMon.println("GPS Task non attivo, riavvio...");
                startGPSTask();
            }
        }
        
        sentSampleCount++;
        SerialMon.printf("Samples sent: %d/%d\n", sentSampleCount, MAX_SAMPLES_BEFORE_SLEEP);
    }

    // Rilascia flag MQTT per permettere al GPS task di riprendere
    mqttInProgress = false;
}

// ============== FUNZIONI DIAGNOSTICA ==============

// Invia diagnostica completa via MQTT
void sendDiagnostics() {
    SerialMon.println("Preparing diagnostics...");

    JsonDocument doc;

    // Info firmware e device
    doc["firmware_version"] = FIRMWARE_VERSION;
    doc["device_id"] = DEVICE_ID;
    doc["uptime_ms"] = millis();
    doc["free_heap"] = ESP.getFreeHeap();

    // Batteria
    doc["battery_voltage"] = readBatteryVoltage();

    // Segnale (con mutex)
    int signal = 0;
    if (modemMutex != NULL && xSemaphoreTake(modemMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        signal = modem.getSignalQuality();
        xSemaphoreGive(modemMutex);
    }
    doc["signal_quality"] = signal;

    // GPS
    float gpsLat = 0, gpsLon = 0;
    char gpsUtc[32] = "";
    bool hasGps = getGPSCacheSafe(gpsLat, gpsLon, gpsUtc, sizeof(gpsUtc));
    doc["gps_fix"] = hasGps;
    if (hasGps) {
        doc["gps_lat"] = gpsLat;
        doc["gps_lon"] = gpsLon;
        if (xSemaphoreTake(gpsCacheMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            doc["gps_cache_age_s"] = (millis() - gpsCache.lastFixTime) / 1000;
            xSemaphoreGive(gpsCacheMutex);
        }
    }

    // Temperatura
    sensors.requestTemperatures();
    float temp = sensors.getTempCByIndex(0);
    if (temp != DEVICE_DISCONNECTED_C) {
        doc["temperature_c"] = temp;
    }

    // Stato connessioni
    doc["mqtt_connected"] = mqtt.connected();
    doc["gprs_connected"] = connectionOK;
    doc["connection_attempts"] = connManager.attemptCount;
    doc["total_samples_sent"] = sentSampleCount;

    // Ultimo errore
    doc["last_error"] = lastError;

    // OTA status
    doc["ota_in_progress"] = otaInProgress;

    // Serializza e invia
    char jsonBuffer[512];
    serializeJson(doc, jsonBuffer, sizeof(jsonBuffer));

    mqtt.publish(topicDiagnostics, jsonBuffer);
    SerialMon.println("Diagnostics sent:");
    SerialMon.println(jsonBuffer);
}

// ============== FUNZIONI OTA ==============

// Pubblica stato OTA via MQTT
void publishOtaStatus(const char* status, int progress, const char* error) {
    JsonDocument doc;
    doc["status"] = status;
    doc["current_version"] = FIRMWARE_VERSION;

    if (progress >= 0) {
        doc["progress"] = progress;
    }
    if (error != nullptr) {
        doc["error"] = error;
    }

    char jsonBuffer[256];
    serializeJson(doc, jsonBuffer, sizeof(jsonBuffer));
    mqtt.publish(topicOtaStatus, jsonBuffer);
    mqtt.loop();  // Assicura invio immediato

    SerialMon.print("OTA Status: ");
    SerialMon.println(jsonBuffer);
}

// Gestisce comando OTA ricevuto via MQTT
void handleOtaCommand(const char* payload, unsigned int len) {
    SerialMon.println("OTA command received");

    // Verifica batteria prima di procedere
    float batt = readBatteryVoltage();
    if (batt < 3.8 && batt > 0.5) {
        SerialMon.printf("Battery too low for OTA: %.2fV\n", batt);
        publishOtaStatus("rejected", -1, "battery_too_low");
        lastError = "OTA rejected: battery too low";
        return;
    }

    // Se già in corso, rifiuta
    if (otaInProgress) {
        SerialMon.println("OTA already in progress");
        publishOtaStatus("rejected", -1, "already_in_progress");
        return;
    }

    // Parse JSON payload: {"url": "...", "checksum": "sha256:..."}
    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, payload, len);

    if (err) {
        SerialMon.print("JSON parse error: ");
        SerialMon.println(err.c_str());
        publishOtaStatus("failed", -1, "invalid_json");
        lastError = "OTA failed: invalid JSON";
        return;
    }

    if (!doc["url"].is<const char*>() || !doc["checksum"].is<const char*>()) {
        SerialMon.println("Missing url or checksum");
        publishOtaStatus("failed", -1, "missing_parameters");
        lastError = "OTA failed: missing parameters";
        return;
    }

    otaUrl = doc["url"].as<String>();
    otaChecksum = doc["checksum"].as<String>();

    SerialMon.println("OTA URL: " + otaUrl);
    SerialMon.println("OTA Checksum: " + otaChecksum);

    // Avvia OTA
    otaInProgress = true;
    publishOtaStatus("starting");

    // Ferma GPS task per liberare risorse
    stopGPSTask();

    // Esegui OTA
    performOtaUpdate();
}

// Esegue l'aggiornamento OTA scaricando via HTTP/GPRS
void performOtaUpdate() {
    SerialMon.println("\n=== STARTING OTA UPDATE ===");

    // Estrai checksum (rimuovi prefisso sha256: se presente)
    String checksum = otaChecksum;
    if (checksum.startsWith("sha256:")) {
        checksum = checksum.substring(7);
    }

    publishOtaStatus("downloading", 0);

    bool success = httpGetToUpdate(otaUrl.c_str(), checksum.c_str());

    if (success) {
        publishOtaStatus("success");
        SerialMon.println("OTA SUCCESS! Rebooting in 3 seconds...");
        delay(3000);
        ESP.restart();
    } else {
        otaInProgress = false;
        publishOtaStatus("failed", -1, lastError.c_str());
        SerialMon.println("OTA FAILED: " + lastError);

        // Riavvia GPS task
        startGPSTask();
    }
}

// Scarica firmware via SSLClient (SSL software con mbedtls)
bool httpGetToUpdate(const char* url, const char* expectedChecksum) {
    SerialMon.println("Starting HTTPS download via SSLClient...");
    SerialMon.println(url);

    // Prendi mutex modem
    if (modemMutex == NULL || xSemaphoreTake(modemMutex, pdMS_TO_TICKS(10000)) != pdTRUE) {
        lastError = "modem_mutex_timeout";
        return false;
    }

    String urlStr = String(url);
    bool isHttps = urlStr.startsWith("https://");

    // Parse URL
    int startIdx = urlStr.indexOf("://") + 3;
    int pathIdx = urlStr.indexOf("/", startIdx);
    if (pathIdx < 0) pathIdx = urlStr.length();

    String host = urlStr.substring(startIdx, pathIdx);
    String path = urlStr.substring(pathIdx);
    if (path.length() == 0) path = "/";
    int port = isHttps ? 443 : 80;

    SerialMon.println("Host: " + host);
    SerialMon.println("Path: " + path);
    SerialMon.printf("Port: %d, HTTPS: %s\n", port, isHttps ? "YES" : "NO");

    // Crea client TCP base
    TinyGsmClient gsmTransport(modem);

    // Wrap con SSLClient per HTTPS (SSL software via mbedtls)
    SSLClient sslClient(&gsmTransport);
    sslClient.setInsecure();  // Skip certificate validation (GitHub usa certificati validi)

    SerialMon.println("Connecting to server via SSLClient...");

    // Per GitHub, dobbiamo seguire i redirect
    String currentHost = host;
    String currentPath = path;
    int currentPort = port;
    int redirectCount = 0;
    const int maxRedirects = 5;
    int contentLength = 0;

    while (redirectCount < maxRedirects) {
        SerialMon.printf("Attempt %d: %s:%d%s\n", redirectCount + 1, currentHost.c_str(), currentPort, currentPath.c_str());

        if (!sslClient.connect(currentHost.c_str(), currentPort)) {
            lastError = "connection_failed";
            SerialMon.println("Connection failed!");
            xSemaphoreGive(modemMutex);
            return false;
        }

        SerialMon.println("Connected! Sending HTTP request...");

        // Invia richiesta HTTP
        sslClient.print("GET ");
        sslClient.print(currentPath);
        sslClient.println(" HTTP/1.1");
        sslClient.print("Host: ");
        sslClient.println(currentHost);
        sslClient.println("User-Agent: ESP32-SIM7000G-OTA");
        sslClient.println("Accept: */*");
        sslClient.println("Connection: close");
        sslClient.println();

        // Leggi risposta header
        String statusLine = "";
        unsigned long timeout = millis() + 30000;

        while (sslClient.connected() && millis() < timeout) {
            if (sslClient.available()) {
                statusLine = sslClient.readStringUntil('\n');
                statusLine.trim();
                break;
            }
            delay(10);
        }

        SerialMon.println("Status: " + statusLine);

        // Parse HTTP status code
        int httpCode = 0;
        int spaceIdx = statusLine.indexOf(' ');
        if (spaceIdx > 0) {
            httpCode = statusLine.substring(spaceIdx + 1, spaceIdx + 4).toInt();
        }

        // Leggi headers
        contentLength = 0;
        String location = "";

        while (sslClient.connected() && millis() < timeout) {
            String line = sslClient.readStringUntil('\n');
            line.trim();

            if (line.length() == 0) {
                break;  // Fine headers
            }

            SerialMon.println("H: " + line);

            if (line.startsWith("Content-Length:") || line.startsWith("content-length:")) {
                contentLength = line.substring(15).toInt();
            }
            if (line.startsWith("Location:") || line.startsWith("location:")) {
                location = line.substring(9);
                location.trim();
            }
        }

        // Handle redirect
        if (httpCode == 301 || httpCode == 302 || httpCode == 307 || httpCode == 308) {
            SerialMon.println("Redirect to: " + location);
            sslClient.stop();

            if (location.length() == 0) {
                lastError = "redirect_no_location";
                xSemaphoreGive(modemMutex);
                return false;
            }

            // Parse nuova location
            if (location.startsWith("http://") || location.startsWith("https://")) {
                bool newHttps = location.startsWith("https://");
                int newStartIdx = location.indexOf("://") + 3;
                int newPathIdx = location.indexOf("/", newStartIdx);
                if (newPathIdx < 0) newPathIdx = location.length();

                currentHost = location.substring(newStartIdx, newPathIdx);
                currentPath = location.substring(newPathIdx);
                if (currentPath.length() == 0) currentPath = "/";
                currentPort = newHttps ? 443 : 80;
            } else {
                // Relative redirect
                currentPath = location;
            }

            redirectCount++;
            delay(1000);
            continue;
        }

        if (httpCode != 200) {
            lastError = "http_error_" + String(httpCode);
            SerialMon.printf("HTTP error: %d\n", httpCode);
            sslClient.stop();
            xSemaphoreGive(modemMutex);
            return false;
        }

        // HTTP 200 OK - procedi con download
        break;
    }

    if (redirectCount >= maxRedirects) {
        lastError = "too_many_redirects";
        xSemaphoreGive(modemMutex);
        return false;
    }

    SerialMon.printf("HTTP 200 OK, Content-Length: %d\n", contentLength);

    if (contentLength <= 0) {
        // Prova a leggere chunked transfer
        lastError = "no_content_length";
        SerialMon.println("Warning: No Content-Length header");
        sslClient.stop();
        xSemaphoreGive(modemMutex);
        return false;
    }

    publishOtaStatus("downloading", 5);

    // Inizia Update
    if (!Update.begin(contentLength)) {
        lastError = "update_begin_failed";
        SerialMon.println("Update.begin() failed");
        sslClient.stop();
        xSemaphoreGive(modemMutex);
        return false;
    }

    // Leggi e scrivi body
    const int chunkSize = 1024;
    uint8_t buffer[chunkSize];
    int bytesRead = 0;
    int lastProgress = 0;
    unsigned long lastActivityTime = millis();

    while (bytesRead < contentLength) {
        if (sslClient.available()) {
            int toRead = min(chunkSize, contentLength - bytesRead);
            int actualRead = sslClient.read(buffer, toRead);

            if (actualRead > 0) {
                size_t written = Update.write(buffer, actualRead);
                if (written != (size_t)actualRead) {
                    lastError = "update_write_failed";
                    Update.abort();
                    sslClient.stop();
                    xSemaphoreGive(modemMutex);
                    return false;
                }
                bytesRead += actualRead;
                lastActivityTime = millis();

                // Aggiorna progresso ogni 10%
                int progress = (bytesRead * 100) / contentLength;
                if (progress >= lastProgress + 10) {
                    lastProgress = progress;
                    publishOtaStatus("downloading", progress);
                    SerialMon.printf("Download: %d%% (%d/%d)\n", progress, bytesRead, contentLength);
                }
            }
        } else if (!sslClient.connected()) {
            if (bytesRead < contentLength) {
                lastError = "connection_lost";
                SerialMon.println("Connection lost during download");
                Update.abort();
                xSemaphoreGive(modemMutex);
                return false;
            }
            break;
        }

        // Timeout check
        if (millis() - lastActivityTime > 60000) {
            lastError = "download_timeout";
            SerialMon.println("Download timeout");
            Update.abort();
            sslClient.stop();
            xSemaphoreGive(modemMutex);
            return false;
        }

        yield();
        delay(1);
    }

    sslClient.stop();
    xSemaphoreGive(modemMutex);

    SerialMon.printf("Download complete: %d bytes\n", bytesRead);
    publishOtaStatus("verifying");

    // Finalizza update
    if (!Update.end(true)) {
        lastError = "update_end_failed";
        SerialMon.println("Update.end() failed");
        return false;
    }

    SerialMon.println("Update completed successfully!");
    return true;
}

// Deep sleep con calcolo tempo basato su ora del giorno
void goToDeepSleep(uint64_t seconds) {
    float v = readBatteryVoltage();
    if (v < 0.5) {
        sentSampleCount = 0;
        return;
    }

    SerialMon.printf("Deep-sleep %llu s\n", seconds);

    // Ferma task GPS prima di andare in deep sleep
    stopGPSTask();

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

    // Crea mutex per accesso thread-safe al modem e GPS cache
    if (modemMutex == NULL) {
        modemMutex = xSemaphoreCreateMutex();
    }
    if (gpsCacheMutex == NULL) {
        gpsCacheMutex = xSemaphoreCreateMutex();
    }

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