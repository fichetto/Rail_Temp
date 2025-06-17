#include <Arduino.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// ===== CONFIGURAZIONE DISPOSITIVO - MODIFICARE SOLO QUESTE DUE RIGHE =====
const char* DEVICE_ID = "Railtemp03";              // ID del dispositivo
const char* APN = "shared.tids.tim.it";                          // APN dell'operatore "WSIM per wherever"
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

// NUOVO: Configurazione attesa GPS
#define GPS_INITIAL_WAIT_TIME 120000    // 2 minuti di attesa iniziale per GPS
#define GPS_CHECK_INTERVAL 5000         // Controlla GPS ogni 5 secondi

// DEBUG: Disabilita attesa GPS per test (decommenta per saltare attesa GPS)
// #define SKIP_GPS_WAIT

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
unsigned long gpsWaitStartTime = 0;
const unsigned long GPS_WAIT_TIMEOUT = 600000;  // 10 minuti
const float BATTERY_GPS_WAIT_THRESHOLD = 3.9;   // Soglia per attesa GPS
bool waitingForGPS = false;
bool gpsInitialWaitDone = false;  // NUOVO: flag per attesa iniziale

// Dichiarazioni funzioni
void modemPowerOn();
void modemPowerOff();
void modemRestart();
void enableGPS();
void disableGPS();
void waitForInitialGPSFix();
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
    digitalWrite(PWR_PIN, LOW);
    delay(1000);
    digitalWrite(PWR_PIN, HIGH);
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

// Gestione GPS MIGLIORATA
void enableGPS() {
    SerialMon.println("\n=== ENABLING GPS ===");
    
    // Prima verifica lo stato attuale
    SerialMon.println("Checking current GPS status...");
    modem.sendAT("+CGPS?");
    String response = "";
    if (modem.waitResponse(2000L, response) == 1) {
        SerialMon.println("GPS Status Response: " + response);
    }
    
    // Reset GPS se necessario
    SerialMon.println("Resetting GPS...");
    modem.sendAT("+CGPS=0");
    if (modem.waitResponse(3000L, response) == 1) {
        SerialMon.println("GPS Reset Response: " + response);
    }
    delay(2000);
    
    // NON usiamo SGPIO se causa problemi - potrebbe essere già alimentato
    // SerialMon.println("Skipping SGPIO power command - GPS may be auto-powered");
    
    // Prova comandi di configurazione alternativi
    SerialMon.println("Testing GPS configuration commands...");
    
    // Test CGPSNMEA
    modem.sendAT("+CGPSNMEA=0");  // Disable NMEA output
    modem.waitResponse(1000L);
    
    // Abilita GPS - metodo semplice
    SerialMon.println("Enabling GPS (simple mode)...");
    modem.sendAT("+CGPS=1");    // Solo enable, senza reset mode
    if (modem.waitResponse(5000L, response) == 1) {
        SerialMon.println("GPS Enable Response: " + response);
        
        // Verifica se è davvero abilitato
        delay(2000);
        modem.sendAT("+CGPS?");
        if (modem.waitResponse(2000L, response) == 1) {
            SerialMon.println("GPS Status After Enable: " + response);
            
            // Se contiene "+CGPS: 1" è abilitato
            if (response.indexOf("+CGPS: 1") >= 0) {
                SerialMon.println("GPS is ENABLED successfully!");
            } else {
                SerialMon.println("GPS may not be enabled properly");
            }
        }
    } else {
        SerialMon.println("No response to CGPS=1 command");
        
        // Prova metodo TinyGSM
        SerialMon.println("Trying TinyGSM enableGPS method...");
        if (modem.enableGPS()) {
            SerialMon.println("GPS enabled with TinyGSM method");
        } else {
            SerialMon.println("TinyGSM enableGPS also failed");
        }
    }
    
    // Test comandi informativi
    delay(2000);
    SerialMon.println("\nTesting GPS info commands:");
    
    modem.sendAT("+CGPSINFO");
    if (modem.waitResponse(2000L, response) == 1) {
        SerialMon.println("CGPSINFO: " + response);
    }
    
    modem.sendAT("+CGPSSTATUS?");
    if (modem.waitResponse(2000L, response) == 1) {
        SerialMon.println("CGPSSTATUS: " + response);
    }
    
    SerialMon.println("=== GPS ENABLE COMPLETE ===\n");
}

// NUOVA FUNZIONE: Attesa iniziale GPS fix
void waitForInitialGPSFix() {
    if (gpsInitialWaitDone) return;
    
    SerialMon.println("\n=== INITIAL GPS FIX WAIT ===");
    SerialMon.println("Waiting up to 2 minutes for initial GPS fix...");
    SerialMon.println("Make sure you are OUTDOORS or near a window!");
    SerialMon.println("GPS needs clear sky view to work.\n");
    
    unsigned long startWait = millis();
    int checkCount = 0;
    
    while (millis() - startWait < GPS_INITIAL_WAIT_TIME) {
        // Check GPS ogni 5 secondi
        if (millis() - startWait > (checkCount * GPS_CHECK_INTERVAL)) {
            checkCount++;
            
            SerialMon.printf("\nGPS Check #%d (Time: %lus)\n", checkCount, (millis() - startWait) / 1000);
            
            // Prima controlla lo stato del GPS
            modem.sendAT("+CGPSSTATUS?");
            String statusResponse = "";
            if (modem.waitResponse(2000L, statusResponse) == 1) {
                SerialMon.println("GPS Status: " + statusResponse);
            }
            
            // Controlla info GPS raw
            modem.sendAT("+CGPSINFO");
            String gpsRaw = "";
            if (modem.waitResponse(3000L, gpsRaw) == 1) {
                SerialMon.println("GPS Raw: " + gpsRaw);
                
                // Se CGPSINFO contiene coordinate valide (non vuote)
                if (gpsRaw.length() > 20 && gpsRaw.indexOf(",,,,") < 0) {
                    SerialMon.println("GPS seems to have data!");
                }
            }
            
            // Prova a ottenere coordinate
            float lat = 0, lon = 0, speed = 0, alt = 0;
            int vsat = 0, usat = 0;
            
            if (modem.getGPS(&lat, &lon, &speed, &alt, &vsat, &usat)) {
                SerialMon.printf("GPS data received - Lat: %.6f, Lon: %.6f\n", lat, lon);
                SerialMon.printf("Satellites: %d visible, %d used\n", vsat, usat);
                
                if (lat != 0 || lon != 0) {
                    SerialMon.printf("*** GPS FIX OBTAINED! ***\n");
                    gpsFixObtained = true;
                    break;
                } else {
                    SerialMon.println("GPS active but coordinates are 0,0");
                }
            } else {
                SerialMon.println("No GPS fix yet...");
                
                // Prova comando alternativo per info
                modem.sendAT("+CGPSINF=0");
                String altInfo = "";
                if (modem.waitResponse(2000L, altInfo) == 1) {
                    SerialMon.println("Alt GPS Info: " + altInfo);
                }
            }
            
            // LED flash durante attesa
            digitalWrite(LED_PIN, HIGH);
            delay(100);
            digitalWrite(LED_PIN, LOW);
        }
        delay(100);
    }
    
    gpsInitialWaitDone = true;
    
    if (gpsFixObtained) {
        SerialMon.println("\n=== GPS FIX SUCCESS ===");
    } else {
        SerialMon.println("\n=== GPS TIMEOUT - Proceeding without fix ===");
        SerialMon.println("GPS Tips:");
        SerialMon.println("1. Move device outdoors with clear sky view");
        SerialMon.println("2. Check GPS antenna is connected");
        SerialMon.println("3. First fix can take 30-120 seconds");
        SerialMon.println("4. Indoor GPS reception is usually impossible\n");
    }
}

void disableGPS() {
    modem.sendAT("+CGPS=0");
    modem.waitResponse(3000L);
    modem.sendAT("+SGPIO=0,4,1,0");
    modem.waitResponse(2000L);
}

// Inizializzazione modem con gestione errori
bool initializeModem() {
    SerialMon.println("Initializing modem...");
    
    // Test comunicazione AT
    for (int i = 0; i < 3; i++) {
        if (modem.testAT()) {
            SerialMon.println("Modem responding");
            break;
        }
        if (i == 2) {
            SerialMon.println("Modem not responding - restarting");
            modemRestart();
            delay(5000);
            return false;
        }
        delay(1000);
    }
    
    // Configurazione modem
    modem.sendAT("+CFUN=0");
    modem.waitResponse(10000L);
    delay(3000);
    
    modem.sendAT("+CFUN=1");
    if (modem.waitResponse(30000L) != 1) {
        SerialMon.println("Failed to set full functionality");
        return false;
    }
    
    // Configurazione APN
    SerialMon.print("Setting APN: ");
    SerialMon.println(APN);
    modem.sendAT("+CGDCONT=1,\"IP\",\"" + String(APN) + "\"");
    if (modem.waitResponse(10000L) != 1) {
        SerialMon.println("Failed to set APN");
        return false;
    }
    delay(2000);
    
    // Modalità rete LTE
    modem.sendAT("+CNMP=38");
    modem.waitResponse(5000L);
    
    modem.sendAT("+CMNB=1");
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
        // Controlla il segnale - se non è 99, siamo connessi
        int signal = modem.getSignalQuality();
        
        if (signal != 99 && signal > 0) {
            SerialMon.printf("Network connected! Signal: %d\n", signal);
            
            // Ottieni operatore
            String cop = modem.getOperator();
            SerialMon.println("Operator: " + cop);
            
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
    
    // Usa direttamente il metodo gprsConnect di TinyGSM
    SerialMon.printf("Attempting GPRS connection with APN: %s\n", APN);
    
    if (modem.gprsConnect(APN, gprsUser, gprsPass)) {
        SerialMon.println("GPRS connected!");
        
        // Verifica IP
        String ip = modem.getLocalIP();
        SerialMon.println("IP address: " + ip);
        
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
    if (battVoltage < 3.6 && battVoltage > 0.5) {  // Modificato: aggiungo check > 0.5 per evitare trigger con USB
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
    
    // NUOVO: Attendi GPS fix dopo connessione
    #ifndef SKIP_GPS_WAIT
    waitForInitialGPSFix();
    #else
    SerialMon.println("GPS wait skipped (SKIP_GPS_WAIT defined)");
    #endif
    
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
    
    // Se non siamo connessi, esci
    if (!connectionOK) {
        return;
    }
    
    // MODIFICATO: Rimuovo la logica di attesa GPS basata su batteria
    // dato che sei alimentato via USB
    
    // Controllo per deep sleep dopo 3 invii
    if (sentSampleCount >= MAX_SAMPLES_BEFORE_SLEEP) {
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
            modemPowerOff();
            delay(5000);
            goToDeepSleep(TIME_TO_SLEEP_DAY);
        } else {
            SerialMon.println("USB powered (voltage < 0.5V) - skip deep sleep");
            sentSampleCount = 0;  // Reset counter
            
            // Lampeggio lungo per indicare reset counter
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
        
        // MODIFICATO: Debug GPS migliorato
        SerialMon.println("\n--- GPS DEBUG ---");
        
        // Controlla stato GPS
        modem.sendAT("+CGPS?");
        String gpsState = "";
        if (modem.waitResponse(2000L, gpsState) == 1) {
            SerialMon.println("GPS State: " + gpsState);
        }
        
        // Controlla status GPS
        modem.sendAT("+CGPSSTATUS?");
        String gpsStatus = "";
        if (modem.waitResponse(2000L, gpsStatus) == 1) {
            SerialMon.println("GPS Status: " + gpsStatus);
        }
        
        // Ottieni info GPS raw
        modem.sendAT("+CGPSINFO");
        String gpsInfo = "";
        if (modem.waitResponse(3000L, gpsInfo) == 1) {
            SerialMon.println("GPS Info: " + gpsInfo);
            
            // Analizza se ci sono dati validi
            if (gpsInfo.length() > 20 && gpsInfo.indexOf(",,,,") < 0) {
                SerialMon.println("GPS has satellite data but may not have fix yet");
            }
        }
        
        // Prova a ottenere i dati GPS
        float lat = 0, lon = 0, speed = 0, alt = 0, accuracy = 0;
        int vsat = 0, usat = 0;
        int year = 0, month = 0, day = 0, hour = 0, min = 0, sec = 0;

        bool gpsSuccess = modem.getGPS(&lat, &lon, &speed, &alt, &vsat, &usat, &accuracy, 
                                       &year, &month, &day, &hour, &min, &sec);

        if (gpsSuccess) {
            SerialMon.printf("GPS READ SUCCESS\n");
            SerialMon.printf("Coordinates: %.6f, %.6f\n", lat, lon);
            SerialMon.printf("Satellites: %d visible, %d used\n", vsat, usat);
            SerialMon.printf("Accuracy: %.1f meters\n", accuracy);
            SerialMon.printf("Speed: %.1f km/h, Altitude: %.1f m\n", speed, alt);
            
            if (lat != 0 || lon != 0) {
                char latStr[15], lonStr[15];
                dtostrf(lat, 10, 6, latStr);
                dtostrf(lon, 10, 6, lonStr);
                mqtt.publish(topicGPSlat, latStr);
                mqtt.publish(topicGPSlon, lonStr);
                SerialMon.printf("GPS PUBLISHED: %s, %s\n", latStr, lonStr);
                
                // Segnala che abbiamo ottenuto almeno un fix
                if (!gpsFixObtained) {
                    gpsFixObtained = true;
                    SerialMon.println("*** FIRST GPS FIX OBTAINED IN MAIN LOOP! ***");
                }
            } else {
                SerialMon.println("GPS coordinates are 0,0 - not publishing");
            }
        } else {
            SerialMon.println("GPS READ FAILED");
            
            // Prova metodo alternativo
            modem.sendAT("+CGPSINF=0");
            String gpsAlt = "";
            if (modem.waitResponse(3000L, gpsAlt) == 1) {
                SerialMon.println("GPS Alt method: " + gpsAlt);
            }
        }
        SerialMon.println("--- END GPS DEBUG ---\n");
        
        sentSampleCount++;
        SerialMon.printf("Samples sent: %d/%d\n", sentSampleCount, MAX_SAMPLES_BEFORE_SLEEP);
    }
}

// Deep sleep con calcolo tempo basato su ora del giorno
void goToDeepSleep(uint64_t time_in_seconds) {
    float voltage = readBatteryVoltage();
    
    if (voltage < 0.5) {  // Modificato da == 0 a < 0.5 per maggiore affidabilità
        SerialMon.println("USB powered - skip deep sleep");
        sentSampleCount = 0;
        return;
    }

    SerialMon.println("Preparing for deep sleep...");
    
    // Spegnimento modem
    disableGPS();
    modem.sendAT("AT+CPOWD=1");
    modem.poweroff();
    delay(1000);
    digitalWrite(PWR_PIN, LOW);
    
    // Determina tempo di sleep
    if (voltage <= 3.7) {
        time_in_seconds = TIME_TO_SLEEP_LOW_BATTERY;
        SerialMon.println("Low battery - 1 hour sleep");
    } else {
        // Tentativo di ottenere l'ora per sleep adattivo
        int hour;
        if (modem.getGPS(nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, 
                         nullptr, nullptr, nullptr, nullptr, &hour, nullptr, nullptr)) {
            int localHour = (hour + 1) % 24; // UTC+1 per l'Italia
            
            if (localHour >= 6 && localHour < 20) {
                time_in_seconds = TIME_TO_SLEEP_DAY;
                SerialMon.println("Daytime - 10 minutes sleep");
            } else {
                time_in_seconds = TIME_TO_SLEEP_NIGHT;
                SerialMon.println("Nighttime - 30 minutes sleep");
            }
        } else {
            // Default a sleep notturno se GPS non disponibile
            time_in_seconds = TIME_TO_SLEEP_NIGHT;
            SerialMon.println("No GPS time - defaulting to night sleep");
        }
    }
    
    SerialMon.print("Deep sleep for ");
    SerialMon.print(time_in_seconds);
    SerialMon.println(" seconds");
    
    esp_sleep_enable_timer_wakeup(time_in_seconds * uS_TO_S_FACTOR);
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
        SerialMon.println("========================\n");
    }
}

// Setup principale
void setup() {
    // Inizializzazione seriali
    SerialMon.begin(115200);
    delay(100);
    
    SerialAT.begin(UART_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);
    delay(100);
    
    SerialMon.println("\n\n=== RailTemp Temperature Monitor ===");
    SerialMon.println("Version: 2.2 - GPS Debug Enhanced");
    SerialMon.printf("Device ID: %s\n", DEVICE_ID);
    SerialMon.printf("APN: %s\n", APN);
    SerialMon.println("====================================\n");
    
    // Configurazione pin
    pinMode(LED_PIN, OUTPUT);
    pinMode(BATTERY_PIN, INPUT);
    
    // Test LED all'avvio
    for(int i = 0; i < 3; i++) {
        digitalWrite(LED_PIN, HIGH);
        delay(200);
        digitalWrite(LED_PIN, LOW);
        delay(200);
    }
    
    // Aggiorna topic MQTT
    updateMqttTopics();
    
    // Inizializza sensore temperatura
    sensors.begin();
    if (sensors.getDeviceCount() == 0) {
        SerialMon.println("WARNING: No temperature sensor found!");
    } else {
        SerialMon.println("Temperature sensor initialized");
        sensors.requestTemperatures();
        float testTemp = sensors.getTempCByIndex(0);
        SerialMon.printf("Test reading: %.1f°C\n", testTemp);
    }
    
    // Disabilita WiFi per risparmio energetico
    WiFi.mode(WIFI_OFF);
    WiFi.disconnect(true);
    
    // Reset contatore campioni
    sentSampleCount = 0;
    
    // Reset variabili GPS
    gpsFixObtained = false;
    gpsWaitStartTime = 0;
    waitingForGPS = false;
    gpsInitialWaitDone = false;
    
    // Inizializza modem
    modemPowerOn();
    delay(2000);  // Aumentato delay per stabilizzazione
    
    // Abilita GPS per localizzazione
    enableGPS();
    
    // Configurazione MQTT
    mqtt.setServer(broker, 1883);
    mqtt.setCallback(mqttCallback);
    
    SerialMon.println("Setup completed - Starting main loop\n");
}

// Loop principale
void loop() {
    ConnectTask();
    SendMqttDataTask();
    logConnectionStats();
    
    // Indicazione LED dello stato
    static unsigned long lastBlink = 0;
    
    // Lampeggio normale per indicare campioni inviati
    if (connectionOK && millis() - lastBlink > 10000) {
        lastBlink = millis();
        
        // Numero di lampeggi = numero di campioni inviati
        for(int i = 0; i < sentSampleCount; i++) {
            digitalWrite(LED_PIN, HIGH);
            delay(200);
            digitalWrite(LED_PIN, LOW);
            delay(200);
        }
    }
    
    delay(100);
}