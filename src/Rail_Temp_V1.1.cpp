//Config Railtemp08
#include <Arduino.h>
#include <TimeUtility.h>
#include <EEPROM.h>
#include <HardwareSerial.h>
#include <WiFi.h>
//#include <WebServer.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS 18  // Definisci il pin per il DS18B20

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

#define EnableConnection
//#define UseWifi
#define SerialMon Serial                    // Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialAT Serial1
#define OPENLOG_SERIAL Serial2
#define OPENLOG_RX 32  // Pin RX per OpenLog, collegare al TX di OpenLog
#define OPENLOG_TX 33  // Pin TX per OpenLog, collegare al RX di OpenLog

#define TINY_GSM_MODEM_SIM7000
#define TINY_GSM_RX_BUFFER 1024 // Set RX buffer to 1Kb
#define GSM_PIN ""

#define uS_TO_S_FACTOR      1000000ULL  // Conversion factor for micro seconds to seconds
#define TIME_TO_SLEEP       60          // Time ESP32 will go to sleep (in seconds)
#define TIME_TO_SLEEP_LONG  600        // Time ESP32 will go to sleep when battery is low (in seconds)

#define UART_BAUD           115200       //uart modem
#define PIN_DTR             25
#define PIN_TX              27
#define PIN_RX              26
#define PWR_PIN             4

#define LED_PIN             12
#define SEND_INTERVAL_MS    10000        //Intervallo di tempo tra un invio dati e il successivo

#define BATTERY_PIN 35  // Definisci il pin ADC per la lettura della tensione della batteria

// Definizioni per la gestione della batteria
#define BATTERY_CRITICAL 3.3     // Voltaggio critico per spegnimento
#define BATTERY_LOW 3.7          // Voltaggio per deep sleep lungo
#define BATTERY_NORMAL 3.9       // Voltaggio per operazioni normali

// Definizioni per i tentativi di riconnessione
#define MAX_RECONNECT_ATTEMPTS 5
#define MAX_NETWORK_RETRIES 10   // Ridotto da 30
#define INITIAL_RETRY_DELAY 3000
#define MAX_RETRY_DELAY 60000UL    // Massimo delay tra tentativi (unsigned long)


#include <TinyGsmClient.h>
#include <SPI.h>
//#include <SD.h>
#include <Ticker.h>
#include <PubSubClient.h>
#include <ArduinoHttpClient.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

// SD card pin assignments
const int pinCS = 13;   // Chip Select for SD card
const int pinMOSI = 15; // Master Out Slave In
const int pinMISO = 2;  // Master In Slave Out
const int pinSCK = 14;  // Serial Clock

// Your GPRS credentials, if any
const char apn[] = "shared.tids.tim.it";     //SET TO YOUR APN
const char gprsUser[] = "";
const char gprsPass[] = "";

const char server[] = "track.tecnocons.com";
const int port = 5055;
String myid = "20240729Railtemp08";

// Local WiFi Network credentials
const char* ssid = "ESP32-AP";
const char* password = "123456789";

// MQTT details
const char* broker = "telemetry.tecnocons.com";

const char* topicTestLed = "Railtemp08/TestLed";
const char* topicStop = "Railtemp08/Stop";
const char* topicStatus = "Railtemp08/Status";
const char* topicInit = "GsmClientTest/init";
const char* topicBatteryStatus = "Railtemp08/BatteryVoltage";
const char* topicGPSlat = "Railtemp08/lat";
const char* topicGPSlon = "Railtemp08/lon";
const char* topicGPSspeed = "Railtemp08/speed";
const char* topicAmpMotore = "Railtemp08/AmpMotore";
const char* topicWatt = "Railtemp08/Watt";
const char* topicTemperature = "Railtemp08/Temperature";
const char* topicSignalQuality = "Railtemp08/SignalQuality"; //topic temporaneo per leggere stato segnale

// DeepSleep intervals
uint64_t daySleepInterval = 600;  // 10 minuti in secondi
uint64_t nightSleepInterval = 3600;  // 60 minuti in secondi


TimeTrigger timerAccel;
Timer timerButton;
Timer timerSpecialBttFunction;
Timer timerPotCalibration;
TaskHandle_t Task3;
HardwareSerial SerialVesc(2);

#ifdef EnableConnection
TinyGsm modem(SerialAT);
#ifdef UseWifi
WiFiClient client;
#else
TinyGsmClient client(modem);
#endif // UseWifi
PubSubClient mqtt(client);
HttpClient http(client, server, port);
#endif

int ledStatus = LOW;
uint32_t lastReconnectAttempt = 0;
bool connectionOK = false;
bool MQTTcommandStop = false;

float speedFromGPS = 0;
int SignalQuality = -1;
bool Connected = false;
unsigned int mqttSent = 0;

float TensioneBatt;
float batteryVoltage;

// Variabili per la gestione delle riconnessioni
static int reconnectAttempts = 0;
static unsigned long retryDelay = INITIAL_RETRY_DELAY;

// Additional sensor pins
const int analogPin1 = 34; // Sensore di pressione 1
const int analogPin2 = 35; // Sensore di pressione 2
const int analogPin3 = 36; // Sensore di pressione 3

// Accelerometer
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
bool ADXL_Present = false;
// Variables for pulse count and speed calculation
volatile unsigned long pulseCount = 0;
float speed = 0.0;
char charLat[10];
char charLon[10];
char charSpeed[10];

// Function declarations
void enableGPS();
void modemPowerOn();
void modemPowerOff();
void modemRestart();
void ConnectTask();
void SendMqttDataTask();
void IRAM_ATTR onPulse();
float calculateSpeed();
void mqttCallback(char* topic, byte* payload, unsigned int len);
void disableGPS();
void SendDataToTraccar(float lat, float lon);
void goToDeepSleep(uint64_t time_in_seconds);

float readBatteryVoltage();

void IRAM_ATTR onPulse() {
  pulseCount++;
}

float readBatteryVoltage() {
    int analogValue = analogRead(BATTERY_PIN);
    float voltage = analogValue * (3.3 / 4095.0);  // 3.3V è la tensione di riferimento e 4095 è il valore massimo dell'ADC a 12 bit
    voltage *= 2.16; // Se c'è un partitore di tensione 1:2, moltiplica per 2
    return voltage;
}

float calculateSpeed() {
  static unsigned long lastTime = 0;
  static unsigned long lastPulseCount = 0;
  
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - lastTime;
  unsigned long pulseDiff = pulseCount - lastPulseCount;

  float frequency = pulseDiff / (elapsedTime / 1000.0); // Frequenza in Hz
  float speed = frequency * 0.1; // Calibrazione con il GPS

  lastTime = currentTime;
  lastPulseCount = pulseCount;

  return speed;
}

void enableGPS() {
  SerialMon.println("Start positioning. Make sure to locate outdoors.");
  modem.sendAT("+SGPIO=0,4,1,1");
  if (modem.waitResponse(10000L) != 1) {
    SerialMon.println("SGPIO=0,4,1,1 false");
  }
  modem.enableGPS();
}

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

// Modifiche alla funzione SendMqttDataTask per gestione MQTT
boolean mqttConnect() {
    static int mqttRetries = 0;
    static unsigned long mqttRetryDelay = INITIAL_RETRY_DELAY;
    
    if (mqttRetries >= MAX_RECONNECT_ATTEMPTS) {
        SerialMon.println("Max MQTT reconnection attempts reached");
        goToDeepSleep(TIME_TO_SLEEP_LONG);
        return false;
    }

    SerialMon.printf("Connecting to MQTT (attempt %d/%d)...\n", 
                     mqttRetries + 1, MAX_RECONNECT_ATTEMPTS);
    
    boolean status = mqtt.connect("Railtemp08", "tecnocons", "nonserve");
    if (!status) {
        mqttRetries++;
        mqttRetryDelay = min(mqttRetryDelay * 2, MAX_RETRY_DELAY);
        return false;
    }
    
    // Reset counters on successful connection
    mqttRetries = 0;
    mqttRetryDelay = INITIAL_RETRY_DELAY;
    return true;
}


void SendDataToTraccar(float lat, float lon) {
  String latStr = String(lat, 6);
  String lonStr = String(lon, 6);

  int err = http.post("/?id=" + myid + "&lat=" + latStr + "&lon=" + lonStr);
  if (err != 0) {
    delay(10000);
    return;
  }

  int status = http.responseStatusCode();
  if (!status) {
    delay(10000);
    return;
  }

  String body = http.responseBody();
  http.stop();
}

int sentSampleCount = 0; // Variabile globale per contare i campioni inviati


void SendMqttDataTask() {
  batteryVoltage = readBatteryVoltage();
 
  // Modifica la condizione per il deep sleep
  if (sentSampleCount >= 3 && batteryVoltage > 0) { // Verifica che non siamo alimentati via USB
      SerialMon.println("Going in deep sleep mode");
      modemPowerOff();
      delay(5000);
      goToDeepSleep(TIME_TO_SLEEP_LONG);
  }


  if (connectionOK == true) {
    static unsigned long memMillisGPS = 0;
    int year, month, day, hour, minute, second;
    float lat, lon, speed, alt, accuracy;
    int vsat, usat;
    static String Coord;
    static char charLat[10];
    static char charLon[10];
    static char charSpeed[10];
    static char charCoord[20];
    static bool GPSdataAvailable;
    if (millis() >= memMillisGPS + SEND_INTERVAL_MS)
    {
      memMillisGPS = millis();

      if (modem.getGPS(&lat, &lon, &speed, &alt, &vsat, &usat, &accuracy, &year, &month, &day, &hour, &minute, &second))
      {
        //Serial.printf("lat:%f lon:%f\n", lat, lon);
        String Stringlat = String(lat, 6);
        String Stringlon = String(lon, 6);
        String StringSpeed = String(speed, 6);
        Stringlat.toCharArray(charLat, 10);
        Stringlon.toCharArray(charLon, 10);
        StringSpeed.toCharArray(charSpeed, 10);
        Coord = Stringlat + "," + Stringlon;
        Coord.toCharArray(charCoord, 20);
        speedFromGPS = speed;
        SerialMon.println("GPS data available");
        GPSdataAvailable = true;
      }
      else {
        Serial.print("no GPS data...");
        Serial.println(millis());
        GPSdataAvailable = false;
        enableGPS();
      }
    }

if (!mqtt.connected()) {
    uint32_t t = millis();
    if (t - lastReconnectAttempt > 10000L) {
        SerialMon.println("=== MQTT NOT CONNECTED ===");
        lastReconnectAttempt = t;
        
        // Verifica prima lo stato della connessione GPRS
        if (!modem.isGprsConnected()) {
            SerialMon.println("GPRS connection lost - Resetting connection...");
            connectionOK = false;  // Forza riconnessione in ConnectTask
            return;
        }
        
        if (mqttConnect()) {
            lastReconnectAttempt = 0;
            SerialMon.println("MQTT reconnected successfully");
        } else {
            SerialMon.println("MQTT reconnection failed");
        }
    }
    delay(100);
    return;
}
    else {  //se mqtt connesso
      mqtt.loop();
      //Send voltage over MQTT
      static unsigned long memMillis = 0;
      if (millis() >= (memMillis + SEND_INTERVAL_MS)) {
      memMillis = millis();

      // Pubblica il livello del segnale su MQTT
      int signalQuality = modem.getSignalQuality();
      char charSignalQuality[10];
      String(signalQuality).toCharArray(charSignalQuality, 10);
      mqtt.publish(topicSignalQuality, charSignalQuality);
      
      // Stampa la qualità del segnale sul terminale
      SerialMon.print("Signal Quality: ");
      SerialMon.println(signalQuality);

      String Tensione = "";
      String AmpereMotore = "";

      // Leggi la temperatura dal sensore DS18B20
      sensors.requestTemperatures(); // Invia la richiesta di temperatura
      float temperatureC = sensors.getTempCByIndex(0); // Ottieni la temperatura in Celsius
      String StringTemperature = String(temperatureC, 1); // Converti la temperatura in stringa

      SerialMon.print("Sending Voltage, Amp and Temperature over MQTT: ");
      SerialMon.print(Tensione);
      SerialMon.print("   ");
      SerialMon.print(AmpereMotore);
      SerialMon.print("   ");
      SerialMon.println(StringTemperature);

      float Watt = TensioneBatt * 0;
      String StringWatt = String(Watt, 1);
      SerialMon.print("Sending Voltage, Amp and Watt over MQTT: ");
      SerialMon.print(Tensione);
      SerialMon.print("   ");
      SerialMon.print(AmpereMotore);
      SerialMon.print("   ");
      SerialMon.println(StringWatt);

      char charTensione[10];
      char charAmpereMot[10];
      char charWatt[10];
      char charTemperature[10];
      
      String StringBatteryVoltage = String(batteryVoltage, 2); // Converti la tensione in stringa
      SerialMon.print("Sending Battery Voltage over MQTT: ");
      SerialMon.println(StringBatteryVoltage);
      char charBatteryVoltage[10];
      StringBatteryVoltage.toCharArray(charBatteryVoltage, 10);
      mqtt.publish(topicBatteryStatus, charBatteryVoltage); // Invia la tensione della batteria

      StringTemperature.toCharArray(charTemperature, 10);
      mqtt.publish(topicTemperature, charTemperature);  // Invia la temperatura al topic specifico

      if (GPSdataAvailable) {
        SerialMon.print("Sending GPS data over MQTT: ");
        SerialMon.println(Coord);
        mqtt.publish(topicGPSlat, charLat);
        mqtt.publish(topicGPSlon, charLon);
        mqtt.publish(topicGPSspeed, charSpeed);
      }
      sentSampleCount++;  // Incrementa il contatore dei campioni inviati
      mqttSent++;
    }
    }

    static unsigned long memMillisTraccar = 0;
    if (millis() >= memMillisTraccar + 60000)
    {
      memMillisTraccar = millis();
      SendDataToTraccar(lat, lon);
      SerialMon.println("Sending GPS data to Traccar");
    }

    if (batteryVoltage > 3.5) { // Condizione di risveglio se sopra 35%
      modemPowerOn();
      return;
    }
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int len) {
  SerialMon.print("Message arrived [");
  SerialMon.print(topic);
  SerialMon.print("]: ");
  SerialMon.write(payload, len);
  SerialMon.println();

  if (String(topic) == topicTestLed) {
    ledStatus = !ledStatus;
    digitalWrite(LED_PIN, ledStatus);
    mqtt.publish(topicStatus, ledStatus ? "1" : "0");
  }

  if (String(topic) == topicStop) {
    if (strncmp((char*)payload, "STOP", len) == 0) {
      MQTTcommandStop = true;
    }
    if (strncmp((char*)payload, "START", len) == 0) {
      MQTTcommandStop = false;
    }
    mqtt.publish(topicStatus, MQTTcommandStop ? "Ebike Stopped!" : "Ebike Operational!");
  }
}

void disableGPS() {
  modem.sendAT("+SGPIO=0,4,1,0");
  modem.disableGPS();
}

void ConnectTask() {
    if (!connectionOK || !modem.isGprsConnected()) {
        // Check batteria prima di tentare riconnessione
        float batteryVoltage = readBatteryVoltage();
        if (batteryVoltage <= BATTERY_CRITICAL) {
            SerialMon.println("Battery critical - powering off");
            modemPowerOff();
            goToDeepSleep(3600); // 1 ora di sleep
            return;
        }

        // Limita i tentativi di riconnessione
        if (reconnectAttempts >= MAX_RECONNECT_ATTEMPTS) {
            SerialMon.println("Max reconnection attempts reached - going to sleep");
            goToDeepSleep(nightSleepInterval);
            return;
        }

        // Verifica modem
        if (!modem.testAT()) {
            SerialMon.println("Modem not responding - Restarting...");
            modemRestart();
            delay(2000);
            reconnectAttempts++;
            return;
        }

        // Tentativo di connessione alla rete con backoff esponenziale
        bool isConnected = false;
        int networkTries = MAX_NETWORK_RETRIES;
        
        while (networkTries-- && !isConnected) {
            int16_t signal = modem.getSignalQuality();
            SerialMon.printf("Signal quality: %d\n", signal);
            
            isConnected = modem.isNetworkConnected();
            if (isConnected) {
                SerialMon.println("Network connected!");
                reconnectAttempts = 0;  // Reset counter on success
                retryDelay = INITIAL_RETRY_DELAY;
                break;
            }
            
            delay(retryDelay);
            retryDelay = (retryDelay * 2 <= MAX_RETRY_DELAY) ? retryDelay * 2 : MAX_RETRY_DELAY;
            digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        }

        if (!isConnected) {
            reconnectAttempts++;
            SerialMon.printf("Network connection failed, attempt %d/%d\n", 
                           reconnectAttempts, MAX_RECONNECT_ATTEMPTS);
            return;
        }

        // GPRS Connection
        if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
            SerialMon.println("GPRS connection failed");
            reconnectAttempts++;
            delay(retryDelay);
            return;
        }
        
        connectionOK = true;
    }

    // Update connection status
    SignalQuality = modem.getSignalQuality();
    Connected = modem.isNetworkConnected() && modem.isGprsConnected();
    
    if (!Connected) {
        connectionOK = false;
    }
}

// Definizioni per gli orari stagionali (orari approssimativi medi per l'Italia)
struct SeasonTime {
    int startMonth;
    int endMonth;
    int sunriseHour;
    int sunsetHour;
};

const SeasonTime seasons[] = {
    {1,  2,  7, 17},  // Inverno (Gennaio-Febbraio)
    {3,  4,  6, 19},  // Primavera precoce (Marzo-Aprile)
    {5,  8,  5, 21},  // Estate (Maggio-Agosto)
    {9,  10, 6, 19},  // Autunno precoce (Settembre-Ottobre)
    {11, 12, 7, 17}   // Inverno (Novembre-Dicembre)
};

bool isDST(int day, int month) {
    // Ora legale in Europa: ultima domenica di marzo - ultima domenica di ottobre
    if (month < 3 || month > 10) return false;
    if (month > 3 && month < 10) return true;
    
    // Approssimazione semplificata: consideriamo dopo il 25 del mese
    return (month == 3 && day >= 25) || (month == 10 && day < 25);
}

void goToDeepSleep(uint64_t time_in_seconds) {
    float batteryVoltage = readBatteryVoltage();
    
    // Se batteryVoltage è 0, significa che siamo alimentati via USB
    if (batteryVoltage == 0) {
        SerialMon.println("USB Power detected - Skip deep sleep");
        sentSampleCount = 0; // Azzera il contatore quando siamo su USB
        return; // Esce dalla funzione senza entrare in deep sleep
    }

    modem.sendAT("+SGPIO=0,4,1,0");
    modem.sendAT("AT+CPOWD=1");
    modem.poweroff();
    delay(1000); 
    pinMode(PWR_PIN, OUTPUT);
    digitalWrite(PWR_PIN, LOW);
    
    // Se la batteria è sotto 3.6V, imposta sleep di 1 ora indipendentemente dall'ora del giorno
    if (batteryVoltage <= 3.7) {
        time_in_seconds = 3600; // 1 ora in secondi
        SerialMon.println("Low battery - Setting 1 hour sleep interval");
    } else {
        // Ottieni l'ora corrente dal GPS o dall'orologio interno
        int year, month, day, hour, minute, second;
        modem.getGPS(nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, 
                    &year, &month, &day, &hour, &minute, &second);
        
        // Trova la stagione corrente
        const SeasonTime* currentSeason = &seasons[0];
        for (int i = 0; i < 5; i++) {
            if (month >= seasons[i].startMonth && month <= seasons[i].endMonth) {
                currentSeason = &seasons[i];
                break;
            }
        }
        
        // Applica correzione per ora legale
        int hourOffset = isDST(day, month) ? 2 : 1; // GMT+1 (ora solare) o GMT+2 (ora legale)
        int localHour = (hour + hourOffset) % 24;
        
        // Imposta l'intervallo di deep sleep in base all'ora locale e alla stagione
        if (localHour >= currentSeason->sunriseHour && 
            localHour < currentSeason->sunsetHour) {
            time_in_seconds = daySleepInterval;    // 10 minuti
            SerialMon.println("Daylight period - Setting 10 minutes sleep interval");
        } else {
            time_in_seconds = nightSleepInterval;  // 30 minuti
            SerialMon.println("Night period - Setting 30 minutes sleep interval");
        }
        
        SerialMon.printf("Current time: %02d:%02d (GMT+%d), Month: %d\n", 
                        localHour, minute, hourOffset, month);
        SerialMon.printf("Season limits - Sunrise: %02d:00, Sunset: %02d:00\n", 
                        currentSeason->sunriseHour, currentSeason->sunsetHour);
    }
    
    SerialMon.print("Going to deep sleep for ");
    SerialMon.print(time_in_seconds);
    SerialMon.println(" seconds");
    
    esp_sleep_enable_timer_wakeup(time_in_seconds * uS_TO_S_FACTOR);
    esp_deep_sleep_start();
}

void setup() {
  SerialMon.begin(115200);
  OPENLOG_SERIAL.begin(9600, SERIAL_8N1, OPENLOG_RX, OPENLOG_TX);
  delay(1000);
  
  sensors.begin();  // Inizializza il sensore DS18B20

  WiFi.mode(WIFI_OFF);
  WiFi.disconnect(true);

  pinMode(BATTERY_PIN, INPUT); // Pin per la lettura della batteria

  // Resetta il contatore dei campioni al risveglio
  sentSampleCount = 0;

  // Initialize accelerometer
  ADXL_Present = true;
  if (!accel.begin()) {
    SerialMon.println("No ADXL345 detected");
    ADXL_Present = false;
  }
  if (ADXL_Present) accel.setRange(ADXL345_RANGE_16_G);

  // Setup interrupt for speed sensor
  attachInterrupt(digitalPinToInterrupt(26), onPulse, RISING);

  // Initialize modem and MQTT
  modemPowerOn();
  SerialAT.begin(UART_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);
  enableGPS();
  mqtt.setServer(broker, 1883);
  mqtt.setCallback(mqttCallback);

  // Task for testing
  xTaskCreatePinnedToCore(
    [] (void*) {
      for (;;) {
        vTaskDelay(10000 / portTICK_PERIOD_MS);
      }
    },
    "Task3",
    1000,
    NULL,
    0,
    &Task3,
    0
  );
}

void loop() {
  ConnectTask();
  SendMqttDataTask();
#define SAMPLE_INTERVAL_MS 30000

  if (ADXL_Present) {

  
    static unsigned long lastSampleTime = 0;
    static unsigned long lastAverageTime = 0;
    float sumX = 0.0;
    float sumY = 0.0;
    float sumZ = 0.0;
    unsigned int sampleCount = 0;

    // Reading accelerometer data at high frequency
    if (millis() - lastSampleTime >= SAMPLE_INTERVAL_MS) {
      lastSampleTime = millis();

      if (ADXL_Present) {
        sensors_event_t event;
        accel.getEvent(&event);
        float accelX = event.acceleration.x;
        float accelY = event.acceleration.y;
        float accelZ = event.acceleration.z;

        sumX += accelX;
        sumY += accelY;
        sumZ += accelZ;

        sampleCount++;
      }
    }

    #define AVERAGE_INTERVAL_MS 1000
    
    // Calculate average values every second
    if (millis() - lastAverageTime >= AVERAGE_INTERVAL_MS) {
      lastAverageTime = millis();
    
      float avgX = sumX / sampleCount;
      float avgY = sumY / sampleCount;
      float avgZ = sumZ / sampleCount;
    
      sumX = 0;
      sumY = 0;
      sumZ = 0;
      sampleCount = 0;
    
      float minX = 1000;
      float minY = 1000;
      float minZ = 1000;
      float maxX = -1000;
      float maxY = -1000;
      float maxZ = -1000;
    
      static unsigned long lastTime = 0;
      if (millis() - lastTime >= 1000) {
        lastTime = millis();
    
        float speed = calculateSpeed();
        int pressure1 = analogRead(analogPin1);
        int pressure2 = analogRead(analogPin2);
        int pressure3 = analogRead(analogPin3);
    
        String dataString = String(millis()) + ";" + String(speed) + ";" + String(pressure1) + ";" + String(pressure2) + ";" + String(pressure3) + ";" + String(avgX) + ";" + String(avgY) + ";" + String(avgZ) + ";" + String(minX) + ";" + String(minY) + ";" + String(minZ) + ";" + String(maxX) + ";" + String(maxY) + ";" + String(maxZ);
        OPENLOG_SERIAL.println(dataString);
        SerialMon.println(dataString);
    
        minX = 1000;
        minY = 1000;
        minZ = 1000;
        maxX = -1000;
        maxY = -1000;
        maxZ = -1000;
      }
    }
    }
  
}
