/*
 * GPS DIAGNOSTIC TEST per LilyGO T-SIM7000G
 *
 * Questo programma testa in modo approfondito il modulo GPS
 * per diagnosticare problemi hardware o di configurazione.
 *
 * Compila con: pio run
 * Upload con: pio run -t upload
 */

#include <Arduino.h>

#define SerialMon Serial
#define SerialAT Serial1

#define UART_BAUD   115200
#define PIN_TX      27
#define PIN_RX      26
#define PWR_PIN     4
#define LED_PIN     12

void modemPowerOn();
void sendCommand(const char* cmd, int timeout = 2000);
String sendCommandGetResponse(const char* cmd, int timeout = 2000);

void modemPowerOn() {
    pinMode(PWR_PIN, OUTPUT);
    digitalWrite(PWR_PIN, HIGH);
    delay(300);
    digitalWrite(PWR_PIN, LOW);
    delay(5000);
}

void sendCommand(const char* cmd, int timeout) {
    SerialMon.print(">>> ");
    SerialMon.println(cmd);
    SerialAT.println(cmd);

    unsigned long start = millis();
    String response = "";
    while (millis() - start < timeout) {
        while (SerialAT.available()) {
            char c = SerialAT.read();
            response += c;
        }
        delay(10);
    }

    response.trim();
    if (response.length() > 0) {
        SerialMon.print("<<< ");
        SerialMon.println(response);
    } else {
        SerialMon.println("<<< [NO RESPONSE]");
    }
    SerialMon.println();
}

String sendCommandGetResponse(const char* cmd, int timeout) {
    SerialAT.println(cmd);

    unsigned long start = millis();
    String response = "";
    while (millis() - start < timeout) {
        while (SerialAT.available()) {
            char c = SerialAT.read();
            response += c;
        }
        delay(10);
    }
    response.trim();
    return response;
}

void setup() {
    SerialMon.begin(115200);
    delay(1000);

    pinMode(LED_PIN, OUTPUT);

    SerialMon.println("\n");
    SerialMon.println("==============================================");
    SerialMon.println("   GPS DIAGNOSTIC TEST - LilyGO T-SIM7000G");
    SerialMon.println("==============================================\n");

    // Accendi modem
    SerialMon.println("[1] Accensione modem...");
    modemPowerOn();

    SerialAT.begin(UART_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);
    delay(3000);

    // Test comunicazione base
    SerialMon.println("\n[2] Test comunicazione modem...");
    sendCommand("AT");
    sendCommand("AT+CGMM");  // Model
    sendCommand("AT+CGMR");  // Firmware version

    // Test GPIO antenna GPS
    SerialMon.println("\n[3] Test GPIO antenna GPS...");
    sendCommand("AT+SGPIO=0,4,1,0");  // OFF
    delay(500);
    sendCommand("AT+SGPIO=0,4,1,1");  // ON
    delay(1000);

    // Test GNSS power
    SerialMon.println("\n[4] Test GNSS Power Control...");
    sendCommand("AT+CGNSPWR=0");  // Spegni
    delay(1000);
    sendCommand("AT+CGNSPWR?");   // Stato
    sendCommand("AT+CGNSPWR=1");  // Accendi
    delay(2000);
    sendCommand("AT+CGNSPWR?");   // Verifica

    // Test comandi GNSS disponibili
    SerialMon.println("\n[5] Test comandi GNSS disponibili...");
    sendCommand("AT+CGNSINF");     // Info GPS
    sendCommand("AT+CGNSURC=1");   // Abilita URC
    sendCommand("AT+CGNSMOD?");    // Modalità GNSS

    // Test cold/hot start
    SerialMon.println("\n[6] Test Cold Start...");
    sendCommand("AT+CGNSCOLD", 5000);
    delay(2000);

    // Lettura continua per 60 secondi
    SerialMon.println("\n[7] Lettura GPS per 60 secondi...");
    SerialMon.println("    Osserva se il campo 'fix status' passa da 0 a 1\n");

    unsigned long startTime = millis();
    int readCount = 0;

    while (millis() - startTime < 60000) {
        readCount++;
        SerialMon.printf("--- Lettura #%d (%.0f sec) ---\n", readCount, (millis() - startTime) / 1000.0);

        String resp = sendCommandGetResponse("AT+CGNSINF", 3000);
        SerialMon.println("CGNSINF: " + resp);

        // Analizza risposta
        // Formato: +CGNSINF: run,fix,utc,lat,lon,alt,speed,course,fixmode,reserved,HDOP,PDOP,VDOP,reserved,gnss_sats,glonass_sats,reserved,C/N0_max,HPA,VPA
        if (resp.indexOf("+CGNSINF: 1,1") >= 0) {
            SerialMon.println("\n*** GPS FIX OTTENUTO! ***");
            SerialMon.println("Il GPS funziona correttamente.\n");

            // Estrai e mostra coordinate
            int idx = resp.indexOf("+CGNSINF: 1,1,");
            if (idx >= 0) {
                String data = resp.substring(idx + 14);
                SerialMon.println("Dati: " + data);
            }
        } else if (resp.indexOf("+CGNSINF: 1,0") >= 0) {
            SerialMon.println("GPS ON, cercando satelliti...");
        } else if (resp.indexOf("+CGNSINF: 0") >= 0) {
            SerialMon.println("!!! GPS SPENTO - problema hardware/config !!!");
        }

        // Lampeggia LED
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));

        delay(5000);
        SerialMon.println();
    }

    // Riepilogo
    SerialMon.println("\n==============================================");
    SerialMon.println("              RIEPILOGO DIAGNOSTICA");
    SerialMon.println("==============================================");
    SerialMon.println("Se vedi sempre '+CGNSINF: 1,0,...' significa:");
    SerialMon.println("  - GPS acceso ma non trova satelliti");
    SerialMon.println("  - Prova a posizionare il dispositivo all'aperto");
    SerialMon.println("  - Verifica antenna GPS connessa");
    SerialMon.println();
    SerialMon.println("Se vedi sempre '+CGNSINF: 0,...' significa:");
    SerialMon.println("  - GPS non si accende");
    SerialMon.println("  - Possibile problema hardware");
    SerialMon.println("  - Verifica alimentazione e GPIO");
    SerialMon.println();
    SerialMon.println("Se vedi 'ERROR' sui comandi CGNS:");
    SerialMon.println("  - Firmware modem potrebbe essere diverso");
    SerialMon.println("  - Prova comandi CGPS invece di CGNS");
    SerialMon.println("==============================================\n");

    // Continua lettura infinita
    SerialMon.println("\n[8] Lettura continua (premi reset per terminare)...\n");
}

void loop() {
    static unsigned long lastRead = 0;

    if (millis() - lastRead > 10000) {
        lastRead = millis();

        String resp = sendCommandGetResponse("AT+CGNSINF", 3000);

        // Parsing semplificato
        if (resp.indexOf("+CGNSINF: 1,1") >= 0) {
            SerialMon.print("[FIX] ");
        } else if (resp.indexOf("+CGNSINF: 1,0") >= 0) {
            SerialMon.print("[SEARCHING] ");
        } else if (resp.indexOf("+CGNSINF: 0") >= 0) {
            SerialMon.print("[OFF] ");
        } else {
            SerialMon.print("[???] ");
        }
        SerialMon.println(resp);

        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    }

    // Passa dati dalla seriale del modem al monitor
    while (SerialAT.available()) {
        SerialMon.write(SerialAT.read());
    }
}
