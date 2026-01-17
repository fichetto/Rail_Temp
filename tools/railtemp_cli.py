#!/usr/bin/env python3
"""
RailTemp CLI - Gestione dispositivi RailTemp via MQTT

Comandi:
    status      - Richiede diagnostica dal dispositivo
    reboot      - Riavvia il dispositivo
    ota         - Avvia aggiornamento OTA
    monitor     - Ascolta tutti i messaggi in tempo reale
    led         - Toggle LED di test

Uso:
    python railtemp_cli.py status
    python railtemp_cli.py reboot
    python railtemp_cli.py ota v1.2.0
    python railtemp_cli.py ota --url https://... --checksum sha256:...
    python railtemp_cli.py monitor
    python railtemp_cli.py led
"""

import argparse
import json
import sys
import time
from datetime import datetime

try:
    import paho.mqtt.client as mqtt
except ImportError:
    print("Errore: libreria paho-mqtt non installata")
    print("Installa con: pip install paho-mqtt")
    sys.exit(1)

# Configurazione MQTT
MQTT_BROKER = "telemetry.tecnocons.com"
MQTT_PORT = 1883
MQTT_USER = "tecnocons"
MQTT_PASS = "nonserve"

# Device default
DEFAULT_DEVICE_ID = "Railtemp03"

# GitHub repo per OTA
GITHUB_REPO = "fichetto/Rail_Temp"


class RailTempCLI:
    def __init__(self, device_id: str, verbose: bool = False):
        self.device_id = device_id
        self.verbose = verbose
        # paho-mqtt v2 usa CallbackAPIVersion
        try:
            self.client = mqtt.Client(
                callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
                client_id=f"railtemp_cli_{int(time.time())}"
            )
        except (AttributeError, TypeError):
            # Fallback per paho-mqtt v1
            self.client = mqtt.Client(client_id=f"railtemp_cli_{int(time.time())}")

        self.client.username_pw_set(MQTT_USER, MQTT_PASS)
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message
        self.connected = False
        self.waiting_for_response = False
        self.response_received = False
        self.last_response = None

    def _on_connect(self, client, userdata, flags, rc, properties=None):
        # rc in v2 e' un ReasonCode object, in v1 e' un int
        rc_value = rc.value if hasattr(rc, 'value') else rc
        if rc_value == 0:
            self.connected = True
            if self.verbose:
                print(f"[MQTT] Connesso a {MQTT_BROKER}")
        else:
            print(f"[MQTT] Errore connessione: {rc}")

    def _on_message(self, client, userdata, msg):
        topic = msg.topic
        try:
            payload = msg.payload.decode('utf-8')
        except:
            payload = str(msg.payload)

        # Formatta timestamp
        timestamp = datetime.now().strftime("%H:%M:%S")

        # Prova a parsare JSON per output formattato
        try:
            data = json.loads(payload)
            payload_formatted = json.dumps(data, indent=2)
        except:
            payload_formatted = payload

        # Colori ANSI per terminale
        CYAN = "\033[96m"
        GREEN = "\033[92m"
        YELLOW = "\033[93m"
        RED = "\033[91m"
        RESET = "\033[0m"

        # Determina colore in base al topic
        if "diagnostics" in topic:
            color = GREEN
        elif "ota/status" in topic:
            color = YELLOW
        elif "cmd/" in topic:
            color = CYAN
        else:
            color = RESET

        print(f"{color}[{timestamp}] {topic}{RESET}")
        print(f"{payload_formatted}")
        print()

        # Se stavamo aspettando una risposta
        if self.waiting_for_response:
            self.response_received = True
            self.last_response = payload

    def connect(self) -> bool:
        """Connette al broker MQTT"""
        try:
            self.client.connect(MQTT_BROKER, MQTT_PORT, 60)
            self.client.loop_start()

            # Attendi connessione
            timeout = 10
            while not self.connected and timeout > 0:
                time.sleep(0.1)
                timeout -= 0.1

            return self.connected
        except Exception as e:
            print(f"Errore connessione: {e}")
            return False

    def disconnect(self):
        """Disconnette dal broker"""
        self.client.loop_stop()
        self.client.disconnect()

    def subscribe_all(self):
        """Subscribe a tutti i topic del dispositivo"""
        topics = [
            f"{self.device_id}/#"
        ]
        for topic in topics:
            self.client.subscribe(topic)
            if self.verbose:
                print(f"[MQTT] Subscribed: {topic}")

    def publish(self, topic: str, payload: str):
        """Pubblica un messaggio"""
        full_topic = f"{self.device_id}/{topic}"
        self.client.publish(full_topic, payload)
        if self.verbose:
            print(f"[MQTT] Pubblicato su {full_topic}: {payload}")

    def wait_for_response(self, timeout: float = 30) -> bool:
        """Attende una risposta dal dispositivo"""
        self.waiting_for_response = True
        self.response_received = False

        start = time.time()
        while not self.response_received and (time.time() - start) < timeout:
            time.sleep(0.1)

        self.waiting_for_response = False
        return self.response_received

    # ========== COMANDI ==========

    def cmd_status(self):
        """Richiede diagnostica dal dispositivo"""
        print(f"Richiesta diagnostica a {self.device_id}...")
        print("(In attesa di risposta...)\n")

        self.subscribe_all()
        time.sleep(0.5)  # Attendi subscription

        self.publish("cmd/diagnostics", "1")

        if self.wait_for_response(30):
            print("\nDiagnostica ricevuta!")
        else:
            print("\nTimeout: nessuna risposta dal dispositivo")
            print("Il dispositivo potrebbe essere in deep sleep o offline")

    def cmd_reboot(self):
        """Riavvia il dispositivo"""
        confirm = input(f"Sei sicuro di voler riavviare {self.device_id}? [y/N] ")
        if confirm.lower() != 'y':
            print("Operazione annullata")
            return

        print(f"Invio comando reboot a {self.device_id}...")

        self.subscribe_all()
        time.sleep(0.5)

        self.publish("cmd/reboot", "1")

        print("Comando inviato. Il dispositivo si riavvierà.")

    def cmd_led(self):
        """Toggle LED di test"""
        print(f"Toggle LED su {self.device_id}...")

        self.subscribe_all()
        time.sleep(0.5)

        self.publish("TestLed", "1")

        if self.wait_for_response(10):
            print("LED toggled!")
        else:
            print("Nessuna conferma ricevuta")

    def cmd_ota(self, version: str = None, url: str = None, checksum: str = None):
        """Avvia aggiornamento OTA"""

        # Se specificata versione, costruisci URL da GitHub
        if version and not url:
            if not version.startswith('v'):
                version = f"v{version}"

            url = f"https://github.com/{GITHUB_REPO}/releases/download/{version}/firmware.bin"

            print(f"Versione: {version}")
            print(f"URL: {url}")
            print()

            # Prova a scaricare version.json per checksum
            try:
                import urllib.request
                version_url = f"https://github.com/{GITHUB_REPO}/releases/download/{version}/version.json"
                print(f"Scarico metadati da {version_url}...")

                with urllib.request.urlopen(version_url, timeout=10) as response:
                    version_data = json.loads(response.read().decode())
                    checksum = version_data.get('checksum', '')
                    print(f"Checksum: {checksum}")
            except Exception as e:
                print(f"Impossibile scaricare metadati: {e}")
                if not checksum:
                    checksum = input("Inserisci checksum manualmente (sha256:...): ")

        if not url or not checksum:
            print("Errore: URL e checksum sono obbligatori")
            print("Uso: railtemp_cli.py ota v1.2.0")
            print("  oppure: railtemp_cli.py ota --url <url> --checksum <checksum>")
            return

        print()
        print("=" * 50)
        print("ATTENZIONE: Aggiornamento OTA")
        print("=" * 50)
        print(f"Device: {self.device_id}")
        print(f"URL: {url}")
        print(f"Checksum: {checksum}")
        print()

        confirm = input("Procedere con l'aggiornamento? [y/N] ")
        if confirm.lower() != 'y':
            print("Operazione annullata")
            return

        print()
        print("Avvio OTA...")

        self.subscribe_all()
        time.sleep(0.5)

        payload = json.dumps({
            "url": url,
            "checksum": checksum
        })

        self.publish("cmd/ota", payload)

        print("Comando OTA inviato. Monitoraggio progresso...\n")

        # Monitora progresso per 5 minuti
        start = time.time()
        while (time.time() - start) < 300:
            time.sleep(0.1)
            # I messaggi vengono stampati dal callback _on_message

    def cmd_monitor(self):
        """Monitora tutti i messaggi in tempo reale"""
        print(f"Monitoraggio {self.device_id} in tempo reale")
        print("Premi Ctrl+C per uscire\n")

        self.subscribe_all()

        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\nMonitoraggio terminato")


def main():
    parser = argparse.ArgumentParser(
        description="RailTemp CLI - Gestione dispositivi via MQTT",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Esempi:
  %(prog)s status                    Richiede diagnostica
  %(prog)s reboot                    Riavvia dispositivo
  %(prog)s ota v1.2.0                Aggiorna a versione specifica
  %(prog)s ota --url URL --checksum CHECKSUM
  %(prog)s monitor                   Monitora messaggi in tempo reale
  %(prog)s led                       Toggle LED di test
  %(prog)s -d Railtemp04 status      Usa device diverso
        """
    )

    parser.add_argument(
        'command',
        choices=['status', 'reboot', 'ota', 'monitor', 'led'],
        help='Comando da eseguire'
    )

    parser.add_argument(
        '-d', '--device',
        default=DEFAULT_DEVICE_ID,
        help=f'ID dispositivo (default: {DEFAULT_DEVICE_ID})'
    )

    parser.add_argument(
        '-v', '--verbose',
        action='store_true',
        help='Output verboso'
    )

    # Argomenti per OTA
    parser.add_argument(
        'version',
        nargs='?',
        help='Versione firmware per OTA (es. v1.2.0)'
    )

    parser.add_argument(
        '--url',
        help='URL diretto al firmware.bin'
    )

    parser.add_argument(
        '--checksum',
        help='Checksum SHA256 (formato: sha256:...)'
    )

    args = parser.parse_args()

    # Crea client
    cli = RailTempCLI(args.device, args.verbose)

    print(f"RailTemp CLI - Device: {args.device}")
    print(f"Broker: {MQTT_BROKER}:{MQTT_PORT}")
    print()

    # Connetti
    if not cli.connect():
        print("Impossibile connettersi al broker MQTT")
        sys.exit(1)

    try:
        # Esegui comando
        if args.command == 'status':
            cli.cmd_status()
        elif args.command == 'reboot':
            cli.cmd_reboot()
        elif args.command == 'led':
            cli.cmd_led()
        elif args.command == 'ota':
            cli.cmd_ota(args.version, args.url, args.checksum)
        elif args.command == 'monitor':
            cli.cmd_monitor()
    finally:
        cli.disconnect()


if __name__ == '__main__':
    main()
