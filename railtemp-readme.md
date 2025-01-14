# Rail Temperature Monitoring System

This project implements an ESP32-based IoT system for monitoring rail temperature and related environmental conditions. The system is designed for railway infrastructure monitoring, combining temperature sensing, GPS tracking, and cellular connectivity for real-time data transmission.

## Features

- **Temperature Monitoring**: Uses DS18B20 temperature sensor for precise rail temperature measurements
- **GPS Tracking**: Built-in GPS functionality for location tracking
- **GSM/GPRS Connectivity**: Uses SIM7000 modem for cellular data transmission
- **MQTT Communication**: Real-time data publishing to a MQTT broker
- **Power Management**: Intelligent sleep cycles based on:
  - Time of day (different intervals for day/night)
  - Season (adjusts for sunrise/sunset times)
  - Battery level
- **Data Collection**:
  - Rail temperature
  - GPS coordinates
  - Battery voltage
  - GSM signal quality
  - Accelerometer data (ADXL345)
  - Pressure sensor readings

## Hardware Requirements

- ESP32 microcontroller
- SIM7000 GSM/GPRS modem
- DS18B20 temperature sensor
- ADXL345 accelerometer
- Pressure sensors (3x analog inputs)
- Battery with voltage monitoring
- OpenLog serial data logger

## Pin Configuration

- DS18B20 Temperature Sensor: Pin 18
- GSM Modem:
  - RX: Pin 26
  - TX: Pin 27
  - Power: Pin 4
- OpenLog Logger:
  - RX: Pin 32
  - TX: Pin 33
- Battery Monitoring: Pin 35
- Pressure Sensors:
  - Sensor 1: Pin 34
  - Sensor 2: Pin 35
  - Sensor 3: Pin 36

## Communication Protocols

### MQTT Topics

The system publishes data to the following MQTT topics:
- `Railtemp08/TestLed`: LED control
- `Railtemp08/Stop`: Remote stop/start control
- `Railtemp08/Status`: System status
- `Railtemp08/BatteryVoltage`: Battery level
- `Railtemp08/Temperature`: Rail temperature
- `Railtemp08/lat`: GPS latitude
- `Railtemp08/lon`: GPS longitude
- `Railtemp08/speed`: GPS speed
- `Railtemp08/SignalQuality`: GSM signal strength

### GPS Tracking

- Integrates with Traccar tracking server
- Sends position updates every 60 seconds
- Includes device ID, latitude, and longitude

## Power Management

### Sleep Cycles
The system implements adaptive sleep cycles based on:
1. **Time of Day**:
   - Day: 10-minute intervals
   - Night: 60-minute intervals

2. **Battery Voltage Thresholds**:
   - Critical: < 3.3V (emergency shutdown)
   - Low: < 3.7V (extended sleep periods)
   - Normal: > 3.9V (regular operation)

3. **Seasonal Adjustments**:
   - Automatically adjusts sleep cycles based on sunrise/sunset times
   - Accounts for Daylight Saving Time

## Data Logging

The system logs comprehensive data to the OpenLog device including:
- Timestamp
- Speed calculations
- Pressure sensor readings
- Accelerometer data (X, Y, Z axes)
- Min/max acceleration values

## Error Handling

- Implements exponential backoff for connection retries
- Maximum reconnection attempts before forced sleep
- Battery voltage monitoring with protective shutdowns
- Signal quality monitoring and reporting

## Network Configuration

### GSM/GPRS Settings
- APN: "shared.tids.tim.it"
- MQTT Broker: telemetry.tecnocons.com
- Traccar Server: track.tecnocons.com:5055

## Usage

1. Configure the device ID and network settings
2. Install required libraries
3. Upload the code to ESP32
4. Power on the device
5. Monitor data through MQTT broker or Traccar server

## Dependencies

- TinyGSM
- PubSubClient
- ArduinoHttpClient
- OneWire
- DallasTemperature
- Adafruit_ADXL345
- Wire
- SPI
- EEPROM

## Installation

1. Install all required libraries through Arduino IDE
2. Configure your network settings in the code
3. Set your device ID
4. Upload to ESP32

## Notes

- The system automatically manages power consumption based on battery levels
- GPS is disabled during sleep cycles to conserve power
- Data transmission intervals are adjusted based on power availability
- System includes watchdog functionality for reliability

## Contributing

Feel free to submit issues and enhancement requests!
