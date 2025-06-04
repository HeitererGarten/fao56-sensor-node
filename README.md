# FAO56 Sensor Node

An ESP32-based sensor node that collects environmental data and transmits it wirelessly via ESP-NOW protocol to a central hub. This is part of the FAO56 IoT system for agricultural monitoring and irrigation management.

## Overview

The Sensor Node captures temperature, humidity, and soil moisture data at configurable intervals. It operates on battery power with efficient sleep modes to conserve energy. Data is transmitted wirelessly to an ESP-NOW UART Hub, which forwards the information to the cloud.

## Features

- **Low Power Operation**: Optimized for battery-powered deployment
- **Multi-Sensor Support**: Temperature, humidity, and soil moisture sensing
- **ESP-NOW Communication**: Efficient short-range wireless data transmission
- **Automatic Pairing**: Self-registers with ESP-NOW hub
- **Deep Sleep Modes**: Extends battery life through power management
- **Configurable Sampling**: Adjustable measurement intervals

## Hardware Requirements

- ESP32 development board (NodeMCU-32S or ESP32-30pin)
- DHT11/DHT22 temperature and humidity sensor
- Capacitive soil moisture sensor
- Power source (Battery or USB)
- Waterproof enclosure for field deployment

## Pin Configuration

```cpp
#define DHT_PIN 33       // DHT temperature/humidity sensor pin
#define DHT_TYPE DHT11   // DHT sensor type
#define MOISTURE_PIN 32  // Analog pin for soil moisture sensor
```

## Data Structure

```cpp
struct sensorData {
    uint8_t msgType;    // Message type (DATA or PAIRING)
    char nodeID[8];     // Node identifier
    float temp;         // Temperature reading in °C
    float humidity;     // Humidity reading in %
    long moisture;      // Soil moisture reading in %
};
```

## Operation Flow

1. **Wake Up**: Device wakes from deep sleep or powers on
2. **Hub Connection**: Pairs with ESP-NOW hub if not already paired
3. **Sensor Reading**: Collects temperature, humidity, and soil moisture data
4. **Data Transmission**: Sends readings to hub via ESP-NOW
5. **Sleep**: Enters deep sleep mode to conserve power
6. **Repeat**: Wakes after configured interval to send new readings

## Power Management

The sensor node implements several strategies to minimize power consumption:
- Deep sleep between measurement cycles
- Quick sensor reads and transmissions
- Power-efficient wireless protocol (ESP-NOW)
- Configurable sleep duration based on requirements

## Configuration

The config file is located in config.h:

### Node Identity
```cpp
#define NODE_ID "S-0"  // Unique identifier for this node
```

### Sleep Settings
```cpp
#define SLEEP_TIMER 10  // Sleep duration in minutes
```

### Soil Moisture Calibration
```cpp
#define AIR_VALUE 2559    // ADC reading for dry soil
#define WATER_VALUE 1225  // ADC reading for wet soil
#define DEFAULT_MEASUREMENT -100  // Default value for failed readings
```

## Building and Deployment

### Prerequisites
- PlatformIO IDE or Arduino IDE with ESP32 support
- ESP32 board package installed

### Build Configuration
The project uses PlatformIO with configuration in platformio.ini:
- Target: ESP32 NodeMCU-32S or ESP32-30pin
- Framework: Arduino
- Required libraries: DHT sensor library, ESP-NOW

### Upload
1. Connect ESP32 to computer via USB
2. Select correct COM port and board
3. Build and upload firmware
4. Monitor serial output for debugging

### Field Deployment
1. Ensure battery is fully charged
2. Confirm sensors are properly connected
3. Verify successful pairing with hub before final placement
4. Place in waterproof enclosure
5. Install at desired location in field

## Debugging

Serial output provides detailed information about:
- Sensor initialization
- Reading values from sensors
- ESP-NOW pairing status
- Data transmission success/failure
- Sleep cycle information

Monitor at 115200 baud to view debug information during setup and testing.

## Integration

This sensor node works in conjunction with:
- **ESP-NOW UART Hub**: Receives sensor data and forwards to MQTT hub
- **MQTT Hub**: Processes data and sends to cloud services
- **FAO56 System**: Uses collected data for irrigation management

## Troubleshooting

### Common Issues
1. **Failed sensor readings**: Check sensor connections and power
2. **Pairing failures**: Ensure hub is in range and operational
3. **Short battery life**: Check for sleep mode issues or short circuits
4. **Incorrect readings**: Verify sensor calibration values (AIR_VALUE and WATER_VALUE)
5. **No data transmission**: Check ESP-NOW configuration and hub status

## Hardware Setup Tips

1. **Battery Connection**: Connect battery to VIN and GND pins
2. **Sensor Placement**: Ensure soil moisture sensor is properly inserted in soil
3. **Waterproofing**: Use silicone sealant around sensor connections
4. **Antenna Position**: Keep antenna clear of metal objects and ground

## License

Part of the FAO56 IoT agricultural monitoring system.