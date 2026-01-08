# GravelPing - Driveway Monitor

A driveway vehicle detection system utilizing the ESP32-C6 SuperMini and DX-LR02 LoRa modules. Triggered via relay output from an inductive loop detector or similar sensor.

## Table of Contents

- [Overview](#overview)
- [Hardware Components](#hardware-components)
  - [Transmitter Unit](#transmitter-unit)
  - [Receiver Unit](#receiver-unit)
  - [ESP32-C6 SuperMini LEDs](#esp32-c6-supermini-leds)
  - [Battery Monitoring (Optional)](#battery-monitoring-optional)
- [Wiring Diagrams](#wiring-diagrams)
  - [ESP32-C6 to LR-02 LoRa Module](#esp32-c6-to-lr-02-lora-module)
  - [ESP32-C6 to Loop Detector](#esp32-c6-to-loop-detector)
- [RGB LED Status Indicators](#rgb-led-status-indicators)
- [Pin Summary](#pin-summary)
- [LR-02 Module Pin Reference](#lr-02-module-pin-reference)
- [Software Configuration](#software-configuration)
  - [PlatformIO Setup](#platformio-setup)
  - [LR-02 Default Configuration](#lr-02-default-configuration)
  - [LR-02 Sleep Mode](#lr-02-sleep-mode)
  - [AT Command Reference](#at-command-reference)
- [Message Format](#message-format)
  - [Event Types](#event-types)
- [AUX Pin Behavior](#aux-pin-behavior)
- [Building and Flashing](#building-and-flashing)
  - [Prerequisites](#prerequisites)
  - [Transmitter (GravelPingTX)](#transmitter-gravelpingtx)
  - [Receiver (GravelPingRX)](#receiver-gravelpingrx)
  - [Configuration](#configuration)
  - [Troubleshooting Build Issues](#troubleshooting-build-issues)
- [Home Assistant Integration](#home-assistant-integration)
- [Serial Monitor Output](#serial-monitor-output)
- [Operation Flow](#operation-flow)
- [Troubleshooting](#troubleshooting)
  - [LoRa AUX Timeout](#lora-aux-timeout)
  - [No Wake from Sleep](#no-wake-from-sleep)
  - [No Vehicle Detection](#no-vehicle-detection)
  - [Messages Not Received](#messages-not-received)
- [License](#license)

## Overview

GravelPing consists of two units:
- **Transmitter Unit** (current): Detects vehicles via an inductive loop sensor and transmits LoRa messages
- **Receiver Unit** (asap): Receives messages and triggers alerts/actions

The system is designed to be battery-powered with deep sleep on both the ESP32-C6 and LR-02 module.

**Potential future featuress:**
- ⬜ Message acknowledgment/retry (currently supports duplicate sending via a build flag)

## Hardware Components

### Transmitter Unit

| Component | Model | Description |
|-----------|-------|-------------|
| Microcontroller | [ESP32-C6 SuperMini](https://www.espboards.dev/esp32/esp32-c6-super-mini/) | Low-power WiFi/BLE MCU with deep sleep support |
| LoRa Module | [DX-LR02](https://en.szdx-smart.com/EN/tczw/114.html) | LoRa UART module, 22dBm TX power (I use the 900T22D 915MHz version in the US, but choose the 433MHz variant if that's what correct for your region) |
| Loop Detector | [EMX LP D-TEK](https://www.emxaccesscontrolsensors.com/product/lp-d-tek/) | Inductive vehicle loop sensor with 2 relay outputs |

### ESP32-C6 SuperMini LEDs

| LED | GPIO | Purpose |
|-----|------|---------|
| Status LED | GPIO15 | Simple on/off indicator |
| RGB LED (WS2812) | GPIO8 | Color-coded status |
| Green (battery) | N/A | Battery charge indicator (not controllable) |

### Battery Monitoring (Optional)

The transmitter supports optional battery voltage monitoring for LiFePO4 4S batteries (12.8V nominal):

| Component | Model | Description |
|-----------|-------|-------------|
| Voltage Sensor | 5x Voltage Divider Module | Scales 0-25V input to 0-5V output. I already had some prebuilt modules on hand, but most people should  just [make one from 2 resistors](https://ohmslawcalculator.com/voltage-divider-calculator). |
| Battery | LiFePO4 4S | 12.8V nominal (14.6V max, 10.0V cutoff) |

**Features:**
- Monitors battery voltage via GPIO2 (ADC1_CH1)
- 12-bit ADC with 10-sample averaging for accuracy
- Voltage included in every LoRa message
- Low battery warning (≤10.5V) - Yellow LED flash
- Critical battery alert (≤9.5V) - Red LED flash
- Automatic MQTT publishing to Home Assistant
- Battery voltage sensor auto-discovery in Home Assistant

**Wiring:**
```
Battery (+) → Voltage Divider Input
Voltage Divider Output → ESP32-C6 GPIO2
Voltage Divider GND → ESP32-C6 GND
```

**Detailed Documentation:** [Battery Voltage Monitoring Guide](TechnicalDocs/Battery-Voltage-Monitoring.md)

## Wiring Diagrams

### ESP32-C6 to LR-02 LoRa Module

```
ESP32-C6 SuperMini          DX-LR02-900T22D
┌─────────────────┐         ┌─────────────────┐
│                 │         │                 │
│  GPIO17 (RX) ◄──┼─────────┼── TX (Pin 4)    │
│                 │         │                 │
│  GPIO16 (TX) ──►┼─────────┼── RX (Pin 3)    │
│                 │         │                 │
│  GPIO18 ◄───────┼─────────┼── AUX (Pin 5)   │
│                 │         │                 │
│  3.3V ──────────┼─────────┼── VCC (Pin 6)   │
│                 │         │                 │
│  GND ───────────┼─────────┼── GND (Pin 7)   │
│                 │         │                 │
└─────────────────┘         └─────────────────┘
```

### ESP32-C6 to Loop Detector

The D-TEKs have two relay outputs. Each relay has COM (Common) and NO (Normally Open) terminals.

```
ESP32-C6 SuperMini          EMX LP D-TEK
┌─────────────────┐         ┌─────────────────┐
│                 │         │                 │
│  GPIO4 ◄────────┼─────────┼── Relay 1 NO    │
│   (Vehicle)     │         │   (Presence)    │
│  GPIO5 ◄────────┼─────────┼── Relay 2 NO    │
│   (Loop Fault)  │         │   (Fail-Secure) │
│  GND ───────────┼─────────┼── Relay 1 COM   │
│                 │         │                 │
│  GND ───────────┼─────────┼── Relay 2 COM   │
│                 │         │                 │
└─────────────────┘         └─────────────────┘
```

**Relay Behavior:**
- **Relay 1 (Vehicle):** Closes to ground when a vehicle is detected over the loop
- **Relay 2 (Loop Fault):** Closes to ground when the D-TEK detects a loop fault condition. This behavior requires the D-TEK to be configured for "fail-secure." (See manual for details.)

Both GPIO4 and GPIO5 are configured as deep sleep wake sources - when either goes LOW, the ESP32 wakes from deep sleep.

## RGB LED Status Indicators

The ESP32-C6 SuperMini has a WS2812 RGB LED on GPIO8 used for status indication:

| Color | Pattern | Meaning |
|-------|---------|---------|
| Magenta (bright) | Solid | Waking from sleep / LR-02 waking |
| Blue | Solid | Transmitting LoRa message |
| Red | Flash 3x | Error / critical battery (≤9.5V) |
| Yellow | Flash 2x | Low battery warning (≤10.5V) |
| Green | Dim pulse | Idle, about to enter deep sleep |
| Magenta (dim) | Solid | About to enter deep sleep |

## Pin Summary (Transmitter & Receiver)

| ESP32-C6 Pin | Function | Connected To |
|--------------|----------|--------------|
| GPIO2 | ADC Input | **TX only:** Voltage divider output (optional battery monitoring) |
| GPIO4 | Deep Sleep Wake Source | **TX only:** LP D-TEK Relay 1 NO (Vehicle) |
| GPIO5 | Deep Sleep Wake Source | **TX only:** LP D-TEK Relay 2 NO (Loop Fault) |
| GPIO8 | WS2812 RGB LED | Onboard LED |
| GPIO15 | Status LED | Onboard LED |
| GPIO16 | UART1 TX | LR-02 RX |
| GPIO17 | UART1 RX | LR-02 TX |
| GPIO18 | Digital Input | LR-02 AUX |
| 3.3V | Power | LR-02 VCC |
| GND | Ground | LR-02 GND, **TX only:** LP D-TEK Relay COM  |

## LR-02 Module Pin Reference

| Pin # | Pin Name | Function |
|-------|----------|----------|
| 1 | M0 | Reserved (customizable IO) |
| 2 | M1 | Reserved (customizable IO) |
| 3 | UART_RX | Serial data input |
| 4 | UART_TX | Serial data output |
| 5 | AUX | RF status indicator (LOW=busy, HIGH=ready) |
| 6 | VCC | Power input (3.0V - 5.5V) |
| 7 | GND | Ground |

## Software Configuration

### PlatformIO Setup

The project uses PlatformIO with the pioarduino ESP32 platform for ESP32-C6 support (because the Platform.IO leadership stubbornly refuses to support new chips).

### LR-02 Default Configuration

The LR-02 module ships with these defaults (no configuration needed for basic operation):

| Parameter | Default Value |
|-----------|---------------|
| Baud Rate | 9600 |
| Working Mode | 0 (Transparent Transmission) |
| Power Mode | 2 (High Efficiency Mode) |
| Frequency | 850 MHz (Channel 00) |
| Air Rate Level | 0 (244 bit/s, ~8km range) |
| TX Power | 22 dBm |
| Device Address | FFFF |

### LR-02 Sleep Mode

The LR-02 supports a low-power sleep mode (~59µA) controlled via AT commands:

| Command | Description | Response |
|---------|-------------|----------|
| `AT+SLEEP0` | Enter full sleep mode | "SLEEP MODE" |
| `AT+SLEEP1` | Enter auto-sleep mode | "SLEEP MODE" |
| `AT+SLEEP2` | Exit sleep / normal operation | "Entry AT" |

**Waking from Sleep:** Send any 4+ bytes via serial to wake the module from sleep mode.

### AT Command Reference

To enter AT command mode, send `+++` (the module will respond with "Entry AT").

| Command | Description | Example |
|---------|-------------|---------|
| `AT` | Test connection | Response: `OK` |
| `AT+HELP` | Query all parameters | Shows full configuration |
| `AT+MODE0` | Set transparent transmission | Default mode |
| `AT+LEVEL<n>` | Set air rate level (0-7) | `AT+LEVEL0` for max range |
| `AT+CHANNEL<hex>` | Set channel (00-A2) | `AT+CHANNEL00` for 850MHz |
| `AT+SLEEP<n>` | Set sleep mode (0, 1, 2) | `AT+SLEEP0` for full sleep |
| `AT+RESET` | Restart module | Required after config changes |
| `+++` | Exit AT mode | Returns to transmission mode |

## Message Format

Messages are sent as compact JSON for efficient LoRa transmission:

```json
{
  "event": "entry",
  "seq": 123,
  "vbat": 12.3
}
```

| Field | Type | Description |
|-------|------|-------------|
| event | string | Event type (see below) |
| seq | int | Message sequence number (persists through sleep) |
| vbat | float | Battery voltage in volts (optional, if battery monitoring enabled) |

**Message Size:** 27 bytes without voltage, 39 bytes with voltage (both fit in single LoRa packet)

### Event Types

| Event | Description |
|-------|-------------|
| `entry` | Vehicle detected over the inductive loop (Relay 1) |
| `fault` | Loop fault condition detected (Relay 2, D-TEK in fail-secure mode) |
| `clear` | Loop fault cleared (Relay 2 released) |

## AUX Pin Behavior

The LR-02 AUX pin indicates module state:

| AUX State | Module Status |
|-----------|---------------|
| HIGH | Ready - can send data |
| LOW | Busy - receiving or transmitting |

The firmware monitors this pin to:
1. Confirm module is ready before sending
2. Detect when transmission completes

## Operation Flow

1. **Wake:** ESP32 wakes from deep sleep when GPIO4 goes LOW (relay triggered)
2. **Indicate:** RGB LED shows magenta (wake)
3. **Wake LR-02:** Send 4 bytes to wake LoRa module from sleep
4. **Transmit:** Send JSON message via LoRa, LED shows blue
5. **Sleep LR-02:** Send AT+SLEEP0 to put module in low-power mode
6. **Sleep ESP32:** Configure GPIO4 wake source, enter deep sleep

Both the ESP32-C6 and LR-02 sleep between events for maximum battery life.

## Building and Flashing

### Prerequisites
1. Install [PlatformIO Core](https://docs.platformio.org/en/latest/core/installation/index.html) or [PlatformIO IDE](https://marketplace.visualstudio.com/items?itemName=platformio.platformio-ide) or [pioarduino IDE](https://marketplace.visualstudio.com/items?itemName=pioarduino.pioarduino-ide). 
2. Clone this repository
3. Navigate to the project directory

### Transmitter (GravelPingTX)

```bash
# Navigate to transmitter directory
cd GravelPingTx

# Build the firmware
pio run

# Upload to connected ESP32-C6
pio run -t upload

# Monitor serial output
pio device monitor

# Build + Upload + Monitor (all in one)
pio run -t upload && pio device monitor
```

### Receiver (GravelPingRX)

```bash
# Navigate to receiver directory
cd GravelPingRx

# Build the firmware
pio run

# Upload to connected ESP32-C6
pio run -t upload

# Monitor serial output
pio device monitor

# Build + Upload + Monitor (all in one)
pio run -t upload && pio device monitor
```

### Configuration

#### Transmitter Configuration
`GravelPingTx/platformio.ini`
```ini
build_flags = 
    -D DEBUG_MODE=0              ; 0=production (deep sleep), 1=debug (no sleep)
    -D DUPLICATE_MESSAGES=0      ; 0=send once, 1=send twice for redundancy
```

#### Receiver Configuration
`GravelPingRx/platformio.ini`
```ini
build_flags = 
    ; WiFi Configuration
    -D WIFI_SSID=YourSSID
    -D WIFI_PASSWORD=YourPassword
    
    ; MQTT Configuration
    -D MQTT_BROKER=192.168.1.100
    -D MQTT_PORT=1883
    -D MQTT_USER=mqttUser
    -D MQTT_PASSWORD=mqttPassword
    
    ; Device Configuration
    -D DEVICE_NAME=GravelPingRX
```

### Troubleshooting Build Issues

**Clean build:**
```bash
pio run -t clean
pio run
```

**Specify upload port manually:**
```bash
pio run -t upload --upload-port COM3  # Windows
pio run -t upload --upload-port /dev/ttyUSB0  # Linux
```

## Home Assistant Integration

The receiver automatically publishes to Home Assistant via MQTT with autodiscovery support. Once the receiver is connected and messages are received, the following entities will automatically appear:

### Available Sensors

| Entity | Type | Description |
|--------|------|-------------|
| `binary_sensor.gravelping_vehicle_detected` | Binary Sensor | Vehicle detection (ON when vehicle enters) |
| `binary_sensor.gravelping_loop_fault` | Binary Sensor | Loop fault status (ON when fault detected) |
| `sensor.gravelping_message_count` | Sensor | Total messages received |
| `sensor.gravelping_battery_voltage` | Sensor | Battery voltage (if battery monitoring enabled) |

### Features

- **Auto-discovery**: Sensors automatically appear in Home Assistant
- **Auto-clear**: Loop fault automatically clears when vehicle detected (proves loop is working)
- **Manual clear**: Loop fault clears when transmitter sends "clear" event

## Serial Monitor Output

Example output when a vehicle is detected (requires `DEBUG_MODE=1` build flag):

```
========================================
   GravelPing TX - Phase 2 (Sleep)
========================================

[BOOT] Wake count: 1
[BOOT] Wake cause: GPIO (vehicle detected)
[BOOT] GPIO4 state: LOW (triggered)
[INIT] Configuring pins...
[INIT] LoRa UART: GPIO16(RX), GPIO17(TX)
[INIT] LoRa AUX:  GPIO18
[INIT] Relay 1:   GPIO4 (wake source)
[LORA] Waking LR-02 module...
[LORA] LR-02 ready
[EVENT] Vehicle detected on Relay 1!
[LORA] Preparing message...
[LORA] Sending: {"device":"TX01","version":1,"event":"vehicle_enter","relay":1,"seq":1,"wake":1}
[LORA] TX complete
[LORA] Putting LR-02 to sleep...
[SLEEP] Entering deep sleep...
[SLEEP] Wake source: GPIO4 LOW
```

## Troubleshooting

### LoRa AUX Timeout
- Check wiring between ESP32-C6 and LR-02
- Verify LR-02 is powered (3.3V-5V)
- Module may need a few seconds to initialize after power-on
- Red LED flashes 3 times on AUX timeout

### No Wake from Sleep
- Verify GPIO4 is correctly wired to relay
- Check relay closes to GND when triggered
- LED should show magenta on wake

### No Vehicle Detection
- Check relay wiring - COM to GND, NO to GPIO4
- Verify LP D-TEK is detecting vehicles (LED indicator) and closing the relay

### Messages Not Received
- Ensure receiver LR-02 has matching configuration:
  - Same LEVEL (air rate)
  - Same CHANNEL
  - MODE 0 (transparent transmission)
- Check antenna connections on both units

## License
Do whatever you want!
