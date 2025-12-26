# GravelPing - Driveway Monitor

A LoRa-based driveway vehicle detection system using ESP32-C6 and DX-LR02-900T22D modules.

## Overview

GravelPing consists of two units:
- **Transmitter Unit** (this phase): Detects vehicles via an inductive loop sensor and transmits LoRa messages
- **Receiver Unit** (future): Receives messages and triggers alerts/actions

The system is designed to be battery-powered with deep sleep capabilities (to be implemented in later phases).

## Hardware Components

### Transmitter Unit

| Component | Model | Description |
|-----------|-------|-------------|
| Microcontroller | ESP32-C6 SuperMini | Low-power WiFi/BLE MCU with deep sleep support |
| LoRa Module | DX-LR02-900T22D | 850-930MHz LoRa UART module, 22dBm TX power |
| Loop Detector | EMX LP D-TEK | Inductive vehicle loop sensor with 2 relay outputs |

## Wiring Diagram

### ESP32-C6 to LR-02 LoRa Module

```
ESP32-C6 SuperMini          DX-LR02-900T22D
┌─────────────────┐         ┌─────────────────┐
│                 │         │                 │
│  GPIO16 (RX) ◄──┼─────────┼── TX (Pin 4)    │
│                 │         │                 │
│  GPIO17 (TX) ──►┼─────────┼── RX (Pin 3)    │
│                 │         │                 │
│  GPIO18 ◄───────┼─────────┼── AUX (Pin 5)   │
│                 │         │                 │
│  3.3V ──────────┼─────────┼── VCC (Pin 6)   │
│                 │         │                 │
│  GND ───────────┼─────────┼── GND (Pin 7)   │
│                 │         │                 │
└─────────────────┘         └─────────────────┘
```

### ESP32-C6 to EMX LP D-TEK Loop Detector

The LP D-TEK has two relay outputs. Each relay has COM (Common) and NO (Normally Open) terminals.

```
ESP32-C6 SuperMini          EMX LP D-TEK
┌─────────────────┐         ┌─────────────────┐
│                 │         │                 │
│  GPIO4 ◄────────┼─────────┼── Relay 1 NO    │
│                 │         │                 │
│  GPIO5 ◄────────┼─────────┼── Relay 2 NO    │
│                 │         │                 │
│  GND ───────────┼─────────┼── Relay 1 COM   │
│                 │         │                 │
│  GND ───────────┼─────────┼── Relay 2 COM   │
│                 │         │                 │
└─────────────────┘         └─────────────────┘
```

**Note:** The relay outputs close to ground when a vehicle is detected. GPIO pins use internal pull-up resistors, so they read HIGH normally and LOW when triggered.

## Pin Summary

| ESP32-C6 Pin | Function | Connected To |
|--------------|----------|--------------|
| GPIO16 | UART1 RX | LR-02 TX |
| GPIO17 | UART1 TX | LR-02 RX |
| GPIO18 | Digital Input | LR-02 AUX |
| GPIO4 | Digital Input (Pull-up) | LP D-TEK Relay 1 NO |
| GPIO5 | Digital Input (Pull-up) | LP D-TEK Relay 2 NO |
| 3.3V | Power | LR-02 VCC |
| GND | Ground | LR-02 GND, LP D-TEK Relay COM |

## LR-02 Module Pin Reference

| Pin # | Pin Name | Function |
|-------|----------|----------|
| 1 | M0 | Reserved (customizable IO) |
| 2 | M1 | Reserved (customizable IO) |
| 3 | UART_RX | Serial data input |
| 4 | UART_TX | Serial data output |
| 5 | AUX | RF status indicator (LOW=ready, HIGH=busy) |
| 6 | VCC | Power input (3.0V - 5.5V) |
| 7 | GND | Ground |

## Software Configuration

### PlatformIO Setup

The project uses PlatformIO with the pioarduino ESP32 platform for ESP32-C6 support.

**platformio.ini:**
```ini
[env:esp32c6]
platform = https://github.com/pioarduino/platform-espressif32.git#main
board = esp32-c6-devkitm-1
framework = arduino
upload_speed = 921600
monitor_speed = 115200
build_flags = 
    -D ARDUINO_USB_MODE=1
    -D ARDUINO_USB_CDC_ON_BOOT=1
lib_deps = 
    bblanchon/ArduinoJson@^7.2.1
```

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

### AT Command Reference

To enter AT command mode, send `+++` (the module will respond with "Entry AT").

| Command | Description | Example |
|---------|-------------|---------|
| `AT` | Test connection | Response: `OK` |
| `AT+HELP` | Query all parameters | Shows full configuration |
| `AT+MODE0` | Set transparent transmission | Default mode |
| `AT+LEVEL<n>` | Set air rate level (0-7) | `AT+LEVEL0` for max range |
| `AT+CHANNEL<hex>` | Set channel (00-A2) | `AT+CHANNEL00` for 850MHz |
| `AT+RESET` | Restart module | Required after config changes |
| `+++` | Exit AT mode | Returns to transmission mode |

## Message Format

Messages are sent as JSON for easy parsing:

```json
{
  "device": "TX01",
  "version": 1,
  "event": "vehicle_enter",
  "relay": 1,
  "seq": 1,
  "uptime": 123
}
```

| Field | Type | Description |
|-------|------|-------------|
| device | string | Transmitter identifier |
| version | int | Protocol version number |
| event | string | Event type (e.g., "vehicle_enter") |
| relay | int | Which relay triggered (1 or 2) |
| seq | int | Message sequence number |
| uptime | int | Device uptime in seconds |

## AUX Pin Behavior

The LR-02 AUX pin indicates module state:

| AUX State | Module Status |
|-----------|---------------|
| LOW | Idle/ready - can send data |
| HIGH | Busy - receiving or transmitting |

The firmware monitors this pin to:
1. Confirm module is ready before sending
2. Detect when transmission completes

## Current Phase: Phase 1

**Implemented:**
- ✅ Basic relay detection (Relay 1 only)
- ✅ LoRa message transmission
- ✅ JSON message format
- ✅ AUX pin monitoring
- ✅ Debouncing and cooldown

**Planned for Future Phases:**
- ⬜ ESP32-C6 deep sleep with GPIO wake
- ⬜ LR-02 sleep mode (AT+SLEEP0)
- ⬜ Relay 2 support with different behavior
- ⬜ Receiver unit implementation
- ⬜ Battery voltage monitoring
- ⬜ Message acknowledgment/retry

## Building and Flashing

1. Install [PlatformIO](https://platformio.org/)
2. Clone this repository
3. Open in VS Code with PlatformIO extension
4. Build: `Ctrl+Alt+B` or click Build button
5. Upload: `Ctrl+Alt+U` or click Upload button
6. Monitor: `Ctrl+Alt+M` or click Serial Monitor button

## Serial Monitor Output

Example output when a vehicle is detected:

```
========================================
   GravelPing Transmitter - Phase 1
========================================

[INIT] Configuring pins...
[INIT] Pins configured:
       - LoRa TX (ESP RX): GPIO16
       - LoRa RX (ESP TX): GPIO17
       - LoRa AUX:         GPIO18
       - Relay 1:          GPIO4
       - Relay 2:          GPIO5
[INIT] Initializing LoRa module...
[INIT] LoRa module ready (AUX pin LOW)
[INIT] LoRa UART configured at 9600 baud
[INIT] Setup complete. Waiting for vehicle detection...

[EVENT] Relay 1 triggered - Vehicle detected!
[LORA] Preparing to send message...
[LORA] Sending: {"device":"TX01","version":1,"event":"vehicle_enter","relay":1,"seq":1,"uptime":45}
[LORA] Message sent successfully
```

## Troubleshooting

### LoRa AUX Timeout
- Check wiring between ESP32-C6 and LR-02
- Verify LR-02 is powered (3.3V-5V)
- Module may need a few seconds to initialize after power-on

### No Vehicle Detection
- Check relay wiring - COM to GND, NO to GPIO4
- Verify LP D-TEK is detecting vehicles (LED indicator)
- Check if cooldown period is active (2 seconds between triggers)

### Messages Not Received
- Ensure receiver LR-02 has matching configuration:
  - Same LEVEL (air rate)
  - Same CHANNEL
  - MODE 0 (transparent transmission)
- Check antenna connections on both units

## License

[Add your license here]
