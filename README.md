# GravelPing - Driveway Monitor

A LoRa-based driveway vehicle detection system using ESP32-C6 and DX-LR02-900T22D modules.

## Overview

GravelPing consists of two units:
- **Transmitter Unit** (current): Detects vehicles via an inductive loop sensor and transmits LoRa messages
- **Receiver Unit** (future): Receives messages and triggers alerts/actions

The system is designed to be battery-powered with deep sleep on both the ESP32-C6 and LR-02 module.

## Hardware Components

### Transmitter Unit

| Component | Model | Description |
|-----------|-------|-------------|
| Microcontroller | ESP32-C6 SuperMini | Low-power WiFi/BLE MCU with deep sleep support |
| LoRa Module | DX-LR02-900T22D | 850-930MHz LoRa UART module, 22dBm TX power |
| Loop Detector | EMX LP D-TEK | Inductive vehicle loop sensor with 2 relay outputs |

### ESP32-C6 SuperMini LEDs

| LED | GPIO | Purpose |
|-----|------|---------|
| Status LED | GPIO15 | Simple on/off indicator |
| RGB LED (WS2812) | GPIO8 | Color-coded status |
| Green (battery) | N/A | Battery charge indicator (not controllable) |

## Wiring Diagram

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

### ESP32-C6 to EMX LP D-TEK Loop Detector

The LP D-TEK has two relay outputs. Each relay has COM (Common) and NO (Normally Open) terminals.

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
- **Relay 2 (Loop Fault):** Closes to ground when the D-TEK is in "fail-secure" mode and detects a loop fault condition

Both GPIO4 and GPIO5 are configured as deep sleep wake sources - when either goes LOW, the ESP32 wakes from deep sleep.

## RGB LED Status Indicators

The ESP32-C6 SuperMini has a WS2812 RGB LED on GPIO8 used for status indication:

| Color | Pattern | Meaning |
|-------|---------|---------|
| Magenta (bright) | Solid | Waking from sleep / LR-02 waking |
| Blue | Solid | Transmitting LoRa message |
| Red | Flash 3x | Error condition (TX failed, AUX timeout) |
| Green | Dim pulse | Idle, about to enter deep sleep |
| Magenta (dim) | Solid | About to enter deep sleep |

## Pin Summary

| ESP32-C6 Pin | Function | Connected To |
|--------------|----------|--------------|
| GPIO4 | Deep Sleep Wake Source | LP D-TEK Relay 1 NO (Vehicle) |
| GPIO5 | Deep Sleep Wake Source | LP D-TEK Relay 2 NO (Loop Fault) |
| GPIO8 | WS2812 RGB LED | Onboard LED |
| GPIO15 | Status LED | Onboard LED |
| GPIO16 | UART1 TX | LR-02 RX |
| GPIO17 | UART1 RX | LR-02 TX |
| GPIO18 | Digital Input | LR-02 AUX |
| 3.3V | Power | LR-02 VCC |
| GND | Ground | LR-02 GND, LP D-TEK Relay COM |

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
    fastled/FastLED@^3.9.0
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

Messages are sent as JSON for easy parsing:

```json
{
  "device": "TX01",
  "version": 1,
  "event": "vehicle_enter",
  "relay": 1,
  "seq": 1,
  "wake": 1
}
```

| Field | Type | Description |
|-------|------|-------------|
| device | string | Transmitter identifier |
| version | int | Protocol version number |
| event | string | Event type (see below) |
| relay | int | Which relay triggered (1 or 2) |
| seq | int | Message sequence number (persists through sleep) |
| wake | int | Wake cycle counter (how many times ESP32 has woken) |

### Event Types

| Event | Relay | Description |
|-------|-------|-------------|
| `vehicle_enter` | 1 | Vehicle detected over the inductive loop |
| `loop_fault` | 2 | Loop fault condition (D-TEK in fail-secure mode) |

## AUX Pin Behavior

The LR-02 AUX pin indicates module state:

| AUX State | Module Status |
|-----------|---------------|
| HIGH | Ready - can send data |
| LOW | Busy - receiving or transmitting |

The firmware monitors this pin to:
1. Confirm module is ready before sending
2. Detect when transmission completes

## Current Phase: Phase 2

**Implemented:**
- ✅ Basic relay detection (Relay 1 only)
- ✅ LoRa message transmission
- ✅ JSON message format
- ✅ AUX pin monitoring
- ✅ ESP32-C6 deep sleep with GPIO4 wake
- ✅ LR-02 sleep mode (AT+SLEEP0)
- ✅ RGB LED status indicators (WS2812)

**Planned for Future Phases:**
- ⬜ Relay 2 support with different behavior
- ⬜ Receiver unit implementation
- ⬜ Battery voltage monitoring
- ⬜ Message acknowledgment/retry (or just multiple sends)

## Operation Flow

1. **Wake:** ESP32 wakes from deep sleep when GPIO4 goes LOW (relay triggered)
2. **Indicate:** RGB LED shows magenta (wake)
3. **Wake LR-02:** Send 4 bytes to wake LoRa module from sleep
4. **Transmit:** Send JSON message via LoRa, LED shows blue
5. **Sleep LR-02:** Send AT+SLEEP0 to put module in low-power mode
6. **Sleep ESP32:** Configure GPIO4 wake source, enter deep sleep

Both the ESP32-C6 and LR-02 sleep between events for maximum battery life.

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
- Verify LP D-TEK is detecting vehicles (LED indicator)

### Messages Not Received
- Ensure receiver LR-02 has matching configuration:
  - Same LEVEL (air rate)
  - Same CHANNEL
  - MODE 0 (transparent transmission)
- Check antenna connections on both units

## License

[Add your license here]
