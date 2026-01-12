# GravelPingRX-S3

ESP32-S3 Super Mini receiver with dual-core architecture.

## MQTT Library Note

This implementation uses **espMqttClient** instead of the standard PubSubClient library used in the ESP32-C6 version.

**Why the different library?**
- PubSubClient experienced **persistent connection timeout issues** on the ESP32-S3's WiFi stack
- Multiple connection attempts would timeout even with reliable WiFi/MQTT broker
- espMqttClient is **ESP32-native** and integrates better with the ESP32 WiFi stack
- espMqttClient is **fully async/non-blocking**, making it ideal for dual-core architecture
- The ESP32-C6 version works fine with PubSubClient - this is S3-specific

**Benefits of espMqttClient:**
- ✅ Reliable connections on ESP32-S3
- ✅ Async operations (non-blocking)
- ✅ Better integration with ESP32 event loop
- ✅ Handles reconnections in background
- ✅ Compatible with FreeRTOS watchdog timer

## Dual-Core Architecture

This receiver implementation takes full advantage of the ESP32-S3's dual-core processor:

### Core 0 (PRO_CPU) - Network Management
- WiFi connection and reconnection
- MQTT broker connection and maintenance
- Publishing messages to Home Assistant
- Auto-reconnection with rate limiting
- Never blocks LoRa message reception

### Core 1 (APP_CPU) - LoRa Reception
- Time-critical LoRa message handling
- Runs on the default Arduino core
- Dedicated to receiving and parsing messages
- Guaranteed to never miss messages due to network delays

### Inter-Core Communication
- **Message Queue**: LoRa messages are queued from Core 1 to Core 0
- **MQTT Mutex**: Protects MQTT client from concurrent access
- Queue size: 10 messages (configurable)

## Hardware: ESP32-S3 Super Mini

### Key Specifications
- **CPU**: Dual-core Xtensa LX7 @ 240 MHz
- **RAM**: 512 KB SRAM
- **Flash**: 4 MB
- **WiFi**: 802.11 b/g/n (2.4 GHz)
- **Bluetooth**: BLE 5.0
- **Size**: 22.52 x 18 mm (ultra-compact)
- **Power**: Deep sleep ~43 μA

### Pin Configuration

| Function | GPIO | Notes |
|----------|------|-------|
| RGB LED (WS2812) | GPIO48 | Shared with red power LED |
| LoRa RX | GPIO17 | ESP32-S3 RX ← LR-02 TX |
| LoRa TX | GPIO16 | ESP32-S3 TX → LR-02 RX |
| LoRa AUX | GPIO18 | Module status indicator |

**Safe GPIOs for expansion**: IO1, IO2, IO4, IO5, IO6, IO7, IO8, IO15, IO16, IO17, IO18, IO21

**Avoid**: IO9-IO14 (flash), IO19-IO20 (USB), IO3 (strapping), IO0/IO45/IO46 (strapping)

## Why Dual-Core?

### Problem
In single-core implementations, WiFi/MQTT reconnection attempts can block the main loop for seconds, causing missed LoRa messages.

### Solution
By dedicating Core 1 exclusively to LoRa reception and moving all network management to Core 0, we guarantee:
- ✅ Zero message loss during network reconnections
- ✅ Reliable operation even with poor WiFi
- ✅ Foundation for future audio backup system

### Future: Audio Backup
When Home Assistant is unavailable, Core 0 will handle audio notifications:
- Play audio alerts for vehicle detection
- Manage audio buffer and playback
- No impact on LoRa message reception on Core 1

## Configuration

Edit `platformio.ini` to configure:

```ini
; WiFi Configuration
-D WIFI_SSID=YourSSID
-D WIFI_PASSWORD=YourPassword

; MQTT Configuration
-D MQTT_BROKER=192.168.1.100
-D MQTT_PORT=1883
-D MQTT_USER=username
-D MQTT_PASSWORD=password

; Device Configuration
-D DEVICE_NAME=GravelPing-S3
```

## Home Assistant Integration

The receiver publishes these entities with unique IDs to avoid conflicts with other receivers:

- `binary_sensor.gravelping_s3_vehicle` - Vehicle detection (auto-off 5s)
- `binary_sensor.gravelping_s3_loop_fault` - Loop fault indicator
- `sensor.gravelping_s3_message_count` - Total messages received
- `sensor.gravelping_s3_battery_voltage` - Transmitter battery voltage

## Building and Uploading

```bash
# Build
pio run

# Upload
pio run --target upload

# Monitor serial output
pio device monitor --baud 115200
```

## LED Status Indicators

| Color | Pattern | Meaning |
|-------|---------|---------|
| Cyan | Solid | System boot |
| Yellow | Solid | WiFi connecting |
| Magenta | Solid | MQTT connecting |
| Green | Dim | Idle, ready to receive |
| Blue | Flash | Message received |
| Red | Flash | Error/invalid message |

## Performance

- **Compilation**: ~165 seconds (includes FastLED with all features)
- **Flash Usage**: 31.3% (1,047,675 bytes)
- **RAM Usage**: 14.6% (47,952 bytes)
- **Message Queue**: 10 messages buffered
- **Reconnection Delay**: 5 seconds between attempts

## Serial Monitor Output

```
========================================
   GravelPing Receiver - ESP32-S3
   Dual-Core Architecture
========================================
Device ID: RX02-S3
Core: 1 (APP_CPU)
----------------------------------------

[INIT] Initializing RGB LED...
[INIT] ✓ RGB LED ready
[INIT] Initializing LoRa UART...
[INIT] ✓ LoRa UART ready
[NETWORK] Task started on Core 0 (PRO_CPU)
[WIFI] Connecting to WiFi...
[WIFI] ✓ Connected
       IP: 192.168.1.XXX
[MQTT] ✓ Connected
[MQTT] Publishing Home Assistant autodiscovery...
[INIT] Setup complete

========================================
[MESSAGE RECEIVED]
[RAW]
{"event":"entry","seq":42,"vbat":12.8}
[PARSED]
  Event:   entry
  Seq:     42
  VBat:    12.8V
>>> VEHICLE DETECTED <<<
----------------------------------------

[MQTT] ✓ Published vehicle detection
[MQTT] ✓ Published battery voltage: 12.8V
```

## Comparison with ESP32-C6 Version

| Feature | ESP32-C6 (GravelPingRX) | ESP32-S3 (This) |
|---------|-------------------------|-----------------|
| Cores | Single (RISC-V) | Dual (Xtensa) |
| Clock | 160 MHz | 240 MHz |
| LoRa Handling | Main loop | Dedicated Core 1 |
| Network | Main loop | Dedicated Core 0 |
| Message Loss Risk | Low | Near-zero |
| Audio Ready | No | Yes |
| Flash Used | 83.1% | 31.3% |

## Troubleshooting

### WiFi Won't Connect
- Check SSID/password in platformio.ini
- Ensure 2.4 GHz network (ESP32-S3 doesn't support 5 GHz)
- Check serial monitor for error messages

### MQTT Won't Connect
- Verify broker IP and port
- Check username/password if required
- Ensure broker is accessible from ESP32-S3's network
- Try `mosquitto_pub` from another device to test broker

### Missing Messages
- Check LoRa wiring (GPIO16/17/18)
- Verify baud rate matches transmitter (9600)
- Monitor both TX and RX serial outputs
- Check for JSON parsing errors in serial monitor

### Task Watchdog Errors
- Should not occur with current implementation
- If seen, Core 0 network task may be blocking
- Check WiFi reconnection delays

## Future Enhancements

1. **Audio Backup System**
   - I2S audio output on Core 0
   - MP3/WAV playback for alerts
   - Fallback when Home Assistant unavailable

2. **Message Prioritization**
   - High-priority queue for vehicle detections
   - Lower priority for status updates

3. **Network Statistics**
   - WiFi signal strength monitoring
   - MQTT reconnection count
   - Message latency tracking

4. **Local Display**
   - SPI/I2C display on Core 0
   - Real-time status visualization
   - No impact on LoRa reception
