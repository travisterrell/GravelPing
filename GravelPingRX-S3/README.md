# GravelPingRX-S3

ESP32-S3 Super Mini receiver with dual-core architecture.

### Core 0 (PRO_CPU) - Network Management
- WiFi connection and reconnection
- MQTT broker connection and maintenance
- Publishing messages to Home Assistant
- Auto-reconnection with rate limiting
- Never blocks LoRa message reception

### Core 1 (APP_CPU) - LoRa Reception & Audio Alerts
- Time-critical LoRa message handling
- Parses JSON and queues messages to Core 0
- Plays buzzer alerts when Home Assistant is unavailable
- No Serial output (ESP32-S3 USB CDC is not thread-safe)
- Should never miss messages due to network delays

### Inter-Core Communication
- **Message Queue**: Parsed LoRa messages are queued from Core 1 to Core 0 via FreeRTOS `xQueue`
- **Volatile flags**: `haAvailable` read by Core 1 to decide buzzer alerts
- Queue size: 10 messages (configurable)

### Audio Backup
When Home Assistant is unavailable, Core 1 plays buzzer alerts directly:
- Three quick beeps + one long beep for vehicle detection
- Uses `vTaskDelay` (non-blocking, FreeRTOS-safe)
- No impact on network operations on Core 0

## MQTT Library Note

This implementation uses **espMqttClient** instead of the standard PubSubClient library used in the ESP32-C6 version. PubSubClient experienced unresolvable persistent connection timeout issues on the ESP32-S3's WiFi stack. The ESP32-C6 version works fine with PubSubClient - this is S3-specific

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
| LoRa TX | GPIO4 | ESP32-S3 TX → LR-02 RX |
| LoRa RX | GPIO5 | ESP32-S3 RX ← LR-02 TX (INPUT_PULLUP) |
| LoRa AUX | GPIO6 | Module status (LOW = ready, HIGH = busy) |
| Buzzer | GPIO8 | Active buzzer output |

## Configuration

Edit `platformio.ini` to configure WiFi & MQTT credentials or change the device name.

## Home Assistant Integration

### Required: Heartbeat Automation

**⚠️ IMPORTANT:** The receiver requires a Home Assistant automation be created to publish a MQTT heartbeat every 10 seconds. This allows the device to detect when Home Assistant is unavailable and trigger local audio alerts. (If this isn't created, the receiver will produce a local audio alert every time.)

Add this automation to your Home Assistant configuration:

```yaml
automation:
  - alias: "GravelPing Heartbeat"
    description: "Publish heartbeat every 10 seconds for GravelPing monitoring"
    trigger:
      - platform: time_pattern
        seconds: "/10"  # Every 10 seconds
    action:
      - service: mqtt.publish
        data:
          topic: "homeassistant/heartbeat"
          payload: "alive"
          qos: 0
          retain: false
```

**📖 For detailed information**, see [HA-Heartbeat-Monitoring.md](../TechnicalDocs/HA-Heartbeat-Monitoring.md)

### Home Assistant Entities

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

- **Compilation**: ~85 seconds (cached libs), ~165 seconds (full rebuild)
- **Flash Usage**: 83.3% (1,092,347 bytes of 1,310,720)
- **RAM Usage**: 16.0% (52,352 bytes of 327,680)
- **Message Queue**: 10 messages buffered
- **Reconnection Delay**: 5 seconds between attempts

## Serial Monitor Output

```
========================================
   GravelPing Receiver - ESP32-S3
   Dual-Core Architecture
========================================
Device ID: RX02-S3
----------------------------------------

[NETWORK] Task started on Core 0 (PRO_CPU)
[WIFI] Starting WiFi connection (non-blocking)...
[WIFI] ✓ Connected
       IP: 192.168.1.XXX
       RSSI: -69 dBm
[MQTT] ✓ Connected
[MQTT] Publishing Home Assistant autodiscovery...
[MQTT] ✓ Autodiscovery complete
[HEALTH] LoRa task heartbeat age: 5 ms ✓
[HA] ✓ Home Assistant available

========================================
[MESSAGE RECEIVED via Core 1 queue]
  Raw:     {"event":"entry","seq":42}
  Event:   entry
  Seq:     42
>>> VEHICLE DETECTED <<<
----------------------------------------
[MQTT] ✓ Published vehicle detection
[MQTT] ✓ Loop fault auto-cleared (vehicle detected)
```

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
- Check LoRa wiring (GPIO4 TX, GPIO5 RX, GPIO6 AUX)
- Verify baud rate matches transmitter (9600)
- Monitor serial output for queue messages
- Check for JSON parsing errors
- Confirm LoRa task heartbeat is healthy (not STALE)

### Task Watchdog Errors
- Should not occur with current implementation
- If seen, Core 0 network task may be blocking
- Check WiFi reconnection delays

## Future Enhancements

1. **Message Prioritization**
   - High-priority queue for vehicle detections
   - Lower priority for status updates

2. **Network Statistics**
   - WiFi signal strength monitoring
   - MQTT reconnection count
   - Message latency tracking

3. **Local Display**
   - SPI/I2C display on Core 0
   - Real-time status visualization
   - No impact on LoRa reception
