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
| RGB LED (WS2812) | GPIO48 | Onboard status indicator |
| Alert LED (WS2812B) | GPIO7 | External alert status indicator (optional) |
| LoRa TX | GPIO4 | ESP32-S3 TX → LR-02 RX |
| LoRa RX | GPIO5 | ESP32-S3 RX ← LR-02 TX (INPUT_PULLUP) |
| LoRa AUX | GPIO6 | Module status (LOW = ready, HIGH = busy) |
| Buzzer | GPIO8 | Active buzzer output |

### Optional: External Alert Status LED

An external WS2812B LED can be connected to **GPIO7** to show alert enabled/disabled status:
- **Purple**: Alerts enabled (default)
- **Red**: Alerts disabled

This feature is completely optional. If you don't connect an LED to GPIO7, the firmware will work normally - you'll just see the onboard status LED instead.

**To use this feature**, you need to:
1. Connect a WS2812B LED to GPIO7 + appropriate power and ground. (Pro-tip: You can buy a cheap WS2812-based strip and end up with 100+ individual WS2812 LEDs for ≈$10.)
2. Create a Home Assistant automation that publishes alert state changes to: `homeassistant/binary_sensor/gravelping/alerts_enabled/state`
3. The automation should publish `"ON"` when alerts are enabled, `"OFF"` when paused/disabled

**Example Home Assistant automation:**
```yaml
automation:
  - alias: "GravelPing Alert Status Publisher"
    description: "Publish alert enabled/disabled state to MQTT"
    triggers:
      - trigger: state
        entity_id:
          - input_boolean.gravelping_alerts_enabled  # Your pause/resume toggle
    conditions: []
    actions:
      - data:
          topic: homeassistant/binary_sensor/gravelping/alerts_enabled/state
          retain: true
          payload: "{{ states('input_boolean.gravelping_alerts_enabled') | upper }}"
        action: mqtt.publish
```

## Configuration

Edit `platformio.ini` to configure WiFi & MQTT credentials or change the device name.

### MQTT Watchdog

The firmware includes an automatic watchdog that monitors MQTT connection health:
- If MQTT fails to reconnect for **5 minutes** continuously, the ESP32 will automatically reboot
- This resolves stuck connection states that sometimes occur after network disruptions
- The timer resets whenever:
  - MQTT successfully connects
  - Any MQTT message is received
- You'll see a warning message in serial output before the reboot

**To adjust the timeout**, edit `MQTT_FAILURE_REBOOT_TIMEOUT` in `main.cpp` (default: 300000ms = 5 minutes).

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

### Onboard Status LED (GPIO48)
| Color | Pattern | Meaning |
|-------|---------|---------|
| Cyan | Solid | System boot |
| Yellow | Solid | WiFi connecting |
| Magenta | Solid | MQTT connecting |
| Green | Dim | Idle, ready to receive |
| Blue | Flash | Message received |
| Red | Flash | Error/invalid message |

### External Alert LED (GPIO7) - Optional
| Color | Meaning |
|-------|---------|
| Cyan | Local backup audio active (WiFi/MQTT/HA down) |
| Dark Violet | Alerts enabled (HA available) |
| Red | Alerts disabled/paused (HA available) |

**Priority**: Cyan (backup audio) overrides user preference. If WiFi, MQTT, or HA heartbeat is missing, the LED shows cyan to indicate local buzzer alerts will sound regardless of the enabled/disabled state.

*Note: Color changes occur when network state changes or MQTT messages are received on the alerts status topic.*

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

### Won't Boot / Only Red LED (LoRa Module Issue)
**Symptoms:** ESP32-S3 won't boot when LoRa module is connected during power-up. Works fine if LoRa is connected after boot. LoRa module's LED indicates it IS powering up correctly.

**Root Cause:** LoRa module transmits data during ESP32 boot sequence, potentially interfering with bootloader on GPIO5 (RX pin).

**Temporary Workaround, because the below haven't fully worked thus far.**
Wrap hand around antenna when connecting power. Keep hand around it until you see the status or alert LEDs illuminate.

**Solutions (in order of effectiveness):**
1. **Hardware - Add 10kΩ pull-down resistor on GPIO5:**
   - GPIO5 (ESP32 RX / LoRa TX) → 10kΩ resistor → GND
   - Keeps the pin low during boot, prevents LoRa TX from interfering
   - Most reliable solution

2. **Hardware - RC delay circuit on LoRa power:**
   - Add RC circuit to delay LoRa power-up by ~1 second after ESP32
   - Allows ESP32 to boot before LoRa begins transmitting

3. **Software delay (Already implemented):**
   - 2-second delay before LoRa UART initialization
   - Reduces occurrence but doesn't eliminate it

4. **100nF ceramic caps to ground on TX, RX, Aux, 5v (Already implemented)**

**Note:** Bulk capacitor (100-470µF) helps with power stability but doesn't solve signal interference during boot.

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
