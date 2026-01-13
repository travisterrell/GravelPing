# MQTT Audio Test Commands

## Quick Reference

**MQTT Topic:** `gravelping/s3/audio/test`

Upload your updated firmware, then publish to this topic to test different sounds!

**Note:** The GravelPingRX-S3 uses an **active buzzer** with a fixed tone frequency. Commands control beep *duration* and *patterns* only (not frequency/pitch).

## Predefined Patterns

| Command | Description |
|---------|-------------|
| `pattern` | Default alert (3 quick beeps + 1 long beep) |
| `startup` | Two-tone startup sound (2x 100ms beeps) |
| `short` | Single short beep (200ms) |
| `medium` or `med` | Single medium beep (500ms) |
| `long` | Single long beep (1000ms) |
| `siren` | Rapid beeping (6x 200ms beeps with 100ms gaps) |
| `alarm` | Fast alarm (5x 150ms beeps with 50ms gaps) |
| `stop` | Stop any playing tone |

## Custom Durations

**Format:** Just the duration in milliseconds (no frequency needed for active buzzer)

**Examples:**
- `100` - Very short beep (100ms)
- `500` - Half-second beep
- `1000` - One second beep
- `2000` - Two second beep (long alert)

**Limits:**
- Duration: 1-9999 ms

## Testing Methods

### Method 1: MQTT Explorer / MQTT.fx
1. Connect to your MQTT broker
2. Publish to: `gravelping/s3/audio/test`
3. Payload: One of the commands above
4. QoS: 0

### Method 2: Home Assistant Developer Tools
1. Go to **Developer Tools** → **Services**
2. Service: `mqtt.publish`
3. Service Data:
```yaml
topic: gravelping/s3/audio/test
payload: pattern
```
4. Click **Call Service**

### Method 3: Mosquitto Command Line
```bash
# Predefined pattern
mosquitto_pub -h 192.168.1.175 -t "gravelping/s3/audio/test" -m "pattern"

# Custom duration (500ms beep)
mosquitto_pub -h 192.168.1.175 -t "gravelping/s3/audio/test" -m "500"

# Short beep
mosquitto_pub -h 192.168.1.175 -t "gravelping/s3/audio/test" -m "short"

# Siren pattern
mosquitto_pub -h 192.168.1.175 -t "gravelping/s3/audio/test" -m "siren"
```

### Method 4: Home Assistant Automation
Create a test automation:

```yaml
automation:
  - alias: "Test GravelPing Audio"
    trigger:
      - platform: state
        entity_id: input_button.test_audio  # Create this helper first
        to: ~
    action:
      - service: mqtt.publish
        data:
          topic: "gravelping/s3/audio/test"
          payload: "pattern"
```

Or use a script:

```yaml
script:
  gravelping_test_audio:
    alias: "GravelPing Test Audio"
    sequence:
      - service: mqtt.publish
        data:
          topic: "gravelping/s3/audio/test"
          payload: "{{ pattern | default('pattern') }}"
```

Call it with:
```yaml
service: script.gravelping_test_audio
data:
  pattern: "siren"
```

## Testing Different Durations

Try these to find what works best for your use case:

```bash
# Very short beep (attention-getter)
mosquitto_pub -h 192.168.1.175 -t "gravelping/s3/audio/test" -m "100"

# Quick beep
mosquitto_pub -h 192.168.1.175 -t "gravelping/s3/audio/test" -m "200"

# Standard beep
mosquitto_pub -h 192.168.1.175 -t "gravelping/s3/audio/test" -m "500"

# Long alert
mosquitto_pub -h 192.168.1.175 -t "gravelping/s3/audio/test" -m "1000"

# Very long (emergency)
mosquitto_pub -h 192.168.1.175 -t "gravelping/s3/audio/test" -m "2000"
```

## Pattern Examples

Test the built-in patterns:

```bash
# Default vehicle alert
mosquitto_pub -h 192.168.1.175 -t "gravelping/s3/audio/test" -m "pattern"

# Startup confirmation
mosquitto_pub -h 192.168.1.175 -t "gravelping/s3/audio/test" -m "startup"

# Emergency siren
mosquitto_pub -h 192.168.1.175 -t "gravelping/s3/audio/test" -m "siren"

# Alarm sound
mosquitto_pub -h 192.168.1.175 -t "gravelping/s3/audio/test" -m "alarm"
```

## Creating Custom Patterns in Home Assistant

You can create custom beep patterns with automations:

**Double Beep:**
```yaml
automation:
  - alias: "Double Beep"
    action:
      - service: mqtt.publish
        data:
          topic: "gravelping/s3/audio/test"
          payload: "200"
      - delay:
          milliseconds: 300
      - service: mqtt.publish
        data:
          topic: "gravelping/s3/audio/test"
          payload: "200"
```

**Triple Short Alert:**
```yaml
automation:
  - alias: "Triple Alert"
    action:
      - repeat:
          count: 3
          sequence:
            - service: mqtt.publish
              data:
                topic: "gravelping/s3/audio/test"
                payload: "150"
            - delay:
                milliseconds: 200
```

**Morse Code SOS:**
```yaml
automation:
  - alias: "SOS Pattern"
    action:
      # S (three short)
      - repeat:
          count: 3
          sequence:
            - service: mqtt.publish
              data:
                topic: "gravelping/s3/audio/test"
                payload: "100"
            - delay:
                milliseconds: 150
      - delay:
          milliseconds: 300
      # O (three long)
      - repeat:
          count: 3
          sequence:
            - service: mqtt.publish
              data:
                topic: "gravelping/s3/audio/test"
                payload: "300"
            - delay:
                milliseconds: 150
      - delay:
          milliseconds: 300
      # S (three short)
      - repeat:
          count: 3
          sequence:
            - service: mqtt.publish
              data:
                topic: "gravelping/s3/audio/test"
                payload: "100"
            - delay:
                milliseconds: 150"
```

## Monitoring Results

Watch the serial output to see what's happening:

```
[AUDIO] Test command received: pattern
[AUDIO] Playing alert pattern...
[AUDIO] Alert pattern complete
```

Or for custom durations:
```
[AUDIO] Test command received: 500
[AUDIO] Playing tone for 500 ms
```

## Error Messages

If you see this:
```
[AUDIO] Unknown command. Valid: pattern, startup, short, medium, long, siren, alarm, stop, or duration_ms
```

Check your payload spelling or format.

If you see this after sending a number:
```
(No error - just plays the tone for that duration)
```

Your duration is within valid range (1-9999 ms).

## Advanced: Node-RED Integration

Create a flow with an inject node:

**Flow JSON:**
```json
[
  {
    "id": "audio_test",
    "type": "inject",
    "name": "Test Alert",
    "topic": "gravelping/s3/audio/test",
    "payload": "pattern",
    "payloadType": "str",
    "repeat": "",
    "crontab": "",
    "once": false,
    "onceDelay": 0.1,
    "x": 150,
    "y": 100,
    "wires": [["mqtt_out"]]
  },
  {
    "id": "mqtt_out",
    "type": "mqtt out",
    "broker": "your_mqtt_broker",
    "topic": "",
    "qos": "0",
    "retain": "false",
    "x": 350,
    "y": 100,
    "wires": []
  }
]
```

## Tips for Finding the Best Alert Pattern

1. **Start simple:** Test `short`, `medium`, `long` to see what gets your attention
2. **Test distance:** Walk to where you'll actually hear it (inside house, garage, etc.)
3. **Consider ambient noise:** Longer beeps may work better in noisy environments
4. **Test urgency:** The `pattern` command has good attention-grabbing rhythm
5. **Custom patterns:** Use Home Assistant automations for complex sequences

## Troubleshooting

### Commands Not Working

1. **Check MQTT connection:**
   - Serial output should show: `[MQTT] ✓ Subscribed to audio test topic`
   - If not subscribed, check MQTT broker connectivity

2. **Verify topic:**
   - Must be exactly: `gravelping/s3/audio/test`
   - Case sensitive!

3. **Check payload:**
   - Commands are case-sensitive
   - Numbers must be 1-9999 (no decimals)

### Buzzer Not Making Sound

1. **Verify audio system initialized:**
   - Serial should show: `[INIT] ✓ Audio ready on GPIO6`

2. **Check wiring:**
   - See [Active-Buzzer-Audio-Guide.md](Active-Buzzer-Audio-Guide.md) for wiring diagram

3. **Test with `long` command:**
   ```bash
   mosquitto_pub -h 192.168.1.175 -t "gravelping/s3/audio/test" -m "long"
   ```
   - 1 second beep is easier to hear than short beeps

### Multiple Beeps When Sending One Command

- This is normal for pattern commands (`siren`, `alarm`, `pattern`)
- They're designed to repeat
- Use `stop` command to silence

## Integration with Existing Alerts

The audio test uses the same functions as the automatic HA-unavailable alert, so:
- ✅ If it works via MQTT, it'll work automatically
- ✅ Test your buzzer setup without triggering false alarms
- ✅ Tune the alert pattern before an actual outage

## Quick Test Script (Bash/Linux/macOS)

Save as `test_audio.sh`:

```bash
#!/bin/bash
BROKER="192.168.1.175"
TOPIC="gravelping/s3/audio/test"

echo "Testing predefined patterns..."
mosquitto_pub -h $BROKER -t $TOPIC -m "startup"
sleep 2
mosquitto_pub -h $BROKER -t $TOPIC -m "pattern"
sleep 3
mosquitto_pub -h $BROKER -t $TOPIC -m "alarm"
sleep 5

echo "Testing duration range..."
for dur in 100 200 500 1000 2000; do
    echo "Testing ${dur}ms beep..."
    mosquitto_pub -h $BROKER -t $TOPIC -m "${dur}"
    sleep 1.5
done

echo "Test complete!"
```

Make executable: `chmod +x test_audio.sh`  
Run: `./test_audio.sh`

## Quick Test Script (PowerShell/Windows)

Save as `test_audio.ps1`:

```powershell
$broker = "192.168.1.175"
$topic = "gravelping/s3/audio/test"

Write-Host "Testing predefined patterns..."
mosquitto_pub -h $broker -t $topic -m "startup"
Start-Sleep -Seconds 2
mosquitto_pub -h $broker -t $topic -m "pattern"
Start-Sleep -Seconds 3
mosquitto_pub -h $broker -t $topic -m "alarm"
Start-Sleep -Seconds 5

Write-Host "Testing duration range..."
foreach ($dur in @(100, 200, 500, 1000, 2000)) {
    Write-Host "Testing ${dur}ms beep..."
    mosquitto_pub -h $broker -t $topic -m "$dur"
    Start-Sleep -Milliseconds 1500
}

Write-Host "Test complete!"
```

Run: `.\test_audio.ps1`

---

**Related Documentation:**
- [Active-Buzzer-Audio-Guide.md](Active-Buzzer-Audio-Guide.md) - Hardware setup and wiring guide
- [HA-Heartbeat-Monitoring.md](HA-Heartbeat-Monitoring.md) - How HA unavailable detection triggers audio alerts

