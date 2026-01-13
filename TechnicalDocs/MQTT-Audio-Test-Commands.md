# MQTT Audio Test Commands

## Quick Reference

**MQTT Topic:** `gravelping/s3/audio/test`

Upload your updated firmware, then publish to this topic to test different sounds!

## Predefined Patterns

| Command | Description |
|---------|-------------|
| `pattern` | Default alert (3 quick beeps + 1 long beep) |
| `startup` | Two-tone startup sound (1kHz → 1.5kHz) |
| `low` | Single low tone (500Hz for 500ms) |
| `med` | Single medium tone (1000Hz for 500ms) |
| `high` | Single high tone (2000Hz for 500ms) |
| `siren` | Rising siren effect (500Hz → 2000Hz, 3 cycles) |
| `alarm` | Alternating alarm (1000Hz ↔ 1500Hz, 5 cycles) |
| `stop` | Stop any playing tone |

## Custom Tones

**Format:** `frequency,duration`

**Examples:**
- `1234,500` - Play 1234Hz for 500ms
- `440,1000` - Play 440Hz (musical A) for 1 second
- `2500,250` - Play 2500Hz for 250ms

**Limits:**
- Frequency: 1-19999 Hz (practical range: 200-5000 Hz)
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

# Custom tone
mosquitto_pub -h 192.168.1.175 -t "gravelping/s3/audio/test" -m "1500,300"

# Low beep
mosquitto_pub -h 192.168.1.175 -t "gravelping/s3/audio/test" -m "low"
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

## Testing Different Frequencies

Try these to find what works best with your speaker/amplifier:

```bash
# Very low (rumble)
mosquitto_pub -h 192.168.1.175 -t "gravelping/s3/audio/test" -m "200,500"

# Low (doorbell-ish)
mosquitto_pub -h 192.168.1.175 -t "gravelping/s3/audio/test" -m "500,500"

# Medium (standard beep)
mosquitto_pub -h 192.168.1.175 -t "gravelping/s3/audio/test" -m "1000,500"

# High (attention-grabbing)
mosquitto_pub -h 192.168.1.175 -t "gravelping/s3/audio/test" -m "2000,500"

# Very high (piercing)
mosquitto_pub -h 192.168.1.175 -t "gravelping/s3/audio/test" -m "4000,500"
```

## Musical Notes (for fun)

Standard musical notes:

| Note | Frequency | Command |
|------|-----------|---------|
| A4 (concert pitch) | 440 Hz | `440,500` |
| C4 (middle C) | 261.63 Hz | `262,500` |
| E4 | 329.63 Hz | `330,500` |
| G4 | 392 Hz | `392,500` |
| C5 | 523.25 Hz | `523,500` |

Play a simple melody:
```bash
mosquitto_pub -h 192.168.1.175 -t "gravelping/s3/audio/test" -m "262,300"
sleep 0.4
mosquitto_pub -h 192.168.1.175 -t "gravelping/s3/audio/test" -m "330,300"
sleep 0.4
mosquitto_pub -h 192.168.1.175 -t "gravelping/s3/audio/test" -m "392,300"
sleep 0.4
mosquitto_pub -h 192.168.1.175 -t "gravelping/s3/audio/test" -m "523,600"
```

## Monitoring Results

Watch the serial output to see what's happening:

```
[AUDIO] Test command received: pattern
[AUDIO] Playing alert pattern...
[AUDIO] Alert pattern complete
```

Or for custom tones:
```
[AUDIO] Test command received: 1234,500
[AUDIO] Playing custom tone: 1234 Hz for 500 ms
```

## Error Messages

If you see this:
```
[AUDIO] Unknown command. Valid: pattern, startup, low, med, high, siren, alarm, stop, or freq,duration
```

Check your payload spelling or format.

If you see this:
```
[AUDIO] Invalid frequency or duration
```

Your frequency or duration is out of range (freq: 1-19999, duration: 1-9999).

## Advanced: Node-RED Integration

Create a flow with an inject node:

```json
[
    {
        "id": "audio_test",
        "type": "inject",
        "name": "Test Pattern",
        "props": [
            {
                "p": "payload"
            }
        ],
        "repeat": "",
        "crontab": "",
        "once": false,
        "payload": "pattern",
        "payloadType": "str",
        "topic": "gravelping/s3/audio/test"
    },
    {
        "id": "mqtt_out",
        "type": "mqtt out",
        "broker": "your_mqtt_broker",
        "topic": "",
        "qos": "0",
        "retain": "false"
    }
]
```

## Tips for Finding the Right Sound

1. **Start with predefined patterns** - Try `pattern`, `alarm`, `siren`
2. **Test frequency range** - Start at 1000Hz, go up/down to find sweet spot
3. **Adjust duration** - 200-500ms is usually good for alerts
4. **Consider environment** - Outdoor may need lower frequencies
5. **Volume test** - Use `low` first, then increase frequency if too quiet

## Integration with Existing Alerts

The audio test uses the same functions as the automatic HA-unavailable alert, so:
- ✅ If it works via MQTT, it'll work automatically
- ✅ Test your speaker setup without triggering false alarms
- ✅ Tune the sound before an actual outage

## Quick Test Script

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

echo "Testing frequency range..."
for freq in 500 1000 1500 2000 2500; do
    echo "Testing ${freq}Hz..."
    mosquitto_pub -h $BROKER -t $TOPIC -m "${freq},300"
    sleep 1
done

echo "Test complete!"
```

Make executable: `chmod +x test_audio.sh`  
Run: `./test_audio.sh`
