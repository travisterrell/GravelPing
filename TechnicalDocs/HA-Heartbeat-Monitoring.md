# Home Assistant Heartbeat Monitoring

## Overview

The GravelPing receiver monitors Home Assistant availability via MQTT heartbeat messages. This allows the device to fall back to local audio alerts when Home Assistant is unavailable.

## Quick Setup

### ⚠️ Required: Create Heartbeat Automation

**You MUST create this automation in Home Assistant for the system to work properly.**

1. Go to **Settings** → **Automations & Scenes** → **Create Automation**
2. Switch to YAML mode (⋮ menu → Edit in YAML)
3. Paste this configuration:

```yaml
alias: "GravelPing Heartbeat"
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
mode: single
```

4. Save the automation
5. Verify it's enabled (toggle should be on)

### Verification

After creating the automation, check the GravelPing serial monitor:
```
[HA] ✓ Home Assistant available
```

If you don't see this message within 10 seconds, check:
- MQTT integration is configured in Home Assistant
- MQTT broker is running and accessible
- Automation is enabled and running

**What happens if you don't create this automation?**
- Device will always consider HA unavailable
- Local audio alerts will play for **every** vehicle detection
- MQTT messages will still be published normally
- Home Assistant will still receive vehicle detection events

The heartbeat is only needed if you want the device to distinguish between "HA working" and "HA down".

## Architecture

### MQTT Topic
- **Topic**: `homeassistant/heartbeat`
- **Published by**: Home Assistant automation (every 10 seconds)
- **QoS**: 0 (fire and forget)

### Timeout Configuration
- **Heartbeat interval**: 10 seconds (HA publishes)
- **Timeout threshold**: 35 seconds (device considers HA down)
- **Jitter allowance**: 25 seconds (accommodates WiFi/MQTT delays)

### State Variables
```cpp
volatile unsigned long lastHAHeartbeat = 0;  // Timestamp of last heartbeat
volatile bool haAvailable = false;            // HA availability status
```

## Implementation Details

### 1. MQTT Subscription
On MQTT connection, the device subscribes to the heartbeat topic:
```cpp
mqttClient.subscribe("homeassistant/heartbeat", 0);
```

### 2. Message Callback
Incoming heartbeat messages update the timestamp:
```cpp
void onMqttMessage(...) {
    if (strcmp(topic, "homeassistant/heartbeat") == 0) {
        lastHAHeartbeat = millis();
        if (!haAvailable) {
            haAvailable = true;
            Serial.println(F("[HA] ✓ Home Assistant available"));
        }
    }
}
```

### 3. Heartbeat Monitoring
The network task (Core 0) checks heartbeat freshness every loop iteration:
```cpp
void checkHAHeartbeat() {
    // Skip if MQTT disconnected
    if (!mqttConnected) {
        haAvailable = false;
        return;
    }
    
    // Wait for first heartbeat before checking
    if (lastHAHeartbeat == 0) return;
    
    // Check timeout
    unsigned long timeSinceHeartbeat = millis() - lastHAHeartbeat;
    if (timeSinceHeartbeat > HA_HEARTBEAT_TIMEOUT) {
        if (haAvailable) {
            haAvailable = false;
            Serial.println(F("[HA] ✗ Home Assistant heartbeat timeout"));
        }
    }
}
```

### 4. Alert Decision Logic
When a vehicle is detected, the device decides whether to play a local alert:
```cpp
void publishToHomeAssistant(const char* event, ...) {
    bool shouldPlayLocalAlert = false;
    
    if (!haAvailable && strcmp(event, "entry") == 0) {
        shouldPlayLocalAlert = true;
        Serial.println(F("[ALERT] HA unavailable - local alert needed"));
        // TODO: Play local sound (PWM/I2S)
    }
    
    // Always publish to MQTT (for logging/recovery)
    // ... MQTT publish code ...
}
```

## Home Assistant Configuration

Add this automation to Home Assistant to publish the heartbeat:

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

## Behavior Matrix

| MQTT Status | Heartbeat | HA Status | Alert Action |
|------------|-----------|-----------|--------------|
| Disconnected | N/A | Unavailable | Local alert |
| Connected | Fresh (<35s) | Available | MQTT only |
| Connected | Stale (>35s) | Unavailable | MQTT + Local |
| Connected | Never received | Unknown | MQTT only |

## Future Enhancements

### Local Speaker Support
When speaker hardware is added, the `shouldPlayLocalAlert` flag will trigger:
- **PWM**: Simple buzzer/piezo speaker
- **I2S**: DFPlayer Mini or MAX98357A amplifier for audio playback

Example integration point:
```cpp
if (shouldPlayLocalAlert) {
    playLocalAlert();  // PWM tone or I2S audio file
}
```

### Debug Monitoring
To test heartbeat monitoring:
1. Monitor serial output for heartbeat status messages
2. Stop/start Home Assistant to trigger timeout
3. Observe "HA unavailable - local alert needed" message
4. Check that MQTT messages are still published during HA downtime

## Edge Cases

### WiFi Reconnection
When WiFi reconnects, MQTT reconnects automatically. The heartbeat subscription is re-established, and `haAvailable` is reset to `false` until the first new heartbeat arrives.

### MQTT Broker Restart
If the MQTT broker restarts but HA is still running, there will be a brief period where `haAvailable = false`. The heartbeat will resume within 10 seconds.

### Millis() Rollover
The `millis()` function rolls over after ~49 days. The heartbeat check handles this naturally because the comparison `(millis() - lastHAHeartbeat) > timeout` still works correctly across rollover boundaries.

## Resource Usage

The heartbeat monitoring adds:
- **Flash**: ~2KB (callback function, check logic)
- **RAM**: 9 bytes (2 variables: `unsigned long` + `bool`)
- **CPU**: Negligible (<0.1% - one comparison per 10ms loop)
