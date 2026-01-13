# Active Buzzer Audio Guide

## Overview

The GravelPingRX-S3 uses an **active buzzer** on GPIO6 for backup audio alerts. An active buzzer has a built-in oscillator and produces a loud fixed-frequency tone when powered - no PWM frequency control needed, just simple on/off.

## Hardware Setup

### What is an Active Buzzer?

**Active Buzzer:**
- Has built-in oscillator circuit
- Just needs power (GPIO HIGH = on, LOW = off)
- Produces fixed tone
- Very loud (85-95 dB typical)
- Simple 2-wire connection

### Pin Configuration
- **GPIO6**: Digital output to buzzer
- **Ground**: Common ground between ESP32-S3 and buzzer

### Wiring Options

**Option 1: Direct Connection (3.3v)**
```
ESP32-S3 GPIO6 ────► Buzzer Positive (+)
ESP32-S3 GND   ────► Buzzer Negative (-)
```

✅ **Use this if:**
- Your buzzer is rated for 3.3V
- Buzzer draws < 40mA (check datasheet or markings)
- Most small active buzzers work with this

**Option 2: Transistor Driver (Higher Current)**
```
ESP32-S3 5V         ──► Buzzer (+)
ESP32-S3 GND        ──┬─► Transistor Emitter (2N2222, etc.)
                      │
ESP32-S3 GPIO6 ─[1kΩ]─┴─► Transistor Base

Transistor Collector ──► Buzzer (-)
```

✅ **Use this if:**
- Buzzer needs 5V operation
- Buzzer draws > 40mA
- Want maximum volume

### Identifying Polarity

Most active buzzers have:
- **"+" or longer lead**: Positive terminal (to GPIO6 or 5V)
- **"-" or shorter lead**: Negative terminal (to GND or transistor)
- **Sticker/label side**: Usually shows polarity markings

## Software Functions

### Available Functions

```cpp
// Play tone for specified duration
playTone(duration);
// duration: milliseconds

// Stop any playing tone
stopTone();

// Pre-programmed alert pattern
playBeepPattern();  // 3 quick beeps + 1 long beep
```

### Current Implementation

**Startup Test (on boot):**
```cpp
playTone(100);  // 100ms beep
delay(50);
playTone(100);  // 100ms beep
```

**Alert Pattern (HA unavailable):**
```cpp
playBeepPattern();
// Three 150ms beeps with 100ms gaps
// One 500ms beep
```

### Customization Examples

**To disable startup beeps:**
Comment out lines in `setupAudio()`:
```cpp
// digitalWrite(PIN_BUZZER, HIGH);
// delay(100);
// digitalWrite(PIN_BUZZER, LOW);
```

**Create custom patterns:**
```cpp
void playCustomAlert() {
    // Rapid beeping
    for (int i = 0; i < 10; i++) {
        playTone(100);  // 100ms on
        delay(50);      // 50ms off
    }
}
```

**Morse code SOS:**
```cpp
void playSOSPattern() {
    // S: three short
    for (int i = 0; i < 3; i++) {
        playTone(100);
        delay(100);
    }
    delay(200);
    
    // O: three long
    for (int i = 0; i < 3; i++) {
        playTone(300);
        delay(100);
    }
    delay(200);
    
    // S: three short
    for (int i = 0; i < 3; i++) {
        playTone(100);
        delay(100);
    }
}
```

## Testing via MQTT

See [MQTT-Audio-Test-Commands.md](MQTT-Audio-Test-Commands.md) for full details.

**Quick tests:**
```bash
# Default alert pattern
mosquitto_pub -h YOUR_BROKER -t "gravelping/s3/audio/test" -m "pattern"

# Short beep
mosquitto_pub -h YOUR_BROKER -t "gravelping/s3/audio/test" -m "short"

# Siren effect
mosquitto_pub -h YOUR_BROKER -t "gravelping/s3/audio/test" -m "siren"

# Custom duration (milliseconds)
mosquitto_pub -h YOUR_BROKER -t "gravelping/s3/audio/test" -m "750"
```

## Current Integration

### Home Assistant Unavailable Detection

When a vehicle is detected and HA is unavailable:
1. System checks HA heartbeat timeout (35 seconds)
2. If HA is down, triggers: `playBeepPattern()`
3. Alert plays locally while still publishing to MQTT (for logging/recovery)

**To test this scenario:**
1. Stop Home Assistant or stop the heartbeat automation
2. Wait 35+ seconds for timeout
3. Trigger vehicle detection via LoRa transmitter
4. Buzzer should play alert pattern

### Startup Notification

On boot, plays two quick beeps to confirm audio is working.

## Troubleshooting

### No Sound

1. **Check polarity**: Swap buzzer wires if no sound
2. **Test with LED**: Replace buzzer with LED temporarily
   - LED lights up = circuit works, buzzer is bad/wrong type
   - LED doesn't light = wiring issue
3. **Verify GPIO**: Monitor serial output for `[AUDIO]` messages
4. **Power issue**: Check ESP32-S3 has adequate power supply

### Very Quiet Sound

1. **Wrong type**: You may have a passive piezo (needs PWM, different circuit)
2. **Voltage**: Try transistor circuit with 5V instead of 3.3V direct
3. **Bad buzzer**: Some cheap buzzers are duds - try another

### Continuous Buzzing (Won't Stop)

1. **Code issue**: Check for missing `stopTone()` calls
2. **Hardware stuck**: GPIO may be damaged - try different pin
3. **Buzzer problem**: Some buzzers have capacitance that holds charge

### MQTT Commands Not Working

1. **Check subscription**: Serial should show "✓ Subscribed to audio test topic"
2. **Verify topic**: Must be exactly `gravelping/s3/audio/test`
3. **MQTT connection**: Ensure MQTT broker is reachable
4. **Payload format**: Commands are case-sensitive

## Volume Control

Active buzzers have **fixed volume** determined by:
- Supply voltage (3.3V vs 5V)
- Buzzer design (85dB vs 95dB vs 105dB models)

**To change volume:**
- Use transistor circuit with 5V for louder
- Buy different buzzer (check dB rating - higher = louder)
- Add external volume control (potentiometer in series)

**Typical loudness:**
- 85 dB: Audible indoors, quiet outdoors
- 95 dB: Loud, good for outdoor use
- 105 dB: Very loud, can be annoying
- 110+ dB: Industrial/alarm level

For reference: Normal conversation ≈ 60 dB, Lawnmower ≈ 90 dB

## Shopping List

**Recommended Active Buzzers:**
- 5V active buzzer, 85-95 dB ($0.50-2.00)
- 12mm or 16mm diameter (common sizes)
- Look for "active buzzer" or "TMB12A05" style

**Where to Buy:**
- Amazon: Search "5V active buzzer" or "3.3V active buzzer"
- AliExpress: Very cheap ($0.20-1.00 each)
- Digikey/Mouser: Quality parts, more expensive

**Optional (for high-current buzzers):**
- 2N2222 or 2N3904 NPN transistor ($0.10)
- 1kΩ resistor ($0.01)

## Future Enhancements

### Phase 1: Current Implementation ✅
- [x] Simple on/off control
- [x] Startup test beeps
- [x] Integration with HA unavailable detection
- [x] MQTT test commands

### Phase 2: Enhanced Patterns 🚧
- [ ] Multiple alert patterns for different events
- [ ] Priority-based alerts (fault vs. vehicle)
- [ ] Configurable pattern via MQTT

### Phase 3: Advanced Options (Future)
- [ ] Switch to passive piezo + PWM for tone control
- [ ] I2S audio with WAV file playback
- [ ] Voice prompts ("Vehicle detected", "System fault")
- [ ] Integration with external siren/alarm system

---

**Note:** This guide assumes **active buzzer** (simple on/off). If you want frequency-controlled tones, you'll need a passive piezo element and PWM code (more complex, generally quieter without amplification).

