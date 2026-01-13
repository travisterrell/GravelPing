# PWM Audio Testing Guide

## Overview

The GravelPingRX-S3 now includes PWM tone generation on GPIO6 for audio backup alerts. This allows you to experiment with different tones and patterns without creating a separate test project.

## Hardware Setup

### Pin Configuration
- **GPIO6**: PWM audio output
- **Ground**: Common ground between ESP32-S3 and amplifier

### Amplifier Options

**Option 1: Simple Transistor Amplifier**
```
GPIO6 ──[1kΩ]──┬── NPN Base (2N2222, 2N3904, etc.)
               │
              GND

NPN Collector ── Speaker (+)
Speaker (-) ── GND
NPN Emitter ── GND
```

**Option 2: PAM8403 Amplifier Module** (Recommended)
```
ESP32-S3           PAM8403 Module
GPIO6    ────────► L-IN (Left Input)
GND      ────────► GND
                   R-IN (can tie to L-IN or leave floating)
                   
                   L+ / L- ──► Speaker
```

**Option 3: MAX98357A I2S Amplifier** (Future upgrade path)
- Better quality, requires I2S instead of PWM
- Keep for future enhancement

## Software Functions

### Available Functions

```cpp
// Play a single tone
playTone(frequency, duration);
// frequency: Hz (20-20000, typical 200-5000 for alerts)
// duration: milliseconds (0 = continuous until stopTone())

// Stop any playing tone
stopTone();

// Pre-programmed alert pattern
playBeepPattern();  // 3 quick beeps + 1 long beep
```

### Experimentation Examples

**Test different frequencies:**
```cpp
// Add to setup() or call via serial command
playTone(500, 200);   // Low beep
delay(100);
playTone(1000, 200);  // Medium beep
delay(100);
playTone(2000, 200);  // High beep
```

**Create custom patterns:**
```cpp
void playCustomAlert() {
    // Siren effect
    for (int i = 0; i < 3; i++) {
        for (uint32_t freq = 500; freq < 2000; freq += 50) {
            playTone(freq, 20);
        }
    }
    stopTone();
}
```

**Alarm sound:**
```cpp
void playAlarm() {
    for (int i = 0; i < 10; i++) {
        playTone(1000, 100);
        delay(50);
        playTone(1500, 100);
        delay(50);
    }
}
```

## Current Integration

### Startup Test
On boot, the system plays two quick beeps:
- 1000 Hz for 100ms
- 1500 Hz for 100ms

**To disable:** Comment out lines in `setupAudio()`:
```cpp
// playTone(1000, 100);
// delay(50);
// playTone(1500, 100);
```

### Home Assistant Unavailable
When a vehicle is detected and HA is unavailable, the system automatically calls:
```cpp
playBeepPattern();  // 3 quick beeps + 1 long beep
```

**To test this:**
1. Stop Home Assistant or stop the heartbeat automation
2. Wait 35 seconds for timeout
3. Trigger a vehicle detection via LoRa
4. Speaker should play alert pattern

## Testing via Serial Commands

You can add serial commands to test tones interactively. Add this to your `loop()` or network task:

```cpp
// In loop() or networkTask()
if (Serial.available()) {
    char cmd = Serial.read();
    switch(cmd) {
        case '1':
            playTone(500, 500);
            break;
        case '2':
            playTone(1000, 500);
            break;
        case '3':
            playTone(2000, 500);
            break;
        case '4':
            playBeepPattern();
            break;
        case 's':
            stopTone();
            break;
    }
}
```

Then send `1`, `2`, `3`, `4`, or `s` via serial monitor to test.

## PWM Technical Details

### Configuration
- **Resolution**: 8-bit (0-255 duty cycle)
- **Duty Cycle**: 50% (128/255) for square wave
- **Frequency Range**: 20 Hz - 40 kHz (typical use: 200-5000 Hz)

### ESP32-S3 LEDC API
```cpp
ledcAttach(pin, frequency, resolution);  // Configure PWM
ledcWrite(pin, duty_cycle);              // Set duty cycle (0-255 for 8-bit)
```

### Typical Alert Frequencies
| Frequency | Use Case |
|-----------|----------|
| 200-500 Hz | Low rumble, serious alerts |
| 500-1000 Hz | Standard beeps |
| 1000-2000 Hz | Attention-grabbing alerts |
| 2000-5000 Hz | Piercing alarms, emergencies |

## Volume Control

PWM amplitude is fixed at 3.3V. Volume control options:

**Hardware:**
1. **Potentiometer** between GPIO6 and amplifier input
2. **Amplifier gain** adjustment (if module has volume knob)
3. **Resistor divider** for fixed attenuation

**Software (future):**
- Vary duty cycle (not true volume, affects tone quality)
- Pulse width modulation envelope (requires more complex code)

## Next Steps

### Phase 1: Current Implementation ✅
- [x] Basic PWM tone generation
- [x] Startup test beeps
- [x] Integration with HA unavailable detection

### Phase 2: Enhanced Patterns
- [ ] Multiple alert patterns (different priorities)
- [ ] Serial command interface for testing
- [ ] RTTTL (ringtone) support for custom melodies

### Phase 3: I2S Audio (Future)
- [ ] Switch from PWM to I2S for better quality
- [ ] Play WAV files from SPIFFS/LittleFS
- [ ] Voice prompts ("Vehicle detected", "System error")

## Troubleshooting

### No Sound
1. Check wiring: GPIO6 → Amplifier input
2. Verify amplifier is powered
3. Check speaker connections
4. Monitor serial output for "[AUDIO]" messages
5. Try higher frequency (2000 Hz more audible than 500 Hz)

### Distorted Sound
1. Reduce frequency (stay 200-4000 Hz range)
2. Check amplifier gain (may be too high)
3. Verify 3.3V PWM input to amplifier (not 5V)

### Quiet Sound
1. Increase amplifier gain/volume
2. Use larger speaker (more efficient)
3. Try different amplifier module (PAM8403 is cheap but decent)

### Interferes with Other Functions
- PWM runs on Core 0 in network task
- Should not affect LoRa reception on Core 1
- If issues occur, move audio calls to separate task

## Example Amplifier Shopping List

**Budget Option ($1-2):**
- PAM8403 module (2x3W, cheap, good enough)
- 4Ω or 8Ω speaker (0.5W+)

**Better Option ($5-10):**
- PAM8610 module (2x10W, better quality)
- 8Ω speaker (2W+)

**Future Upgrade ($8-15):**
- MAX98357A I2S module (3W, excellent quality)
- Requires code rewrite for I2S

All available on Amazon, AliExpress, or eBay.
