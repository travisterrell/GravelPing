# Battery Voltage Monitoring Implementation

## Overview
Added battery voltage monitoring to the GravelPing transmitter to track LiFePO4 4S battery health and prevent unexpected shutdowns.

## Hardware

### Voltage Sensor Module
- **Type**: 5x voltage divider module
- **Input Range**: 0-25V (suitable for LiFePO4 4S: 9.5V-14.6V)
- **Output Range**: 0-5V (scaled to 0-3.3V max, ESP32 safe)
- **Connection**: Voltage divider output → ESP32-C6 GPIO2 (ADC1_CH1)

### ESP32-C6 ADC Configuration
- **Pin**: GPIO2 (ADC1_CH1)
- **Resolution**: 12-bit (0-4095 counts)
- **Attenuation**: ADC_11db (~0-3100mV range on ESP32-C6)
- **Sampling**: 10 samples averaged for stability

**Note**: The ESP32-C6 ADC with 11dB attenuation provides approximately 3100mV range (not 2500mV like older ESP32). This is sufficient for measuring up to 15.5V batteries (3.1V × 5 = 15.5V) after the voltage divider.

## Battery Specifications (LiFePO4 4S)

| Parameter | Voltage |
|-----------|---------|
| Nominal | 12.8V |
| Full Charge | 14.6V |
| Low Threshold | 10.5V |
| Critical Threshold | 9.5V |
| Cutoff | 10.0V (typical) |

## Implementation Details

### Transmitter (TX) Changes

#### Pin Configuration
```cpp
constexpr int PIN_VOLTAGE_SENSE = 2;  // ADC1_CH1 (GPIO2)
```

#### ADC Setup (in setupPins())
```cpp
analogReadResolution(12);                  // 12-bit resolution
analogSetAttenuation(ADC_11db);            // 11dB attenuation (~0-3100mV on ESP32-C6)
pinMode(PIN_VOLTAGE_SENSE, INPUT);         // Voltage divider input
```

#### Voltage Reading Function
- **Function**: `float readBatteryVoltage()`
- **Sampling**: 10 samples averaged with 10ms delay between samples
- **Conversion**:
  1. ADC reading → millivolts: `(reading / 4095) * 3100`
  2. millivolts → actual voltage: `(mV / 1000) * 5.0` (voltage divider ratio)
- **Precision**: Rounded to 0.1V
- **Debug Output**: Enabled in DEBUG_MODE

**Voltage Range Support:**
- ADC measures: 0-3.1V (with ADC_11db on ESP32-C6)
- After 5x voltage divider: Can measure batteries up to 15.5V
- LiFePO4 4S range: 9.5V-14.6V ✓ Fully supported

#### Voltage Thresholds
```cpp
constexpr float BATTERY_LOW_THRESHOLD      = 10.5;   // Low battery warning
constexpr float BATTERY_CRITICAL_THRESHOLD = 9.5;    // Critical battery level
```

#### Visual Indicators
- **Normal**: No special indication
- **Low Battery (≤10.5V)**: Yellow LED flash (2 times, 200ms on/off)
- **Critical (≤9.5V)**: Red LED flash (3 times, 200ms on/off)

#### Message Format
JSON message now includes battery voltage:
```json
{"event":"entry","seq":123,"vbat":12.3}
```

**Message Size**: 39 bytes (still fits in single LoRa packet, <41 byte limit)

### Receiver (RX) Changes

#### Message Parsing
- Added `vbat` field parsing from JSON
- Default value: 0.0 if not present (backward compatible)

#### MQTT Publishing
- **Topic**: `homeassistant/sensor/gravelping/battery_voltage/state`
- **Payload**: Battery voltage as string with 1 decimal place (e.g., "12.3")
- **Frequency**: Published with every LoRa message received

#### Home Assistant Discovery
Added battery voltage sensor with:
- **Device Class**: `voltage`
- **Unit**: `V` (volts)
- **State Class**: `measurement`
- **Icon**: `mdi:battery`
- **Unique ID**: `gravelping_battery_voltage`

## Serial Output Examples

### Transmitter
```
[BATTERY] ADC: 2048.3, mV: 1250.5, Voltage: 12.5V
[BATTERY] Voltage: 12.5V (OK)
[LORA] Sending: {"event":"entry","seq":123,"vbat":12.5}
```

### Low Battery Warning
```
[BATTERY] LOW: 10.3V (threshold: 10.5V)
```

### Critical Battery Warning
```
[BATTERY] CRITICAL: 9.4V (threshold: 9.5V)
```

### Receiver
```
[PARSED]
  Event:   entry
  Seq:     123
  VBat:    12.5V
>>> VEHICLE DETECTED <<<
[MQTT] ✓ Published battery voltage: 12.5V
```

## Calibration

If voltage readings are inaccurate, add a calibration factor:

```cpp
// In readBatteryVoltage() function
constexpr float CALIBRATION_FACTOR = 1.02;  // Adjust based on multimeter
voltage = voltage * CALIBRATION_FACTOR;
```

To calibrate:
1. Measure battery voltage with multimeter
2. Compare to ESP32 reading in serial output
3. Calculate: `CALIBRATION_FACTOR = actual_voltage / measured_voltage`
4. Update and recompile

## Home Assistant Integration

The battery voltage sensor will automatically appear in Home Assistant after the first message is received. You can:

1. **Create automations** based on battery voltage
2. **Monitor battery health** over time using history graphs
3. **Set up notifications** for low battery conditions

Example automation:
```yaml
automation:
  - alias: "GravelPing Low Battery Alert"
    trigger:
      platform: numeric_state
      entity_id: sensor.gravelping_battery_voltage
      below: 10.5
    action:
      service: notify.mobile_app
      data:
        message: "GravelPing battery is low ({{ states('sensor.gravelping_battery_voltage') }}V)"
```

## Power Consumption Impact

The voltage monitoring adds minimal power consumption:
- **ADC Reading**: ~100µs per sample (10 samples = 1ms total)
- **Additional Wake Time**: ~110ms (10 samples × 10ms delay + 1ms reading)
- **Impact**: Negligible compared to LoRa transmission time (~50-200ms)

The voltage reading is only performed when the device wakes to send a message, so it doesn't affect deep sleep current consumption.

## Troubleshooting

### Voltage Reads 0.0V
- Check GPIO2 connection to voltage divider output
- Verify voltage divider module is powered
- Check battery is connected to voltage divider input

### Voltage Inaccurate
- Verify 5x voltage divider ratio
- Check for loose connections
- Apply calibration factor (see Calibration section)
- Verify 11dB attenuation is set correctly

### Voltage Fluctuates
- Increase NUM_SAMPLES in `readBatteryVoltage()`
- Add capacitor to voltage divider output (0.1µF - 1µF)
- Check for poor battery connection

## Future Enhancements

1. **Battery Percentage**: Convert voltage to percentage estimate
2. **Voltage History**: Track voltage over time to predict battery life
3. **Low Battery Sleep**: Disable features or extend sleep time when battery is low
4. **Voltage Compensation**: Adjust LoRa transmit power based on battery level
