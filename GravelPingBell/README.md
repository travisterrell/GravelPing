# GravelPingBell - Secondary Alert Controller

An ESP32-C6 based secondary alerting device for the GravelPing system. Listens for MQTT commands **from Home Assistant** and drives a DC load a fire alarm bell, siren, et. via a MOSFET. It can be used as the main notification mechanism, or as a secondary alert outdoors or any other location.

## Hardware

- **ESP32-C6 SuperMini**
- DC bell, siren, or other 12V load (I'm using NOS 6" fire alarm bell from eBay that can be heard around my property)
- N-channel MOSFET (e.g. IRLZ44N) wired between `BELL_PIN` and load ground
- 12V supply for the load; ESP powered separately via USB or 3.3V rail

## Wiring
```
        +12V
          │
     Bell/Siren (+)
          │
     Bell/Siren (−)
          │                    ┌── Cathode
          │              1N4007│
          └──────────────┤     │── Anode ──┐
                         │                 │
                    MOSFET Drain           │
                         │                 │
                    MOSFET Source ─────────┘
                         │
                        GND (common with ESP32)
                         
MOSFET Gate ── 330Ω ── GPIO (BELL_PIN) on ESP32
MOSFET Gate ── 10kΩ ── GND
```
(The flyback diode (e.g. 1N4007) across the load is not required if the load isn't inductive (like a motor-driven bell or relay coil are))


## MQTT Topics

| Topic | Direction | Payload | Action |
|---|---|---|---|
| `gravelping/bell/ring/single` | Subscribe | any | Ring once for `RING_SINGLE_MS` ms |
| `gravelping/bell/ring/pattern` | Subscribe | any | Ring `RING_PATTERN_COUNT` short bursts |
| `gravelping/bell/status` | Publish | `idle` / `ringing` | Current state |

## Home Assistant

MQTT Discovery is published on boot. HA will automatically create a **GravelPingBell** device with:
- **Ring Once** button
- **Ring Pattern** button
- **Bell Status** sensor

## Configuration

All timing is set via build flags in `platformio.ini` — no changes needed on the HA side.

| Flag | Default | Description |
|---|---|---|
| `BELL_PIN` | `3` | GPIO driving the MOSFET gate |
| `RING_SINGLE_MS` | `1500` | Single ring duration (ms) |
| `RING_PATTERN_COUNT` | `3` | Number of bursts in pattern |
| `RING_PATTERN_ON_MS` | `300` | On time per burst (ms) |
| `RING_PATTERN_OFF_MS` | `200` | Off gap between bursts (ms) |

## LED Indicators

| Color | Meaning |
|---|---|
| Cyan solid | Boot |
| Yellow | WiFi connecting |
| Magenta | MQTT connecting |
| Green dim | Idle / connected |
| Blue flash | Command received |
| Red strobe | Load active (ringing) |
| Red flash ×2 | Connection error |
