# GravelPing – Driveway Transmitter

Battery-friendly driveway ingress sensor built on an **ESP32-C6 SuperMini** paired with a **DX-LR02 UART LoRa module** and an **EMX LP D-TEK loop detector**. The transmitter sleeps until the loop detector's relay closes, wakes both radios, pushes a JSON status frame, then drops everything back into deep sleep.

## Hardware blocks

| Component | Notes |
| --- | --- |
| ESP32-C6 SuperMini | Runs Arduino firmware from this repo. Needs a LiFePO4/Li-ion pack plus a low-Iq regulator. |
| EMX LP D-TEK loop sensor | Provides two isolated relay contacts. Relay 1 = "vehicle present" (used today). Relay 2 reserved for future behaviors. |
| DX-LR02 (900T22D) LoRa UART module | UART-based radio. AUX pin asserts HIGH when the module is awake/idle. We hard-cut power with an enable pin so it idles at ~0 µA. |
| Load switch / P-MOSFET | Lets the ESP disable the LR-02 entirely in sleep. Gate is driven by `LORA_ENABLE_PIN`. |
| Battery + charge circuitry | Size for the expected duty cycle. The firmware only wakes on relay edges, so idle current is dominated by leakage. |

## Pinout

All pins are referenced to the SuperMini silk labels. Choose RTC-capable pins for wake-up (GPIO4/5 are RTC on the C6). Adjust if your board exposes a different mapping.

| Function | ESP32-C6 pin | External connection |
| --- | --- | --- |
| `RELAY1_PIN` | GPIO4 (RTC) | EMX relay 1 NO contact. Relay COM → ESP GND. Internal pull-up keeps the line HIGH while open. |
| `RELAY2_PIN` | GPIO5 (RTC) | EMX relay 2 NO contact (placeholder for future events). |
| `LORA_UART_TX_PIN` | GPIO19 | UART TX → LR-02 RX. |
| `LORA_UART_RX_PIN` | GPIO18 | UART RX ← LR-02 TX. |
| `LORA_AUX_PIN` | GPIO9 | LR-02 AUX/busy output. HIGH when module is awake/idle. Pulled down when the radio is unpowered. |
| `LORA_ENABLE_PIN` | GPIO8 | Drives the high-side switch (or LR-02 EN) to cut/en restore power. Keep LOW during deep sleep. |
| 3V3 / VBAT | — | Battery/regulator output feeding both ESP32-C6 and LR-02 (through the load switch). |
| GND | — | Common ground between ESP, LR-02, loop sensor relay commons, and the battery negative. |

**Relay wiring tip:** Power the EMX detector per its manual (typically 12–24 VAC/DC). Only the relay contacts leave the housing. Keep the relay commons tied to the ESP ground and land the NO contacts on GPIO4/GPIO5.

## Firmware behavior

1. **Deep sleep by default.** The ESP configures GPIO4 as an RTC wake source with an internal pull-up.
2. **Relay 1 closes.** The contact pulls GPIO4 LOW and wakes the MCU.
3. **Validation + debounce.** The firmware waits ~40 ms to ensure the relay is really closed.
4. **Wake LR-02.** `LORA_ENABLE_PIN` drives the LoRa module's load switch, then the code waits for `AUX` to report HIGH.
5. **Transmit frame.** A JSON payload like `{"device":"gravelping-tx","event":"car_enter","relay":1,"seq":42}` is printed over UART at 9600 bps. `AUX` is polled to confirm the frame cleared.
6. **Shut everything down.** LR-02 power is cut, the wakeup source is re-armed, and `esp_deep_sleep_start()` is called.

Relay 2 is already debounced and monitored so we can add a distinct transmission later without touching the sleep plumbing.

## Building and flashing

1. Install [PlatformIO](https://platformio.org/).
2. Connect the ESP32-C6 SuperMini via USB (CDC is enabled by default).
3. From this folder run `pio run -t upload -e esp32c6`.
4. Use `pio device monitor -b 115200` to watch debug prints when forcing relay closures manually.

The `platformio.ini` points to the upstream Arduino core fork that includes ESP32-C6 support—do not change unless you know you need a different commit.

## Power + mechanical notes

- Keep the LR-02 supply behind the same switch as `LORA_ENABLE_PIN`, otherwise its idle current will dominate battery life.
- AUX is left floating when the module is off, so we enable the ESP32's pulldown to force a deterministic LOW level.
- If your load switch inverts polarity, adjust `LORA_ENABLE_PIN` logic accordingly.
- House the electronics in a weatherproof box near the loop detector junction, and keep the LR antennas away from the loop wires to avoid coupling.

## Roadmap

- Distinct message for Relay 2 (exit vs. enter, tamper input, etc.).
- Optional heartbeat timer wake to report battery health.
- CRC or encryption once the receive-side protocol is defined.
