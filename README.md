# GravelPing – Driveway Detection Transmitter

Battery-friendly driveway ingress sensor built on an **ESP32-C6 SuperMini** paired with a **DX-LR02 UART LoRa module** and an **EMX LP D-TEK loop detector**. The transmitter sleeps until the loop detector's relay closes, wakes both radios, pushes a JSON status frame, then drops everything back into deep sleep.

## Hardware blocks

| Component | Notes |
| --- | --- |
| ESP32-C6 SuperMini | Runs Arduino firmware from this repo. I'm powering mine through a DC-DC buck converter using the same 12v LiFePO4 battery that powers my loop detector, but the [ESP32-C6 SuperMini](https://www.espboards.dev/esp32/esp32-c6-super-mini/) also natively supports a lithium battery & charging. |
| DX-LR02 LoRa UART module | [UART-based LoRa radio](https://en.szdx-smart.com/EN/tczw/114.html). Choose the module with the correct LoRa frequency for your location (433/900MHz). The AUX pin is LOW when idle and HIGH while busy. We keep the module powered and use `AT+SLEEP0` / 4-byte wake bursts (per docs §2.3.4/§5.1.8) for sub-100 µA sleep current. 
| Vehicle Detector | Any driveway loop detector, magnetic detector, etc. can be used as long as it has a dry contact relay output. I use the [EMX LP D-TEK vehicle loop detector](https://www.emxaccesscontrolsensors.com/product/lp-d-tek/), which provides two isolated relay contacts. Relay 1 = "vehicle present." Relay 2 = "loop fault" (configure the detector for "Fail Secure" to get this Relay 2 behavior).  ||

## Pinout

All pins are referenced to the SuperMini silk labels. Choose RTC-capable pins for wake-up (GPIO4/5 are RTC on the C6). Adjust if your board exposes a different mapping.

| Function | ESP32-C6 pin | External connection |
| --- | --- | --- |
| `RELAY1_PIN` | GPIO4 (RTC) | EMX relay 1 NO contact. Relay COM → ESP GND. Internal pull-up keeps the line HIGH while open. |
| `RELAY2_PIN` | GPIO5 (RTC) | EMX relay 2 NO contact (loop fault / fail-secure alarm). |
| `LORA_UART_TX_PIN` | GPIO19 | UART TX → LR-02 RX. |
| `LORA_UART_RX_PIN` | GPIO18 | UART RX ← LR-02 TX. |
| `LORA_AUX_PIN` | GPIO9 | LR-02 AUX/busy output. LOW means the module is idle/ready; HIGH means it's busy or still waking. |
| 3V3 / VBAT | — | Battery/regulator output feeding both ESP32-C6 and LR-02. |
| GND | — | Common ground between ESP, LR-02, loop sensor relay commons, and the battery negative. |

**Relay wiring tip:** Power the EMX detector per its manual (12–24V AC/DC). Only the relay contacts leave the housing. Keep the relay commons tied to the ESP ground and land the NO contacts on GPIO4/GPIO5.

## Firmware behavior

1. **Deep sleep by default.** The ESP configures GPIO4 as an RTC wake source with an internal pull-up.
2. **Relay 1 closes.** The contact pulls GPIO4 LOW and wakes the MCU.
3. **Validation + debounce.** The firmware waits ~40 ms to ensure the relay is really closed.
4. **Wake LR-02.** A 4-byte dummy burst is sent on UART (per §2.3.4 of the spec) so the module exits its sleep state, then `AUX` is polled until it sits LOW (= idle).
5. **Transmit frame.** A JSON payload like `{"device":"gravelping-tx","event":"car_enter","relay":1,"seq":42}` is printed over UART at 9600 bps. `AUX` rises HIGH while the radio is busy and drops LOW once the frame clears.
6. **Return LR-02 to sleep.** While still in AT mode the ESP issues `AT+SLEEP0` (per §5.1.8 of the serial guide), keeping the module powered but drawing only tens of µA. Finally the ESP re-arms its RTC wake and calls `esp_deep_sleep_start()`.

Relay 2 is wired to the loop detector's fault relay (set the LP D-TEK to "Fail Secure" so the contact closes on a fault). When it trips we transmit a `"loop_fault"` event with `"relay":2`, letting the receiver surface loop wiring issues immediately.

## Building and flashing

1. Install [PlatformIO](https://platformio.org/) or [pioarduino](https://github.com/pioarduino/platform-espressif32). (or minimally the [PIO Core CLI](https://docs.platformio.org/en/latest/core/installation/index.html) tools)
    - GravelPing internally uses the pioarduino fork of PlatformIO for modern device support on Arduino (since PlatformIO is run by morons). However, for the tooling itself, the original PIO works fine.
2. Connect the ESP32-C6 SuperMini via USB (CDC is enabled by default).
3. From this folder run `pio run -t upload -e esp32c6`.
4. Use `pio device monitor -b 115200` to watch debug prints when forcing relay closures manually.

The `platformio.ini` points to the upstream Arduino core fork that includes ESP32-C6 support—do not change unless you know you need a different commit.

## USB debug logging

- Runtime status messages (wake causes, LR-02 state, etc.) are emitted on the ESP32-C6's native USB CDC port at **115200 bps**.
- Logging is enabled by default via the `GP_DEBUG_SERIAL` compile-time flag. This keeps development simple—just plug in USB and run `pio device monitor` to see traffic.
- For battery-powered scenarios, disable the logger by setting `-D GP_DEBUG_SERIAL=0` in the platform.ini `build_flags` for the environment. The firmware excludes all debug calls when the flag is set to `0`.

## Power + mechanical notes

- The LR-02 stays powered from the same regulator as the ESP32-C6; firmware uses the documented `AT+SLEEP0` command so the module drops to ~60 µA. 
- AUX (pin 5) is actively driven LOW when idle and HIGH while busy/waking. Route it to GPIO9 so the ESP can synchronize UART traffic with the radio.
- When waking from sleep, the LR-02 expects four arbitrary bytes on UART before it will respond to commands—see §2.3.4 in the module spec.
- House the electronics in a weatherproof box near the loop detector junction, and keep the LR antennas away from the loop wires to avoid coupling.

## Roadmap (potential)

- Optional battery health/solar charging performance reporting (heartbeat timer wake).
- Distinguish vehicle exit vs. entry events (e.g. by adding a second loop or timing logic).
- CRC or encryption once the receive-side protocol is defined.