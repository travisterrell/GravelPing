#include <Arduino.h>
#include <driver/rtc_io.h>
#include <esp_sleep.h>
#include <esp_err.h>
#include <cstring>

constexpr gpio_num_t RELAY1_PIN = GPIO_NUM_4;   // EMX loop sensor relay 1 (vehicle present)
constexpr gpio_num_t RELAY2_PIN = GPIO_NUM_5;   // EMX loop sensor relay 2 (future use)
constexpr gpio_num_t LORA_AUX_PIN = GPIO_NUM_9;    // Reads AUX/busy signal from the LR-02
constexpr int LORA_UART_RX_PIN = 18; // ESP32-C6 RX <- LR-02 TX
constexpr int LORA_UART_TX_PIN = 19; // ESP32-C6 TX -> LR-02 RX
constexpr uint32_t LORA_BAUD = 9600;
constexpr uint32_t AUX_READY_TIMEOUT_MS = 2000;
constexpr uint32_t AUX_STATE_STABLE_MS = 20;
constexpr uint32_t RELAY_ACTIVE_MIN_MS = 40;
constexpr uint32_t LORA_WAKE_GUARD_MS = 20;
constexpr size_t LORA_WAKE_BURST_LEN = 4;
constexpr uint8_t LORA_WAKE_BYTE = 0x00;
constexpr uint32_t AT_GUARD_TIME_MS = 50;
constexpr uint32_t AT_RESPONSE_TIMEOUT_MS = 250;
constexpr char DEVICE_NAME[] = "gravelping-tx";

RTC_DATA_ATTR uint32_t relay1EventCount = 0;

HardwareSerial LoraSerial(1);

constexpr uint64_t RELAY_WAKE_MASK = (1ULL << RELAY1_PIN) | (1ULL << RELAY2_PIN);

bool g_atModeActive = false;
bool g_loraSleeping = false;

bool waitForPinState(uint8_t pin, uint8_t targetLevel, uint32_t stableMs, uint32_t timeoutMs) {
    const uint32_t start = millis();
    uint32_t stableStart = 0;
    while ((millis() - start) < timeoutMs) {
        if (digitalRead(pin) == targetLevel) {
            if (stableStart == 0) {
                stableStart = millis();
            }
            if (millis() - stableStart >= stableMs) {
                return true;
            }
        } else {
            stableStart = 0;
        }
        delay(5);
    }
    return false;
}

void beginLoraSerial() {
    static bool initialized = false;
    if (!initialized) {
        LoraSerial.begin(LORA_BAUD, SERIAL_8N1, LORA_UART_RX_PIN, LORA_UART_TX_PIN);
        initialized = true;
    }
}

void drainLoraSerial() {
    while (LoraSerial.available()) {
        LoraSerial.read();
    }
}

bool waitForAuxLow(uint32_t timeoutMs) {
    return waitForPinState(LORA_AUX_PIN, LOW, AUX_STATE_STABLE_MS, timeoutMs);
}

bool waitForAuxHigh(uint32_t timeoutMs) {
    return waitForPinState(LORA_AUX_PIN, HIGH, AUX_STATE_STABLE_MS, timeoutMs);
}

void sendWakeBurst() {
    uint8_t burst[LORA_WAKE_BURST_LEN];
    memset(burst, LORA_WAKE_BYTE, sizeof(burst));
    LoraSerial.write(burst, sizeof(burst));
    LoraSerial.flush();
    delay(LORA_WAKE_GUARD_MS);
}

enum class AtFeedback { None, Entry, Exit };

AtFeedback readAtFeedback(uint32_t timeoutMs) {
    String line;
    const uint32_t start = millis();
    while ((millis() - start) < timeoutMs) {
        while (LoraSerial.available()) {
            const char c = LoraSerial.read();
            if (c == '\r' || c == '\n') {
                if (line.length() > 0) {
                    if (line.indexOf("Entry") != -1) {
                        return AtFeedback::Entry;
                    }
                    if (line.indexOf("Exit") != -1) {
                        return AtFeedback::Exit;
                    }
                    line = "";
                }
            } else {
                line += c;
            }
        }
    }
    return AtFeedback::None;
}

bool toggleAtMode() {
    delay(AT_GUARD_TIME_MS);
    drainLoraSerial();
    LoraSerial.print("+++");
    LoraSerial.flush();
    const AtFeedback fb = readAtFeedback(AT_RESPONSE_TIMEOUT_MS);
    if (fb == AtFeedback::Entry) {
        g_atModeActive = true;
        return true;
    }
    if (fb == AtFeedback::Exit) {
        g_atModeActive = false;
        return true;
    }
    return false;
}

bool ensureAtMode(bool enable) {
    beginLoraSerial();
    if (g_atModeActive == enable) {
        return true;
    }
    for (int attempt = 0; attempt < 4; ++attempt) {
        if (toggleAtMode()) {
            if (g_atModeActive == enable) {
                return true;
            }
            continue;
        }
        sendWakeBurst();
        waitForAuxLow(AUX_READY_TIMEOUT_MS);
    }
    return g_atModeActive == enable;
}

bool waitForResponseToken(const char* token, uint32_t timeoutMs) {
    String line;
    const uint32_t start = millis();
    while ((millis() - start) < timeoutMs) {
        while (LoraSerial.available()) {
            const char c = LoraSerial.read();
            if (c == '\r' || c == '\n') {
                if (line.length() > 0) {
                    if (line.indexOf(token) != -1) {
                        return true;
                    }
                    if (line.indexOf("ERROR") != -1) {
                        return false;
                    }
                    line = "";
                }
            } else {
                line += c;
            }
        }
    }
    return false;
}

void wakeLoraFromSleep() {
    beginLoraSerial();
    sendWakeBurst();
    if (!waitForAuxLow(AUX_READY_TIMEOUT_MS)) {
        Serial.println("[LR02] AUX failed to indicate idle after wake burst.");
    }
    g_loraSleeping = false;
    ensureAtMode(false);
}

void enterLoraSleep() {
    beginLoraSerial();
    if (!ensureAtMode(true)) {
        Serial.println("[LR02] Could not enter AT mode to request sleep.");
        return;
    }
    drainLoraSerial();
    LoraSerial.print("AT+SLEEP0\r\n");
    LoraSerial.flush();
    if (!waitForResponseToken("OK", AT_RESPONSE_TIMEOUT_MS)) {
        Serial.println("[LR02] No OK received after AT+SLEEP0.");
    }
    g_loraSleeping = true;
}

void ensureLoraSleeping() {
    if (g_loraSleeping) {
        return;
    }
    wakeLoraFromSleep();
    enterLoraSleep();
}

void configureRelayInputs() {
    pinMode(RELAY1_PIN, INPUT_PULLUP);
    pinMode(RELAY2_PIN, INPUT_PULLUP);

    if (rtc_gpio_is_valid_gpio(RELAY1_PIN)) {
        rtc_gpio_pulldown_dis(RELAY1_PIN);
        rtc_gpio_pullup_en(RELAY1_PIN);
    } else {
        Serial.println("Warning: RELAY1_PIN is not RTC-capable; deep-sleep wake may fail.");
    }

    if (rtc_gpio_is_valid_gpio(RELAY2_PIN)) {
        rtc_gpio_pulldown_dis(RELAY2_PIN);
        rtc_gpio_pullup_en(RELAY2_PIN);
    }
}

void configureLoraPins() {
    pinMode(LORA_AUX_PIN, INPUT);
}

void primeWakeupSources() {
    const esp_err_t err = esp_sleep_enable_ext1_wakeup(RELAY_WAKE_MASK, ESP_EXT1_WAKEUP_ANY_LOW);
    if (err != ESP_OK) {
        Serial.print("Failed to enable EXT1 wakeup: ");
        Serial.println(err);
    }
}

void ensureRelayHasSettled() {
    // Wait for relay 1 to remain active for a short window to avoid bouncing
    waitForPinState(RELAY1_PIN, LOW, RELAY_ACTIVE_MIN_MS, 250);
}

bool isRelay1Active() {
    return digitalRead(RELAY1_PIN) == LOW;
}

void sendRelay1Event() {
    beginLoraSerial();
    wakeLoraFromSleep();

    relay1EventCount++;

    String payload = "{";
    payload += "\"device\":\"";
    payload += DEVICE_NAME;
    payload += "\",";
    payload += "\"event\":\"car_enter\",";
    payload += "\"relay\":1,";
    payload += "\"seq\":";
    payload += relay1EventCount;
    payload += "}";

    Serial.print("[LR02] TX -> ");
    Serial.println(payload);

    if (!waitForAuxLow(AUX_READY_TIMEOUT_MS)) {
        Serial.println("[LR02] AUX did not report idle before transmit.");
    }

    LoraSerial.println(payload);
    LoraSerial.flush();

    if (!waitForAuxHigh(AUX_READY_TIMEOUT_MS)) {
        Serial.println("[LR02] AUX never went busy after transmit request.");
    }
    if (!waitForAuxLow(AUX_READY_TIMEOUT_MS)) {
        Serial.println("[LR02] AUX did not return to idle after transmit.");
    }

    enterLoraSleep();
}

void goToSleep() {
    Serial.flush();
    delay(10);
    esp_deep_sleep_start();
}

void setup() {
    Serial.begin(115200);
    delay(50);

    configureRelayInputs();
    configureLoraPins();
    primeWakeupSources();

    const esp_sleep_wakeup_cause_t wakeCause = esp_sleep_get_wakeup_cause();
    const uint64_t ext1Mask = esp_sleep_get_ext1_wakeup_status();

    Serial.print("Wake cause: ");
    Serial.println(wakeCause);

    if (wakeCause == ESP_SLEEP_WAKEUP_EXT1 && (ext1Mask & (1ULL << RELAY1_PIN))) {
        ensureRelayHasSettled();
        if (isRelay1Active()) {
            sendRelay1Event();
        } else {
            Serial.println("Relay released before we could confirm event; skipping transmit.");
        }
    } else if (wakeCause == ESP_SLEEP_WAKEUP_EXT1 && (ext1Mask & (1ULL << RELAY2_PIN))) {
        Serial.println("Relay 2 wake detected; behavior not implemented yet.");
    } else {
        Serial.println("Cold boot; entering sentinel sleep until loop sensor fires.");
    }

    ensureLoraSleeping();

    Serial.println("Entering deep sleep...");
    goToSleep();
}

void loop() {
    // never reached
}
