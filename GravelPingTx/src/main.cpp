/**
 * GravelPing - Driveway Monitor Transmitter
 * 
 * This is the transmitter unit that detects vehicle presence via an EMX LP D-TEK
 * loop sensor and sends LoRa messages via the DX-LR02-900T22D module.
 * 
 * Hardware:
 *   - ESP32-C6 SuperMini
 *   - DX-LR02-900T22D LoRa UART Module
 *   - EMX LP D-TEK Vehicle Loop Detector (2 relay outputs)
 * 
 * Phase 2: With deep sleep support
 * 
 * LED Indicators (focused on sleep/wake debugging):
 *   RGB LED:
 *     - MAGENTA solid:     Woke from sleep / boot
 *     - MAGENTA flash x2:  Entering sleep
 *     - BLUE solid:        Transmitting LoRa message  
 *     - RED flash:         Error (LoRa timeout, etc.)
 *     - GREEN dim:         Awake/idle (brief, before sleep)
 *   
 *   Status LED (GPIO15):
 *     - ON:  ESP is awake and processing
 *     - OFF: ESP is asleep (or about to sleep)
 * 
 * Sleep Behavior:
 *   - ESP32-C6 enters deep sleep after transmission
 *   - Wakes on GPIO4 LOW (relay 1 closes to ground)
 *   - LR-02 put into sleep mode (AT+SLEEP0), wakes on serial data
 */

#include <Arduino.h>
#include <ArduinoJson.h>
#include <FastLED.h>
#include "esp_sleep.h"

// ============================================================================
// PIN DEFINITIONS
// ============================================================================

// Onboard LEDs (ESP32-C6 SuperMini)
constexpr int PIN_LED_STATUS = 15;  // Simple status LED (active HIGH)
constexpr int PIN_LED_RGB    = 8;   // WS2812 RGB LED

// LR-02 LoRa Module Connections
// HardwareSerial.begin(baud, config, rxPin, txPin)
constexpr int PIN_LORA_RX  = 17;  // ESP32-C6 RX (GPIO17) <- LR-02 TX (Pin 4)
constexpr int PIN_LORA_TX  = 16;  // ESP32-C6 TX (GPIO16) -> LR-02 RX (Pin 3)
constexpr int PIN_LORA_AUX = 18;  // LR-02 AUX pin (LOW = busy, HIGH = ready)

// EMX LP D-TEK Relay Connections (Active LOW - relay closes to ground)
constexpr int PIN_RELAY1   = 4;   // Relay 1: Vehicle presence detection
constexpr int PIN_RELAY2   = 5;   // Relay 2: Loop fault indicator (fail-secure mode)

// ============================================================================
// CONFIGURATION
// ============================================================================

// DEBUG MODE: Set to true to disable esp32 sleep and allow serial monitoring
// Set to false for production (battery-powered) use
constexpr bool DEBUG_MODE = false;

// Send each message twice for redundancy (set to false to send only once)
constexpr bool DUPLICATE_MESSAGES = false;

// Serial configuration
constexpr unsigned long SERIAL_BAUD      = 115200;  // Debug serial
constexpr unsigned long LORA_BAUD        = 9600;    // LR-02 default baud rate

// Timing configuration
constexpr unsigned long DEBOUNCE_MS            = 50;      // General debounce time
constexpr unsigned long RELAY_DEBOUNCE_MS      = 100;     // Relay release debounce time
constexpr unsigned long LOOP_FAULT_DEBOUNCE_MS = 2000;    // Loop fault must persist this long before reporting
constexpr unsigned long LORA_AUX_TIMEOUT       = 5000;    // Max wait for AUX pin (ms)
constexpr unsigned long LORA_WAKE_DELAY        = 100;     // Time for LR-02 to wake from sleep (ms)
constexpr unsigned long PRE_SLEEP_DELAY        = 500;     // Delay before entering sleep (ms)

// Device identification
constexpr const char* DEVICE_ID          = "TX01";  // Transmitter ID
constexpr uint32_t    MESSAGE_VERSION    = 1;       // Protocol version

// RGB LED configuration
constexpr int NUM_LEDS              = 1;
constexpr int LED_BRIGHTNESS        = 50;    // 0-255, normal brightness
constexpr int LED_BRIGHTNESS_DIM    = 10;    // Dim brightness for idle

// ============================================================================
// LED COLOR DEFINITIONS
// ============================================================================

namespace Colors {
    const CRGB OFF        = CRGB::Black;
    const CRGB WAKE       = CRGB::Magenta;   // Woke from sleep
    const CRGB TRANSMIT   = CRGB::Blue;      // Transmitting
    const CRGB ERROR      = CRGB::Red;       // Error condition
    const CRGB IDLE       = CRGB::Green;     // Brief idle before sleep
}

// ============================================================================
// GLOBALS
// ============================================================================

// Hardware serial for LR-02 communication
HardwareSerial LoRaSerial(1);  // Use UART1

// RGB LED array
CRGB leds[NUM_LEDS];

// State tracking
RTC_DATA_ATTR uint32_t messageCounter = 0;  // Persists through deep sleep
RTC_DATA_ATTR uint32_t wakeCount = 0;       // Track wake cycles

// ============================================================================
// FUNCTION DECLARATIONS
// ============================================================================

void setupPins();
void setupLEDs();
void setupLoRa();
bool waitForLoRaReady(unsigned long timeoutMs = LORA_AUX_TIMEOUT);
void sendLoRaMessage(const char* eventType, int relayNum);
void wakeLoRaModule();
void sleepLoRaModule();
void enterDeepSleep();
bool wasWokenByGPIO();
uint64_t getWakeGPIOStatus();
void waitForRelaysRelease();

// LED functions
void setRGB(CRGB color);
void setRGBDim(CRGB color, uint8_t brightness);
void setStatusLED(bool on);
void flashRGB(CRGB color, int count = 1, int onMs = 100, int offMs = 100);
void flashStatusLED(int count = 1, int onMs = 100, int offMs = 100);

// ============================================================================
// SETUP
// ============================================================================

void setup() {
    // =========================================================================
    // FLASH WINDOW: 3 second delay to allow flashing when sleep is enabled
    // Hold BOOT button during reset, or flash during this window
    // =========================================================================
    delay(3000);
    
    // Initialize LEDs FIRST for immediate visual feedback
    setupLEDs();
    
    // Immediately show we're awake - MAGENTA = woke up
    setRGB(Colors::WAKE);
    setStatusLED(true);
    
    // Initialize debug serial
    Serial.begin(SERIAL_BAUD);
    delay(500);  // Give USB CDC time to initialize
    
    // Check wake reason - capture GPIO status immediately
    wakeCount++;
    bool wokeFromSleep = wasWokenByGPIO();
    uint64_t wakeGPIOs = getWakeGPIOStatus();  // Get which GPIO(s) triggered wake
    
    Serial.println();
    Serial.println(F("========================================"));
    Serial.println(F("   GravelPing Transmitter - Phase 2"));
    Serial.println(F("========================================"));
    Serial.printf("[WAKE] Wake count: %lu\n", wakeCount);
    Serial.printf("[WAKE] Message counter: %lu\n", messageCounter);
    
    if (wokeFromSleep) {
        Serial.printf("[WAKE] Woke from deep sleep via GPIO (status: 0x%llX)\n", wakeGPIOs);
        if (wakeGPIOs & (1ULL << PIN_RELAY1)) Serial.println(F("[WAKE]   - GPIO4 (Relay 1) triggered"));
        if (wakeGPIOs & (1ULL << PIN_RELAY2)) Serial.println(F("[WAKE]   - GPIO5 (Relay 2) triggered"));
    } else {
        Serial.println(F("[WAKE] Fresh boot (power-on or reset)"));
    }
    Serial.println();
    
    // Initialize hardware
    setupPins();
    setupLoRa();
    
    // Handle LR-02 module state based on how we woke up
    if (wokeFromSleep) {
        // ESP woke from deep sleep, so LR-02 should also be asleep
        Serial.println(F("[LORA] Waking LR-02 module from sleep..."));
        wakeLoRaModule();
    } else {
        // Fresh boot - LR-02 is already awake and in transparent mode
        Serial.println(F("[LORA] Fresh boot - LR-02 should already be ready"));
        // Just verify it's ready
        if (waitForLoRaReady(1000)) {
            Serial.println(F("[LORA] LR-02 is ready (AUX LOW)"));
        } else {
            Serial.println(F("[LORA] Warning: AUX not LOW, module may need reset"));
        }
    }
    
    // If we woke from GPIO, use the wake status to determine which relay triggered
    if (wokeFromSleep) {
        bool relay1Triggered = (wakeGPIOs & (1ULL << PIN_RELAY1)) != 0;
        bool relay2Triggered = (wakeGPIOs & (1ULL << PIN_RELAY2)) != 0;
        bool messageSent = false;
        
        Serial.printf("[DEBUG] relay1Triggered=%d, relay2Triggered=%d\n", relay1Triggered, relay2Triggered);
        
        // Handle relay 1 (vehicle detection) - send immediately
        if (relay1Triggered) {
            Serial.println(F("[EVENT] Vehicle detected (Relay 1) - sending LoRa message"));
            setRGB(Colors::TRANSMIT);
            sendLoRaMessage("vehicle_enter", 1);
            messageSent = true;
        }
        
        // Handle relay 2 (loop fault) - only send if still active after debounce period
        // This filters out brief false triggers that can occur during vehicle detection
        if (relay2Triggered) {
            Serial.println(F("[EVENT] Relay 2 triggered - checking if loop fault persists..."));
            setRGB(Colors::ERROR);  // Red to indicate potential fault
            
            // Check if relay 2 is still LOW after debounce period
            bool faultConfirmed = true;
            unsigned long debounceStart = millis();
            
            while (millis() - debounceStart < LOOP_FAULT_DEBOUNCE_MS) {
                if (digitalRead(PIN_RELAY2) == HIGH) {
                    // Relay released - was just a brief glitch
                    faultConfirmed = false;
                    Serial.println(F("[EVENT] Relay 2 released - false trigger, ignoring"));
                    break;
                }
                delay(50);  // Check every 50ms
            }
            
            if (faultConfirmed) {
                Serial.println(F("[EVENT] Loop fault confirmed - sending LoRa message"));
                sendLoRaMessage("loop_fault", 2);
                messageSent = true;
            } else {
                // Clear the error color if we already sent a vehicle message
                if (relay1Triggered) {
                    setRGBDim(Colors::IDLE, LED_BRIGHTNESS_DIM);
                }
            }
        }
        
        // Fallback: if we woke from GPIO but couldn't determine which pin, send anyway
        if (!messageSent) {
            Serial.println(F("[WARN] Wake GPIO status unclear - sending default message"));
            setRGB(Colors::TRANSMIT);
            sendLoRaMessage("vehicle_enter", 1);
        }
    } else {
        // Fresh boot - just show we're ready
        Serial.println(F("[INIT] Fresh boot complete"));
        flashRGB(Colors::IDLE, 2, 200, 100);
    }
    
    // Brief idle indication before sleep
    setRGBDim(Colors::IDLE, LED_BRIGHTNESS_DIM);
    
    if (DEBUG_MODE) {
        Serial.println(F("[DEBUG] Debug mode enabled - sleep disabled"));
        Serial.println(F("[DEBUG] Waiting for relay trigger (polling mode)..."));
        // Don't sleep - fall through to loop() for polling
    } else {
        // Wait for relays to release before sleeping (prevents immediate re-wake)
        waitForRelaysRelease();
        
        Serial.println(F("[SLEEP] Preparing to enter deep sleep..."));
        Serial.flush();
        delay(PRE_SLEEP_DELAY);
        
        // Put LR-02 to sleep
        sleepLoRaModule();
        
        // Enter deep sleep
        enterDeepSleep();
    }
}

// ============================================================================
// MAIN LOOP
// ============================================================================

// Track relay state for debug mode polling
static bool lastRelay1State = HIGH;
static bool lastRelay2State = HIGH;
static unsigned long lastRelay1TriggerTime = 0;
static unsigned long lastRelay2TriggerTime = 0;
constexpr unsigned long COOLDOWN_MS = 2000;  // 2 second cooldown between triggers
static bool loraAsleep = false;  // Track LR-02 sleep state for debug mode

void loop() {
    if (DEBUG_MODE) {
        // Poll relay states instead of using sleep
        bool relay1State = digitalRead(PIN_RELAY1);
        bool relay2State = digitalRead(PIN_RELAY2);
        unsigned long now = millis();
        bool messageSent = false;
        
        // Check for Relay 1 HIGH->LOW transition (vehicle detected) - send immediately
        if (relay1State == LOW && lastRelay1State == HIGH) {
            if (now - lastRelay1TriggerTime >= COOLDOWN_MS) {
                lastRelay1TriggerTime = now;
                
                // Wake LR-02 if it was asleep
                if (loraAsleep) {
                    Serial.println(F("[LORA] Waking LR-02..."));
                    wakeLoRaModule();
                    loraAsleep = false;
                }
                
                Serial.println(F("[EVENT] Vehicle detected on Relay 1!"));
                setRGB(Colors::TRANSMIT);
                sendLoRaMessage("vehicle_enter", 1);
                setRGBDim(Colors::IDLE, LED_BRIGHTNESS_DIM);
                messageSent = true;
            } else {
                Serial.println(F("[EVENT] Relay 1 triggered but in cooldown"));
            }
        }
        
        // Check for Relay 2 HIGH->LOW transition (loop fault) - debounce before sending
        if (relay2State == LOW && lastRelay2State == HIGH) {
            if (now - lastRelay2TriggerTime >= COOLDOWN_MS) {
                lastRelay2TriggerTime = now;
                
                Serial.println(F("[EVENT] Relay 2 triggered - checking if loop fault persists..."));
                setRGB(Colors::ERROR);
                
                // Wait for debounce period, checking if relay stays LOW
                bool faultConfirmed = true;
                unsigned long debounceStart = millis();
                
                while (millis() - debounceStart < LOOP_FAULT_DEBOUNCE_MS) {
                    if (digitalRead(PIN_RELAY2) == HIGH) {
                        faultConfirmed = false;
                        Serial.println(F("[EVENT] Relay 2 released - false trigger, ignoring"));
                        break;
                    }
                    delay(50);
                }
                
                if (faultConfirmed) {
                    // Wake LR-02 if it was asleep
                    if (loraAsleep) {
                        Serial.println(F("[LORA] Waking LR-02..."));
                        wakeLoRaModule();
                        loraAsleep = false;
                    }
                    
                    Serial.println(F("[EVENT] Loop fault confirmed on Relay 2!"));
                    sendLoRaMessage("loop_fault", 2);
                    messageSent = true;
                }
                setRGBDim(Colors::IDLE, LED_BRIGHTNESS_DIM);
            } else {
                Serial.println(F("[EVENT] Relay 2 triggered but in cooldown"));
            }
        }
        
        // After sending, wait for relay release then put LR-02 to sleep
        if (messageSent) {
            waitForRelaysRelease();
            Serial.println(F("[LORA] Putting LR-02 to sleep..."));
            sleepLoRaModule();
            loraAsleep = true;
            Serial.println(F("[DEBUG] Waiting for next trigger..."));
        }
        
        lastRelay1State = relay1State;
        lastRelay2State = relay2State;
        delay(10);  // Small delay to prevent busy-waiting
    } else {
        // We should never get here in production - setup() always ends with deep sleep
        // But just in case, show error and retry sleep
        Serial.println(F("[ERROR] Unexpected loop execution!"));
        flashRGB(Colors::ERROR, 5, 100, 100);
        delay(1000);
        enterDeepSleep();
    }
}

// ============================================================================
// LED SETUP
// ============================================================================

void setupLEDs() {
    // Initialize status LED
    pinMode(PIN_LED_STATUS, OUTPUT);
    digitalWrite(PIN_LED_STATUS, LOW);
    
    // Initialize WS2812 RGB LED
    FastLED.addLeds<NEOPIXEL, PIN_LED_RGB>(leds, NUM_LEDS);
    FastLED.setBrightness(LED_BRIGHTNESS);
    FastLED.clear();
    FastLED.show();
}

// ============================================================================
// PIN SETUP
// ============================================================================

void setupPins() {
    Serial.println(F("[INIT] Configuring pins..."));
    
    // Configure LR-02 AUX pin as input
    // AUX is LOW when module is idle/ready, HIGH when busy
    pinMode(PIN_LORA_AUX, INPUT);
    
    // Configure relay inputs with internal pull-up
    // Relay closes to ground when triggered, so we use INPUT_PULLUP
    pinMode(PIN_RELAY1, INPUT_PULLUP);
    pinMode(PIN_RELAY2, INPUT_PULLUP);
    
    // Note: No interrupt needed - we use deep sleep with GPIO wake
    
    Serial.println(F("[INIT] Pins configured:"));
    Serial.printf("       - Status LED:       GPIO%d\n", PIN_LED_STATUS);
    Serial.printf("       - RGB LED (WS2812): GPIO%d\n", PIN_LED_RGB);
    Serial.printf("       - LoRa TX (ESP RX): GPIO%d\n", PIN_LORA_TX);
    Serial.printf("       - LoRa RX (ESP TX): GPIO%d\n", PIN_LORA_RX);
    Serial.printf("       - LoRa AUX:         GPIO%d\n", PIN_LORA_AUX);
    Serial.printf("       - Relay 1 (vehicle): GPIO%d\n", PIN_RELAY1);
    Serial.printf("       - Relay 2 (fault):   GPIO%d\n", PIN_RELAY2);
}

// ============================================================================
// LORA SETUP
// ============================================================================

void setupLoRa() {
    Serial.println(F("[INIT] Initializing LoRa UART..."));
    
    // HardwareSerial.begin(baud, config, rxPin, txPin)
    LoRaSerial.begin(LORA_BAUD, SERIAL_8N1, PIN_LORA_RX, PIN_LORA_TX);
    
    // Wait for UART to be ready
    delay(100);
    
    Serial.printf("[INIT] LoRa UART configured at %lu baud\n", LORA_BAUD);
    Serial.printf("[INIT] RX=GPIO%d, TX=GPIO%d\n", PIN_LORA_RX, PIN_LORA_TX);
    
    // Check AUX pin state
    int auxState = digitalRead(PIN_LORA_AUX);
    Serial.printf("[INIT] AUX pin state: %s\n", auxState == LOW ? "LOW (ready)" : "HIGH (busy)");
    
    // Clear any pending data in RX buffer
    while (LoRaSerial.available()) {
        LoRaSerial.read();
    }
}

// ============================================================================
// LORA FUNCTIONS
// ============================================================================

/**
 * Wait for the LR-02 AUX pin to go LOW (indicating ready state)
 * Returns true if ready, false if timeout
 */
bool waitForLoRaReady(unsigned long timeoutMs) {
    unsigned long startTime = millis();
    
    while (digitalRead(PIN_LORA_AUX) == HIGH) {
        if (millis() - startTime >= timeoutMs) {
            return false;
        }
        delay(1);
    }
    
    return true;
}

/**
 * Wake the LR-02 module from sleep mode
 * After waking from AT+SLEEP0, the module is still in AT mode!
 * We need to send +++ to exit AT mode and return to transparent mode.
 */
void wakeLoRaModule() {
    Serial.println(F("[LORA] Waking LR-02 from sleep..."));
    
    // Clear any pending data
    while (LoRaSerial.available()) {
        LoRaSerial.read();
    }
    
    // Send wake bytes (need at least 4 to wake from sleep)
    // Module will respond with "leave deepsleep..." and "ERROR=102"
    Serial.println(F("[LORA] Sending wake bytes..."));
    LoRaSerial.print("WAKE");  // 4 bytes to wake
    LoRaSerial.flush();
    
    // Wait for wake response
    delay(500);
    
    Serial.print(F("[LORA] Wake response: "));
    String response = "";
    while (LoRaSerial.available()) {
        char c = LoRaSerial.read();
        response += c;
        Serial.print(c);
    }
    if (response.length() == 0) {
        Serial.println(F("(none - module may not have been asleep)"));
    }
    Serial.println();
    
    // After waking from deep sleep, module is STILL in AT mode
    // Send +++ to EXIT AT mode and return to transparent mode
    Serial.println(F("[LORA] Exiting AT mode (sending +++)..."));
    delay(200);
    
    LoRaSerial.print("+++\r\n");
    LoRaSerial.flush();
    
    // Wait for "Exit AT" and "Power on" response
    delay(500);
    
    Serial.print(F("[LORA] Exit response: "));
    response = "";
    while (LoRaSerial.available()) {
        char c = LoRaSerial.read();
        response += c;
        Serial.print(c);
    }
    Serial.println();
    
    if (response.indexOf("Exit AT") >= 0) {
        Serial.println(F("[LORA] Successfully exited AT mode"));
    } else if (response.indexOf("Entry AT") >= 0) {
        // We weren't in AT mode (maybe module wasn't asleep), now we are - exit it
        Serial.println(F("[LORA] Entered AT mode unexpectedly - exiting..."));
        delay(200);
        LoRaSerial.print("+++\r\n");
        LoRaSerial.flush();
        delay(500);
        while (LoRaSerial.available()) {
            Serial.print((char)LoRaSerial.read());
        }
        Serial.println();
    } else {
        Serial.println(F("[LORA] (Module may have already been in transparent mode)"));
    }
    
    // Wait for module to be ready after reset ("Power on")
    delay(500);
    
    // Wait for AUX to indicate ready
    if (waitForLoRaReady(2000)) {
        Serial.println(F("[LORA] Module awake and ready"));
    } else {
        Serial.println(F("[LORA] WARNING: Module may not be ready (AUX timeout)"));
        flashRGB(Colors::ERROR, 2, 100, 100);
    }
}

/**
 * Put the LR-02 module into sleep mode (AT+SLEEP0)
 * This reduces current to ~59µA
 * Expected flow:
 *   Send: +++\r\n       -> Response: Entry AT\r\n
 *   Send: AT+SLEEP0\r\n -> Response: OK\r\n enter deepsleep...\r\n
 */
void sleepLoRaModule() {
    Serial.println(F("[LORA] Putting LR-02 to sleep..."));
    
    // Wait for any ongoing transmission to complete (AUX should be LOW when ready)
    Serial.println(F("[LORA] Waiting for module ready (AUX LOW)..."));
    if (!waitForLoRaReady(2000)) {
        Serial.println(F("[LORA] Warning: AUX timeout - module may be busy"));
        // Continue anyway, but note the issue
    } else {
        Serial.println(F("[LORA] Module ready"));
    }
    
    // Clear RX buffer
    while (LoRaSerial.available()) {
        LoRaSerial.read();
    }
    
    // Flush TX buffer and wait for silence before +++
    LoRaSerial.flush();
    delay(200);
    
    // Enter AT command mode
    Serial.println(F("[LORA] Sending +++..."));
    LoRaSerial.print("+++\r\n");
    LoRaSerial.flush();
    
    // Wait for response
    delay(500);
    
    Serial.print(F("[LORA] +++ response: "));
    String response = "";
    while (LoRaSerial.available()) {
        char c = LoRaSerial.read();
        response += c;
        Serial.print(c);
    }
    Serial.println();
    
    if (response.indexOf("Entry AT") >= 0) {
        Serial.println(F("[LORA] Entered AT command mode"));
    } else if (response.indexOf("Exit AT") >= 0) {
        // We were already in AT mode, now we're out - re-enter
        Serial.println(F("[LORA] Was in AT mode - re-entering..."));
        delay(500);  // Wait for "Power on" reset
        while (LoRaSerial.available()) LoRaSerial.read();
        LoRaSerial.flush();
        delay(200);
        LoRaSerial.print("+++\r\n");
        LoRaSerial.flush();
        delay(500);
        while (LoRaSerial.available()) {
            Serial.print((char)LoRaSerial.read());
        }
        Serial.println();
    } else {
        Serial.println(F("[LORA] Warning: No AT mode response, trying sleep anyway..."));
        // NOTE: Future consideration - add retry logic here
    }
    
    // Wait for module ready before sending command
    waitForLoRaReady(500);
    
    // Clear buffer
    while (LoRaSerial.available()) {
        LoRaSerial.read();
    }
    
    // Send sleep command
    Serial.println(F("[LORA] Sending AT+SLEEP0..."));
    LoRaSerial.print("AT+SLEEP0\r\n");
    LoRaSerial.flush();
    
    // Wait for "OK" and "enter deepsleep..." response
    delay(500);
    
    Serial.print(F("[LORA] Sleep response: "));
    response = "";
    while (LoRaSerial.available()) {
        char c = LoRaSerial.read();
        response += c;
        Serial.print(c);
    }
    Serial.println();
    
    if (response.indexOf("OK") >= 0 || response.indexOf("deepsleep") >= 0) {
        Serial.println(F("[LORA] LR-02 entered sleep mode successfully!"));
    } else {
        Serial.println(F("[LORA] Warning: Sleep confirmation not received"));
        // NOTE: Future consideration - add retry logic here
    }
}

/**
 * Send a JSON message via the LR-02 LoRa module
 * Sends the message twice for redundancy
 */
void sendLoRaMessage(const char* eventType, int relayNum) {
    Serial.println(F("[LORA] Preparing to send message..."));
    
    // Check AUX pin state
    Serial.printf("[LORA] AUX pin state: %s\n", digitalRead(PIN_LORA_AUX) == HIGH ? "HIGH (busy)" : "LOW (ready)");
    
    // Wait for LoRa module to be ready (reduced timeout for faster failure)
    if (!waitForLoRaReady(2000)) {
        Serial.println(F("[LORA] ERROR: Module not ready (AUX timeout) - attempting send anyway"));
        // Don't return - try to send anyway, module might still work
    }
    
    // Build JSON payload
    JsonDocument doc;
    doc["device"]  = DEVICE_ID;
    doc["version"] = MESSAGE_VERSION;
    doc["event"]   = eventType;
    doc["relay"]   = relayNum;
    doc["seq"]     = ++messageCounter;
    doc["wake"]    = wakeCount;
    
    // Serialize to string
    String jsonPayload;
    serializeJson(doc, jsonPayload);
    
    // Determine how many times to send (1 or 2 based on DUPLICATE_MESSAGES flag)
    const int sendCount = DUPLICATE_MESSAGES ? 2 : 1;
    
    for (int attempt = 1; attempt <= sendCount; attempt++) {
        if (DUPLICATE_MESSAGES) {
            Serial.printf("[LORA] Sending (%d/%d): ", attempt, sendCount);
        } else {
            Serial.print(F("[LORA] Sending: "));
        }
        Serial.println(jsonPayload);
        
        // Send via LoRa (transparent mode - just send the data)
        LoRaSerial.print(jsonPayload);
        LoRaSerial.print("\n");  // Add newline as message delimiter
        LoRaSerial.flush();
        
        // Wait for transmission to complete (AUX goes HIGH during TX, then LOW)
        delay(50);  // Small delay for module to start transmitting
        
        if (waitForLoRaReady()) {
            Serial.printf("[LORA] Message %d sent successfully\n", attempt);
        } else {
            Serial.printf("[LORA] WARNING: AUX timeout after message %d\n", attempt);
            flashRGB(Colors::ERROR, 1, 100, 50);
        }
        
        // Brief delay between transmissions (only if sending multiple)
        if (DUPLICATE_MESSAGES && attempt < sendCount) {
            delay(100);
        }
    }
    
    // Brief status LED blink to confirm TX complete
    flashStatusLED(2, 100, 100);
}

// ============================================================================
// SLEEP FUNCTIONS
// ============================================================================

/**
 * Check if we woke from deep sleep via GPIO
 */
bool wasWokenByGPIO() {
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    return (wakeup_reason == ESP_SLEEP_WAKEUP_GPIO);
}

/**
 * Get bitmask of which GPIOs triggered the wake
 * Returns 0 if not woken by GPIO
 */
uint64_t getWakeGPIOStatus() {
    if (!wasWokenByGPIO()) {
        return 0;
    }
    return esp_sleep_get_gpio_wakeup_status();
}

/**
 * Wait for all relay inputs to go HIGH (released) before sleeping
 * This prevents immediate re-wake if a vehicle is parked on the sensor
 * Includes debounce to ensure relay is truly released
 */
void waitForRelaysRelease() {
    bool relay1Active = (digitalRead(PIN_RELAY1) == LOW);
    bool relay2Active = (digitalRead(PIN_RELAY2) == LOW);
    
    if (!relay1Active && !relay2Active) {
        // Both appear released - verify with debounce
        delay(RELAY_DEBOUNCE_MS);
        relay1Active = (digitalRead(PIN_RELAY1) == LOW);
        relay2Active = (digitalRead(PIN_RELAY2) == LOW);
        if (!relay1Active && !relay2Active) {
            return;  // Confirmed released
        }
    }
    
    Serial.println(F("[WAIT] Waiting for relay(s) to release before sleeping..."));
    if (relay1Active) Serial.println(F("[WAIT]   - Relay 1 still active"));
    if (relay2Active) Serial.println(F("[WAIT]   - Relay 2 still active"));
    
    // Show a different color while waiting
    setRGBDim(Colors::WAKE, LED_BRIGHTNESS_DIM);  // Dim magenta = waiting for release
    
    unsigned long startTime = millis();
    unsigned long lastPrintTime = 0;
    unsigned long releaseTime = 0;
    bool waitingForDebounce = false;
    
    while (true) {
        relay1Active = (digitalRead(PIN_RELAY1) == LOW);
        relay2Active = (digitalRead(PIN_RELAY2) == LOW);
        
        if (relay1Active || relay2Active) {
            // Still active or bounced back - reset debounce
            waitingForDebounce = false;
            releaseTime = 0;
        } else {
            // Both released - start/continue debounce timer
            if (!waitingForDebounce) {
                waitingForDebounce = true;
                releaseTime = millis();
            } else if (millis() - releaseTime >= RELAY_DEBOUNCE_MS) {
                // Debounce complete - truly released
                break;
            }
        }
        
        // Print status every 5 seconds
        unsigned long elapsed = millis() - startTime;
        if (elapsed - lastPrintTime >= 5000) {
            lastPrintTime = elapsed;
            Serial.printf("[WAIT] Still waiting... (%lu seconds)\n", elapsed / 1000);
        }
        
        delay(10);  // Poll at 100Hz for better debounce resolution
    }
    
    Serial.printf("[WAIT] Relay(s) released after %lu ms\n", millis() - startTime);
    setRGBDim(Colors::IDLE, LED_BRIGHTNESS_DIM);  // Back to idle color
}

/**
 * Enter deep sleep mode with GPIO4 and GPIO5 as wake sources
 * The ESP will wake when either relay goes LOW
 *   - GPIO4 (Relay 1): Vehicle detected
 *   - GPIO5 (Relay 2): Loop fault (fail-secure mode)
 */
void enterDeepSleep() {
    Serial.println(F("[SLEEP] Configuring deep sleep..."));
    
    // Flash magenta twice to indicate entering sleep
    flashRGB(Colors::WAKE, 2, 150, 150);
    
    // Turn off LEDs
    setRGB(Colors::OFF);
    setStatusLED(false);
    FastLED.show();
    
    // Ensure serial output is complete
    Serial.println(F("[SLEEP] Entering deep sleep. Will wake on GPIO4 or GPIO5 LOW."));
    Serial.flush();
    delay(10);
    
    // Configure GPIO4 and GPIO5 as wake sources (wake on LOW level)
    // ESP32-C6 uses esp_deep_sleep_enable_gpio_wakeup with bitmask
    uint64_t wake_mask = (1ULL << PIN_RELAY1) | (1ULL << PIN_RELAY2);
    esp_deep_sleep_enable_gpio_wakeup(wake_mask, ESP_GPIO_WAKEUP_GPIO_LOW);
    
    // Enter deep sleep (never returns)
    esp_deep_sleep_start();
}

// ============================================================================
// LED FUNCTIONS
// ============================================================================

/**
 * Set the RGB LED to a specific color at default brightness
 */
void setRGB(CRGB color) {
    FastLED.setBrightness(LED_BRIGHTNESS);
    leds[0] = color;
    FastLED.show();
}

/**
 * Set the RGB LED to a specific color at custom brightness
 */
void setRGBDim(CRGB color, uint8_t brightness) {
    FastLED.setBrightness(brightness);
    leds[0] = color;
    FastLED.show();
}

/**
 * Turn the status LED on or off
 */
void setStatusLED(bool on) {
    digitalWrite(PIN_LED_STATUS, on ? HIGH : LOW);
}

/**
 * Flash the RGB LED with a specific color
 */
void flashRGB(CRGB color, int count, int onMs, int offMs) {
    for (int i = 0; i < count; i++) {
        setRGB(color);
        delay(onMs);
        setRGB(Colors::OFF);
        if (i < count - 1) {
            delay(offMs);
        }
    }
}

/**
 * Flash the status LED
 */
void flashStatusLED(int count, int onMs, int offMs) {
    for (int i = 0; i < count; i++) {
        setStatusLED(true);
        delay(onMs);
        setStatusLED(false);
        if (i < count - 1) {
            delay(offMs);
        }
    }
}
