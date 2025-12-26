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
 * Phase 1: Basic transmission without sleep modes
 * 
 * LED Color Codes:
 *   - Startup:      Rainbow cycle (initialization)
 *   - Ready/Idle:   Green pulse (waiting for vehicle)
 *   - Transmitting: Blue (sending LoRa message)
 *   - TX Success:   Cyan flash (message sent)
 *   - TX Error:     Red flash (transmission failed)
 *   - Cooldown:     Yellow flash (trigger ignored)
 *   - Sleep:        Off (future: when in deep sleep)
 */

#include <Arduino.h>
#include <ArduinoJson.h>
#include <FastLED.h>

// ============================================================================
// PIN DEFINITIONS
// ============================================================================

// Onboard LEDs (ESP32-C6 SuperMini)
constexpr int PIN_LED_STATUS = 15;  // Simple status LED (active HIGH)
constexpr int PIN_LED_RGB    = 8;   // WS2812 RGB LED

// LR-02 LoRa Module Connections
// Note: LR-02 uses 3.3V-5V logic, ESP32-C6 is 3.3V compatible
constexpr int PIN_LORA_TX  = 16;  // ESP32-C6 RX <- LR-02 TX
constexpr int PIN_LORA_RX  = 17;  // ESP32-C6 TX -> LR-02 RX
constexpr int PIN_LORA_AUX = 18;  // LR-02 AUX pin (LOW = idle/ready, HIGH = busy)

// EMX LP D-TEK Relay Connections (Active LOW - relay closes to ground)
constexpr int PIN_RELAY1   = 4;   // Relay 1 output from loop detector
constexpr int PIN_RELAY2   = 5;   // Relay 2 output from loop detector (future use)

// ============================================================================
// CONFIGURATION
// ============================================================================

// Serial configuration
constexpr unsigned long SERIAL_BAUD      = 115200;  // Debug serial
constexpr unsigned long LORA_BAUD        = 9600;    // LR-02 default baud rate

// Timing configuration
constexpr unsigned long DEBOUNCE_MS      = 50;      // Relay debounce time
constexpr unsigned long LORA_AUX_TIMEOUT = 5000;    // Max wait for AUX pin (ms)
constexpr unsigned long RELAY_COOLDOWN   = 2000;    // Min time between triggers (ms)

// Device identification
constexpr const char* DEVICE_ID          = "TX01";  // Transmitter ID
constexpr uint32_t    MESSAGE_VERSION    = 1;       // Protocol version

// RGB LED configuration
constexpr int NUM_LEDS        = 1;
constexpr int LED_BRIGHTNESS  = 50;   // 0-255, keep low for battery savings

// ============================================================================
// LED COLOR DEFINITIONS
// ============================================================================

namespace Colors {
    const CRGB OFF        = CRGB::Black;
    const CRGB READY      = CRGB::Green;
    const CRGB TRANSMIT   = CRGB::Blue;
    const CRGB SUCCESS    = CRGB::Cyan;
    const CRGB ERROR      = CRGB::Red;
    const CRGB COOLDOWN   = CRGB::Yellow;
    const CRGB INIT       = CRGB::Magenta;
    const CRGB LORA_READY = CRGB::Green;
    const CRGB LORA_WAIT  = CRGB::Orange;
}

// ============================================================================
// GLOBALS
// ============================================================================

// Hardware serial for LR-02 communication
HardwareSerial LoRaSerial(1);  // Use UART1

// RGB LED array
CRGB leds[NUM_LEDS];

// State tracking
volatile bool relay1Triggered = false;
unsigned long lastTriggerTime = 0;
uint32_t messageCounter = 0;

// Idle animation state
unsigned long lastIdleUpdate = 0;
uint8_t idleBrightness = 0;
int8_t idleDirection = 1;

// ============================================================================
// FUNCTION DECLARATIONS
// ============================================================================

void setupPins();
void setupLEDs();
void setupLoRa();
bool waitForLoRaReady(unsigned long timeoutMs = LORA_AUX_TIMEOUT);
void sendLoRaMessage(const char* eventType, int relayNum);
void onRelay1Change();

// LED functions
void setRGB(CRGB color);
void setStatusLED(bool on);
void flashRGB(CRGB color, int count = 1, int onMs = 100, int offMs = 100);
void rainbowCycle(int cycles = 1, int delayMs = 10);
void updateIdleAnimation();

// ============================================================================
// SETUP
// ============================================================================

void setup() {
    // Initialize LEDs first for visual feedback during boot
    setupLEDs();
    setRGB(Colors::INIT);
    setStatusLED(true);
    
    // Initialize debug serial
    Serial.begin(SERIAL_BAUD);
    delay(1000);  // Give USB CDC time to initialize
    
    Serial.println();
    Serial.println(F("========================================"));
    Serial.println(F("   GravelPing Transmitter - Phase 1"));
    Serial.println(F("========================================"));
    Serial.println();
    
    // Rainbow cycle to indicate initialization
    Serial.println(F("[INIT] Starting initialization..."));
    rainbowCycle(2, 15);
    
    // Initialize hardware
    setupPins();
    setupLoRa();
    
    // Final ready indication
    setStatusLED(false);
    flashRGB(Colors::SUCCESS, 3, 200, 100);
    
    Serial.println(F("[INIT] Setup complete. Waiting for vehicle detection..."));
    Serial.println(F("[INIT] LED Colors: Green=Ready, Blue=TX, Cyan=Success, Red=Error, Yellow=Cooldown"));
    Serial.println();
    
    // Set to ready state
    setRGB(Colors::READY);
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
    // Check if relay 1 was triggered
    if (relay1Triggered) {
        unsigned long currentTime = millis();
        
        // Apply cooldown to prevent rapid re-triggering
        if (currentTime - lastTriggerTime >= RELAY_COOLDOWN) {
            Serial.println(F("[EVENT] Relay 1 triggered - Vehicle detected!"));
            
            // Visual indicator - blue during transmission
            setRGB(Colors::TRANSMIT);
            setStatusLED(true);
            
            // Send the LoRa message
            sendLoRaMessage("vehicle_enter", 1);
            
            setStatusLED(false);
            lastTriggerTime = currentTime;
            
            // Brief pause then back to ready
            delay(500);
            setRGB(Colors::READY);
        } else {
            Serial.println(F("[EVENT] Relay 1 triggered but in cooldown period, ignoring."));
            // Yellow flash to indicate cooldown
            flashRGB(Colors::COOLDOWN, 2, 100, 100);
            setRGB(Colors::READY);
        }
        
        relay1Triggered = false;
    }
    
    // Update idle animation (gentle green pulse)
    updateIdleAnimation();
    
    // Small delay to prevent tight loop
    delay(10);
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
    pinMode(PIN_RELAY2, INPUT_PULLUP);  // Future use
    
    // Attach interrupt for relay 1 (trigger on falling edge = relay closing)
    attachInterrupt(digitalPinToInterrupt(PIN_RELAY1), onRelay1Change, FALLING);
    
    Serial.println(F("[INIT] Pins configured:"));
    Serial.printf("       - Status LED:       GPIO%d\n", PIN_LED_STATUS);
    Serial.printf("       - RGB LED (WS2812): GPIO%d\n", PIN_LED_RGB);
    Serial.printf("       - LoRa TX (ESP RX): GPIO%d\n", PIN_LORA_TX);
    Serial.printf("       - LoRa RX (ESP TX): GPIO%d\n", PIN_LORA_RX);
    Serial.printf("       - LoRa AUX:         GPIO%d\n", PIN_LORA_AUX);
    Serial.printf("       - Relay 1:          GPIO%d\n", PIN_RELAY1);
    Serial.printf("       - Relay 2:          GPIO%d\n", PIN_RELAY2);
}

// ============================================================================
// LORA SETUP
// ============================================================================

void setupLoRa() {
    Serial.println(F("[INIT] Initializing LoRa module..."));
    setRGB(Colors::LORA_WAIT);
    
    // Initialize UART1 for LR-02 communication
    LoRaSerial.begin(LORA_BAUD, SERIAL_8N1, PIN_LORA_TX, PIN_LORA_RX);
    
    // Wait for module to be ready
    delay(100);
    
    // Check if module is responsive
    if (waitForLoRaReady(2000)) {
        Serial.println(F("[INIT] LoRa module ready (AUX pin LOW)"));
        flashRGB(Colors::LORA_READY, 2, 150, 100);
    } else {
        Serial.println(F("[WARN] LoRa AUX pin not LOW - module may still be initializing"));
        flashRGB(Colors::ERROR, 3, 100, 100);
    }
    
    // The LR-02 defaults to transparent transmission mode (MODE 0)
    // and high efficiency mode (SLEEP 2), which is what we want for now.
    // No AT command configuration needed for basic operation.
    
    Serial.printf("[INIT] LoRa UART configured at %lu baud\n", LORA_BAUD);
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
 * Send a JSON message via the LR-02 LoRa module
 */
void sendLoRaMessage(const char* eventType, int relayNum) {
    Serial.println(F("[LORA] Preparing to send message..."));
    
    // Wait for LoRa module to be ready
    if (!waitForLoRaReady()) {
        Serial.println(F("[LORA] ERROR: Module not ready (AUX timeout)"));
        flashRGB(Colors::ERROR, 3, 100, 50);
        return;
    }
    
    // Build JSON payload
    JsonDocument doc;
    doc["device"]  = DEVICE_ID;
    doc["version"] = MESSAGE_VERSION;
    doc["event"]   = eventType;
    doc["relay"]   = relayNum;
    doc["seq"]     = ++messageCounter;
    doc["uptime"]  = millis() / 1000;  // Uptime in seconds
    
    // Serialize to string
    String jsonPayload;
    serializeJson(doc, jsonPayload);
    
    Serial.print(F("[LORA] Sending: "));
    Serial.println(jsonPayload);
    
    // Send via LoRa (transparent mode - just send the data)
    LoRaSerial.print(jsonPayload);
    LoRaSerial.print("\n");  // Add newline as message delimiter
    
    // Wait for transmission to complete (AUX goes HIGH during TX, then LOW)
    delay(50);  // Small delay for module to start transmitting
    
    if (waitForLoRaReady()) {
        Serial.println(F("[LORA] Message sent successfully"));
        flashRGB(Colors::SUCCESS, 2, 150, 100);
    } else {
        Serial.println(F("[LORA] WARNING: AUX did not return LOW after transmission"));
        flashRGB(Colors::ERROR, 2, 150, 100);
    }
}

// ============================================================================
// INTERRUPT HANDLERS
// ============================================================================

/**
 * ISR for Relay 1 state change
 * Called on falling edge (relay closing to ground)
 */
void IRAM_ATTR onRelay1Change() {
    // Simple flag set - debouncing handled in main loop
    relay1Triggered = true;
}

// ============================================================================
// LED FUNCTIONS
// ============================================================================

/**
 * Set the RGB LED to a specific color
 */
void setRGB(CRGB color) {
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
 * @param color The color to flash
 * @param count Number of flashes
 * @param onMs Duration LED is on (ms)
 * @param offMs Duration LED is off (ms)
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
 * Display a rainbow cycle animation
 * @param cycles Number of complete rainbow cycles
 * @param delayMs Delay between hue steps
 */
void rainbowCycle(int cycles, int delayMs) {
    for (int c = 0; c < cycles; c++) {
        for (int hue = 0; hue < 256; hue++) {
            leds[0] = CHSV(hue, 255, 255);
            FastLED.show();
            delay(delayMs);
        }
    }
    setRGB(Colors::OFF);
}

/**
 * Update the idle animation (gentle green pulse)
 * Call this from the main loop
 */
void updateIdleAnimation() {
    // Only update every 30ms for smooth but efficient animation
    if (millis() - lastIdleUpdate < 30) {
        return;
    }
    lastIdleUpdate = millis();
    
    // Pulse brightness up and down
    idleBrightness += idleDirection * 3;
    
    if (idleBrightness >= 50) {
        idleBrightness = 50;
        idleDirection = -1;
    } else if (idleBrightness <= 5) {
        idleBrightness = 5;
        idleDirection = 1;
    }
    
    // Apply pulsing green
    leds[0] = CRGB(0, idleBrightness, 0);
    FastLED.show();
}
