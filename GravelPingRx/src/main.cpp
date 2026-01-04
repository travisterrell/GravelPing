/**
 * GravelPing - Driveway Monitor Receiver
 * 
 * This is the receiver unit that listens for LoRa messages from the transmitter
 * and outputs them to USB serial for monitoring/logging.
 * 
 * Hardware:
 *   - ESP32-C6 SuperMini
 *   - DX-LR02-900T22D LoRa UART Module
 * 
 * LED Indicators:
 *   RGB LED:
 *     - GREEN dim:    Idle, ready to receive
 *     - BLUE flash:   Message received
 *     - RED flash:    Invalid/error message
 *     - CYAN:         System boot
 *   
 *   Status LED (GPIO15):
 *     - ON:  System active
 * 
 * Output Format:
 *   All received messages are printed to USB serial in JSON format
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
constexpr int PIN_LORA_RX  = 17;  // ESP32-C6 RX (GPIO17) <- LR-02 TX (Pin 4)
constexpr int PIN_LORA_TX  = 16;  // ESP32-C6 TX (GPIO16) -> LR-02 RX (Pin 3)
constexpr int PIN_LORA_AUX = 18;  // LR-02 AUX pin (LOW = ready, HIGH = busy)

// ============================================================================
// CONFIGURATION
// ============================================================================

// Serial configuration
constexpr unsigned long SERIAL_BAUD      = 115200;  // Debug/output serial
constexpr unsigned long LORA_BAUD        = 9600;    // LR-02 default baud rate

// Device identification
constexpr const char* DEVICE_ID = "RX01";  // Receiver ID

// RGB LED configuration
constexpr int NUM_LEDS              = 1;
constexpr int LED_BRIGHTNESS        = 50;    // 0-255, normal brightness
constexpr int LED_BRIGHTNESS_DIM    = 10;    // Dim brightness for idle

// Message timeout - consider message complete if no data for this duration
constexpr unsigned long MESSAGE_TIMEOUT_MS = 100;

// ============================================================================
// LED COLOR DEFINITIONS
// ============================================================================

namespace Colors {
    const CRGB OFF        = CRGB::Black;
    const CRGB BOOT       = CRGB::Cyan;      // System boot
    const CRGB IDLE       = CRGB::Green;     // Ready to receive
    const CRGB RECEIVE    = CRGB::Blue;      // Message received
    const CRGB ERROR      = CRGB::Red;       // Error/invalid message
}

// ============================================================================
// GLOBALS
// ============================================================================

// Hardware serial for LR-02 communication
HardwareSerial LoRaSerial(1);  // Use UART1

// RGB LED array
CRGB leds[NUM_LEDS];

// Message buffer
String messageBuffer = "";
unsigned long lastCharTime = 0;
uint32_t messageCount = 0;

// ============================================================================
// FUNCTION DECLARATIONS
// ============================================================================

void setupLEDs();
void setupLoRa();
void processLoRaData();
void handleMessage(const String& message);

// LED functions
void setRGB(CRGB color);
void setRGBDim(CRGB color, uint8_t brightness);
void setStatusLED(bool on);
void flashRGB(CRGB color, int count = 1, int onMs = 100, int offMs = 100);

// ============================================================================
// SETUP
// ============================================================================

void setup() {
    // Initialize LEDs first for immediate feedback
    setupLEDs();
    setRGB(Colors::BOOT);
    setStatusLED(true);
    
    // Initialize USB serial
    Serial.begin(SERIAL_BAUD);
    delay(1000);  // Give USB CDC time to initialize
    
    Serial.println();
    Serial.println(F("========================================"));
    Serial.println(F("   GravelPing Receiver"));
    Serial.println(F("========================================"));
    Serial.printf("[INIT] Device ID: %s\n", DEVICE_ID);
    Serial.println();
    
    // Initialize LoRa module
    setupLoRa();
    
    // Ready to receive
    Serial.println(F("[READY] Listening for messages..."));
    Serial.println();
    setRGBDim(Colors::IDLE, LED_BRIGHTNESS_DIM);
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
    processLoRaData();
    delay(1);  // Small delay to prevent busy-waiting
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
// LORA SETUP
// ============================================================================

void setupLoRa() {
    Serial.println(F("[INIT] Initializing LoRa UART..."));
    
    // Configure AUX pin as input
    pinMode(PIN_LORA_AUX, INPUT);
    
    // Initialize UART for LR-02
    LoRaSerial.begin(LORA_BAUD, SERIAL_8N1, PIN_LORA_RX, PIN_LORA_TX);
    delay(100);
    
    Serial.printf("[INIT] LoRa UART configured at %lu baud\n", LORA_BAUD);
    Serial.printf("[INIT] RX=GPIO%d, TX=GPIO%d, AUX=GPIO%d\n", PIN_LORA_RX, PIN_LORA_TX, PIN_LORA_AUX);
    
    // Check AUX pin state
    int auxState = digitalRead(PIN_LORA_AUX);
    Serial.printf("[INIT] AUX pin state: %s\n", auxState == LOW ? "LOW (ready)" : "HIGH (busy)");
    
    // Clear any pending data
    while (LoRaSerial.available()) {
        LoRaSerial.read();
    }
    
    Serial.println(F("[INIT] LoRa module ready"));
}

// ============================================================================
// MESSAGE PROCESSING
// ============================================================================

/**
 * Read data from LoRa serial and process complete messages
 * Messages are delimited by newlines
 */
void processLoRaData() {
    // Read available characters
    while (LoRaSerial.available()) {
        char c = LoRaSerial.read();
        lastCharTime = millis();
        
        if (c == '\n' || c == '\r') {
            // End of message - process if buffer has content
            if (messageBuffer.length() > 0) {
                handleMessage(messageBuffer);
                messageBuffer = "";
            }
        } else {
            // Add character to buffer
            messageBuffer += c;
            
            // Prevent buffer overflow
            if (messageBuffer.length() > 256) {
                Serial.println(F("[ERROR] Message buffer overflow - discarding"));
                messageBuffer = "";
            }
        }
    }
    
    // Check for message timeout (incomplete message)
    if (messageBuffer.length() > 0 && (millis() - lastCharTime) > MESSAGE_TIMEOUT_MS) {
        Serial.print(F("[WARN] Message timeout, partial data: "));
        Serial.println(messageBuffer);
        messageBuffer = "";
    }
}

/**
 * Process a complete message received from LoRa
 */
void handleMessage(const String& message) {
    messageCount++;
    
    // Flash blue to indicate reception
    flashRGB(Colors::RECEIVE, 1, 50, 0);
    
    Serial.println(F("----------------------------------------"));
    Serial.printf("[RX #%lu] Raw: %s\n", messageCount, message.c_str());
    
    // Try to parse as JSON
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, message);
    
    if (error) {
        Serial.printf("[ERROR] JSON parse failed: %s\n", error.c_str());
        flashRGB(Colors::ERROR, 1, 100, 0);
        Serial.println(F("----------------------------------------"));
        setRGBDim(Colors::IDLE, LED_BRIGHTNESS_DIM);
        return;
    }
    
    // Extract fields
    const char* device  = doc["device"]  | "unknown";
    uint32_t    version = doc["version"] | 0;
    const char* event   = doc["event"]   | "unknown";
    uint8_t     relay   = doc["relay"]   | 0;
    uint32_t    seq     = doc["seq"]     | 0;
    uint32_t    wake    = doc["wake"]    | 0;
    
    // Print parsed data
    Serial.println(F("[PARSED]"));
    Serial.printf("  Device:  %s\n", device);
    Serial.printf("  Version: %u\n", version);
    Serial.printf("  Event:   %s\n", event);
    Serial.printf("  Relay:   %u\n", relay);
    Serial.printf("  Seq:     %u\n", seq);
    Serial.printf("  Wake:    %u\n", wake);
    
    // Event-specific output
    if (strcmp(event, "vehicle_enter") == 0) {
        Serial.println(F(">>> VEHICLE DETECTED <<<"));
    } else if (strcmp(event, "loop_fault") == 0) {
        Serial.println(F(">>> LOOP FAULT DETECTED <<<"));
    } else if (strcmp(event, "loop_fault_cleared") == 0) {
        Serial.println(F(">>> LOOP FAULT CLEARED <<<"));
    }
    
    Serial.println(F("----------------------------------------"));
    Serial.println();
    
    // Return to idle
    setRGBDim(Colors::IDLE, LED_BRIGHTNESS_DIM);
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
