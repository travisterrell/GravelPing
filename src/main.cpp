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
 */

#include <Arduino.h>
#include <ArduinoJson.h>

// ============================================================================
// PIN DEFINITIONS
// ============================================================================

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

// ============================================================================
// GLOBALS
// ============================================================================

// Hardware serial for LR-02 communication
HardwareSerial LoRaSerial(1);  // Use UART1

// State tracking
volatile bool relay1Triggered = false;
unsigned long lastTriggerTime = 0;
uint32_t messageCounter = 0;

// ============================================================================
// FUNCTION DECLARATIONS
// ============================================================================

void setupPins();
void setupLoRa();
bool waitForLoRaReady(unsigned long timeoutMs = LORA_AUX_TIMEOUT);
void sendLoRaMessage(const char* eventType, int relayNum);
void onRelay1Change();

// ============================================================================
// SETUP
// ============================================================================

void setup() {
    // Initialize debug serial
    Serial.begin(SERIAL_BAUD);
    delay(1000);  // Give USB CDC time to initialize
    
    Serial.println();
    Serial.println(F("========================================"));
    Serial.println(F("   GravelPing Transmitter - Phase 1"));
    Serial.println(F("========================================"));
    Serial.println();
    
    // Initialize hardware
    setupPins();
    setupLoRa();
    
    Serial.println(F("[INIT] Setup complete. Waiting for vehicle detection..."));
    Serial.println();
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
            
            // Send the LoRa message
            sendLoRaMessage("vehicle_enter", 1);
            
            lastTriggerTime = currentTime;
        } else {
            Serial.println(F("[EVENT] Relay 1 triggered but in cooldown period, ignoring."));
        }
        
        relay1Triggered = false;
    }
    
    // Small delay to prevent tight loop
    delay(10);
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
    
    // Initialize UART1 for LR-02 communication
    LoRaSerial.begin(LORA_BAUD, SERIAL_8N1, PIN_LORA_TX, PIN_LORA_RX);
    
    // Wait for module to be ready
    delay(100);
    
    // Check if module is responsive
    if (waitForLoRaReady(2000)) {
        Serial.println(F("[INIT] LoRa module ready (AUX pin LOW)"));
    } else {
        Serial.println(F("[WARN] LoRa AUX pin not LOW - module may still be initializing"));
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
    } else {
        Serial.println(F("[LORA] WARNING: AUX did not return LOW after transmission"));
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
