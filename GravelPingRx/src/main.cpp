/**
 * GravelPing - Driveway Monitor Receiver
 * 
 * This is the receiver unit that listens for LoRa messages from the transmitter
 * and forwards them to Home Assistant via MQTT.
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
 *     - YELLOW:       WiFi connecting
 *     - MAGENTA:      MQTT connecting
 *   
 *   Status LED (GPIO15):
 *     - ON:  System active
 * 
 * Output:
 *   - USB Serial: Human-readable message log
 *   - MQTT: Home Assistant autodiscovery + state updates
 */

#include <Arduino.h>
#include <ArduinoJson.h>
#include <FastLED.h>
#include <WiFi.h>
#include <PubSubClient.h>

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
// CONFIGURATION (from platformio.ini build_flags)
// ============================================================================

// Serial configuration
constexpr unsigned long SERIAL_BAUD      = 115200;  // Debug/output serial
constexpr unsigned long LORA_BAUD        = 9600;    // LR-02 default baud rate

// Device identification
constexpr const char* DEVICE_ID = "RX01";  // Receiver ID

// WiFi credentials (defined in platformio.ini)
#ifndef WIFI_SSID
#define WIFI_SSID "YourSSID"
#endif
#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD "YourPassword"
#endif

// MQTT configuration (defined in platformio.ini)
#ifndef MQTT_BROKER
#define MQTT_BROKER "192.168.1.100"
#endif
#ifndef MQTT_PORT
#define MQTT_PORT 1883
#endif
#ifndef MQTT_USER
#define MQTT_USER ""
#endif
#ifndef MQTT_PASSWORD
#define MQTT_PASSWORD ""
#endif
#ifndef DEVICE_NAME
#define DEVICE_NAME "GravelPing Driveway"
#endif

// RGB LED configuration
constexpr int NUM_LEDS              = 1;
constexpr int LED_BRIGHTNESS        = 50;    // 0-255, normal brightness
constexpr int LED_BRIGHTNESS_DIM    = 10;    // Dim brightness for idle

// Message timeout - consider message complete if no data for this duration
constexpr unsigned long MESSAGE_TIMEOUT_MS = 100;

// Reconnection intervals
constexpr unsigned long WIFI_RECONNECT_INTERVAL = 5000;   // 5 seconds
constexpr unsigned long MQTT_RECONNECT_INTERVAL = 5000;   // 5 seconds

// ============================================================================
// LED COLOR DEFINITIONS
// ============================================================================

namespace Colors {
    const CRGB OFF        = CRGB::Black;
    const CRGB BOOT       = CRGB::Cyan;      // System boot
    const CRGB WIFI       = CRGB::Yellow;    // WiFi connecting
    const CRGB MQTT       = CRGB::Magenta;   // MQTT connecting
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

// WiFi and MQTT clients
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// Message buffer
String messageBuffer = "";
unsigned long lastCharTime = 0;
uint32_t messageCount = 0;

// Reconnection tracking
unsigned long lastWifiAttempt = 0;
unsigned long lastMqttAttempt = 0;

// ============================================================================
// FUNCTION DECLARATIONS
// ============================================================================

void setupLEDs();
void setupWiFi();
void setupMQTT();
void setupLoRa();
void connectWiFi();
void connectMQTT();
void publishDiscovery();
void processLoRaData();
void handleMessage(const String& message);
void publishToHomeAssistant(const char* event, uint32_t seq);

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
    Serial.printf("[INIT] Device Name: %s\n", DEVICE_NAME);
    Serial.println();
    
    // Initialize LoRa module
    setupLoRa();
    
    // Connect to WiFi
    setupWiFi();
    connectWiFi();
    
    // Setup MQTT
    if (WiFi.status() == WL_CONNECTED) {
        setupMQTT();
        connectMQTT();
        
        // Publish Home Assistant discovery messages
        if (mqttClient.connected()) {
            publishDiscovery();
        }
    }
    
    // Ready to receive
    Serial.println(F("[READY] Listening for messages..."));
    Serial.println();
    setRGBDim(Colors::IDLE, LED_BRIGHTNESS_DIM);
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
    // Maintain WiFi connection
    if (WiFi.status() != WL_CONNECTED) {
        if (millis() - lastWifiAttempt >= WIFI_RECONNECT_INTERVAL) {
            lastWifiAttempt = millis();
            Serial.println(F("[WIFI] Connection lost, reconnecting..."));
            connectWiFi();
        }
    }
    
    // Maintain MQTT connection
    if (WiFi.status() == WL_CONNECTED && !mqttClient.connected()) {
        if (millis() - lastMqttAttempt >= MQTT_RECONNECT_INTERVAL) {
            lastMqttAttempt = millis();
            Serial.println(F("[MQTT] Connection lost, reconnecting..."));
            connectMQTT();
        }
    }
    
    // Process MQTT loop (handles keepalive)
    if (mqttClient.connected()) {
        mqttClient.loop();
    }
    
    // Process incoming LoRa data
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
// WIFI SETUP
// ============================================================================

void setupWiFi() {
    Serial.println(F("[WIFI] Initializing WiFi..."));
    WiFi.mode(WIFI_STA);
    WiFi.setHostname("gravelping-rx");
}

void connectWiFi() {
    if (WiFi.status() == WL_CONNECTED) {
        return;  // Already connected
    }
    
    Serial.printf("[WIFI] Connecting to %s...\n", WIFI_SSID);
    setRGB(Colors::WIFI);
    
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
        Serial.print(".");
        attempts++;
    }
    Serial.println();
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println(F("[WIFI] Connected!"));
        Serial.printf("[WIFI] IP Address: %s\n", WiFi.localIP().toString().c_str());
        Serial.printf("[WIFI] Signal: %d dBm\n", WiFi.RSSI());
    } else {
        Serial.println(F("[WIFI] Connection failed!"));
        flashRGB(Colors::ERROR, 3, 200, 200);
    }
}

// ============================================================================
// MQTT SETUP
// ============================================================================

void setupMQTT() {
    Serial.println(F("[MQTT] Initializing MQTT client..."));
    mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
    mqttClient.setBufferSize(512);  // Increase buffer for discovery messages
    Serial.printf("[MQTT] Broker: %s:%d\n", MQTT_BROKER, MQTT_PORT);
}

void connectMQTT() {
    if (mqttClient.connected()) {
        return;  // Already connected
    }
    
    Serial.println(F("[MQTT] Connecting to broker..."));
    setRGB(Colors::MQTT);
    
    String clientId = "gravelping-rx-";
    clientId += String(random(0xffff), HEX);
    
    bool connected = false;
    if (strlen(MQTT_USER) > 0) {
        connected = mqttClient.connect(clientId.c_str(), MQTT_USER, MQTT_PASSWORD);
    } else {
        connected = mqttClient.connect(clientId.c_str());
    }
    
    if (connected) {
        Serial.println(F("[MQTT] Connected!"));
        Serial.printf("[MQTT] Client ID: %s\n", clientId.c_str());
    } else {
        Serial.printf("[MQTT] Connection failed! State: %d\n", mqttClient.state());
        flashRGB(Colors::ERROR, 3, 200, 200);
    }
}

/**
 * Publish Home Assistant MQTT Discovery messages
 * Creates binary sensors for vehicle detection and loop fault
 */
void publishDiscovery() {
    Serial.println(F("[MQTT] Publishing Home Assistant discovery messages..."));
    
    JsonDocument doc;
    String payload;
    String topic;
    
    // Device information (shared across all entities)
    JsonObject device = doc["device"].to<JsonObject>();
    device["identifiers"][0] = "gravelping";
    device["name"] = DEVICE_NAME;
    device["model"] = "GravelPing Transmitter";
    device["manufacturer"] = "Custom";
    
    // =========================================================================
    // Binary Sensor: Vehicle Detection
    // =========================================================================
    doc.clear();
    doc["name"] = "Vehicle Detected";
    doc["unique_id"] = "gravelping_vehicle";
    doc["state_topic"] = "homeassistant/binary_sensor/gravelping/vehicle/state";
    doc["device_class"] = "motion";
    doc["payload_on"] = "ON";
    doc["payload_off"] = "OFF";
    doc["off_delay"] = 5;  // Auto-off after 5 seconds
    device = doc["device"].to<JsonObject>();
    device["identifiers"][0] = "gravelping";
    device["name"] = DEVICE_NAME;
    device["model"] = "GravelPing Transmitter";
    device["manufacturer"] = "Custom";
    
    payload = "";
    serializeJson(doc, payload);
    topic = "homeassistant/binary_sensor/gravelping/vehicle/config";
    
    if (mqttClient.publish(topic.c_str(), payload.c_str(), true)) {
        Serial.println(F("[MQTT]   ✓ Vehicle detection binary sensor"));
    } else {
        Serial.println(F("[MQTT]   ✗ Failed to publish vehicle sensor"));
    }
    
    // =========================================================================
    // Binary Sensor: Loop Fault
    // =========================================================================
    doc.clear();
    doc["name"] = "Loop Fault";
    doc["unique_id"] = "gravelping_loop_fault";
    doc["state_topic"] = "homeassistant/binary_sensor/gravelping/loop_fault/state";
    doc["device_class"] = "problem";
    doc["payload_on"] = "ON";
    doc["payload_off"] = "OFF";
    device = doc["device"].to<JsonObject>();
    device["identifiers"][0] = "gravelping";
    device["name"] = DEVICE_NAME;
    device["model"] = "GravelPing Transmitter";
    device["manufacturer"] = "Custom";
    
    payload = "";
    serializeJson(doc, payload);
    topic = "homeassistant/binary_sensor/gravelping/loop_fault/config";
    
    if (mqttClient.publish(topic.c_str(), payload.c_str(), true)) {
        Serial.println(F("[MQTT]   ✓ Loop fault binary sensor"));
    } else {
        Serial.println(F("[MQTT]   ✗ Failed to publish loop fault sensor"));
    }
    
    // =========================================================================
    // Sensor: Last Detection
    // =========================================================================
    doc.clear();
    doc["name"] = "Last Detection";
    doc["unique_id"] = "gravelping_last_detection";
    doc["state_topic"] = "homeassistant/sensor/gravelping/last_detection/state";
    doc["icon"] = "mdi:clock-outline";
    device = doc["device"].to<JsonObject>();
    device["identifiers"][0] = "gravelping";
    device["name"] = DEVICE_NAME;
    device["model"] = "GravelPing Transmitter";
    device["manufacturer"] = "Custom";
    
    payload = "";
    serializeJson(doc, payload);
    topic = "homeassistant/sensor/gravelping/last_detection/config";
    
    if (mqttClient.publish(topic.c_str(), payload.c_str(), true)) {
        Serial.println(F("[MQTT]   ✓ Last detection sensor"));
    } else {
        Serial.println(F("[MQTT]   ✗ Failed to publish last detection sensor"));
    }
    
    // =========================================================================
    // Sensor: Message Count
    // =========================================================================
    doc.clear();
    doc["name"] = "Message Count";
    doc["unique_id"] = "gravelping_message_count";
    doc["state_topic"] = "homeassistant/sensor/gravelping/message_count/state";
    doc["icon"] = "mdi:counter";
    device = doc["device"].to<JsonObject>();
    device["identifiers"][0] = "gravelping";
    device["name"] = DEVICE_NAME;
    device["model"] = "GravelPing Transmitter";
    device["manufacturer"] = "Custom";
    
    payload = "";
    serializeJson(doc, payload);
    topic = "homeassistant/sensor/gravelping/message_count/config";
    
    if (mqttClient.publish(topic.c_str(), payload.c_str(), true)) {
        Serial.println(F("[MQTT]   ✓ Message count sensor"));
    } else {
        Serial.println(F("[MQTT]   ✗ Failed to publish message count sensor"));
    }
    
    Serial.println(F("[MQTT] Discovery complete!"));
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
    
    // Extract fields (only event and seq now)
    const char* event   = doc["event"]   | "unknown";
    uint32_t    seq     = doc["seq"]     | 0;
    
    // Print parsed data
    Serial.println(F("[PARSED]"));
    Serial.printf("  Event:   %s\n", event);
    Serial.printf("  Seq:     %u\n", seq);
    
    // Event-specific output
    if (strcmp(event, "entry") == 0) {
        Serial.println(F(">>> VEHICLE DETECTED <<<"));
    } else if (strcmp(event, "fault") == 0) {
        Serial.println(F(">>> LOOP FAULT DETECTED <<<"));
    }
    
    Serial.println(F("----------------------------------------"));
    Serial.println();
    
    // Publish to Home Assistant via MQTT
    if (mqttClient.connected()) {
        publishToHomeAssistant(event, seq);
    } else {
        Serial.println(F("[MQTT] Not connected - skipping publish"));
    }
    
    // Return to idle
    setRGBDim(Colors::IDLE, LED_BRIGHTNESS_DIM);
}

/**
 * Publish state updates to Home Assistant via MQTT
 */
void publishToHomeAssistant(const char* event, uint32_t seq) {
    String topic;
    String payload;
    
    // Update message count sensor
    topic = "homeassistant/sensor/gravelping/message_count/state";
    payload = String(messageCount);
    mqttClient.publish(topic.c_str(), payload.c_str());
    
    // Update last detection timestamp
    topic = "homeassistant/sensor/gravelping/last_detection/state";
    payload = String(millis() / 1000);  // Uptime in seconds
    mqttClient.publish(topic.c_str(), payload.c_str());
    
    // Update binary sensors based on event type
    if (strcmp(event, "entry") == 0) {
        topic = "homeassistant/binary_sensor/gravelping/vehicle/state";
        payload = "ON";
        if (mqttClient.publish(topic.c_str(), payload.c_str())) {
            Serial.println(F("[MQTT] ✓ Published vehicle detection"));
        }
        
        // Auto-off will happen via Home Assistant's off_delay setting
        
    } else if (strcmp(event, "fault") == 0) {
        topic = "homeassistant/binary_sensor/gravelping/loop_fault/state";
        payload = "ON";
        if (mqttClient.publish(topic.c_str(), payload.c_str())) {
            Serial.println(F("[MQTT] ✓ Published loop fault"));
        }
    }
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
