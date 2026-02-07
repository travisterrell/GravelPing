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
#include <ESPmDNS.h>
#include <FastLED.h>
#include <PubSubClient.h>
#include <WiFi.h>

// ============================================================================
// PIN DEFINITIONS
// ============================================================================

// Onboard LEDs (ESP32-C6 SuperMini)
constexpr int PIN_LED_STATUS = 15;  // Simple status LED (active HIGH)
constexpr int PIN_LED_RGB    = 8;   // WS2812 RGB LED

// LR02 LoRa Module Connections
constexpr int PIN_LORA_RX  = 17;  // ESP32-C6 RX (GPIO17) <- LR-02 TX (Pin 4)
constexpr int PIN_LORA_TX  = 16;  // ESP32-C6 TX (GPIO16) -> LR-02 RX (Pin 3)
constexpr int PIN_LORA_AUX = 18;  // LR-02 AUX pin (LOW = ready, HIGH = busy)

// ============================================================================
// CONFIGURATION (from platformio.ini build_flags)
// ============================================================================

// Stringify macro helper
#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

// Serial configuration
constexpr unsigned long SERIAL_BAUD      = 115200;  // Debug/output serial
constexpr unsigned long LORA_BAUD        = 9600;    // LR-02 default baud rate

// Device identification
constexpr const char* DEVICE_ID = "RX01";  // Receiver ID

// WiFi credentials (defined in platformio.ini)
#ifndef WIFI_SSID
#define WIFI_SSID YourSSID
#endif
#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD YourPassword
#endif

// MQTT configuration (defined in platformio.ini)
#ifndef MQTT_BROKER
#define MQTT_BROKER 192.168.1.100
#endif
#ifndef MQTT_PORT
#define MQTT_PORT 1883
#endif
#ifndef MQTT_USER
#define MQTT_USER 
#endif
#ifndef MQTT_PASSWORD
#define MQTT_PASSWORD 
#endif
#ifndef DEVICE_NAME
#define DEVICE_NAME GravelPingRx
#endif

// Convert macros to strings
const char* WIFI_SSID_STR = TOSTRING(WIFI_SSID);
const char* WIFI_PASSWORD_STR = TOSTRING(WIFI_PASSWORD);
const char* MQTT_BROKER_STR = TOSTRING(MQTT_BROKER);
const char* MQTT_USER_STR = TOSTRING(MQTT_USER);
const char* MQTT_PASSWORD_STR = TOSTRING(MQTT_PASSWORD);
const char* DEVICE_NAME_STR = TOSTRING(DEVICE_NAME);

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

// Connection state tracking
enum WiFiState {
    WIFI_DISCONNECTED,
    WIFI_CONNECTING,
    WIFI_CONNECTED
};

enum MQTTState {
    MQTT_STATE_DISCONNECTED,
    MQTT_STATE_CONNECTING,
    MQTT_STATE_CONNECTED
};

WiFiState wifiState = WIFI_DISCONNECTED;
MQTTState mqttState = MQTT_STATE_DISCONNECTED;
unsigned long lastWifiAttempt = 0;
unsigned long lastMqttAttempt = 0;
unsigned long wifiConnectStartTime = 0;
unsigned long mqttConnectStartTime = 0;
constexpr unsigned long WIFI_CONNECT_TIMEOUT = 10000;  // 10 seconds
constexpr unsigned long MQTT_CONNECT_TIMEOUT = 5000;   // 5 seconds
bool discoveryPublished = false;  // Track if we've published discovery messages

// ============================================================================
// FUNCTION DECLARATIONS
// ============================================================================

void setupLEDs();
void setupLoRa();
void setupWiFi();
void setupMQTT();
void manageWiFiConnection();
void startWiFiConnection();
void onWiFiConnected();
void manageMQTTConnection();
void startMQTTConnection();
void onMQTTConnected();
void publishDiscovery();
void processLoRaData();
void handleMessage(const String& message);
void publishToHomeAssistant(const char* event, uint32_t seq, float vbat);

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
    Serial.printf("[INIT] Device Name: %s\n", DEVICE_NAME_STR);
    Serial.println();
    
    // Initialize LoRa module
    setupLoRa();
    
    // Initialize WiFi and MQTT (but don't block on connections)
    setupWiFi();
    setupMQTT();
    
    // Trigger first connection attempt (non-blocking)
    wifiState = WIFI_DISCONNECTED;
    mqttState = MQTT_STATE_DISCONNECTED;
    lastWifiAttempt = millis() - WIFI_RECONNECT_INTERVAL;  // Trigger immediate attempt
    
    // Ready to receive
    Serial.println(F("[READY] Listening for messages..."));
    Serial.println(F("[INFO] WiFi/MQTT will connect in background"));
    Serial.println();
    setRGBDim(Colors::IDLE, LED_BRIGHTNESS_DIM);
}

// ============================================================================
// MAIN LOOP (Non-blocking)
// ============================================================================

void loop() {
    // ALWAYS process LoRa data first - highest priority, never blocked
    processLoRaData();
    
    // Non-blocking WiFi connection management
    manageWiFiConnection();
    
    // Non-blocking MQTT connection management (only if WiFi connected)
    if (wifiState == WIFI_CONNECTED) {
        manageMQTTConnection();
    }
    
    // Process MQTT loop (handles keepalive) - non-blocking
    if (mqttState == MQTT_STATE_CONNECTED) {
        mqttClient.loop();
    }
    
    delay(1);  // Minimal delay to prevent busy-waiting
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
// WIFI SETUP (Non-blocking)
// ============================================================================

void setupWiFi() {
    Serial.println(F("[WIFI] Initializing WiFi..."));
    WiFi.mode(WIFI_STA);
    WiFi.disconnect(true);  // Clear any previous connection
    delay(100);
    
    // Set hostname before connecting
    if (WiFi.setHostname(DEVICE_NAME_STR)) {
        Serial.printf("[WIFI] Hostname configured: %s\n", DEVICE_NAME_STR);
    } else {
        Serial.println(F("[WIFI] Failed to set hostname"));
    }
}

void manageWiFiConnection() {
    switch (wifiState) {
        case WIFI_DISCONNECTED:
            // Check if enough time has passed since last attempt
            if (millis() - lastWifiAttempt >= WIFI_RECONNECT_INTERVAL) {
                lastWifiAttempt = millis();
                startWiFiConnection();
            }
            break;
            
        case WIFI_CONNECTING:
            // Check connection status (non-blocking)
            if (WiFi.status() == WL_CONNECTED) {
                onWiFiConnected();
            } else if (millis() - wifiConnectStartTime >= WIFI_CONNECT_TIMEOUT) {
                // Timeout - give up and try again later
                Serial.println(F("[WIFI] Connection timeout"));
                wifiState = WIFI_DISCONNECTED;
                flashRGB(Colors::ERROR, 1, 100, 0);
                setRGBDim(Colors::IDLE, LED_BRIGHTNESS_DIM);
            }
            break;
            
        case WIFI_CONNECTED:
            // Monitor connection status
            if (WiFi.status() != WL_CONNECTED) {
                Serial.println(F("[WIFI] Connection lost"));
                wifiState = WIFI_DISCONNECTED;
                mqttState = MQTT_STATE_DISCONNECTED;  // MQTT also lost
                discoveryPublished = false;  // Need to republish discovery
            }
            break;
    }
}

void startWiFiConnection() {
    Serial.printf("[WIFI] Connecting to %s...\n", WIFI_SSID_STR);
    setRGB(Colors::WIFI);
    
    WiFi.begin(WIFI_SSID_STR, WIFI_PASSWORD_STR);
    wifiState = WIFI_CONNECTING;
    wifiConnectStartTime = millis();
}

void onWiFiConnected() {
    // Set hostname again after connection (some routers need this)
    WiFi.setHostname(DEVICE_NAME_STR);
    
    Serial.println(F("[WIFI] Connected!"));
    Serial.printf("[WIFI] Hostname: %s\n", WiFi.getHostname());
    Serial.printf("[WIFI] IP Address: %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("[WIFI] Gateway: %s\n", WiFi.gatewayIP().toString().c_str());
    Serial.printf("[WIFI] DNS: %s\n", WiFi.dnsIP().toString().c_str());
    Serial.printf("[WIFI] MAC: %s\n", WiFi.macAddress().c_str());
    Serial.printf("[WIFI] Signal: %d dBm\n", WiFi.RSSI());
    
    // Setup mDNS responder
    if (MDNS.begin(DEVICE_NAME_STR)) {
        Serial.printf("[MDNS] Responder started: %s.local\n", DEVICE_NAME_STR);
        MDNS.addService("http", "tcp", 80);
    } else {
        Serial.println(F("[MDNS] Failed to start responder"));
    }
    
    wifiState = WIFI_CONNECTED;
    setRGBDim(Colors::IDLE, LED_BRIGHTNESS_DIM);
    
    // Trigger MQTT connection attempt
    mqttState = MQTT_STATE_DISCONNECTED;
    lastMqttAttempt = millis() - MQTT_RECONNECT_INTERVAL;
}

// ============================================================================
// MQTT SETUP (Non-blocking)
// ============================================================================

void setupMQTT() {
    Serial.println(F("[MQTT] Initializing MQTT client..."));
    mqttClient.setServer(MQTT_BROKER_STR, MQTT_PORT);
    mqttClient.setBufferSize(512);  // Increase buffer for discovery messages
    Serial.printf("[MQTT] Broker: %s:%d\n", MQTT_BROKER_STR, MQTT_PORT);
}

void manageMQTTConnection() {
    switch (mqttState) {
        case MQTT_STATE_DISCONNECTED:
            // Check if enough time has passed since last attempt
            if (millis() - lastMqttAttempt >= MQTT_RECONNECT_INTERVAL) {
                lastMqttAttempt = millis();
                startMQTTConnection();
            }
            break;
            
        case MQTT_STATE_CONNECTING:
            // MQTT connect is somewhat blocking, but only takes 1-2 seconds
            // This is acceptable since it's rare (only during reconnects)
            // The timeout is handled by the library itself
            if (mqttClient.connected()) {
                onMQTTConnected();
            } else if (millis() - mqttConnectStartTime >= MQTT_CONNECT_TIMEOUT) {
                // Timeout - give up and try again later
                Serial.printf("[MQTT] Connection timeout, state: %d\n", mqttClient.state());
                mqttState = MQTT_STATE_DISCONNECTED;
            }
            break;
            
        case MQTT_STATE_CONNECTED:
            // Monitor connection status
            if (!mqttClient.connected()) {
                Serial.println(F("[MQTT] Connection lost"));
                mqttState = MQTT_STATE_DISCONNECTED;
                discoveryPublished = false;  // Need to republish discovery
            }
            break;
    }
}

void startMQTTConnection() {
    Serial.println(F("[MQTT] Connecting to broker..."));
    setRGB(Colors::MQTT);
    
    String clientId = "gravelping-rx-";
    clientId += String(random(0xffff), HEX);
    
    // This call blocks for 1-2 seconds, but that's acceptable for reconnection
    bool connected = false;
    if (strlen(MQTT_USER_STR) > 0) {
        connected = mqttClient.connect(clientId.c_str(), MQTT_USER_STR, MQTT_PASSWORD_STR);
    } else {
        connected = mqttClient.connect(clientId.c_str());
    }
    
    if (connected) {
        Serial.println(F("[MQTT] Connected!"));
        Serial.printf("[MQTT] Client ID: %s\n", clientId.c_str());
        mqttState = MQTT_STATE_CONNECTED;
        mqttConnectStartTime = millis();
        setRGBDim(Colors::IDLE, LED_BRIGHTNESS_DIM);
        
        // Publish discovery if not done yet
        if (!discoveryPublished) {
            publishDiscovery();
            discoveryPublished = true;
        }
    } else {
        Serial.printf("[MQTT] Connection failed! State: %d\n", mqttClient.state());
        mqttState = MQTT_STATE_DISCONNECTED;
        mqttConnectStartTime = millis();
        flashRGB(Colors::ERROR, 1, 100, 0);
        setRGBDim(Colors::IDLE, LED_BRIGHTNESS_DIM);
    }
}

void onMQTTConnected() {
    Serial.println(F("[MQTT] Connected (state check)"));
    mqttState = MQTT_STATE_CONNECTED;
    setRGBDim(Colors::IDLE, LED_BRIGHTNESS_DIM);
    
    // Publish discovery if not done yet
    if (!discoveryPublished) {
        publishDiscovery();
        discoveryPublished = true;
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
    device["name"] = DEVICE_NAME_STR;
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
    device["name"] = DEVICE_NAME_STR;
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
    device["name"] = DEVICE_NAME_STR;
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
    // Sensor: Message Count
    // =========================================================================
    doc.clear();
    doc["name"] = "Message Count";
    doc["unique_id"] = "gravelping_message_count";
    doc["state_topic"] = "homeassistant/sensor/gravelping/message_count/state";
    doc["icon"] = "mdi:counter";
    
    device = doc["device"].to<JsonObject>();
    device["identifiers"][0] = "gravelping";
    device["name"] = DEVICE_NAME_STR;
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
    
    // =========================================================================
    // Sensor: Battery Voltage
    // =========================================================================
    doc.clear();
    doc["name"] = "Battery Voltage";
    doc["unique_id"] = "gravelping_battery_voltage";
    doc["state_topic"] = "homeassistant/sensor/gravelping/battery_voltage/state";
    doc["device_class"] = "voltage";
    doc["unit_of_measurement"] = "V";
    doc["state_class"] = "measurement";
    doc["icon"] = "mdi:battery";
    
    device = doc["device"].to<JsonObject>();
    device["identifiers"][0] = "gravelping";
    device["name"] = DEVICE_NAME_STR;
    device["model"] = "GravelPing Transmitter";
    device["manufacturer"] = "Custom";

    payload = "";
    serializeJson(doc, payload);
    
    topic = "homeassistant/sensor/gravelping/battery_voltage/config";
    
    if (mqttClient.publish(topic.c_str(), payload.c_str(), true)) {
        Serial.println(F("[MQTT]   ✓ Battery voltage sensor"));
    } else {
        Serial.println(F("[MQTT]   ✗ Failed to publish battery voltage sensor"));
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
    float       vbat    = doc["vbat"]    | 0.0;
    
    // Print parsed data
    Serial.println(F("[PARSED]"));
    Serial.printf("  Event:   %s\n", event);
    Serial.printf("  Seq:     %u\n", seq);
    if (vbat > 0.0) {
        Serial.printf("  VBat:    %.1fV\n", vbat);
    }
    
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
        publishToHomeAssistant(event, seq, vbat);
    } else {
        Serial.println(F("[MQTT] Not connected - skipping publish"));
    }
    
    // Return to idle
    setRGBDim(Colors::IDLE, LED_BRIGHTNESS_DIM);
}

/**
 * Publish state updates to Home Assistant via MQTT
 */
void publishToHomeAssistant(const char* event, uint32_t seq, float vbat) {
    String topic;
    String payload;
    
    // Update message count sensor
    topic = "homeassistant/sensor/gravelping/message_count/state";
    payload = String(messageCount);
    mqttClient.publish(topic.c_str(), payload.c_str());
    
    // Update battery voltage sensor (if available)
    if (vbat > 0.0) {
        topic = "homeassistant/sensor/gravelping/battery_voltage/state";
        payload = String(vbat, 1);  // 1 decimal place
        if (mqttClient.publish(topic.c_str(), payload.c_str())) {
            Serial.printf("[MQTT] ✓ Published battery voltage: %.1fV\n", vbat);
        }
    }
    
    // Update binary sensors based on event type
    if (strcmp(event, "entry") == 0) {
        topic = "homeassistant/binary_sensor/gravelping/vehicle/state";
        payload = "ON";
        if (mqttClient.publish(topic.c_str(), payload.c_str())) {
            Serial.println(F("[MQTT] ✓ Published vehicle detection"));
        }
        
        // Auto-clear any loop fault (vehicle detection proves loop is working)
        topic = "homeassistant/binary_sensor/gravelping/loop_fault/state";
        payload = "OFF";
        if (mqttClient.publish(topic.c_str(), payload.c_str())) {
            Serial.println(F("[MQTT] ✓ Loop fault auto-cleared (vehicle detected)"));
        }
        
    } else if (strcmp(event, "fault") == 0) {
        topic = "homeassistant/binary_sensor/gravelping/loop_fault/state";
        payload = "ON";
        if (mqttClient.publish(topic.c_str(), payload.c_str())) {
            Serial.println(F("[MQTT] ✓ Published loop fault"));
        }
        
    } else if (strcmp(event, "clear") == 0) {
        topic = "homeassistant/binary_sensor/gravelping/loop_fault/state";
        payload = "OFF";
        if (mqttClient.publish(topic.c_str(), payload.c_str())) {
            Serial.println(F("[MQTT] ✓ Loop fault cleared"));
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
