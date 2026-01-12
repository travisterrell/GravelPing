/**
 * GravelPing - Driveway Monitor Receiver (ESP32-S3 Dual-Core)
 * 
 * Optimized for ESP32-S3 Super Mini with dual-core architecture.
 * 
 * DUAL-CORE ARCHITECTURE:
 *   Core 0 (PRO_CPU): WiFi & MQTT management (async, non-blocking)
 *   Core 1 (APP_CPU): LoRa message reception (time-critical, never blocked)
 * 
 * This separation ensures WiFi/MQTT operations never interfere with LoRa reception.
 * Messages are queued from Core 1 to Core 0 for MQTT publishing.
 * 
 * HOME ASSISTANT HEARTBEAT MONITORING:
 *   - Subscribes to homeassistant/heartbeat (published by HA every 10s)
 *   - Monitors heartbeat freshness (timeout: 35s)
 *   - Falls back to local alert if HA is unavailable
 *   - Always publishes to MQTT (for logging/recovery)
 * 
 * Hardware:
 *   - ESP32-S3 Super Mini (Dual-Core Xtensa LX7 @ 240MHz, 4MB Flash)
 *   - DX-LR02-900T22D LoRa UART Module
 *   - WS2812 RGB LED (GPIO48)
 * 
 * LED Status:
 *   - GREEN dim:    Idle, ready
 *   - BLUE flash:   Message received
 *   - CYAN:         System boot
 *   - YELLOW:       WiFi connecting
 *   - MAGENTA:      MQTT connecting
 * 
 * MQTT Library:
 *   - espMqttClient (ESP32-native, async, non-blocking)
 *   - Handles reconnections automatically in background tasks
 *   - Compatible with watchdog timer (operations don't block)
 */

#include <Arduino.h>
#include <ArduinoJson.h>
#include <ESPmDNS.h>
#include <FastLED.h>
#include <espMqttClient.h>
#include <WiFi.h>

// ============================================================================
// PIN DEFINITIONS (ESP32-S3 Super Mini)
// ============================================================================

constexpr int PIN_LED_RGB = 48;  // WS2812 RGB LED

// LoRa Module UART
constexpr int PIN_LORA_RX  = 17;  // ESP32-S3 RX (GPIO17) <- LR-02 TX (Pin 4)
constexpr int PIN_LORA_TX  = 16;  // ESP32-S3 TX (GPIO16) -> LR-02 RX (Pin 3)
constexpr int PIN_LORA_AUX = 18;  // LR-02 AUX pin (LOW = ready, HIGH = busy)

// ============================================================================
// CONFIGURATION (from platformio.ini build_flags)
// ============================================================================

// Stringify macro helper
#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

// Serial configuration
constexpr unsigned long SERIAL_BAUD = 115200;  // Debug/output serial
constexpr unsigned long LORA_BAUD   = 9600;    // LR-02 default baud rate

// Device identification
constexpr const char* DEVICE_ID = "RX02-S3";  // Receiver ID (S3 variant)

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
#define DEVICE_NAME GravelPing-S3
#endif

// Convert macros to strings
const char* WIFI_SSID_STR = TOSTRING(WIFI_SSID);
const char* WIFI_PASSWORD_STR = TOSTRING(WIFI_PASSWORD);
const char* MQTT_BROKER_STR = TOSTRING(MQTT_BROKER);
const char* MQTT_USER_STR = TOSTRING(MQTT_USER);
const char* MQTT_PASSWORD_STR = TOSTRING(MQTT_PASSWORD);
const char* DEVICE_NAME_STR = TOSTRING(DEVICE_NAME);

// RGB LED configuration
constexpr int NUM_LEDS           = 1;
constexpr int LED_BRIGHTNESS     = 50;  // 0-255, normal brightness
constexpr int LED_BRIGHTNESS_DIM = 10;  // Dim brightness for idle

// WiFi & MQTT configuration
constexpr unsigned long WIFI_CONNECT_TIMEOUT  = 20000;  // 20 seconds
constexpr unsigned long MQTT_CONNECT_TIMEOUT  = 10000;  // 10 seconds
constexpr unsigned long MQTT_RECONNECT_DELAY  = 5000;   // 5 seconds between reconnection attempts

// Home Assistant heartbeat monitoring
constexpr unsigned long HA_HEARTBEAT_TIMEOUT  = 25000;  // 25 seconds (HA publishes every 10s)

// ============================================================================
// GLOBAL OBJECTS & STATE
// ============================================================================

// Hardware Serial for LoRa (UART1)
HardwareSerial LoRaSerial(1);

// FastLED for RGB LED
CRGB leds[NUM_LEDS];

// espMqttClient (ESP32-native MQTT client)
espMqttClient mqttClient;

// Message tracking
uint32_t messageCount = 0;
unsigned long lastMessageTime = 0;

// FreeRTOS synchronization
SemaphoreHandle_t mqttMutex;         // Protects MQTT client access
QueueHandle_t messageQueue;           // LoRa messages from Core 1 to Core 0

// Message structure for inter-core communication
struct LoRaMessage {
    char event[16];
    uint32_t seq;
    float vbat;
    unsigned long timestamp;
};

// Network state (managed by Core 0)
volatile bool wifiConnected = false;
volatile bool mqttConnected = false;
volatile unsigned long lastWifiAttempt = 0;
volatile unsigned long lastMqttAttempt = 0;

// Home Assistant heartbeat tracking
volatile unsigned long lastHAHeartbeat = 0;
volatile bool haAvailable = false;

// ============================================================================
// COLOR DEFINITIONS
// ============================================================================

namespace Colors {
    constexpr CRGB IDLE     = CRGB::Green;
    constexpr CRGB RECEIVED = CRGB::Blue;
    constexpr CRGB ERROR    = CRGB::Red;
    constexpr CRGB BOOT     = CRGB::Cyan;
    constexpr CRGB WIFI     = CRGB::Yellow;
    constexpr CRGB MQTT     = CRGB::Magenta;
}

// ============================================================================
// FUNCTION DECLARATIONS
// ============================================================================

// Setup functions
void setupLEDs();
void setupLoRa();

// Core 0 task (WiFi/MQTT management)
void networkTask(void* parameter);
void setupWiFi();
void setupMQTT();
void maintainWiFi();
void maintainMQTT();
void checkHAHeartbeat();
void onMqttMessage(const espMqttClientTypes::MessageProperties& properties, const char* topic, const uint8_t* payload, size_t len, size_t index, size_t total);
void publishDiscovery();
void publishToHomeAssistant(const char* event, uint32_t seq, float vbat);

// Core 1 functions (LoRa message handling)
void handleLoRaMessages();
void handleMessage(const char* jsonStr);

// Helper function for MQTT publish
bool publishMQTT(const char* topic, const char* payload, bool retain = false);

// LED functions
void setRGB(CRGB color);
void setRGBDim(CRGB color, int brightness);
void flashRGB(CRGB color, int times, int onMs, int offMs);

// ============================================================================
// SETUP (Runs on Core 1 by default)
// ============================================================================

void setup() {
    // Initialize serial
    Serial.begin(SERIAL_BAUD);
    delay(1000);  // Give serial time to initialize
    
    Serial.println(F("\n\n========================================"));
    Serial.println(F("   GravelPing Receiver - ESP32-S3"));
    Serial.println(F("   Dual-Core Architecture"));
    Serial.println(F("========================================"));
    Serial.printf("Device ID: %s\n", DEVICE_ID);
    Serial.printf("Core: %d (APP_CPU)\n", xPortGetCoreID());
    Serial.println(F("----------------------------------------\n"));
    
    // Initialize LEDs
    setupLEDs();
    setRGB(Colors::BOOT);
    
    // Initialize LoRa
    setupLoRa();
    
    // Create synchronization primitives
    mqttMutex = xSemaphoreCreateMutex();
    messageQueue = xQueueCreate(10, sizeof(LoRaMessage));  // Queue up to 10 messages
    
    if (mqttMutex == NULL || messageQueue == NULL) {
        Serial.println(F("[ERROR] Failed to create synchronization primitives!"));
        flashRGB(Colors::ERROR, 10, 200, 200);
        ESP.restart();
    }
    
    // Create network task on Core 0 (PRO_CPU)
    // This task handles WiFi and MQTT in background
    xTaskCreatePinnedToCore(
        networkTask,      // Task function
        "NetworkTask",    // Task name
        16384,           // Stack size (bytes) - increased for MQTT operations
        NULL,            // Parameters
        2,               // Priority (2 = above idle, prevents watchdog)
        NULL,            // Task handle
        0                // Core 0 (PRO_CPU)
    );
    
    Serial.println(F("[INIT] Network task started on Core 0 (PRO_CPU)"));
    Serial.println(F("[INIT] Main loop will handle LoRa on Core 1 (APP_CPU)"));
    Serial.println(F("[INIT] Setup complete\n"));
    
    // Show ready status
    setRGBDim(Colors::IDLE, LED_BRIGHTNESS_DIM);
}

// ============================================================================
// MAIN LOOP (Runs on Core 1 - APP_CPU)
// ============================================================================

void loop() {
    // Core 1: Focus solely on LoRa message reception
    // This ensures we never miss messages due to WiFi/MQTT delays
    handleLoRaMessages();
    
    // Small delay to prevent task starvation
    delay(1);
}

// ============================================================================
// CORE 0 TASK - NETWORK MANAGEMENT (PRO_CPU)
// ============================================================================

void networkTask(void* parameter) {
    Serial.printf("[NETWORK] Task started on Core %d (PRO_CPU)\n", xPortGetCoreID());
    
    // Initialize WiFi
    setupWiFi();
    
    // Initialize MQTT (will auto-reconnect if WiFi fails initially)
    setupMQTT();
    
    // Process queued messages and maintain connections
    while (true) {
        // Maintain WiFi connection
        maintainWiFi();
        
        // Maintain MQTT connection (if WiFi is up)
        if (wifiConnected) {
            maintainMQTT();
        }
        
        // Check Home Assistant heartbeat status
        checkHAHeartbeat();
        
        // Process queued LoRa messages
        LoRaMessage msg;
        if (xQueueReceive(messageQueue, &msg, 10 / portTICK_PERIOD_MS) == pdTRUE) {
            // Got a message from Core 1, publish to MQTT
            if (mqttConnected && xSemaphoreTake(mqttMutex, 100 / portTICK_PERIOD_MS) == pdTRUE) {
                publishToHomeAssistant(msg.event, msg.seq, msg.vbat);
                xSemaphoreGive(mqttMutex);
            } else {
                Serial.println(F("[NETWORK] WARNING: Could not publish message (MQTT not ready)"));
            }
        }
        
        // espMqttClient handles all background operations (no loop() needed)
        delay(10);
    }
}

// ============================================================================
// LED SETUP
// ============================================================================

void setupLEDs() {
    Serial.println(F("[INIT] Initializing RGB LED..."));
    
    // Configure FastLED for WS2812 on GPIO48
    FastLED.addLeds<NEOPIXEL, PIN_LED_RGB>(leds, NUM_LEDS);
    FastLED.setBrightness(LED_BRIGHTNESS);
    
    // Test LED
    flashRGB(Colors::BOOT, 2, 200, 100);
    
    Serial.println(F("[INIT] ✓ RGB LED ready"));
}

// ============================================================================
// LORA SETUP
// ============================================================================

void setupLoRa() {
    Serial.println(F("[INIT] Initializing LoRa UART..."));
    
    // Initialize UART1 for LoRa communication
    LoRaSerial.begin(LORA_BAUD, SERIAL_8N1, PIN_LORA_RX, PIN_LORA_TX);
    
    // Configure AUX pin as input
    pinMode(PIN_LORA_AUX, INPUT);
    
    Serial.println(F("[INIT] ✓ LoRa UART ready"));
    Serial.printf("       - LoRa TX (ESP RX): GPIO%d\n", PIN_LORA_TX);
    Serial.printf("       - LoRa RX (ESP TX): GPIO%d\n", PIN_LORA_RX);
    Serial.printf("       - LoRa AUX:         GPIO%d\n", PIN_LORA_AUX);
}

// ============================================================================
// WIFI SETUP & MAINTENANCE
// ============================================================================

void setupWiFi() {
    Serial.println(F("[WIFI] Connecting to WiFi..."));
    Serial.printf("       SSID: %s\n", WIFI_SSID_STR);
    
    setRGB(Colors::WIFI);
    
    WiFi.mode(WIFI_STA);
    WiFi.disconnect(true);  // Clear any previous connection
    delay(100);
    
    // Set hostname before connecting
    if (WiFi.setHostname(DEVICE_NAME_STR)) {
        Serial.printf("[WIFI] Hostname configured: %s\n", DEVICE_NAME_STR);
    } else {
        Serial.println(F("[WIFI] Failed to set hostname"));
    }
    
    WiFi.setSleep(false);  // Disable WiFi power save (improves stability)
    WiFi.begin(WIFI_SSID_STR, WIFI_PASSWORD_STR);
    
    unsigned long startTime = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - startTime) < WIFI_CONNECT_TIMEOUT) {
        delay(500);
        Serial.print(".");
        vTaskDelay(1);  // Yield to other tasks
    }
    Serial.println();
    
    if (WiFi.status() == WL_CONNECTED) {
        wifiConnected = true;
        
        // Set hostname again after connection (some routers need this)
        WiFi.setHostname(DEVICE_NAME_STR);
        
        Serial.println(F("[WIFI] ✓ Connected"));
        Serial.printf("       Hostname: %s\n", WiFi.getHostname());
        Serial.printf("       IP: %s\n", WiFi.localIP().toString().c_str());
        Serial.printf("       RSSI: %d dBm\n", WiFi.RSSI());
        
        // Setup mDNS
        if (MDNS.begin(DEVICE_NAME_STR)) {
            Serial.printf("[MDNS] ✓ Hostname: %s.local\n", DEVICE_NAME_STR);
        }
    } else {
        wifiConnected = false;
        Serial.println(F("[WIFI] ✗ Connection failed - will retry"));
        flashRGB(Colors::ERROR, 3, 200, 200);
    }
    
    lastWifiAttempt = millis();
}

void maintainWiFi() {
    if (WiFi.status() == WL_CONNECTED) {
        if (!wifiConnected) {
            wifiConnected = true;
            Serial.println(F("[WIFI] ✓ Reconnected"));
            Serial.printf("       IP: %s\n", WiFi.localIP().toString().c_str());
        }
    } else {
        if (wifiConnected) {
            wifiConnected = false;
            mqttConnected = false;  // MQTT depends on WiFi
            Serial.println(F("[WIFI] ✗ Connection lost"));
        }
        
        // Attempt reconnection (with rate limiting)
        if (millis() - lastWifiAttempt > MQTT_RECONNECT_DELAY) {
            Serial.println(F("[WIFI] Attempting reconnection..."));
            WiFi.disconnect();
            WiFi.begin(WIFI_SSID_STR, WIFI_PASSWORD_STR);
            lastWifiAttempt = millis();
        }
    }
}

// ============================================================================
// MQTT SETUP & MAINTENANCE
// ============================================================================

void setupMQTT() {
    if (!wifiConnected) {
        Serial.println(F("[MQTT] Waiting for WiFi..."));
        return;
    }
    
    Serial.println(F("[MQTT] Connecting to MQTT broker..."));
    Serial.printf("       Broker: %s:%d\n", MQTT_BROKER_STR, MQTT_PORT);
    
    setRGB(Colors::MQTT);
    
    // Configure espMqttClient
    mqttClient.setServer(MQTT_BROKER_STR, MQTT_PORT);
    mqttClient.setCredentials(MQTT_USER_STR, MQTT_PASSWORD_STR);
    mqttClient.setKeepAlive(15);
    
    // Set up message callback for subscriptions
    mqttClient.onMessage(onMqttMessage);
    
    // Generate client ID
    String clientId = "gravelping-s3-";
    clientId += String(random(0xffff), HEX);
    mqttClient.setClientId(clientId.c_str());
    
    Serial.printf("[MQTT] Client ID: %s\n", clientId.c_str());
    Serial.println(F("[MQTT] Connecting..."));
    
    // Connect (async, non-blocking)
    mqttClient.connect();
    
    lastMqttAttempt = millis();
}

void maintainMQTT() {
    // espMqttClient is async - connection status updates automatically
    bool isConnected = mqttClient.connected();
    
    if (isConnected) {
        if (!mqttConnected) {
            mqttConnected = true;
            Serial.println(F("[MQTT] ✓ Connected"));
            
            // Subscribe to Home Assistant heartbeat
            if (mqttClient.subscribe("homeassistant/heartbeat", 0)) {
                Serial.println(F("[MQTT] ✓ Subscribed to HA heartbeat"));
            } else {
                Serial.println(F("[MQTT] ✗ Failed to subscribe to HA heartbeat"));
            }
            
            // Publish Home Assistant autodiscovery
            if (xSemaphoreTake(mqttMutex, 1000 / portTICK_PERIOD_MS) == pdTRUE) {
                publishDiscovery();
                xSemaphoreGive(mqttMutex);
            }
            
            setRGBDim(Colors::IDLE, LED_BRIGHTNESS_DIM);
        }
    } else {
        if (mqttConnected) {
            mqttConnected = false;
            haAvailable = false;  // HA heartbeat lost when MQTT disconnects
            Serial.println(F("[MQTT] ✗ Connection lost"));
        }
        
        // Attempt reconnection (with rate limiting)
        if (millis() - lastMqttAttempt > MQTT_RECONNECT_DELAY) {
            Serial.println(F("[MQTT] Attempting reconnection..."));
            setupMQTT();
        }
    }
}

// ============================================================================
// MQTT MESSAGE CALLBACK
// ============================================================================

void onMqttMessage(const espMqttClientTypes::MessageProperties& properties, const char* topic, const uint8_t* payload, size_t len, size_t index, size_t total) {
    // Handle incoming MQTT messages
    if (strcmp(topic, "homeassistant/heartbeat") == 0) {
        // Update heartbeat timestamp
        lastHAHeartbeat = millis();
        
        // Mark HA as available if this is first heartbeat or was previously down
        if (!haAvailable) {
            haAvailable = true;
            Serial.println(F("[HA] ✓ Home Assistant available"));
        }
    }
}

// ============================================================================
// HOME ASSISTANT HEARTBEAT MONITORING
// ============================================================================

void checkHAHeartbeat() {
    // Only check if MQTT is connected
    if (!mqttConnected) {
        if (haAvailable) {
            haAvailable = false;
            Serial.println(F("[HA] ✗ HA unavailable (MQTT disconnected)"));
        }
        return;
    }
    
    // If we've never received a heartbeat, don't mark as down yet
    if (lastHAHeartbeat == 0) {
        return;
    }
    
    // Check if heartbeat is stale
    unsigned long timeSinceHeartbeat = millis() - lastHAHeartbeat;
    
    if (timeSinceHeartbeat > HA_HEARTBEAT_TIMEOUT) {
        if (haAvailable) {
            haAvailable = false;
            Serial.println(F("[HA] ✗ Home Assistant heartbeat timeout"));
            Serial.printf("[HA]    Last heartbeat: %lu ms ago\n", timeSinceHeartbeat);
        }
    } else {
        if (!haAvailable) {
            haAvailable = true;
            Serial.println(F("[HA] ✓ Home Assistant recovered"));
        }
    }
}

// ============================================================================
// MQTT PUBLISHING HELPER
// ============================================================================

bool publishMQTT(const char* topic, const char* payload, bool retain) {
    // espMqttClient API: publish(topic, qos, retain, payload, length, dup, message_id)
    // For our use case: qos=0, dup=false, message_id=0
    uint16_t packetId = mqttClient.publish(topic, 0, retain, (const uint8_t*)payload, strlen(payload));
    return packetId > 0;
}

// ============================================================================
// HOME ASSISTANT AUTODISCOVERY
// ============================================================================

void publishDiscovery() {
    Serial.println(F("[MQTT] Publishing Home Assistant autodiscovery..."));
    
    JsonDocument doc;
    JsonObject device;
    String topic;
    String payload;
    
    // =========================================================================
    // Binary Sensor: Vehicle Detection
    // =========================================================================
    doc.clear();
    doc["name"] = "Vehicle Detected";
    doc["unique_id"] = "gravelping_s3_vehicle";
    doc["state_topic"] = "homeassistant/binary_sensor/gravelping_s3/vehicle/state";
    doc["device_class"] = "occupancy";
    doc["payload_on"] = "ON";
    doc["payload_off"] = "OFF";
    doc["off_delay"] = 5;  // Auto-off after 5 seconds

    device = doc["device"].to<JsonObject>();
    device["identifiers"][0] = "gravelping_s3";
    device["name"] = DEVICE_NAME_STR;
    device["model"] = "GravelPing Transmitter (S3 RX)";
    device["manufacturer"] = "Custom";

    payload = "";
    serializeJson(doc, payload);

    topic = "homeassistant/binary_sensor/gravelping_s3/vehicle/config";
    
    if (publishMQTT(topic.c_str(), payload.c_str(), true)) {
        Serial.println(F("[MQTT]   ✓ Vehicle detection binary sensor"));
    } else {
        Serial.println(F("[MQTT]   ✗ Failed to publish vehicle sensor"));
    }
    
    // =========================================================================
    // Binary Sensor: Loop Fault
    // =========================================================================
    doc.clear();
    doc["name"] = "Loop Fault";
    doc["unique_id"] = "gravelping_s3_loop_fault";
    doc["state_topic"] = "homeassistant/binary_sensor/gravelping_s3/loop_fault/state";
    doc["device_class"] = "problem";
    doc["payload_on"] = "ON";
    doc["payload_off"] = "OFF";

    device = doc["device"].to<JsonObject>();
    device["identifiers"][0] = "gravelping_s3";
    device["name"] = DEVICE_NAME_STR;
    device["model"] = "GravelPing Transmitter (S3 RX)";
    device["manufacturer"] = "Custom";

    payload = "";
    serializeJson(doc, payload);

    topic = "homeassistant/binary_sensor/gravelping_s3/loop_fault/config";
    
    if (publishMQTT(topic.c_str(), payload.c_str(), true)) {
        Serial.println(F("[MQTT]   ✓ Loop fault binary sensor"));
    } else {
        Serial.println(F("[MQTT]   ✗ Failed to publish loop fault sensor"));
    }
    
    // =========================================================================
    // Sensor: Message Count
    // =========================================================================
    doc.clear();
    doc["name"] = "Message Count";
    doc["unique_id"] = "gravelping_s3_message_count";
    doc["state_topic"] = "homeassistant/sensor/gravelping_s3/message_count/state";
    doc["icon"] = "mdi:counter";

    device = doc["device"].to<JsonObject>();
    device["identifiers"][0] = "gravelping_s3";
    device["name"] = DEVICE_NAME_STR;
    device["model"] = "GravelPing Transmitter (S3 RX)";
    device["manufacturer"] = "Custom";

    payload = "";
    serializeJson(doc, payload);

    topic = "homeassistant/sensor/gravelping_s3/message_count/config";
    
    if (publishMQTT(topic.c_str(), payload.c_str(), true)) {
        Serial.println(F("[MQTT]   ✓ Message count sensor"));
    } else {
        Serial.println(F("[MQTT]   ✗ Failed to publish message count sensor"));
    }
    
    // =========================================================================
    // Sensor: Battery Voltage
    // =========================================================================
#ifdef ENABLE_BATTERY_MONITORING
    doc.clear();
    doc["name"] = "Battery Voltage";
    doc["unique_id"] = "gravelping_s3_battery_voltage";
    doc["state_topic"] = "homeassistant/sensor/gravelping_s3/battery_voltage/state";
    doc["device_class"] = "voltage";
    doc["unit_of_measurement"] = "V";
    doc["state_class"] = "measurement";

    device = doc["device"].to<JsonObject>();
    device["identifiers"][0] = "gravelping_s3";
    device["name"] = DEVICE_NAME_STR;
    device["model"] = "GravelPing Transmitter (S3 RX)";
    device["manufacturer"] = "Custom";

    payload = "";
    serializeJson(doc, payload);

    topic = "homeassistant/sensor/gravelping_s3/battery_voltage/config";
    
    if (publishMQTT(topic.c_str(), payload.c_str(), true)) {
        Serial.println(F("[MQTT]   ✓ Battery voltage sensor"));
    } else {
        Serial.println(F("[MQTT]   ✗ Failed to publish battery voltage sensor"));
    }
#endif
    
    Serial.println(F("[MQTT] ✓ Autodiscovery complete"));
}

// ============================================================================
// LORA MESSAGE HANDLING (Core 1 - Time Critical)
// ============================================================================

void handleLoRaMessages() {
    // Check if data available on LoRa UART
    if (LoRaSerial.available()) {
        String jsonStr = LoRaSerial.readStringUntil('\n');
        jsonStr.trim();
        
        if (jsonStr.length() > 0) {
            handleMessage(jsonStr.c_str());
        }
    }
}

void handleMessage(const char* jsonStr) {
    // Parse JSON on Core 1 (fast, time-critical)
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, jsonStr);
    
    if (error) {
        // Ignore noise/garbage from LoRa module startup
        if (strlen(jsonStr) < 10) {
            return;  // Silently ignore short garbage
        }
        Serial.println(F("[WARN] JSON parsing failed (LoRa noise/garbage)"));
        Serial.printf("       Raw: %s\n", jsonStr);
        return;  // Don't flash LED for startup noise
    }
    
    // Extract fields
    const char* event = doc["event"] | "unknown";
    uint32_t seq = doc["seq"] | 0;
    float vbat = doc["vbat"] | 0.0f;
    
    // Update message count
    messageCount++;
    lastMessageTime = millis();
    
    // Visual feedback
    flashRGB(Colors::RECEIVED, 1, 100, 0);
    
    // Log to serial
    Serial.println(F("========================================"));
    Serial.println(F("[MESSAGE RECEIVED]"));
    Serial.println(F("[RAW]"));
    Serial.println(jsonStr);
    Serial.println(F("[PARSED]"));
    Serial.printf("  Event:   %s\n", event);
    Serial.printf("  Seq:     %u\n", seq);
#ifdef ENABLE_BATTERY_MONITORING
    if (vbat > 0.0) {
        Serial.printf("  VBat:    %.1fV\n", vbat);
    }
#endif
    
    // Event-specific output
    if (strcmp(event, "entry") == 0) {
        Serial.println(F(">>> VEHICLE DETECTED <<<"));
    } else if (strcmp(event, "fault") == 0) {
        Serial.println(F(">>> LOOP FAULT DETECTED <<<"));
    }
    
    Serial.println(F("----------------------------------------"));
    Serial.println();
    
    // Queue message for Core 0 to publish via MQTT
    LoRaMessage msg;
    strncpy(msg.event, event, sizeof(msg.event) - 1);
    msg.event[sizeof(msg.event) - 1] = '\0';
    msg.seq = seq;
    msg.vbat = vbat;
    msg.timestamp = millis();
    
    if (xQueueSend(messageQueue, &msg, 0) != pdTRUE) {
        Serial.println(F("[WARNING] Message queue full - dropping message"));
    }
    
    // Return to idle
    setRGBDim(Colors::IDLE, LED_BRIGHTNESS_DIM);
}

// ============================================================================
// MQTT PUBLISHING (Core 0)
// ============================================================================

void publishToHomeAssistant(const char* event, uint32_t seq, float vbat) {
    String topic;
    String payload;
    
    // Check Home Assistant availability for alert decisions
    bool shouldPlayLocalAlert = false;
    if (!haAvailable && strcmp(event, "entry") == 0) {
        shouldPlayLocalAlert = true;
        Serial.println(F("[ALERT] HA unavailable - local alert needed"));
        // TODO: Play local sound when speaker hardware is added (PWM/I2S)
    }
    
    // Always publish to MQTT (even if HA is down, for logging/recovery)
    
    // Update message count sensor
    topic = "homeassistant/sensor/gravelping_s3/message_count/state";
    payload = String(messageCount);
    publishMQTT(topic.c_str(), payload.c_str());
    
    // Update battery voltage sensor (if available)
#ifdef ENABLE_BATTERY_MONITORING
    if (vbat > 0.0) {
        topic = "homeassistant/sensor/gravelping_s3/battery_voltage/state";
        payload = String(vbat, 1);  // 1 decimal place
        if (publishMQTT(topic.c_str(), payload.c_str())) {
            Serial.printf("[MQTT] ✓ Published battery voltage: %.1fV\n", vbat);
        }
    }
#endif
    
    // Update binary sensors based on event type
    if (strcmp(event, "entry") == 0) {
        topic = "homeassistant/binary_sensor/gravelping_s3/vehicle/state";
        payload = "ON";
        if (publishMQTT(topic.c_str(), payload.c_str())) {
            Serial.println(F("[MQTT] ✓ Published vehicle detection"));
        }
        
        // Auto-clear any loop fault (vehicle detection proves loop is working)
        topic = "homeassistant/binary_sensor/gravelping_s3/loop_fault/state";
        payload = "OFF";
        if (publishMQTT(topic.c_str(), payload.c_str())) {
            Serial.println(F("[MQTT] ✓ Loop fault auto-cleared (vehicle detected)"));
        }
        
    } else if (strcmp(event, "fault") == 0) {
        topic = "homeassistant/binary_sensor/gravelping_s3/loop_fault/state";
        payload = "ON";
        if (publishMQTT(topic.c_str(), payload.c_str())) {
            Serial.println(F("[MQTT] ✓ Published loop fault"));
        }
        
    } else if (strcmp(event, "clear") == 0) {
        topic = "homeassistant/binary_sensor/gravelping_s3/loop_fault/state";
        payload = "OFF";
        if (publishMQTT(topic.c_str(), payload.c_str())) {
            Serial.println(F("[MQTT] ✓ Loop fault cleared"));
        }
    }
}

// ============================================================================
// LED FUNCTIONS
// ============================================================================

void setRGB(CRGB color) {
    FastLED.setBrightness(LED_BRIGHTNESS);
    leds[0] = color;
    FastLED.show();
}

void setRGBDim(CRGB color, int brightness) {
    FastLED.setBrightness(brightness);
    leds[0] = color;
    FastLED.show();
}

void flashRGB(CRGB color, int times, int onMs, int offMs) {
    for (int i = 0; i < times; i++) {
        setRGB(color);
        delay(onMs);
        leds[0] = CRGB::Black;
        FastLED.show();
        if (i < times - 1) delay(offMs);
    }
}
