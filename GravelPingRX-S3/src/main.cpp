/**
 * GravelPing - Driveway Monitor Receiver (ESP32-S3 Dual-Core)
 * 
 * Optimized for ESP32-S3 Super Mini with dual-core architecture:
 *   Core 0 (PRO_CPU): WiFi & MQTT management (async, non-blocking)
 *   Core 1 (APP_CPU): LoRa message reception (time-critical, never blocked)
 *   Messages are queued from Core 1 to Core 0 for MQTT publishing.
 * 
 * Hardware:
 *   - ESP32-S3 Super Mini (Dual-Core Xtensa LX7 @ 240MHz, 4MB Flash)
 *   - DX-LR02-900T22D LoRa UART Module
 * 
 * Onboard WS2812 LED Status:
 *   - GREEN dim:    Idle, ready
 *   - BLUE flash:   Message received
 *   - CYAN:         System boot
 *   - YELLOW:       WiFi connecting
 *   - MAGENTA:      MQTT connecting
 */

#include <Arduino.h>
#include <ArduinoJson.h>
#include <ESPmDNS.h>
#include <FastLED.h>
#include <esp_task_wdt.h>
#include <espMqttClient.h>
#include <WiFi.h>

#ifdef ENABLE_OTA_UPDATES
#include <ArduinoOTA.h>
#endif

// ============================================================================
// PIN DEFINITIONS (ESP32-S3 Super Mini)
// ============================================================================

constexpr int PIN_LED_RGB = 48;  // WS2812 RGB LED

// LoRa Module UART
constexpr int PIN_LORA_TX  = 4;   // ESP32-S3 TX (GPIO4) -> LR-02 RX (Pin 3)
constexpr int PIN_LORA_RX  = 5;   // ESP32-S3 RX (GPIO5) <- LR-02 TX (Pin 4)
constexpr int PIN_LORA_AUX = 6;   // LR-02 AUX pin (LOW = ready, HIGH = busy)

// Audio Output (Active Buzzer)
constexpr int PIN_BUZZER  = 8;   // GPIO output for active buzzer

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
constexpr unsigned long HA_HEARTBEAT_TIMEOUT  = 25000;  // 25 seconds (HA automation publishes this every 10s)

// LoRa message buffering (character-by-character reading)
constexpr unsigned long MESSAGE_TIMEOUT_MS    = 100;    // 100ms timeout for incomplete messages
constexpr size_t MAX_MESSAGE_LENGTH           = 256;    // Maximum message buffer size

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
QueueHandle_t messageQueue;           // LoRa messages from Core 1 to Core 0

// Message structure for inter-core communication (Core 1 -> Core 0)
struct LoRaMessage {
    char event[16];
    uint32_t seq;
    float vbat;
    unsigned long timestamp;
    char raw[256];  // Raw JSON for Core 0 logging
};

// Network state (managed by Core 0)
volatile bool wifiConnected = false;
volatile bool mqttConnected = false;
volatile unsigned long lastWifiAttempt = 0;
volatile unsigned long lastMqttAttempt = 0;

// Home Assistant heartbeat tracking
volatile unsigned long lastHAHeartbeat = 0;
volatile bool haAvailable = false;

// LoRa task health tracking
volatile unsigned long lastLoRaHeartbeat = 0;

// LoRa message buffering (used by Core 1 only — not shared)
// Declared here so Core 1's loraTask owns them exclusively
char loraMsgBuf[MAX_MESSAGE_LENGTH + 1];
size_t loraMsgBufLen = 0;
unsigned long lastCharTime = 0;

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

#ifdef ENABLE_OTA_UPDATES
void setupOTA();
#endif

// Core 1 task (LoRa message handling + backup buzzer)
void loraTask(void* parameter);
void handleLoRaMessages();      // Core 1: reads UART, parses, queues
void handleMessage(const char* jsonStr);  // Core 1: parses JSON, queues to Core 0
void playBeepPatternNonBlocking();  // Core 1: buzzer alert (no Serial output)

// Helper function for MQTT publish
bool publishMQTT(const char* topic, const char* payload, bool retain = false);

// LED functions
void setRGB(CRGB color);
void setRGBDim(CRGB color, int brightness);
void flashRGB(CRGB color, int times, int onMs, int offMs);

// Audio functions (active buzzer)
void setupAudio();
void playTone(uint32_t duration);
void playBeepPattern();
void stopTone();

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
    
    // Initialize audio (active buzzer)
    setupAudio();
    
    // Initialize LoRa
    setupLoRa();
    
    // Create synchronization primitives
    messageQueue = xQueueCreate(10, sizeof(LoRaMessage));  // Queue up to 10 messages
    
    if (messageQueue == NULL) {
        Serial.println(F("[ERROR] Failed to create message queue!"));
        flashRGB(Colors::ERROR, 10, 200, 200);
        ESP.restart();
    }
    
    // Create network task on Core 0 (PRO_CPU)
    // This task handles WiFi and MQTT in background
    BaseType_t networkResult = xTaskCreatePinnedToCore(
        networkTask,      // Task function
        "NetworkTask",    // Task name
        16384,           // Stack size (bytes) - increased for MQTT operations
        NULL,            // Parameters
        2,               // Priority (2 = above idle, prevents watchdog)
        NULL,            // Task handle
        0                // Core 0 (PRO_CPU)
    );
    
    if (networkResult != pdPASS) {
        Serial.println(F("[ERROR] Failed to create NetworkTask!"));
        flashRGB(Colors::ERROR, 10, 200, 200);
        ESP.restart();
    }
    
    // Create LoRa task on Core 1 (APP_CPU)
    // This task handles time-critical LoRa message reception + backup buzzer
    BaseType_t loraResult = xTaskCreatePinnedToCore(
        loraTask,         // Task function
        "LoRaTask",       // Task name
        16384,           // Stack size (bytes) - needs room for JSON parsing
        NULL,            // Parameters
        3,               // Priority (3 = higher than networkTask)
        NULL,            // Task handle
        1                // Core 1 (APP_CPU)
    );
    
    if (loraResult != pdPASS) {
        Serial.println(F("[ERROR] Failed to create LoRaTask!"));
        flashRGB(Colors::ERROR, 10, 200, 200);
        ESP.restart();
    }
    
    Serial.println(F("[INIT] Network task started on Core 0 (PRO_CPU)"));
    Serial.println(F("[INIT] LoRa task started on Core 1 (APP_CPU)"));
    Serial.println(F("[INIT] Setup complete\n"));
    
    // Show ready status
    setRGBDim(Colors::IDLE, LED_BRIGHTNESS_DIM);
}

// ============================================================================
// MAIN LOOP (Empty - All work done in tasks)
// ============================================================================

void loop() {
    // All work is done in dedicated tasks (networkTask on Core 0, loraTask on Core 1)
    // This loop is intentionally empty to avoid conflicts
    delay(1000);
}

// ============================================================================
// CORE 1 TASK - LORA MESSAGE HANDLING + BACKUP BUZZER (APP_CPU)
// ============================================================================
// CRITICAL: No Serial.printf or Serial.println from this task!
// ESP32-S3 USB CDC Serial is NOT thread-safe. Writing from both cores
// causes corruption and crashes. All logging is done via the queue on Core 0.
// ============================================================================

void loraTask(void* parameter) {
    // Small initial delay to let Core 0 finish setup
    vTaskDelay(500 / portTICK_PERIOD_MS);
    
    while (true) {
        lastLoRaHeartbeat = millis();
        
        // Handle LoRa messages (reads UART, parses JSON, queues to Core 0)
        handleLoRaMessages();
        
        // Yield to watchdog — 10ms gives responsive UART reading
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    
    // Should never reach here
    vTaskDelete(NULL);
}

// ============================================================================
// CORE 0 TASK - NETWORK MANAGEMENT (PRO_CPU)
// ============================================================================

void networkTask(void* parameter) {
    Serial.printf("[NETWORK] Task started on Core %d (PRO_CPU)\n", xPortGetCoreID());
    
    // Subscribe this task to the task watchdog timer
    esp_task_wdt_add(NULL);
    
    // Non-blocking WiFi initialization - start connection attempt but don't wait
    Serial.println(F("[WIFI] Starting WiFi connection (non-blocking)..."));
    Serial.printf("       SSID: %s\n", WIFI_SSID_STR);
    setRGB(Colors::WIFI);
    
    WiFi.mode(WIFI_STA);
    WiFi.disconnect(true);
    delay(100);
    
    if (WiFi.setHostname(DEVICE_NAME_STR)) {
        Serial.printf("[WIFI] Hostname configured: %s\n", DEVICE_NAME_STR);
    }
    
    WiFi.setSleep(false);
    WiFi.begin(WIFI_SSID_STR, WIFI_PASSWORD_STR);
    
    // Don't wait for connection - let maintainWiFi() handle it in the main loop
    Serial.println(F("[WIFI] Connection initiated, continuing to main loop..."));
    Serial.println(F("[NETWORK] LoRa handling on Core 1, receiving via queue"));
    
    unsigned long lastLoRaCheck = 0;
    
    // Process queued messages and maintain connections
    while (true) {
        // Core 1 heartbeat monitoring
        if (millis() - lastLoRaCheck >= 30000) {
            unsigned long loraAge = millis() - lastLoRaHeartbeat;
            Serial.printf("[HEALTH] LoRa task heartbeat age: %lu ms %s\n", 
                         loraAge, loraAge > 15000 ? "⚠️ STALE" : "✓");
            lastLoRaCheck = millis();
        }
        
        // Maintain WiFi connection
        maintainWiFi();
        
#ifdef ENABLE_OTA_UPDATES
        // Handle OTA updates (non-blocking)
        ArduinoOTA.handle();
#endif
        
        // Maintain MQTT connection (if WiFi is up)
        if (wifiConnected) {
            maintainMQTT();
        }
        
        // Check Home Assistant heartbeat status
        checkHAHeartbeat();
        
        // Process messages from Core 1's LoRa task via queue
        LoRaMessage msg;
        while (xQueueReceive(messageQueue, &msg, 0) == pdTRUE) {
            // Log the received message (safe here on Core 0)
            Serial.println(F("========================================"));
            Serial.println(F("[MESSAGE RECEIVED via Core 1 queue]"));
            Serial.printf("  Raw:     %s\n", msg.raw);
            Serial.printf("  Event:   %s\n", msg.event);
            Serial.printf("  Seq:     %u\n", msg.seq);
#ifdef ENABLE_BATTERY_MONITORING
            if (msg.vbat > 0.0) {
                Serial.printf("  VBat:    %.1fV\n", msg.vbat);
            }
#endif
            
            if (strcmp(msg.event, "entry") == 0) {
                Serial.println(F(">>> VEHICLE DETECTED <<<"));
            } else if (strcmp(msg.event, "fault") == 0) {
                Serial.println(F(">>> LOOP FAULT DETECTED <<<"));
            }
            Serial.println(F("----------------------------------------"));
            
            // Update message count
            messageCount++;
            lastMessageTime = millis();
            
            // Visual feedback (LED)
            flashRGB(Colors::RECEIVED, 1, 100, 0);
            
            // Publish to MQTT
            if (mqttConnected) {
                publishToHomeAssistant(msg.event, msg.seq, msg.vbat);
            } else {
                Serial.println(F("[MQTT] Not connected - skipping publish"));
            }
            
            // Return to idle
            setRGBDim(Colors::IDLE, LED_BRIGHTNESS_DIM);
        }
        
        // Feed the task watchdog and yield
        esp_task_wdt_reset();
        vTaskDelay(10 / portTICK_PERIOD_MS);
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
    
    // Enable internal pull-up on RX pin to prevent floating (reduces noise)
    pinMode(PIN_LORA_RX, INPUT_PULLUP);
    
    // Initialize UART1 for LoRa communication
    LoRaSerial.begin(LORA_BAUD, SERIAL_8N1, PIN_LORA_RX, PIN_LORA_TX);
    
    // Configure AUX pin as input
    pinMode(PIN_LORA_AUX, INPUT);
    
    Serial.println(F("[INIT] ✓ LoRa UART ready"));
    Serial.printf("       - ESP32 TX (GPIO%d) -> LoRa RX\n", PIN_LORA_TX);
    Serial.printf("       - ESP32 RX (GPIO%d) <- LoRa TX\n", PIN_LORA_RX);
    Serial.printf("       - LoRa AUX:         GPIO%d\n", PIN_LORA_AUX);
    Serial.println(F("       - Waiting for LoRa messages..."));
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
        vTaskDelay(50 / portTICK_PERIOD_MS);  // Yield to watchdog during connection
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
    static bool otaInitialized = false;
    
    if (WiFi.status() == WL_CONNECTED) {
        if (!wifiConnected) {
            wifiConnected = true;
            Serial.println(F("[WIFI] ✓ Connected"));
            Serial.printf("       Hostname: %s\n", DEVICE_NAME_STR);
            Serial.printf("       IP: %s\n", WiFi.localIP().toString().c_str());
            Serial.printf("       RSSI: %d dBm\n", WiFi.RSSI());
            
            // Setup mDNS (once)
            if (MDNS.begin(DEVICE_NAME_STR)) {
                Serial.printf("[MDNS] ✓ Hostname: %s.local\n", DEVICE_NAME_STR);
            }
            
#ifdef ENABLE_OTA_UPDATES
            // Initialize OTA (once, after first WiFi connection)
            if (!otaInitialized) {
                setupOTA();
                otaInitialized = true;
            }
#endif
            
            // Trigger MQTT connection
            setupMQTT();
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
            
            // Subscribe to audio test commands
            if (mqttClient.subscribe("gravelping/s3/audio/test", 0)) {
                Serial.println(F("[MQTT] ✓ Subscribed to audio test topic"));
            } else {
                Serial.println(F("[MQTT] ✗ Failed to subscribe to audio test topic"));
            }
            
            // Subscribe to debug status commands
            if (mqttClient.subscribe("gravelping/s3/debug/status", 0)) {
                Serial.println(F("[MQTT] ✓ Subscribed to debug status topic"));
            } else {
                Serial.println(F("[MQTT] ✗ Failed to subscribe to debug status topic"));
            }
            
            // Publish Home Assistant autodiscovery
            // No mutex needed — MQTT is only accessed from Core 0
            publishDiscovery();
            
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
    else if (strcmp(topic, "gravelping/s3/audio/test") == 0) {
        // Audio test command
        // Convert payload to null-terminated string
        char msg[32];
        size_t copyLen = (len < 31) ? len : 31;
        memcpy(msg, payload, copyLen);
        msg[copyLen] = '\0';
        
        Serial.printf("[AUDIO] Test command received: %s\n", msg);
        
        // Parse command - active buzzer has fixed tone, only duration varies
        if (strcmp(msg, "pattern") == 0) {
            playBeepPattern();
        }
        else if (strcmp(msg, "startup") == 0) {
            // Two quick beeps
            playTone(100);
            delay(50);
            playTone(100);
        }
        else if (strcmp(msg, "short") == 0) {
            playTone(200);
        }
        else if (strcmp(msg, "medium") == 0 || strcmp(msg, "med") == 0) {
            playTone(500);
        }
        else if (strcmp(msg, "long") == 0) {
            playTone(1000);
        }
        else if (strcmp(msg, "siren") == 0) {
            // Siren effect with on/off pattern
            for (int i = 0; i < 6; i++) {
                playTone(200);
                delay(100);
            }
        }
        else if (strcmp(msg, "alarm") == 0) {
            // Rapid beeping alarm
            for (int i = 0; i < 5; i++) {
                playTone(150);
                delay(50);
            }
        }
        else if (strcmp(msg, "stop") == 0) {
            stopTone();
        }
        else {
            // Try parsing as duration in milliseconds
            uint32_t duration = atoi(msg);
            if (duration > 0 && duration < 10000) {
                Serial.printf("[AUDIO] Playing tone for %u ms\n", duration);
                playTone(duration);
            } else {
                Serial.println(F("[AUDIO] Unknown command. Valid: pattern, startup, short, medium, long, siren, alarm, stop, or duration_ms"));
            }
        }
    }
    else if (strcmp(topic, "gravelping/s3/debug/status") == 0) {
        // Status request - report task health
        unsigned long now = millis();
        unsigned long loraAge = now - lastLoRaHeartbeat;
        
        Serial.println(F("========================================"));
        Serial.println(F("[DEBUG] System Status Report:"));
        Serial.printf("  WiFi:        %s\n", wifiConnected ? "Connected" : "Disconnected");
        Serial.printf("  MQTT:        %s\n", mqttConnected ? "Connected" : "Disconnected");
        Serial.printf("  HA:          %s\n", haAvailable ? "Available" : "Unavailable");
        Serial.printf("  Messages:    %u\n", messageCount);
        Serial.printf("  Uptime:      %lu seconds\n", now / 1000);
        Serial.printf("  Free Heap:   %u bytes\n", ESP.getFreeHeap());
        Serial.println(F("  ---"));
        Serial.printf("  Core 0:      NetworkTask (this core)\n");
        Serial.printf("  Core 1:      LoRaTask - last heartbeat %lu ms ago", loraAge);
        if (loraAge < 1000) {
            Serial.println(F(" ✓ HEALTHY"));
        } else if (loraAge < 5000) {
            Serial.println(F(" ⚠ SLOW"));
        } else {
            Serial.println(F(" ✗ FROZEN/DEAD"));
        }
        Serial.println(F("========================================"));
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
    
    // Yield to watchdog between publishes
    vTaskDelay(100 / portTICK_PERIOD_MS);
    
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
    
    // Yield to watchdog between publishes
    vTaskDelay(100 / portTICK_PERIOD_MS);
    
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
    
    // Yield to watchdog between publishes
    vTaskDelay(100 / portTICK_PERIOD_MS);
    
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
    
    // Yield to watchdog after final publish
    vTaskDelay(100 / portTICK_PERIOD_MS);
#endif
    
    Serial.println(F("[MQTT] ✓ Autodiscovery complete"));
}

// ============================================================================
// LORA MESSAGE HANDLING (Core 1 — NO Serial output allowed!)
// ============================================================================
// These functions run on Core 1. They must NEVER call Serial.printf,
// Serial.println, or any other Serial method. ESP32-S3 USB CDC is not
// thread-safe and concurrent writes from both cores cause crashes.
//
// Instead, parsed messages are sent to Core 0 via messageQueue.
// Core 0's networkTask logs everything safely.
// ============================================================================

void handleLoRaMessages() {
    // Read available characters one at a time (no blocking)
    while (LoRaSerial.available()) {
        char c = LoRaSerial.read();
        lastCharTime = millis();
        
        if (c == '\n' || c == '\r') {
            // End of message - process if buffer has content
            if (loraMsgBufLen > 0) {
                loraMsgBuf[loraMsgBufLen] = '\0';  // Null-terminate
                handleMessage(loraMsgBuf);
                loraMsgBufLen = 0;
            }
        } else if (c >= 32 && c <= 126) {
            // Only accept printable ASCII characters (filters binary garbage)
            if (loraMsgBufLen < MAX_MESSAGE_LENGTH) {
                loraMsgBuf[loraMsgBufLen++] = c;
            } else {
                // Buffer overflow — discard
                loraMsgBufLen = 0;
            }
        }
        // Non-printable characters are silently dropped
    }
    
    // Check for message timeout (incomplete message)
    if (loraMsgBufLen > 0 && (millis() - lastCharTime) > MESSAGE_TIMEOUT_MS) {
        loraMsgBufLen = 0;  // Discard partial message
    }
}

void handleMessage(const char* jsonStr) {
    // Parse JSON entirely on Core 1 (fast, no allocations beyond stack)
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, jsonStr);
    
    if (error) {
        // Silently ignore garbage — no Serial output on Core 1!
        return;
    }
    
    // Extract fields
    const char* event = doc["event"] | "unknown";
    uint32_t seq = doc["seq"] | 0;
    float vbat = doc["vbat"] | 0.0f;
    
    // Build the message struct to send to Core 0
    LoRaMessage msg;
    strncpy(msg.event, event, sizeof(msg.event) - 1);
    msg.event[sizeof(msg.event) - 1] = '\0';
    msg.seq = seq;
    msg.vbat = vbat;
    msg.timestamp = millis();
    strncpy(msg.raw, jsonStr, sizeof(msg.raw) - 1);
    msg.raw[sizeof(msg.raw) - 1] = '\0';
    
    // Check if HA is down and this is a vehicle entry — trigger backup buzzer
    // haAvailable is volatile, safe to read from Core 1 (atomic bool read)
    if (!haAvailable && strcmp(event, "entry") == 0) {
        playBeepPatternNonBlocking();
    }
    
    // Queue message to Core 0 for logging + MQTT publish
    // Don't block if queue is full (drop message rather than stall Core 1)
    xQueueSend(messageQueue, &msg, 0);
}

// ============================================================================
// MQTT PUBLISHING (Core 0)
// ============================================================================

void publishToHomeAssistant(const char* event, uint32_t seq, float vbat) {
    String topic;
    String payload;
    
    // NOTE: Buzzer alert is handled by Core 1 in handleMessage() — no buzzer logic here
    
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

// ============================================================================
// AUDIO FUNCTIONS (Active Buzzer)
// ============================================================================

void setupAudio() {
    Serial.println(F("[INIT] Initializing audio (active buzzer)..."));
    
    // Configure GPIO as output for active buzzer
    pinMode(PIN_BUZZER, OUTPUT);
    digitalWrite(PIN_BUZZER, LOW);  // Start silent
    
    Serial.printf("[INIT] ✓ Audio ready on GPIO%d\n", PIN_BUZZER);
    
    // Test beep on startup (optional - comment out if annoying)
    Serial.println(F("[AUDIO] Playing startup beep..."));
    digitalWrite(PIN_BUZZER, HIGH);
    delay(200);
    digitalWrite(PIN_BUZZER, LOW);
    delay(50);
    digitalWrite(PIN_BUZZER, HIGH);
    delay(200);
    digitalWrite(PIN_BUZZER, LOW);
}

void playTone(uint32_t duration) {
    // For active buzzer: turn on for specified duration, then off
    if (duration == 0) {
        stopTone();
        return;
    }
    
    digitalWrite(PIN_BUZZER, HIGH);
    delay(duration);
    digitalWrite(PIN_BUZZER, LOW);
}

void stopTone() {
    digitalWrite(PIN_BUZZER, LOW);
}

void playBeepPattern() {
    // Alert pattern for vehicle detection (Core 0 version with Serial logging)
    Serial.println(F("[AUDIO] Playing alert pattern..."));
    
    // Three quick beeps
    for (int i = 0; i < 3; i++) {
        playTone(250);    // 250ms beep
        delay(120);       // 120ms gap
    }
    
    // One long beep
    playTone(700);        // 700ms beep
    
    Serial.println(F("[AUDIO] Alert pattern complete"));
}

// Core 1 safe version — uses vTaskDelay, no Serial output
void playBeepPatternNonBlocking() {
    // Three quick beeps
    for (int i = 0; i < 3; i++) {
        digitalWrite(PIN_BUZZER, HIGH);
        vTaskDelay(250 / portTICK_PERIOD_MS);
        digitalWrite(PIN_BUZZER, LOW);
        vTaskDelay(120 / portTICK_PERIOD_MS);
    }
    
    // One long beep
    digitalWrite(PIN_BUZZER, HIGH);
    vTaskDelay(700 / portTICK_PERIOD_MS);
    digitalWrite(PIN_BUZZER, LOW);
}

// ============================================================================
// OTA UPDATE FUNCTIONS
// ============================================================================

#ifdef ENABLE_OTA_UPDATES
void setupOTA() {
    Serial.println(F("[OTA] Initializing Over-The-Air updates..."));
    
    // Set hostname for OTA
    #ifndef OTA_HOSTNAME
    #define OTA_HOSTNAME "GravelPing-S3"
    #endif
    
    #ifndef OTA_PASSWORD
    #define OTA_PASSWORD "gravelping"
    #endif
    
    ArduinoOTA.setHostname(TOSTRING(OTA_HOSTNAME));
    ArduinoOTA.setPassword(TOSTRING(OTA_PASSWORD));
    ArduinoOTA.setPort(3232);
    
    // OTA event callbacks
    ArduinoOTA.onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH) {
            type = "sketch";
        } else {  // U_SPIFFS
            type = "filesystem";
        }
        Serial.println("[OTA] Start updating " + type);
        
        // Visual feedback - flash LED during update
        setRGB(CRGB::Magenta);
        
        // Stop LoRa reception during OTA to prevent issues
        // (Network task continues on Core 0, LoRa on Core 1 will be blocked)
    });
    
    ArduinoOTA.onEnd([]() {
        Serial.println(F("\n[OTA] Update complete!"));
        setRGB(CRGB::Green);
        delay(1000);
    });
    
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        static unsigned long lastPrint = 0;
        unsigned long now = millis();
        
        // Print progress every 1 second
        if (now - lastPrint > 1000) {
            Serial.printf("[OTA] Progress: %u%%\n", (progress / (total / 100)));
            lastPrint = now;
            
            // Flash LED to show activity
            static bool ledState = false;
            if (ledState) {
                setRGB(CRGB::Magenta);
            } else {
                setRGB(CRGB::Black);
            }
            ledState = !ledState;
        }
    });
    
    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("[OTA] Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) {
            Serial.println(F("Auth Failed"));
        } else if (error == OTA_BEGIN_ERROR) {
            Serial.println(F("Begin Failed"));
        } else if (error == OTA_CONNECT_ERROR) {
            Serial.println(F("Connect Failed"));
        } else if (error == OTA_RECEIVE_ERROR) {
            Serial.println(F("Receive Failed"));
        } else if (error == OTA_END_ERROR) {
            Serial.println(F("End Failed"));
        }
        
        // Flash red LED on error
        flashRGB(Colors::ERROR, 5, 200, 200);
    });
    
    ArduinoOTA.begin();
    
    Serial.println(F("[OTA] ✓ OTA ready"));
    Serial.printf("[OTA]    Hostname: %s\n", TOSTRING(OTA_HOSTNAME));
    Serial.printf("[OTA]    Port: 3232\n");
    Serial.printf("[OTA]    Password: %s\n", TOSTRING(OTA_PASSWORD));
    Serial.println(F("[OTA]    Use 'pio run -t upload' for serial upload"));
    Serial.println(F("[OTA]    Use 'pio run -t upload --upload-port GravelPing-S3.local' for OTA upload"));
}
#endif

