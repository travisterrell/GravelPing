/**
 * GravelPing - Bell Controller
 *
 * Listens for MQTT commands from Home Assistant and rings a DC fire alarm bell
 * wired to a GPIO pin via a MOSFET. Supports two ring modes:
 *   - Single ring: bell on for RING_SINGLE_MS milliseconds
 *   - Pattern ring: RING_PATTERN_COUNT short bursts
 *
 * Hardware:
 *   - ESP32-C6 DevKitM-1
 *   - DC bell wired via MOSFET to BELL_PIN
 *
 * LED Indicators:
 *   RGB LED:
 *     - CYAN solid:       System boot
 *     - YELLOW:           WiFi connecting
 *     - MAGENTA:          MQTT connecting
 *     - GREEN dim:        Idle / connected
 *     - BLUE flash:       Command received
 *     - RED solid:        Bell ringing
 *     - RED flash:        Error
 *
 * MQTT Topics:
 *   Subscribe:
 *     gravelping/bell/ring/single   - Ring once for RING_SINGLE_MS ms
 *     gravelping/bell/ring/pattern  - Ring in short burst pattern
 *   Publish:
 *     gravelping/bell/status        - "Idle" | "Ringing"
 */

#include <Arduino.h>
#include <ArduinoJson.h>
#include <ESPmDNS.h>
#include <FastLED.h>
#include <PubSubClient.h>
#include <WiFi.h>
#ifdef ENABLE_OTA_UPDATES
#include <ArduinoOTA.h>
#endif

// ============================================================================
// PIN DEFINITIONS
// ============================================================================

constexpr int PIN_LED_STATUS = 15;  // Simple status LED (active HIGH)
constexpr int PIN_LED_RGB    = 8;   // WS2812 RGB LED

// ============================================================================
// CONFIGURATION (from platformio.ini build_flags)
// ============================================================================

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

#ifndef WIFI_SSID
#define WIFI_SSID YourSSID
#endif
#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD YourPassword
#endif
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
#define DEVICE_NAME GravelPingBell
#endif

#ifndef BELL_PIN
#define BELL_PIN 3
#endif
#ifndef RING_SINGLE_MS
#define RING_SINGLE_MS 1500
#endif
#ifndef RING_PATTERN_COUNT
#define RING_PATTERN_COUNT 3
#endif
#ifndef RING_PATTERN_ON_MS
#define RING_PATTERN_ON_MS 300
#endif
#ifndef RING_PATTERN_OFF_MS
#define RING_PATTERN_OFF_MS 200
#endif

const char* WIFI_SSID_STR     = TOSTRING(WIFI_SSID);
const char* WIFI_PASSWORD_STR = TOSTRING(WIFI_PASSWORD);
const char* MQTT_BROKER_STR   = TOSTRING(MQTT_BROKER);
const char* MQTT_USER_STR     = TOSTRING(MQTT_USER);
const char* MQTT_PASSWORD_STR = TOSTRING(MQTT_PASSWORD);
const char* DEVICE_NAME_STR   = TOSTRING(DEVICE_NAME);

// ============================================================================
// MQTT TOPICS
// ============================================================================

constexpr const char* TOPIC_CMD_SINGLE  = "gravelping/bell/ring/single";
constexpr const char* TOPIC_CMD_PATTERN = "gravelping/bell/ring/pattern";
constexpr const char* TOPIC_STATUS      = "gravelping/bell/status";

// ============================================================================
// TIMING
// ============================================================================

constexpr unsigned long WIFI_RECONNECT_INTERVAL = 5000;
constexpr unsigned long MQTT_RECONNECT_INTERVAL = 5000;
constexpr unsigned long WIFI_CONNECT_TIMEOUT    = 10000;
constexpr unsigned long MQTT_CONNECT_TIMEOUT    = 5000;

// ============================================================================
// LED CONFIGURATION
// ============================================================================

constexpr int NUM_LEDS           = 1;
constexpr int LED_BRIGHTNESS     = 50;
constexpr int LED_BRIGHTNESS_DIM = 10;

namespace Colors {
    const CRGB OFF     = CRGB::Black;
    const CRGB BOOT    = CRGB::Cyan;
    const CRGB WIFI    = CRGB::Yellow;
    const CRGB MQTT    = CRGB::Magenta;
    const CRGB IDLE    = CRGB::Green;
    const CRGB RINGING = CRGB::Red;
    const CRGB COMMAND = CRGB::Blue;
    const CRGB ERROR   = CRGB::Red;
}

// ============================================================================
// GLOBALS
// ============================================================================

CRGB leds[NUM_LEDS];

WiFiClient   wifiClient;
PubSubClient mqttClient(wifiClient);

bool discoveryPublished = false;

enum WiFiState  { WIFI_DISCONNECTED, WIFI_CONNECTING, WIFI_CONNECTED };
enum MQTTState  { MQTT_STATE_DISCONNECTED, MQTT_STATE_CONNECTING, MQTT_STATE_CONNECTED };

WiFiState wifiState = WIFI_DISCONNECTED;
MQTTState mqttState = MQTT_STATE_DISCONNECTED;

unsigned long lastWifiAttempt      = 0;
unsigned long lastMqttAttempt      = 0;
unsigned long wifiConnectStartTime = 0;
unsigned long mqttConnectStartTime = 0;

// Bell state machine
enum BellState { BELL_IDLE, BELL_SINGLE, BELL_PATTERN_ON, BELL_PATTERN_OFF };
BellState bellState      = BELL_IDLE;
unsigned long bellTimer  = 0;
int patternPulsesDone    = 0;

// Non-blocking LED strobe (used while ringing)
constexpr unsigned long STROBE_ON_MS  = 40;
constexpr unsigned long STROBE_OFF_MS = 60;
bool strobeActive        = false;
bool strobePhase         = false;   // true = LED on, false = LED off
unsigned long strobeTimer = 0;

// ============================================================================
// FUNCTION DECLARATIONS
// ============================================================================

void setupLEDs();
void setupBell();
void setupWiFi();
void setupMQTT();
#ifdef ENABLE_OTA_UPDATES
void setupOTA();
#endif

void manageWiFiConnection();
void startWiFiConnection();
void onWiFiConnected();

void manageMQTTConnection();
void startMQTTConnection();
void onMQTTConnected();

void publishDiscovery();
void publishStatus(const char* status);
void mqttCallback(char* topic, byte* payload, unsigned int length);

void startSingleRing();
void startPatternRing();
void manageBell();
void manageStrobe();

void bellOn();
void bellOff();

void setRGB(CRGB color);
void setRGBDim(CRGB color, uint8_t brightness);
void flashRGB(CRGB color, int count = 1, int onMs = 150, int offMs = 100);

// ============================================================================
// SETUP
// ============================================================================

void setup() {
    setupLEDs();
    setRGB(Colors::BOOT);

    Serial.begin(115200);
    delay(1000);

    Serial.println();
    Serial.println(F("========================================"));
    Serial.println(F("   GravelPing Bell Controller"));
    Serial.println(F("========================================"));
    Serial.printf("[INIT] Device: %s\n", DEVICE_NAME_STR);
    Serial.printf("[INIT] Bell pin: GPIO%d\n", BELL_PIN);
    Serial.printf("[INIT] Single ring: %d ms\n", RING_SINGLE_MS);
    Serial.printf("[INIT] Pattern: %dx %dms on / %dms off\n",
        RING_PATTERN_COUNT, RING_PATTERN_ON_MS, RING_PATTERN_OFF_MS);
    Serial.println();

    setupBell();
    setupWiFi();
    setupMQTT();

    lastWifiAttempt = millis() - WIFI_RECONNECT_INTERVAL;  // Trigger immediate attempt

    Serial.println(F("[READY] Waiting for commands..."));
    setRGBDim(Colors::IDLE, LED_BRIGHTNESS_DIM);
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
    manageWiFiConnection();

    if (wifiState == WIFI_CONNECTED) {
        manageMQTTConnection();
    }

    if (mqttState == MQTT_STATE_CONNECTED) {
        mqttClient.loop();
    }

#ifdef ENABLE_OTA_UPDATES
    ArduinoOTA.handle();
#endif

    manageBell();
    manageStrobe();

    delay(1);
}

// ============================================================================
// LED HELPERS
// ============================================================================

void setupLEDs() {
    pinMode(PIN_LED_STATUS, OUTPUT);
    digitalWrite(PIN_LED_STATUS, LOW);

    FastLED.addLeds<NEOPIXEL, PIN_LED_RGB>(leds, NUM_LEDS);
    FastLED.setBrightness(LED_BRIGHTNESS);
    FastLED.clear();
    FastLED.show();
}

void setRGB(CRGB color) {
    leds[0] = color;
    FastLED.setBrightness(LED_BRIGHTNESS);
    FastLED.show();
}

void setRGBDim(CRGB color, uint8_t brightness) {
    leds[0] = color;
    FastLED.setBrightness(brightness);
    FastLED.show();
}

void flashRGB(CRGB color, int count, int onMs, int offMs) {
    for (int i = 0; i < count; i++) {
        setRGB(color);
        delay(onMs);
        setRGB(Colors::OFF);
        if (i < count - 1) delay(offMs);
    }
}

// ============================================================================
// BELL CONTROL
// ============================================================================

void setupBell() {
    pinMode(BELL_PIN, OUTPUT);
    digitalWrite(BELL_PIN, LOW);
    Serial.printf("[BELL] Pin GPIO%d configured\n", BELL_PIN);
}

void bellOn() {
    digitalWrite(BELL_PIN, HIGH);
    digitalWrite(PIN_LED_STATUS, HIGH);
    // Start strobe - kick off with LED on immediately
    strobeActive = true;
    strobePhase  = true;
    strobeTimer  = millis();
    setRGB(Colors::RINGING);
}

void bellOff() {
    digitalWrite(BELL_PIN, LOW);
    digitalWrite(PIN_LED_STATUS, LOW);
    // Stop strobe and return to idle colour
    strobeActive = false;
    setRGBDim(Colors::IDLE, LED_BRIGHTNESS_DIM);
}

void startSingleRing() {
    if (bellState != BELL_IDLE) return;
    Serial.println(F("[BELL] Single ring triggered"));
    publishStatus("Ringing");
    bellOn();
    bellState = BELL_SINGLE;
    bellTimer = millis();
}

void startPatternRing() {
    if (bellState != BELL_IDLE) return;
    Serial.println(F("[BELL] Pattern ring triggered"));
    publishStatus("Ringing");
    patternPulsesDone = 0;
    bellOn();
    bellState = BELL_PATTERN_ON;
    bellTimer = millis();
}

/**
 * Non-blocking bell state machine. Called every loop iteration.
 */
void manageBell() {
    switch (bellState) {
        case BELL_IDLE:
            break;

        case BELL_SINGLE:
            if (millis() - bellTimer >= RING_SINGLE_MS) {
                bellOff();
                bellState = BELL_IDLE;
                publishStatus("Idle");
                Serial.println(F("[BELL] Single ring complete"));
            }
            break;

        case BELL_PATTERN_ON:
            if (millis() - bellTimer >= RING_PATTERN_ON_MS) {
                bellOff();
                patternPulsesDone++;
                if (patternPulsesDone >= RING_PATTERN_COUNT) {
                    bellState = BELL_IDLE;
                    publishStatus("Idle");
                    Serial.println(F("[BELL] Pattern ring complete"));
                } else {
                    bellState = BELL_PATTERN_OFF;
                    bellTimer = millis();
                }
            }
            break;

        case BELL_PATTERN_OFF:
            if (millis() - bellTimer >= RING_PATTERN_OFF_MS) {
                bellOn();
                bellState = BELL_PATTERN_ON;
                bellTimer = millis();
            }
            break;
    }
}

/**
 * Non-blocking LED strobe. Runs independently of the bell timer so the
 * strobe cadence stays consistent regardless of ring mode or duration.
 */
void manageStrobe() {
    if (!strobeActive) return;

    unsigned long elapsed = millis() - strobeTimer;

    if (strobePhase && elapsed >= STROBE_ON_MS) {
        // ON phase expired → turn LED off
        setRGB(Colors::OFF);
        strobePhase = false;
        strobeTimer = millis();
    } else if (!strobePhase && elapsed >= STROBE_OFF_MS) {
        // OFF phase expired → turn LED on
        setRGB(Colors::RINGING);
        strobePhase = true;
        strobeTimer = millis();
    }
}

// ============================================================================
// MQTT CALLBACK
// ============================================================================

void mqttCallback(char* topic, byte* payload, unsigned int length) {
    Serial.printf("[MQTT] Message on topic: %s\n", topic);

    flashRGB(Colors::COMMAND, 1, 150, 0);
    setRGBDim(Colors::IDLE, LED_BRIGHTNESS_DIM);

    if (strcmp(topic, TOPIC_CMD_SINGLE) == 0) {
        startSingleRing();
    } else if (strcmp(topic, TOPIC_CMD_PATTERN) == 0) {
        startPatternRing();
    }
}

// ============================================================================
// WIFI
// ============================================================================

void setupWiFi() {
    Serial.println(F("[WIFI] Initializing..."));
    WiFi.mode(WIFI_STA);
    WiFi.disconnect(true);
    delay(100);
    WiFi.setHostname(DEVICE_NAME_STR);
}

void manageWiFiConnection() {
    switch (wifiState) {
        case WIFI_DISCONNECTED:
            if (millis() - lastWifiAttempt >= WIFI_RECONNECT_INTERVAL) {
                lastWifiAttempt = millis();
                startWiFiConnection();
            }
            break;

        case WIFI_CONNECTING:
            if (WiFi.status() == WL_CONNECTED) {
                onWiFiConnected();
            } else if (millis() - wifiConnectStartTime >= WIFI_CONNECT_TIMEOUT) {
                Serial.println(F("[WIFI] Connection timeout"));
                wifiState = WIFI_DISCONNECTED;
                flashRGB(Colors::ERROR, 2, 100, 100);
                setRGBDim(Colors::IDLE, LED_BRIGHTNESS_DIM);
            }
            break;

        case WIFI_CONNECTED:
            if (WiFi.status() != WL_CONNECTED) {
                Serial.println(F("[WIFI] Connection lost"));
                wifiState = WIFI_DISCONNECTED;
                mqttState = MQTT_STATE_DISCONNECTED;
                discoveryPublished = false;
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
    WiFi.setHostname(DEVICE_NAME_STR);
    Serial.println(F("[WIFI] Connected!"));
    Serial.printf("[WIFI] IP: %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("[WIFI] Signal: %d dBm\n", WiFi.RSSI());

    if (MDNS.begin(DEVICE_NAME_STR)) {
        Serial.printf("[MDNS] Started: %s.local\n", DEVICE_NAME_STR);
    }

#ifdef ENABLE_OTA_UPDATES
    static bool otaInitialized = false;
    if (!otaInitialized) {
        setupOTA();
        otaInitialized = true;
    }
#endif

    wifiState = WIFI_CONNECTED;
    setRGBDim(Colors::IDLE, LED_BRIGHTNESS_DIM);
    mqttState = MQTT_STATE_DISCONNECTED;
    lastMqttAttempt = millis() - MQTT_RECONNECT_INTERVAL;
}

// ============================================================================
// MQTT
// ============================================================================

void setupMQTT() {
    Serial.println(F("[MQTT] Initializing..."));
    mqttClient.setServer(MQTT_BROKER_STR, MQTT_PORT);
    mqttClient.setCallback(mqttCallback);
    mqttClient.setBufferSize(512);
    Serial.printf("[MQTT] Broker: %s:%d\n", MQTT_BROKER_STR, MQTT_PORT);
}

void manageMQTTConnection() {
    switch (mqttState) {
        case MQTT_STATE_DISCONNECTED:
            if (millis() - lastMqttAttempt >= MQTT_RECONNECT_INTERVAL) {
                lastMqttAttempt = millis();
                startMQTTConnection();
            }
            break;

        case MQTT_STATE_CONNECTING:
            if (mqttClient.connected()) {
                onMQTTConnected();
            } else if (millis() - mqttConnectStartTime >= MQTT_CONNECT_TIMEOUT) {
                Serial.println(F("[MQTT] Connection timeout"));
                mqttState = MQTT_STATE_DISCONNECTED;
            }
            break;

        case MQTT_STATE_CONNECTED:
            if (!mqttClient.connected()) {
                Serial.println(F("[MQTT] Connection lost"));
                mqttState = MQTT_STATE_DISCONNECTED;
                discoveryPublished = false;
            }
            break;
    }
}

void startMQTTConnection() {
    Serial.println(F("[MQTT] Connecting to broker..."));
    setRGB(Colors::MQTT);

    String clientId = "gravelping-bell-";
    clientId += String(random(0xffff), HEX);

    bool connected = false;
    if (strlen(MQTT_USER_STR) > 0) {
        connected = mqttClient.connect(clientId.c_str(), MQTT_USER_STR, MQTT_PASSWORD_STR);
    } else {
        connected = mqttClient.connect(clientId.c_str());
    }

    if (connected) {
        Serial.println(F("[MQTT] Connected!"));
        mqttState = MQTT_STATE_CONNECTED;
        mqttConnectStartTime = millis();
        setRGBDim(Colors::IDLE, LED_BRIGHTNESS_DIM);

        if (!discoveryPublished) {
            publishDiscovery();
            discoveryPublished = true;
        }
    } else {
        Serial.printf("[MQTT] Connection failed! State: %d\n", mqttClient.state());
        mqttState = MQTT_STATE_DISCONNECTED;
        flashRGB(Colors::ERROR, 2, 100, 100);
        setRGBDim(Colors::IDLE, LED_BRIGHTNESS_DIM);
    }
}

void onMQTTConnected() {
    mqttState = MQTT_STATE_CONNECTED;
    setRGBDim(Colors::IDLE, LED_BRIGHTNESS_DIM);
    if (!discoveryPublished) {
        publishDiscovery();
        discoveryPublished = true;
    }
}

void publishStatus(const char* status) {
    if (mqttState == MQTT_STATE_CONNECTED) {
        mqttClient.publish(TOPIC_STATUS, status, false);
        Serial.printf("[MQTT] Status: %s\n", status);
    }
}

// ============================================================================
// HOME ASSISTANT MQTT DISCOVERY
// ============================================================================

void publishDiscovery() {
    Serial.println(F("[MQTT] Publishing HA discovery..."));

    JsonDocument doc;
    String payload;

    // Shared device block
    auto addDevice = [&](JsonDocument& d) {
        JsonObject device = d["device"].to<JsonObject>();
        device["identifiers"][0] = "gravelping_bell";
        device["name"]           = DEVICE_NAME_STR;
        device["model"]          = "GravelPing Bell Controller";
        device["manufacturer"]   = "Custom";
    };

    // -------------------------------------------------------------------------
    // Button: Single Ring
    // -------------------------------------------------------------------------
    doc.clear();
    doc["name"]               = "Ring Once";
    doc["unique_id"]          = "gravelping_bell_ring_single";
    doc["command_topic"]      = TOPIC_CMD_SINGLE;
    doc["payload_press"]      = "PRESS";
    doc["icon"]               = "mdi:bell";
    addDevice(doc);

    payload = "";
    serializeJson(doc, payload);
    if (mqttClient.publish("homeassistant/button/gravelping_bell/ring_single/config", payload.c_str(), true)) {
        Serial.println(F("[MQTT]   ✓ Button: Ring Once"));
    } else {
        Serial.println(F("[MQTT]   ✗ Failed: Ring Once"));
    }

    // -------------------------------------------------------------------------
    // Button: Pattern Ring
    // -------------------------------------------------------------------------
    doc.clear();
    doc["name"]               = "Ring Pattern";
    doc["unique_id"]          = "gravelping_bell_ring_pattern";
    doc["command_topic"]      = TOPIC_CMD_PATTERN;
    doc["payload_press"]      = "PRESS";
    doc["icon"]               = "mdi:bell-ring";
    addDevice(doc);

    payload = "";
    serializeJson(doc, payload);
    if (mqttClient.publish("homeassistant/button/gravelping_bell/ring_pattern/config", payload.c_str(), true)) {
        Serial.println(F("[MQTT]   ✓ Button: Ring Pattern"));
    } else {
        Serial.println(F("[MQTT]   ✗ Failed: Ring Pattern"));
    }

    // -------------------------------------------------------------------------
    // Sensor: Bell Status
    // -------------------------------------------------------------------------
    doc.clear();
    doc["name"]          = "Bell Status";
    doc["unique_id"]     = "gravelping_bell_status";
    doc["state_topic"]   = TOPIC_STATUS;
    doc["icon"]          = "mdi:bell-check";
    addDevice(doc);

    payload = "";
    serializeJson(doc, payload);
    if (mqttClient.publish("homeassistant/sensor/gravelping_bell/status/config", payload.c_str(), true)) {
        Serial.println(F("[MQTT]   ✓ Sensor: Bell Status"));
    } else {
        Serial.println(F("[MQTT]   ✗ Failed: Bell Status"));
    }

    // Subscribe to command topics
    mqttClient.subscribe(TOPIC_CMD_SINGLE);
    mqttClient.subscribe(TOPIC_CMD_PATTERN);
    Serial.println(F("[MQTT] Subscribed to command topics"));
    Serial.println(F("[MQTT] Discovery complete"));
}

// ============================================================================
// OTA UPDATES
// ============================================================================

#ifdef ENABLE_OTA_UPDATES
void setupOTA() {
    Serial.println(F("[OTA] Initializing Over-The-Air updates..."));

    #ifndef OTA_HOSTNAME
    #define OTA_HOSTNAME "GravelPingBell"
    #endif
    #ifndef OTA_PASSWORD
    #define OTA_PASSWORD "gravelping"
    #endif

    ArduinoOTA.setHostname(TOSTRING(OTA_HOSTNAME));
    ArduinoOTA.setPassword(TOSTRING(OTA_PASSWORD));
    ArduinoOTA.setPort(3232);

    ArduinoOTA.onStart([]() {
        String type = (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem";
        Serial.println("[OTA] Start updating " + type);
        setRGB(CRGB::Magenta);
    });

    ArduinoOTA.onEnd([]() {
        Serial.println(F("\n[OTA] Update complete!"));
        setRGB(CRGB::Green);
        delay(1000);
    });

    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        static unsigned long lastPrint = 0;
        unsigned long now = millis();
        if (now - lastPrint > 1000) {
            Serial.printf("[OTA] Progress: %u%%\n", (progress / (total / 100)));
            lastPrint = now;
            static bool ledState = false;
            setRGB(ledState ? CRGB::Magenta : CRGB::Black);
            ledState = !ledState;
        }
    });

    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("[OTA] Error[%u]: ", error);
        if      (error == OTA_AUTH_ERROR)    Serial.println(F("Auth Failed"));
        else if (error == OTA_BEGIN_ERROR)   Serial.println(F("Begin Failed"));
        else if (error == OTA_CONNECT_ERROR) Serial.println(F("Connect Failed"));
        else if (error == OTA_RECEIVE_ERROR) Serial.println(F("Receive Failed"));
        else if (error == OTA_END_ERROR)     Serial.println(F("End Failed"));
        flashRGB(Colors::ERROR, 5, 200, 200);
    });

    ArduinoOTA.begin();

    Serial.println(F("[OTA] ✓ OTA ready"));
    Serial.printf("[OTA]    Hostname : %s.local\n", TOSTRING(OTA_HOSTNAME));
    Serial.printf("[OTA]    Port     : 3232\n");
    Serial.printf("[OTA]    Password : %s\n", TOSTRING(OTA_PASSWORD));
}
#endif
