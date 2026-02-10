#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <HTTPClient.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <M5Unified.h>
#include <time.h>
#include "secrets.h"

// ------------------------- Agent A: Hardware Abstraction -------------------------
constexpr gpio_num_t REED_PIN = GPIO_NUM_26;  // RTC-capable pin
constexpr int DOOR_CLOSED_LEVEL = LOW;        // INPUT_PULLUP + reed to GND

// --------------------------- Agent B: Power Manager ------------------------------
constexpr uint32_t CPU_FREQ_MHZ = 80;
constexpr uint64_t HEARTBEAT_INTERVAL_US = 6ULL * 60ULL * 60ULL * 1000000ULL;  // 6h
constexpr bool HEARTBEAT_TIMER_ENABLED = true;
constexpr uint32_t NTP_SYNC_TIMEOUT_MS = 1200;

// --------------------------- Agent C: Sensor Manager -----------------------------
constexpr uint8_t DEBOUNCE_SAMPLES = 7;
constexpr uint16_t DEBOUNCE_GAP_MS = 8;
constexpr uint16_t IMU_WINDOW_MS = 240;
constexpr uint16_t IMU_SAMPLE_GAP_MS = 20;
constexpr float TAMPER_G_THRESHOLD = 0.28f;

// ---------------------------- Agent D: Event Engine ------------------------------
constexpr uint32_t TAMPER_COOLDOWN_MS = 2000;
constexpr char TOPIC_STATE[] = "door/state";
constexpr char TOPIC_TAMPER[] = "door/tamper";
constexpr char TOPIC_HEARTBEAT[] = "door/heartbeat";

RTC_DATA_ATTR int rtcLastDoorState = -1;
RTC_DATA_ATTR uint32_t rtcLastTamperUptimeMs = 0;
RTC_DATA_ATTR uint32_t rtcBootCounter = 0;

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

struct RuntimeContext {
  bool doorClosed;
  bool stateChanged;
  bool tamperDetected;
  bool heartbeatWake;
  esp_sleep_wakeup_cause_t wakeReason;
};

String wakeReasonToText(esp_sleep_wakeup_cause_t cause) {
  switch (cause) {
    case ESP_SLEEP_WAKEUP_EXT0: return "EXT0";
    case ESP_SLEEP_WAKEUP_TIMER: return "TIMER";
    case ESP_SLEEP_WAKEUP_UNDEFINED: return "POWER_ON";
    default: return "OTHER";
  }
}

String eventToText(const RuntimeContext& ctx) {
  if (ctx.tamperDetected) return "FORCE_DETECTED";
  if (ctx.stateChanged) return ctx.doorClosed ? "CLOSED" : "OPEN";
  if (ctx.heartbeatWake) return "HEARTBEAT";
  return "NO_EVENT";
}

bool readDoorClosedDebounced() {
  uint8_t lowCount = 0;
  uint8_t highCount = 0;

  for (uint8_t i = 0; i < DEBOUNCE_SAMPLES; ++i) {
    int v = digitalRead(REED_PIN);
    if (v == LOW) {
      lowCount++;
    } else {
      highCount++;
    }
    delay(DEBOUNCE_GAP_MS);
  }

  int stableLevel = (lowCount >= highCount) ? LOW : HIGH;
  return stableLevel == DOOR_CLOSED_LEVEL;
}

bool detectTamperMovement() {
  float baseX, baseY, baseZ;
  M5.Imu.getAccelData(&baseX, &baseY, &baseZ);

  float maxDelta = 0.0f;
  uint16_t elapsed = 0;

  while (elapsed < IMU_WINDOW_MS) {
    float x, y, z;
    M5.Imu.getAccelData(&x, &y, &z);

    float dx = x - baseX;
    float dy = y - baseY;
    float dz = z - baseZ;
    float delta = sqrtf(dx * dx + dy * dy + dz * dz);

    if (delta > maxDelta) {
      maxDelta = delta;
    }

    delay(IMU_SAMPLE_GAP_MS);
    elapsed += IMU_SAMPLE_GAP_MS;
  }

  uint32_t now = millis();
  bool cooldownPassed = (now - rtcLastTamperUptimeMs) >= TAMPER_COOLDOWN_MS;
  bool tamper = cooldownPassed && (maxDelta >= TAMPER_G_THRESHOLD);

  if (tamper) {
    rtcLastTamperUptimeMs = now;
  }

  return tamper;
}

bool connectWifiFast() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  const uint32_t start = millis();
  constexpr uint32_t WIFI_TIMEOUT_MS = 6000;

  while (WiFi.status() != WL_CONNECTED && (millis() - start) < WIFI_TIMEOUT_MS) {
    delay(25);
  }

  return WiFi.status() == WL_CONNECTED;
}

void syncClockFast() {
  configTzTime("UTC0", "pool.ntp.org", "time.nist.gov");

  const uint32_t start = millis();
  time_t now = time(nullptr);
  while (now < 1700000000 && (millis() - start) < NTP_SYNC_TIMEOUT_MS) {
    delay(50);
    now = time(nullptr);
  }
}

String buildTimestamp() {
  time_t now = time(nullptr);
  if (now > 1700000000) {
    struct tm timeInfo;
    gmtime_r(&now, &timeInfo);
    char iso[32];
    strftime(iso, sizeof(iso), "%Y-%m-%dT%H:%M:%SZ", &timeInfo);
    return String(iso);
  }

  return String("uptime_ms:") + String(millis());
}

bool connectMqttFast() {
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);

  const uint32_t start = millis();
  constexpr uint32_t MQTT_TIMEOUT_MS = 2500;

  while (!mqttClient.connected() && (millis() - start) < MQTT_TIMEOUT_MS) {
    bool ok;
    if (strlen(MQTT_USERNAME) > 0) {
      ok = mqttClient.connect(MQTT_CLIENT_ID, MQTT_USERNAME, MQTT_PASSWORD);
    } else {
      ok = mqttClient.connect(MQTT_CLIENT_ID);
    }
    if (ok) return true;
    delay(120);
  }

  return mqttClient.connected();
}

String buildPayload(const RuntimeContext& ctx, const char* eventName) {
  StaticJsonDocument<256> doc;
  doc["device"] = "m5stickc-plus2";
  doc["event"] = eventName;
  doc["timestamp"] = buildTimestamp();
  doc["battery_mv"] = M5.Power.getBatteryVoltage();
  doc["wake_reason"] = wakeReasonToText(ctx.wakeReason);
  doc["boot_count"] = rtcBootCounter;

  String out;
  serializeJson(doc, out);
  return out;
}

void publishEvent(const RuntimeContext& ctx) {
  if (!connectWifiFast()) {
    return;
  }

  syncClockFast();

  bool mqttOk = connectMqttFast();

  if (mqttOk) {
    if (ctx.stateChanged) {
      const char* stateEvent = ctx.doorClosed ? "CLOSED" : "OPEN";
      String payload = buildPayload(ctx, stateEvent);
      mqttClient.publish(TOPIC_STATE, payload.c_str(), true);
    }

    if (ctx.tamperDetected) {
      String payload = buildPayload(ctx, "FORCE_DETECTED");
      mqttClient.publish(TOPIC_TAMPER, payload.c_str(), false);
    }

    if (ctx.heartbeatWake) {
      String payload = buildPayload(ctx, "HEARTBEAT");
      mqttClient.publish(TOPIC_HEARTBEAT, payload.c_str(), false);
    }

    mqttClient.disconnect();
  }

  if (TELEGRAM_ENABLED) {
    String text = String("DoorSensor: ") + eventToText(ctx);
    String url = String("https://api.telegram.org/bot") + TELEGRAM_BOT_TOKEN + "/sendMessage";

    HTTPClient http;
    http.begin(url);
    http.addHeader("Content-Type", "application/json");

    StaticJsonDocument<256> telegramBody;
    telegramBody["chat_id"] = TELEGRAM_CHAT_ID;
    telegramBody["text"] = text;

    String body;
    serializeJson(telegramBody, body);
    http.POST(body);
    http.end();
  }

  WiFi.disconnect(true, true);
  WiFi.mode(WIFI_OFF);
}

void configureDeepSleep(bool doorClosedNow) {
  // EXT0 wakes on a single logic level. Configure the next wake level as the
  // opposite of current stable door level, so both OPEN and CLOSE transitions
  // can wake the ESP32 across cycles.
  const int currentLevel = doorClosedNow ? DOOR_CLOSED_LEVEL : !DOOR_CLOSED_LEVEL;
  const int wakeLevel = (currentLevel == LOW) ? HIGH : LOW;
  esp_sleep_enable_ext0_wakeup(REED_PIN, wakeLevel);

  if (HEARTBEAT_TIMER_ENABLED) {
    esp_sleep_enable_timer_wakeup(HEARTBEAT_INTERVAL_US);
  }
}

void goToSleep(bool doorClosedNow) {
  configureDeepSleep(doorClosedNow);
  delay(20);
  esp_deep_sleep_start();
}

RuntimeContext evaluateState() {
  RuntimeContext ctx{};
  ctx.wakeReason = esp_sleep_get_wakeup_cause();
  ctx.heartbeatWake = (ctx.wakeReason == ESP_SLEEP_WAKEUP_TIMER);

  bool closed = readDoorClosedDebounced();
  ctx.doorClosed = closed;

  if (rtcLastDoorState < 0) {
    ctx.stateChanged = true;
  } else {
    ctx.stateChanged = (static_cast<int>(closed) != rtcLastDoorState);
  }

  bool ignoreImuForTransition = ctx.stateChanged && !closed;
  if (!ignoreImuForTransition && closed) {
    ctx.tamperDetected = detectTamperMovement();
  }

  rtcLastDoorState = static_cast<int>(closed);
  return ctx;
}

void setup() {
  setCpuFrequencyMhz(CPU_FREQ_MHZ);

  auto cfg = M5.config();
  M5.begin(cfg);
  M5.Display.sleep();
  M5.Display.setBrightness(0);

  pinMode(REED_PIN, INPUT_PULLUP);

  rtcBootCounter++;

  RuntimeContext ctx = evaluateState();

  if (ctx.stateChanged || ctx.tamperDetected || ctx.heartbeatWake) {
    publishEvent(ctx);
  }

  goToSleep(ctx.doorClosed);
}

void loop() {}
