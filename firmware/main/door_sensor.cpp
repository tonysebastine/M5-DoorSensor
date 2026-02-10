#include <esp_log.h>
#include <esp_system.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_sleep.h>
#include <nvs_flash.h>
#include <driver/gpio.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "sdkconfig.h"
#include "esp_sntp.h"

// For MQTT
#include <mqtt_client.h>
// For JSON
#include <cJSON.h>
// For HTTP Client (for Telegram)
#include <esp_http_client.h>

// Tag for ESP_LOG messages
static const char *TAG = "DOORSENSOR";

// --- Configuration Constants (will be moved to sdkconfig or CMake defines) ---
// For now, these are direct definitions.
// NOTE: These should ideally come from Kconfig/sdkconfig or CMake for proper ESP-IDF integration.
// For now, using placeholders or direct values.
// Static IP and Wi-Fi credentials will eventually come from CMake/Kconfig.

// ------------------------- Agent A: Hardware Abstraction -------------------------
// No external reed switch used. Door state detected via inbuilt IMU.
#include <driver/i2c.h>
#include "bmi270.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

#define I2C_MASTER_SDA_IO           GPIO_NUM_21
#define I2C_MASTER_SCL_IO           GPIO_NUM_22
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          400000
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0

static bmi270_t bmi270_dev;

// ADC definitions for battery voltage reading (M5StickC Plus2 uses GPIO38)
#define BATTERY_ADC_UNIT            ADC_UNIT_1
#define BATTERY_ADC_CHANNEL         ADC_CHANNEL_2    // GPIO38 is ADC1_CHANNEL_2
#define BATTERY_ADC_ATTEN           ADC_ATTEN_DB_11  // 11dB attenuation for a wider voltage range
#define BATTERY_ADC_WIDTH           ADC_WIDTH_BIT_12 // 12-bit resolution
#define BATTERY_VOLTAGE_DIVIDER_FACTOR 2.0
#define BATTERY_ADC_SAMPLE_COUNT    64 // Number of samples to average

static esp_adc_cal_characteristics_t adc_chars;

// --------------------------- Agent B: Power Manager ------------------------------
// CPU_FREQ_MHZ is usually set in sdkconfig, but 80MHz is a common default for low power.
// constexpr uint32_t CPU_FREQ_MHZ = 80; // This will be handled by sdkconfig
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
static const char* TOPIC_STATE = "door/state";
static const char* TOPIC_TAMPER = "door/tamper";
static const char* TOPIC_HEARTBEAT = "door/heartbeat";

// RTC_DATA_ATTR is already ESP-IDF compatible
RTC_DATA_ATTR int rtcLastDoorState = -1;
RTC_DATA_ATTR uint32_t rtcLastTamperUptimeMs = 0;
RTC_DATA_ATTR uint32_t rtcBootCounter = 0;

// Placeholder for WiFi event group, will be used in connectWifi()
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

// MQTT Client Handle
static esp_mqtt_client_handle_t mqtt_client = NULL;

// Global variables for Wi-Fi and MQTT configuration (will come from Kconfig/CMake)
// These are expected to be defined as compile definitions by CMake.
// If not defined, a compile error will occur for critical ones.

#ifndef WIFI_SSID
#error "WIFI_SSID is not defined. Please define it via CMake/Kconfig or in secrets management."
#endif
#ifndef WIFI_PASSWORD
#error "WIFI_PASSWORD is not defined. Please define it via CMake/Kconfig or in secrets management."
#endif
#ifndef MQTT_HOST
#error "MQTT_HOST is not defined. Please define it via CMake/Kconfig or in secrets management."
#endif
#ifndef MQTT_PORT
#error "MQTT_PORT is not defined. Please define it via CMake/Kconfig or in secrets management."
#endif
#ifndef MQTT_USERNAME
#define MQTT_USERNAME "" // Default to empty if not provided
#endif
#ifndef MQTT_PASSWORD
#define MQTT_PASSWORD "" // Default to empty if not provided
#endif
#ifndef MQTT_CLIENT_ID
#define MQTT_CLIENT_ID "m5-door-sensor" // Provide a default if not set
#endif

// Optional Telegram. Provide defaults if not defined.
#ifndef TELEGRAM_ENABLED
#define TELEGRAM_ENABLED false
#endif
#ifndef TELEGRAM_BOT_TOKEN
#define TELEGRAM_BOT_TOKEN ""
#endif
#ifndef TELEGRAM_CHAT_ID
#define TELEGRAM_CHAT_ID ""
#endif

// Static IP. Provide defaults if not defined.
#ifndef USE_STATIC_IP
#define USE_STATIC_IP false
#endif
#ifndef STATIC_IP_ADDRESS
#define STATIC_IP_ADDRESS ""
#endif
#ifndef STATIC_GATEWAY
#define STATIC_GATEWAY ""
#endif
#ifndef STATIC_SUBNET
#define STATIC_SUBNET ""
#endif
#ifndef STATIC_DNS
#define STATIC_DNS ""
#endif


struct RuntimeContext {
  bool doorClosed;
  bool stateChanged;
  bool tamperDetected;
  esp_sleep_wakeup_cause_t wakeReason;
  bool heartbeatWake; // Added for clarity
};

// --- Function Prototypes ---
// Declarations for functions that will be defined later, to avoid implicit declaration warnings
void configureDeepSleep(bool doorClosedNow);
void goToSleep(bool doorClosedNow);
RuntimeContext evaluateState();
esp_err_t connectWifi(); // Returns ESP_OK on success
void publishEvent(const RuntimeContext& ctx);
esp_err_t syncClock(); // Returns ESP_OK on success
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);


const char* wakeReasonToText(esp_sleep_wakeup_cause_t cause) {
  switch (cause) {
    case ESP_SLEEP_WAKEUP_EXT0: return "EXT0";
    case ESP_SLEEP_WAKEUP_TIMER: return "TIMER";
    case ESP_SLEEP_WAKEUP_UNDEFINED: return "POWER_ON";
    default: return "OTHER";
  }
}

const char* eventToText(const RuntimeContext& ctx) {
  if (ctx.tamperDetected) return "FORCE_DETECTED";
  if (ctx.stateChanged) return ctx.doorClosed ? "CLOSED" : "OPEN";
  if (ctx.heartbeatWake) return "HEARTBEAT";
  return "NO_EVENT";
}

// Function to initialize I2C and BMI270 sensor
esp_err_t initIMU() {
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL; // Use default clock source
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C parameter configuration failed: %s", esp_err_to_name(err));
        return err;
    }
    err = i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver installation failed: %s", esp_err_to_name(err));
        return err;
    }

    // Initialize BMI270
    bmi270_dev.i2c_addr = BMI270_I2C_ADDR_LOW; // Or BMI270_I2C_ADDR_HIGH depending on configuration
    bmi270_dev.i2c_port = I2C_MASTER_NUM;
    err = bmi270_init(&bmi270_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "BMI270 initialization failed: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "BMI270 initialized successfully.");
    return ESP_OK;
}

// Function to initialize ADC for battery voltage reading
esp_err_t initADC() {
    ESP_ERROR_CHECK(adc1_config_width(BATTERY_ADC_WIDTH));
    ESP_ERROR_CHECK(adc1_config_channel_atten(BATTERY_ADC_CHANNEL, BATTERY_ADC_ATTEN));

    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(BATTERY_ADC_UNIT, BATTERY_ADC_ATTEN, BATTERY_ADC_WIDTH, 1100, &adc_chars);
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        ESP_LOGI(TAG, "ADC calibration: eFuse Vref in use");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        ESP_LOGI(TAG, "ADC calibration: eFuse Two Point in use");
    } else {
        ESP_LOGI(TAG, "ADC calibration: Default Vref in use");
    }
    ESP_LOGI(TAG, "ADC for battery voltage initialized successfully.");
    return ESP_OK;
}

bool readDoorClosedDebounced() {
  float sum_z_accel = 0;
  for (uint8_t i = 0; i < DEBOUNCE_SAMPLES; ++i) {
    bmi270_sensor_data_t accel;
    esp_err_t err = bmi270_get_acceleration(&bmi270_dev, &accel);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to read BMI270 acceleration for debounce: %s", esp_err_to_name(err));
      // In case of error, assume previous state or a safe default.
      // For now, let's just use 0 and hope for recovery in next cycle.
      sum_z_accel += 0;
    } else {
      sum_z_accel += accel.z;
    }
    vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_GAP_MS));
  }

  float avg_z_accel = sum_z_accel / DEBOUNCE_SAMPLES;
  ESP_LOGI(TAG, "Average Z-acceleration: %.2f G", avg_z_accel);

  // Assuming:
  // - When door is CLOSED, device is mostly vertical, Z-axis ~1.0 G (or -1.0 G depending on orientation)
  // - When door is OPEN, device is horizontal, Z-axis ~0.0 G
  // This threshold will need calibration based on actual mounting.
  // For a door in vertical position, if Z points upwards (closed), it's +1G. If Z points downwards (closed), it's -1G.
  // When open, it might be horizontal, so Z is ~0G.
  // Let's assume a mounting where Z is around 1G when closed.
  const float DOOR_CLOSED_Z_THRESHOLD_MIN = 0.8f; // Example threshold
  const float DOOR_CLOSED_Z_THRESHOLD_MAX = 1.2f; // Example threshold

  bool currently_closed = (avg_z_accel > DOOR_CLOSED_Z_THRESHOLD_MIN && avg_z_accel < DOOR_CLOSED_Z_THRESHOLD_MAX);

  return currently_closed;
}

bool detectTamperMovement() {
  bmi270_sensor_data_t accel;
  esp_err_t err = bmi270_get_acceleration(&bmi270_dev, &accel);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to read BMI270 acceleration: %s", esp_err_to_name(err));
    return false;
  }

  // Calculate magnitude of acceleration vector
  float accel_magnitude = sqrt(accel.x * accel.x + accel.y * accel.y + accel.z * accel.z);

  // Compare with threshold. This is a simple thresholding, more complex logic might be needed.
  if (accel_magnitude > TAMPER_G_THRESHOLD) {
    ESP_LOGW(TAG, "Tamper detected! Accel magnitude: %.2f G (Threshold: %.2f G)", accel_magnitude, TAMPER_G_THRESHOLD);
    return true;
  }
  return false;
}

// Function to get battery voltage from ADC
uint32_t getBatteryVoltageMv() {
    uint32_t adc_reading = 0;
    // Take multiple samples for better accuracy
    for (int i = 0; i < BATTERY_ADC_SAMPLE_COUNT; i++) {
        adc_reading += adc1_get_raw(BATTERY_ADC_CHANNEL);
    }
    adc_reading /= BATTERY_ADC_SAMPLE_COUNT;

    // Convert raw ADC reading to voltage in mV
    uint32_t voltage_mv = esp_adc_cal_raw_to_voltage(adc_reading, &adc_chars);

    // Apply voltage divider factor
    voltage_mv = (uint32_t)(voltage_mv * BATTERY_VOLTAGE_DIVIDER_FACTOR);

    ESP_LOGI(TAG, "Battery voltage: %d mV", voltage_mv);
    return voltage_mv;
}


static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "Wi-Fi disconnected, trying to reconnect...");
        esp_wifi_connect();
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP address: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        xEventGroupClearBits(s_wifi_event_group, WIFI_FAIL_BIT);
    }
}

esp_err_t connectWifi() {
    s_wifi_event_group = xEventGroupCreate();

    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    esp_event_handler_instance_register(WIFI_EVENT,
                                            ESP_EVENT_ANY_ID,
                                            &wifi_event_handler,
                                            NULL,
                                            &instance_any_id);
    esp_event_handler_instance_register(IP_EVENT,
                                            IP_EVENT_STA_GOT_IP,
                                            &wifi_event_handler,
                                            NULL,
                                            &instance_got_ip);

    // Apply static IP configuration if enabled
    if (USE_STATIC_IP) {
        ESP_LOGI(TAG, "Configuring static IP: %s", STATIC_IP_ADDRESS);
        esp_netif_dhcpc_stop(sta_netif); // Stop DHCP client
        esp_netif_ip_info_t ip_info;
        ip4_addr_t ip, gw, netmask, dns;

        ip4addr_aton(STATIC_IP_ADDRESS, &ip);
        ip4addr_aton(STATIC_GATEWAY, &gw);
        ip4addr_aton(STATIC_SUBNET, &netmask);
        ip4addr_aton(STATIC_DNS, &dns); // Assuming a single DNS

        ip_info.ip = ip;
        ip_info.gw = gw;
        ip_info.netmask = netmask;

        esp_netif_set_ip_info(sta_netif, &ip_info);
        esp_netif_set_dns_info(sta_netif, ESP_NETIF_DNS_MAIN, &dns); // Set main DNS
    }

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = "", // Will be replaced by Kconfig/CMake
            .password = "", // Will be replaced by Kconfig/CMake
            .threshold = {
                .rssi = -127,
                .authmode = WIFI_AUTH_WPA2_PSK,
            },
            .sae_pwe_h2e = WPA3_SAE_PWE_BOTH, // This might need adjustment based on Kconfig.
        },
    };
    // Copy SSID and Password from defines to wifi_config
    strncpy((char*)wifi_config.sta.ssid, WIFI_SSID, sizeof(wifi_config.sta.ssid));
    strncpy((char*)wifi_config.sta.password, WIFI_PASSWORD, sizeof(wifi_config.sta.password));


    if (strlen(WIFI_SSID) == 0 || strlen(WIFI_PASSWORD) == 0) {
        ESP_LOGE(TAG, "Missing Wi-Fi credentials in Kconfig");
        return ESP_FAIL;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Connecting to Wi-Fi SSID: %s", WIFI_SSID);

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           pdMS_TO_TICKS(NTP_SYNC_TIMEOUT_MS)); // Using NTP_SYNC_TIMEOUT_MS as a general WiFi timeout for now.

    esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip);
    esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id);
    vEventGroupDelete(s_wifi_event_group);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Wi-Fi Connected!");
        return ESP_OK;
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGE(TAG, "Failed to connect to Wi-Fi");
        return ESP_FAIL;
    } else {
        ESP_LOGE(TAG, "Wi-Fi connection timed out");
        return ESP_FAIL;
    }
}

esp_err_t syncClock() {
    ESP_LOGI(TAG, "Synchronizing system time with NTP server...");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_setservername(1, "time.nist.gov");
    sntp_init();

    time_t now = 0;
    struct tm timeinfo = { 0 };
    int retry = 0;
    const int retry_count = 10;
    while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++retry < retry_count) {
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    time(&now);
    localtime_r(&now, &timeinfo);

    if (timeinfo.tm_year < (2020 - 1900)) { // Check if time is sensible (e.g., after 2020)
        ESP_LOGE(TAG, "Time not set from NTP server!");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Time synchronized: %s", asctime(&timeinfo));
    return ESP_OK;
}

char* buildTimestamp() {
  static char iso[32]; // Use static buffer as String is not available
  time_t now = 0;
  time(&now);
  struct tm timeinfo;
  localtime_r(&now, &timeinfo);
  strftime(iso, sizeof(iso), "%Y-%m-%dT%H:%M:%SZ", &timeinfo);
  return iso;
}


// MQTT Event Handler
static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event) {
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    // your_context_t *context = event->context;
    switch ((esp_mqtt_event_id_t)event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            // Optionally subscribe here
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            break;
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "MQTT_EVENT_DATA");
            // This device doesn't subscribe, so this case might not be needed.
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
            if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
                ESP_LOGI(TAG, "Last error code reported from mqtt event: 0x%x", event->error_handle->esp_transport_sock_errno);
            }
            break;
        default:
            ESP_LOGI(TAG, "Other event id:%d", event->event_id);
            break;
    }
    return ESP_OK;
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    mqtt_event_handler_cb((esp_mqtt_event_handle_t)event_data);
}


esp_err_t connectMqtt() {
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker = {
            .address = {
                .uri = "mqtt://" MQTT_HOST ":" TOSTRING(MQTT_PORT), // Construct URI
            },
            .username = MQTT_USERNAME,
            .password = MQTT_PASSWORD,
        },
        .client_id = MQTT_CLIENT_ID,
    };

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, mqtt_client);
    esp_err_t err = esp_mqtt_client_start(mqtt_client);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start MQTT client: %s", esp_err_to_name(err));
        return err;
    }

    // Wait for connection (simple check, proper handling would use event groups)
    int retry_count = 0;
    while (esp_mqtt_client_get_state(mqtt_client) != ESP_MQTT_STATE_CONNECTED && retry_count < 5) {
        ESP_LOGI(TAG, "Waiting for MQTT connection... (%d/5)", retry_count + 1);
        vTaskDelay(pdMS_TO_TICKS(500));
        retry_count++;
    }

    if (esp_mqtt_client_get_state(mqtt_client) == ESP_MQTT_STATE_CONNECTED) {
        ESP_LOGI(TAG, "MQTT Connected!");
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "Failed to connect to MQTT broker.");
        esp_mqtt_client_destroy(mqtt_client);
        mqtt_client = NULL;
        return ESP_FAIL;
    }
}


cJSON* buildPayload(const RuntimeContext& ctx, const char* eventName) {
  cJSON *root = cJSON_CreateObject();
  if (root == NULL) {
      ESP_LOGE(TAG, "Failed to create JSON object");
      return NULL;
  }

  cJSON_AddStringToObject(root, "device", "m5stickc-plus2");
  cJSON_AddStringToObject(root, "event", eventName);
  cJSON_AddStringToObject(root, "timestamp", buildTimestamp());
  cJSON_AddNumberToObject(root, "battery_mv", getBatteryVoltageMv());
  cJSON_AddStringToObject(root, "wake_reason", wakeReasonToText(ctx.wakeReason));
  cJSON_AddNumberToObject(root, "boot_count", rtcBootCounter);

  return root;
}


void publishEvent(const RuntimeContext& ctx) {
  // Initialize NVS for Wi-Fi and other functionalities
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  if (connectWifi() != ESP_OK) {
      ESP_LOGE(TAG, "Failed to connect to Wi-Fi. Aborting event publish.");
      return;
  }

  if (syncClock() != ESP_OK) {
      ESP_LOGE(TAG, "Failed to synchronize clock. Using uptime for timestamp.");
  }

  if (connectMqtt() == ESP_OK) {
    if (ctx.stateChanged) {
      const char* stateEvent = ctx.doorClosed ? "CLOSED" : "OPEN";
      cJSON *payload_json = buildPayload(ctx, stateEvent);
      if (payload_json) {
          char *payload_str = cJSON_PrintUnformatted(payload_json);
          esp_mqtt_client_publish(mqtt_client, TOPIC_STATE, payload_str, 0, 1, 0);
          cJSON_Delete(payload_json);
          free(payload_str);
      }
    }

    if (ctx.tamperDetected) {
      cJSON *payload_json = buildPayload(ctx, "FORCE_DETECTED");
      if (payload_json) {
          char *payload_str = cJSON_PrintUnformatted(payload_json);
          esp_mqtt_client_publish(mqtt_client, TOPIC_TAMPER, payload_str, 0, 0, 0);
          cJSON_Delete(payload_json);
          free(payload_str);
      }
    }

    if (ctx.heartbeatWake) {
      cJSON *payload_json = buildPayload(ctx, "HEARTBEAT");
      if (payload_json) {
          char *payload_str = cJSON_PrintUnformatted(payload_json);
          esp_mqtt_client_publish(mqtt_client, TOPIC_HEARTBEAT, payload_str, 0, 0, 0);
          cJSON_Delete(payload_json);
          free(payload_str);
      }
    }
    esp_mqtt_client_stop(mqtt_client);
    esp_mqtt_client_destroy(mqtt_client);
    mqtt_client = NULL;
  } else {
    ESP_LOGE(TAG, "Failed to connect to MQTT broker. Skipping MQTT publish.");
  }


  if (TELEGRAM_ENABLED) {
    // String text = String("DoorSensor: ") + eventToText(ctx); // Replace String with char* and sprintf
    char text_buf[64]; // Max length for now
    snprintf(text_buf, sizeof(text_buf), "DoorSensor: %s", eventToText(ctx));

    // String url = String("https://api.telegram.org/bot") + TELEGRAM_BOT_TOKEN + "/sendMessage"; // Replace with esp_http_client
    char url_buf[128]; // Max URL length for now
    snprintf(url_buf, sizeof(url_buf), "https://api.telegram.org/bot%s/sendMessage", TELEGRAM_BOT_TOKEN);

    cJSON *telegramBody = cJSON_CreateObject();
    if (telegramBody) {
        cJSON_AddStringToObject(telegramBody, "chat_id", TELEGRAM_CHAT_ID);
        cJSON_AddStringToObject(telegramBody, "text", text_buf);
        char *telegram_payload = cJSON_PrintUnformatted(telegramBody);

        if (strlen(TELEGRAM_BOT_TOKEN) > 0 && strlen(TELEGRAM_CHAT_ID) > 0) {
            esp_http_client_config_t http_cfg = {
                .url = url_buf,
                .method = HTTP_METHOD_POST,
            };
            esp_http_client_handle_t http_client = esp_http_client_init(&http_cfg);
            esp_http_client_set_header(http_client, "Content-Type", "application/json");
            esp_http_client_set_post_field(http_client, telegram_payload, strlen(telegram_payload));
            esp_err_t err = esp_http_client_perform(http_client);

            if (err == ESP_OK) {
                ESP_LOGI(TAG, "Telegram message sent successfully");
            } else {
                ESP_LOGE(TAG, "Telegram HTTP POST request failed: %s", esp_err_to_name(err));
            }
            esp_http_client_cleanup(http_client);
        } else {
            ESP_LOGW(TAG, "Telegram not configured, skipping message.");
        }
        cJSON_Delete(telegramBody);
        free(telegram_payload);
    }
  }

  ESP_LOGI(TAG, "Disconnecting WiFi...");
  ESP_ERROR_CHECK(esp_wifi_disconnect());
  ESP_ERROR_CHECK(esp_wifi_stop());
  ESP_ERROR_CHECK(esp_wifi_deinit());
  esp_netif_deinit(); // De-initialize the network interface.
  esp_event_loop_delete_default(); // Delete the default event loop.
  // nvs_flash_deinit(); // NVS de-initialization can be done here if no longer needed.
}


void configureDeepSleep(bool doorClosedNow) {
  // Deep sleep wakeup will be handled by the timer for now.
  // In the future, this should be replaced with BMI270 motion interrupts for more efficient wakeup.
  if (HEARTBEAT_TIMER_ENABLED) {
    ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(HEARTBEAT_INTERVAL_US));
  }
}

void goToSleep(bool doorClosedNow) {
  configureDeepSleep(doorClosedNow);
  vTaskDelay(pdMS_TO_TICKS(20)); // Replace delay with vTaskDelay
  ESP_LOGI(TAG, "Entering deep sleep...");
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

// The main entry point for ESP-IDF applications
extern "C" void app_main() {
  // Initialize NVS (needed for Wi-Fi)
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  ESP_LOGI(TAG, "M5-DoorSensor Starting (Boot Count: %d)", rtcBootCounter);

  // Initialize IMU
  ESP_ERROR_CHECK(initIMU());

  // Initialize ADC for battery voltage
  ESP_ERROR_CHECK(initADC());

  rtcBootCounter++;

  RuntimeContext ctx = evaluateState();

  if (ctx.stateChanged || ctx.tamperDetected || ctx.heartbeatWake) {
    publishEvent(ctx);
  }

  goToSleep(ctx.doorClosed);
}

// loop() is not used in ESP-IDF
// void loop() {}

// Helper macro to stringify values (needed for MQTT URI)
#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)