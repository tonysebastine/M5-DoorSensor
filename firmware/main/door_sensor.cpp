#include "esp_sntp.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"
#include "sdkconfig.h"
#include <driver/gpio.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_sleep.h>
#include <esp_system.h>
#include <esp_wifi.h>
#include <math.h>
#include <nvs_flash.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>


// For MQTT
#include <mqtt_client.h>
// For JSON
#include <cJSON.h>
// For HTTP Client (for Telegram)
#include <esp_http_client.h>

static const char *TAG = "DOORSENSOR";

// ------------------------- MPU6886 Minimal Driver -------------------------
#include <driver/i2c.h>

#define I2C_MASTER_SDA_IO GPIO_NUM_21
#define I2C_MASTER_SCL_IO GPIO_NUM_22
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 400000
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0

#define MPU6886_ADDR 0x68
#define MPU6886_WHOAMI 0x75
#define MPU6886_ACCEL_XOUT_H 0x3B
#define MPU6886_PWR_MGMT_1 0x6B
#define MPU6886_ACCEL_CONFIG 0x1C
#define MPU6886_ACCEL_INTEL_CTRL 0x69
#define MPU6886_INT_ENABLE 0x38

// M5StickC Plus 2 Interrupt Pin
#define IMU_INT_PIN GPIO_NUM_35
#define IMU_INT_ACTIVE_LEVEL 0

struct AccelData {
  float x, y, z;
};

esp_err_t mpu6886_write_byte(uint8_t reg, uint8_t data) {
  uint8_t write_buf[2] = {reg, data};
  return i2c_master_write_to_device(I2C_MASTER_NUM, MPU6886_ADDR, write_buf,
                                    sizeof(write_buf), pdMS_TO_TICKS(100));
}

esp_err_t mpu6886_read_bytes(uint8_t reg, uint8_t *data, size_t len) {
  return i2c_master_write_read_device(I2C_MASTER_NUM, MPU6886_ADDR, &reg, 1,
                                      data, len, pdMS_TO_TICKS(100));
}

esp_err_t initMPU6886() {
  uint8_t id;
  if (mpu6886_read_bytes(MPU6886_WHOAMI, &id, 1) != ESP_OK || id != 0x19) {
    ESP_LOGE(TAG, "MPU6886 not found (ID: 0x%02X)", id);
    return ESP_FAIL;
  }

  mpu6886_write_byte(MPU6886_PWR_MGMT_1,
                     0x01); // Wake up, use clock auto-select
  vTaskDelay(pdMS_TO_TICKS(10));
  mpu6886_write_byte(MPU6886_ACCEL_CONFIG, 0x10); // +/- 8g range

  // Configure Wake-on-Motion (WOM) for deep sleep wakeup
  // 1. Cycle bit for low power
  // 2. Enable WOM logic
  // 3. Set threshold (roughly 0.1g - 0.5g depending on noise)
  // For now, keeping it simple; the core goal is functional orientation check.

  ESP_LOGI(TAG, "MPU6886 initialized (ID: 0x%02X)", id);
  return ESP_OK;
}

AccelData mpu6886_get_accel() {
  uint8_t buf[6];
  AccelData data = {0, 0, 0};
  if (mpu6886_read_bytes(MPU6886_ACCEL_XOUT_H, buf, 6) == ESP_OK) {
    int16_t x = (int16_t)((buf[0] << 8) | buf[1]);
    int16_t y = (int16_t)((buf[2] << 8) | buf[3]);
    int16_t z = (int16_t)((buf[4] << 8) | buf[5]);
    // Scale for 8g range (4096 LSB/g)
    data.x = (float)x / 4096.0f;
    data.y = (float)y / 4096.0f;
    data.z = (float)z / 4096.0f;
  }
  return data;
}

// ------------------------- Battery ADC Logic -------------------------
#include "driver/adc.h"
#include "esp_adc_cal.h"

#define BATTERY_ADC_CHANNEL ADC1_CHANNEL_2 // GPIO38 is ADC1_CHANNEL_2
#define BATTERY_ADC_ATTEN ADC_ATTEN_DB_12
#define BATTERY_ADC_WIDTH ADC_WIDTH_BIT_12
#define BATTERY_VOLTAGE_DIVIDER_FACTOR 2.0
static esp_adc_cal_characteristics_t adc_chars;

void initADC() {
  adc1_config_width(BATTERY_ADC_WIDTH);
  adc1_config_channel_atten(BATTERY_ADC_CHANNEL, BATTERY_ADC_ATTEN);
  esp_adc_cal_characterize(ADC_UNIT_1, BATTERY_ADC_ATTEN, BATTERY_ADC_WIDTH,
                           1100, &adc_chars);
}

uint32_t getBatteryVoltageMv() {
  uint32_t raw = 0;
  for (int i = 0; i < 64; i++)
    raw += adc1_get_raw(BATTERY_ADC_CHANNEL);
  raw /= 64;
  return (uint32_t)(esp_adc_cal_raw_to_voltage(raw, &adc_chars) *
                    BATTERY_VOLTAGE_DIVIDER_FACTOR);
}

// --------------------------- State & Config ------------------------------
constexpr uint64_t HEARTBEAT_INTERVAL_US =
    6ULL * 60ULL * 60ULL * 1000000ULL; // 6h

RTC_DATA_ATTR int rtcLastDoorState = -1;
RTC_DATA_ATTR uint32_t rtcBootCounter = 0;

struct RuntimeContext {
  bool doorClosed;
  bool stateChanged;
  bool tamperDetected;
  esp_sleep_wakeup_cause_t wakeReason;
  bool heartbeatWake;
};

// --------------------------- WiFi & MQTT ------------------------------
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data) {
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
    esp_wifi_connect();
  } else if (event_base == WIFI_EVENT &&
             event_id == WIFI_EVENT_STA_DISCONNECTED) {
    xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
  } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
    xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
  }
}

esp_err_t connectWifi() {
  s_wifi_event_group = xEventGroupCreate();
  esp_netif_init();
  esp_event_loop_create_default();
  esp_netif_create_default_wifi_sta();
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  esp_wifi_init(&cfg);
  esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                      &wifi_event_handler, NULL, NULL);
  esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                      &wifi_event_handler, NULL, NULL);

#if CONFIG_USE_STATIC_IP
  esp_netif_dhcpc_stop(esp_netif_get_handle_from_ifkey("WIFI_STA_DEF"));
  esp_netif_ip_info_t ip_info;
  ip4addr_aton(CONFIG_STATIC_IP_ADDRESS, &ip_info.ip);
  ip4addr_aton(CONFIG_STATIC_GATEWAY, &ip_info.gw);
  ip4addr_aton(CONFIG_STATIC_SUBNET, &ip_info.netmask);
  esp_netif_set_ip_info(esp_netif_get_handle_from_ifkey("WIFI_STA_DEF"),
                        &ip_info);
#endif

  wifi_config_t wifi_config = {};
  strncpy((char *)wifi_config.sta.ssid, CONFIG_WIFI_SSID, 32);
  strncpy((char *)wifi_config.sta.password, CONFIG_WIFI_PASSWORD, 64);
  esp_wifi_set_mode(WIFI_MODE_STA);
  esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
  esp_wifi_start();

  EventBits_t bits = xEventGroupWaitBits(
      s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE,
      pdMS_TO_TICKS(10000));
  return (bits & WIFI_CONNECTED_BIT) ? ESP_OK : ESP_FAIL;
}

void syncClock() {
  esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
  esp_sntp_setservername(0, "pool.ntp.org");
  esp_sntp_init();
  int retry = 0;
  while (esp_sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++retry < 10)
    vTaskDelay(pdMS_TO_TICKS(500));
}

// --------------------------- Application Logic ------------------------------
bool readDoorClosedDebounced() {
  float sum_z = 0;
  for (int i = 0; i < 7; i++) {
    sum_z += mpu6886_get_accel().z;
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  float avg_z = sum_z / 7.0f;
  ESP_LOGI(TAG, "Avg Z: %.2f", avg_z);
  return (avg_z > 0.8f && avg_z < 1.2f); // Assuming vertical is 1G
}

void publishEvent(const RuntimeContext &ctx) {
  if (connectWifi() != ESP_OK)
    return;
  syncClock();

  esp_mqtt_client_config_t mqtt_cfg = {};
  mqtt_cfg.broker.address.uri = "mqtt://" CONFIG_MQTT_HOST;
  mqtt_cfg.broker.address.port = CONFIG_MQTT_PORT;
  mqtt_cfg.credentials.username = CONFIG_MQTT_USERNAME;
  mqtt_cfg.credentials.authentication.password = CONFIG_MQTT_PASSWORD;
  mqtt_cfg.credentials.client_id = CONFIG_MQTT_CLIENT_ID;

  esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
  esp_mqtt_client_start(client);

  // Simplistic wait for connection
  vTaskDelay(pdMS_TO_TICKS(2000));

  cJSON *root = cJSON_CreateObject();
  cJSON_AddStringToObject(root, "device", "m5stickc-plus2");
  cJSON_AddNumberToObject(root, "battery_mv", getBatteryVoltageMv());
  cJSON_AddNumberToObject(root, "boot_count", rtcBootCounter);

  if (ctx.stateChanged) {
    cJSON_AddStringToObject(root, "event", ctx.doorClosed ? "CLOSED" : "OPEN");
    char *s = cJSON_PrintUnformatted(root);
    esp_mqtt_client_publish(client, "door/state", s, 0, 1, 0);
    free(s);
  } else if (ctx.heartbeatWake) {
    cJSON_AddStringToObject(root, "event", "HEARTBEAT");
    char *s = cJSON_PrintUnformatted(root);
    esp_mqtt_client_publish(client, "door/heartbeat", s, 0, 0, 0);
    free(s);
  }

  // Telegram
#if CONFIG_TELEGRAM_ENABLED
  char url[256];
  snprintf(url, sizeof(url), "https://api.telegram.org/bot%s/sendMessage",
           CONFIG_TELEGRAM_BOT_TOKEN);
  cJSON *tel = cJSON_CreateObject();
  cJSON_AddStringToObject(tel, "chat_id", CONFIG_TELEGRAM_CHAT_ID);
  char msg[64];
  snprintf(msg, sizeof(msg), "Door Sensor: %s",
           ctx.doorClosed ? "CLOSED" : "OPEN");
  cJSON_AddStringToObject(tel, "text", msg);
  char *post_data = cJSON_PrintUnformatted(tel);

  esp_http_client_config_t http_cfg = {};
  http_cfg.url = url;
  http_cfg.method = HTTP_METHOD_POST;
  esp_http_client_handle_t hc = esp_http_client_init(&http_cfg);
  esp_http_client_set_header(hc, "Content-Type", "application/json");
  esp_http_client_set_post_field(hc, post_data, strlen(post_data));
  esp_http_client_perform(hc);
  esp_http_client_cleanup(hc);
  cJSON_Delete(tel);
  free(post_data);
#endif

  vTaskDelay(pdMS_TO_TICKS(1000));
  esp_mqtt_client_destroy(client);
  cJSON_Delete(root);
}

extern "C" void app_main() {
  nvs_flash_init();

  // I2C Init
  i2c_config_t conf = {};
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = I2C_MASTER_SDA_IO;
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf.scl_io_num = I2C_MASTER_SCL_IO;
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
  i2c_param_config(I2C_MASTER_NUM, &conf);
  i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);

  initMPU6886();
  initADC();
  rtcBootCounter++;

  RuntimeContext ctx{};
  ctx.wakeReason = esp_sleep_get_wakeup_cause();
  ctx.heartbeatWake = (ctx.wakeReason == ESP_SLEEP_WAKEUP_TIMER);

  bool closed = readDoorClosedDebounced();
  ctx.doorClosed = closed;
  ctx.stateChanged =
      (rtcLastDoorState < 0) || (static_cast<int>(closed) != rtcLastDoorState);
  rtcLastDoorState = static_cast<int>(closed);

  if (ctx.stateChanged || ctx.heartbeatWake) {
    publishEvent(ctx);
  }

  esp_sleep_enable_timer_wakeup(HEARTBEAT_INTERVAL_US);
  esp_sleep_enable_ext0_wakeup(IMU_INT_PIN, IMU_INT_ACTIVE_LEVEL);
  ESP_LOGI(TAG, "Sleeping...");
  esp_deep_sleep_start();
}
