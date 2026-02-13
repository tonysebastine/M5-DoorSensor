#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

#include "esp_cpu.h"
#include "esp_event.h"
#include "esp_http_client.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_pm.h"
#include "esp_sleep.h"
#include "esp_sntp.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "mqtt_client.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_types.h"
#include "soc/rtc_cntl_reg.h"

static const char *TAG = "DOORSENSOR";

// ------------------------- BUTTONS & PINS -------------------------
#define BTN_A_PIN GPIO_NUM_37   // Front
#define BTN_B_PIN GPIO_NUM_39   // Side (Calibration)
#define LCD_BLK_PIN GPIO_NUM_27 // Backlight

// ------------------------- MPU6886 Minimal Driver -------------------------
#include <driver/i2c.h>
#define I2C_MASTER_SDA_IO GPIO_NUM_21
#define I2C_MASTER_SCL_IO GPIO_NUM_22
#define I2C_MASTER_NUM I2C_NUM_0

#define MPU6886_ADDR 0x68
#define MPU6886_PWR_MGMT_1 0x6B
#define MPU6886_ACCEL_XOUT_H 0x3B
#define MPU6886_PWR_MGMT_2 0x6C

struct AccelData {
  float x, y, z;
};

// ------------------------- STATE VARIABLES (RTC) -------------------------
constexpr uint64_t POLL_INTERVAL_US =
    200000ULL; // 0.2s Poll for Instant Response

// Stored in RTC Memory (survives Light Sleep)
RTC_DATA_ATTR float rtcRefZ = 0.98f;     // Door Closed Reference
RTC_DATA_ATTR float rtcRefX = 0.0f;      // Orientation Reference X
RTC_DATA_ATTR float rtcRefY = 1.0f;      // Orientation Reference Y
RTC_DATA_ATTR int rtcLastDoorState = -1; // 1: Closed, 0: Open
RTC_DATA_ATTR uint32_t rtcBootCounter = 0;
RTC_DATA_ATTR bool rtcArmed = true;            // Armed by default
RTC_DATA_ATTR time_t rtcCalibrationTime = 0;   // Timestamp of last calibration
RTC_DATA_ATTR bool rtcLowBattReported = false; // Prevent spamming low batt
RTC_DATA_ATTR time_t rtcLastHeartbeat = 0;     // Heartbeat Timer
RTC_DATA_ATTR int rtcWifiLastStatus =
    0; // 0: None, 1: Success, 2: Failed, 3: Connecting
RTC_DATA_ATTR char rtcCurrentSSID[33] = ""; // Buffer for SSID

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
  mpu6886_write_byte(MPU6886_PWR_MGMT_1, 0x01);
  vTaskDelay(pdMS_TO_TICKS(10));
  mpu6886_write_byte(0x1C, 0x10);               // Accel Config 8G
  mpu6886_write_byte(MPU6886_PWR_MGMT_2, 0x07); // Disable Gyro
  return ESP_OK;
}

AccelData mpu6886_get_accel() {
  uint8_t buf[6];
  AccelData data = {0, 0, 0};
  if (mpu6886_read_bytes(MPU6886_ACCEL_XOUT_H, buf, 6) == ESP_OK) {
    int16_t x = (int16_t)((buf[0] << 8) | buf[1]);
    int16_t y = (int16_t)((buf[2] << 8) | buf[3]);
    int16_t z = (int16_t)((buf[4] << 8) | buf[5]);
    data.x = (float)x / 4096.0f;
    data.y = (float)y / 4096.0f;
    data.z = (float)z / 4096.0f;
  }
  return data;
}

// ------------------------- DISPLAY DRIVER (ST7789) -------------------------
esp_lcd_panel_handle_t panel_handle = NULL;

void initDisplay() {
  spi_bus_config_t buscfg = {};
  buscfg.sclk_io_num = GPIO_NUM_13;
  buscfg.mosi_io_num = GPIO_NUM_15;
  buscfg.miso_io_num = -1;
  buscfg.quadwp_io_num = -1;
  buscfg.quadhd_io_num = -1;
  buscfg.max_transfer_sz = 240 * 135 * 2 + 8;
  spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);

  esp_lcd_panel_io_handle_t io_handle = NULL;
  esp_lcd_panel_io_spi_config_t io_config = {};
  io_config.dc_gpio_num = GPIO_NUM_14;
  io_config.cs_gpio_num = GPIO_NUM_5;
  io_config.pclk_hz = 20 * 1000 * 1000;
  io_config.lcd_cmd_bits = 8;
  io_config.lcd_param_bits = 8;
  io_config.spi_mode = 0;
  io_config.trans_queue_depth = 10;
  esp_lcd_new_panel_io_spi(SPI2_HOST, &io_config, &io_handle);

  esp_lcd_panel_dev_config_t panel_config = {};
  panel_config.reset_gpio_num = GPIO_NUM_12;
  panel_config.rgb_endian = LCD_RGB_ENDIAN_BGR;
  panel_config.bits_per_pixel = 16;
  esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle);

  esp_lcd_panel_reset(panel_handle);
  esp_lcd_panel_init(panel_handle);
  esp_lcd_panel_invert_color(panel_handle, true);

  // Set Orientation based on Calibration (rtcRefX/Y)
  // FORCE LANDSCAPE (M5StickC Plus 2)
  esp_lcd_panel_swap_xy(panel_handle, true);
  esp_lcd_panel_mirror(panel_handle, true, false); // Mirror X to fix text
  esp_lcd_panel_set_gap(
      panel_handle, 40,
      50); // Gap 50 - Moved UP (Total shift: -2px from original)

  esp_lcd_panel_disp_on_off(panel_handle, true);

  // Backlight
  gpio_set_direction(LCD_BLK_PIN, GPIO_MODE_OUTPUT); // Backlight
  gpio_set_level(LCD_BLK_PIN, 0);                    // OFF initially
}

void setBacklight(bool on) { gpio_set_level(LCD_BLK_PIN, on ? 1 : 0); }

// Graphics Helpers
void fillRect(int x, int y, int w, int h, uint16_t color) {
  if (!panel_handle)
    return;
  // Clip constraints
  if (x < 0) {
    w += x;
    x = 0;
  }
  if (y < 0) {
    h += y;
    y = 0;
  }
  if (x >= 240 || y >= 135)
    return;
  if (x + w > 240)
    w = 240 - x;
  if (y + h > 135)
    h = 135 - y;
  if (w <= 0 || h <= 0)
    return;

  uint16_t *buffer = (uint16_t *)heap_caps_malloc(w * h * 2, MALLOC_CAP_DMA);
  if (!buffer)
    return;
  for (int i = 0; i < w * h; i++)
    buffer[i] = (color >> 8) | (color << 8);
  esp_lcd_panel_draw_bitmap(panel_handle, x, y, x + w, y + h, buffer);
  free(buffer);
}

// Modern Color Helper
// Minimal 5x7 Bitmap Font (ASCII 32-90) - Only uppercase needed
const uint8_t font5x7[] = {
    0x00, 0x00, 0x00, 0x00, 0x00, // space
    0x00, 0x00, 0x5F, 0x00, 0x00, // !
    0x00, 0x07, 0x00, 0x07, 0x00, // "
    0x14, 0x7F, 0x14, 0x7F, 0x14, // #
    0x24, 0x2A, 0x7F, 0x2A, 0x12, // $
    0x23, 0x13, 0x08, 0x64, 0x62, // %
    0x36, 0x49, 0x55, 0x22, 0x50, // &
    0x00, 0x05, 0x03, 0x00, 0x00, // '
    0x00, 0x1C, 0x22, 0x41, 0x00, // (
    0x00, 0x41, 0x22, 0x1C, 0x00, // )
    0x14, 0x08, 0x3E, 0x08, 0x14, // *
    0x08, 0x08, 0x3E, 0x08, 0x08, // +
    0x00, 0x50, 0x30, 0x00, 0x00, // ,
    0x08, 0x08, 0x08, 0x08, 0x08, // -
    0x00, 0x60, 0x60, 0x00, 0x00, // .
    0x20, 0x10, 0x08, 0x04, 0x02, // /
    0x3E, 0x51, 0x49, 0x45, 0x3E, // 0
    0x00, 0x42, 0x7F, 0x40, 0x00, // 1
    0x42, 0x61, 0x51, 0x49, 0x46, // 2
    0x21, 0x41, 0x45, 0x4B, 0x31, // 3
    0x18, 0x14, 0x12, 0x7F, 0x10, // 4
    0x27, 0x45, 0x45, 0x45, 0x39, // 5
    0x3C, 0x4A, 0x49, 0x49, 0x30, // 6
    0x01, 0x71, 0x09, 0x05, 0x03, // 7
    0x36, 0x49, 0x49, 0x49, 0x36, // 8
    0x06, 0x49, 0x49, 0x29, 0x1E, // 9
    0x00, 0x36, 0x36, 0x00, 0x00, // :
    0x00, 0x56, 0x36, 0x00, 0x00, // ;
    0x08, 0x14, 0x22, 0x41, 0x00, // <
    0x14, 0x14, 0x14, 0x14, 0x14, // =
    0x00, 0x41, 0x22, 0x14, 0x08, // >
    0x02, 0x01, 0x51, 0x09, 0x06, // ?
    0x32, 0x49, 0x79, 0x41, 0x3E, // @
    0x7E, 0x11, 0x11, 0x11, 0x7E, // A
    0x7F, 0x49, 0x49, 0x49, 0x36, // B
    0x3E, 0x41, 0x41, 0x41, 0x22, // C
    0x7F, 0x41, 0x41, 0x22, 0x1C, // D
    0x7F, 0x49, 0x49, 0x49, 0x41, // E
    0x7F, 0x09, 0x09, 0x09, 0x01, // F
    0x3E, 0x41, 0x49, 0x49, 0x7A, // G
    0x7F, 0x08, 0x08, 0x08, 0x7F, // H
    0x00, 0x41, 0x7F, 0x41, 0x00, // I
    0x20, 0x40, 0x41, 0x3F, 0x01, // J
    0x7F, 0x08, 0x14, 0x22, 0x41, // K
    0x7F, 0x40, 0x40, 0x40, 0x40, // L
    0x7F, 0x02, 0x0C, 0x02, 0x7F, // M
    0x7F, 0x04, 0x08, 0x10, 0x7F, // N
    0x3E, 0x41, 0x41, 0x41, 0x3E, // O
    0x7F, 0x09, 0x09, 0x09, 0x06, // P
    0x3E, 0x41, 0x51, 0x21, 0x5E, // Q
    0x7F, 0x09, 0x19, 0x29, 0x46, // R
    0x46, 0x49, 0x49, 0x49, 0x31, // S
    0x01, 0x01, 0x7F, 0x01, 0x01, // T
    0x3F, 0x40, 0x40, 0x40, 0x3F, // U
    0x1F, 0x20, 0x40, 0x20, 0x1F, // V
    0x3F, 0x40, 0x38, 0x40, 0x3F, // W
    0x63, 0x14, 0x08, 0x14, 0x63, // X
    0x07, 0x08, 0x70, 0x08, 0x07, // Y
    0x61, 0x51, 0x49, 0x45, 0x43  // Z
};

void drawChar(int x, int y, char c, uint16_t color, int size) {
  if (c < 32 || c > 90)
    c = 32;
  const uint8_t *glyph = &font5x7[(c - 32) * 5];
  for (int i = 0; i < 5; i++) {
    uint8_t line = glyph[i];
    for (int j = 0; j < 7; j++) {
      if (line & 1)
        fillRect(x + i * size, y + j * size, size, size, color);
      line >>= 1;
    }
  }
}

void drawString(int x, int y, const char *str, uint16_t color, int size) {
  while (*str) {
    drawChar(x, y, *str++, color, size);
    x += 6 * size;
  }
}

void drawUI(bool closed, uint32_t battMv, uint32_t ticks) {
  // 1. Premium Dark Background
  uint16_t bgColor = 0x0841; // Deep Deep Blue/Grey
  fillRect(0, 0, 240, 135, bgColor);

  // Corner Highlights (Cyber-Deck aesthetic)
  uint16_t accent = 0x3186; // Muted blue accent
  fillRect(0, 0, 15, 2, accent);
  fillRect(0, 0, 2, 15, accent); // Top-Left
  fillRect(225, 0, 15, 2, accent);
  fillRect(238, 0, 2, 15, accent); // Top-Right
  fillRect(0, 133, 15, 2, accent);
  fillRect(0, 120, 2, 15, accent); // Bot-Left
  fillRect(225, 133, 15, 2, accent);
  fillRect(238, 120, 2, 15, accent); // Bot-Right

  // 2. Main Hero Block (Status)
  const char *status = closed ? "CLOSED" : "OPEN";
  uint16_t statusColor = closed ? 0x03E0 : 0x8800; // Muted green/red background
  uint16_t textColor =
      closed ? 0x07E0 : 0xF840; // Bright Emerald / Sunset Orange

  if (!rtcArmed) {
    statusColor = 0x2104; // Dark Steel
    textColor = 0x8410;   // Slate
  }

  // Breathing Pulse (Faster/Larger when OPEN)
  int pulseSpeed = closed ? 12 : 6;
  int pulseMax = closed ? 5 : 10;
  int pulse = (ticks / pulseSpeed) % pulseMax;

  int blockHeight = 62 + pulse;
  int blockY = (70 - blockHeight / 2);

  // Glow Border for Hero
  fillRect(38, blockY - 2, 164, blockHeight + 4, 0x0000); // Black shadow
  fillRect(40, blockY, 160, blockHeight, statusColor);

  // Status Text (Perfectly Centered)
  int len = strlen(status);
  int textWidth = len * 6 * 5;
  int textHeight = 7 * 5; // Scale 5
  int textX = (240 - textWidth) / 2;
  int textY = blockY + (blockHeight - textHeight) / 2;
  drawString(textX, textY, status, textColor, 5);

  // 3. System Badge (Header)
  const char *systemTag = rtcArmed ? "[ SYSTEM ACTIVE ]" : "[ SYSTEM STANDBY ]";
  uint16_t tagColor = rtcArmed ? 0x07FF : 0x8410;
  int tagWidth = strlen(systemTag) * 6;
  drawString((240 - tagWidth) / 2, 10, systemTag, tagColor, 1);

  // 4. Integrated Dashboard (Footer)
  fillRect(0, 115, 240, 20, 0x0000); // Darker tray
  fillRect(0, 114, 240, 1, accent);  // Top Divider

  // Segments: [ BATTERY ] | [ SSID ] | [ WIFI STATUS ]

  // Battery
  char battStr[16];
  snprintf(battStr, sizeof(battStr), "%d.%dV", (int)battMv / 1000,
           (int)(battMv % 1000) / 100);
  drawString(8, 121, battStr, 0xFFFF, 1);
  fillRect(45, 118, 1, 14, 0x2104); // Vertical Divider 1

  // SSID (Dynamic positioning)
  if (strlen(rtcCurrentSSID) > 0) {
    int sLen = (int)strlen(rtcCurrentSSID);
    if (sLen > 18)
      sLen = 18;
    char ssidDisp[20];
    strncpy(ssidDisp, rtcCurrentSSID, sLen);
    ssidDisp[sLen] = 0;
    int sw = sLen * 6;
    drawString((240 - sw) / 2, 121, ssidDisp, 0x8410, 1);
  }
  fillRect(190, 118, 1, 14, 0x2104); // Vertical Divider 2

  // WiFi Status Link
  if (rtcWifiLastStatus == 1)
    drawString(196, 121, "ONLINE", 0x07E0, 1);
  else if (rtcWifiLastStatus == 2)
    drawString(196, 121, "FAILED", 0xF800, 1);
  else
    drawString(196, 121, "WAIT..", 0xFFE0, 1);
}

void drawBootScreen() {
  fillRect(0, 0, 240, 135, 0x0000); // Clear

  // Draw "M" (White) - Shifted +5px to Center
  uint16_t w = 0xFFFF;
  fillRect(70, 40, 10, 40, w); // Left Leg
  fillRect(80, 40, 10, 10, w);
  fillRect(90, 50, 10, 10, w); // Middle
  fillRect(100, 40, 10, 10, w);
  fillRect(110, 40, 10, 40, w); // Right Leg

  // Draw "5" (Red) - Shifted +5px to Center
  uint16_t r = 0xF800;
  fillRect(140, 40, 30, 8, r); // Top
  fillRect(140, 48, 8, 12, r); // Down 1
  fillRect(140, 60, 30, 8, r); // Mid
  fillRect(162, 68, 8, 12, r); // Down 2
  fillRect(140, 80, 30, 8, r); // Bot

  // Loading Bar
  fillRect(50, 105, 140, 6, 0x2104); // BG
  for (int i = 0; i <= 140; i += 5) {
    fillRect(50, 105, i, 6, 0x07E0); // Fill Green
    vTaskDelay(pdMS_TO_TICKS(15));
  }
  vTaskDelay(pdMS_TO_TICKS(200));
}

void fillScreen(uint16_t color) { fillRect(0, 0, 240, 135, color); }

// ------------------------- Battery ADC -------------------------
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#define BATTERY_ADC_CHANNEL ADC_CHANNEL_2
static adc_oneshot_unit_handle_t adc1_handle = NULL;
static adc_cali_handle_t adc1_cali_handle = NULL;

void initADC() {
  if (adc1_handle)
    return;
  adc_oneshot_unit_init_cfg_t init_config1 = {};
  init_config1.unit_id = ADC_UNIT_1;
  adc_oneshot_new_unit(&init_config1, &adc1_handle);
  adc_oneshot_chan_cfg_t config = {};
  config.bitwidth = ADC_BITWIDTH_12;
  config.atten = ADC_ATTEN_DB_12;
  adc_oneshot_config_channel(adc1_handle, BATTERY_ADC_CHANNEL, &config);
  adc_cali_line_fitting_config_t cali_config = {};
  cali_config.unit_id = ADC_UNIT_1;
  cali_config.atten = ADC_ATTEN_DB_12;
  cali_config.bitwidth = ADC_BITWIDTH_12;
  adc_cali_create_scheme_line_fitting(&cali_config, &adc1_cali_handle);
}

uint32_t getBatteryVoltageMv() {
  if (!adc1_handle)
    initADC();
  int raw = 0;
  for (int i = 0; i < 4; i++) {
    int val;
    adc_oneshot_read(adc1_handle, BATTERY_ADC_CHANNEL, &val);
    raw += val;
  }
  raw /= 4;
  int voltage = 0;
  if (adc1_cali_handle)
    adc_cali_raw_to_voltage(adc1_cali_handle, raw, &voltage);
  else
    voltage = raw * 3300 / 4095;
  return (uint32_t)(voltage * 2.0);
}

// ------------------------- WiFi & Logic -------------------------
static EventGroupHandle_t s_wifi_event_group;
static esp_netif_t *sta_netif = NULL; // Netif handle
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

struct StaticIP {
  esp_ip4_addr_t ip;
  esp_ip4_addr_t gw;
  esp_ip4_addr_t netmask;
  uint8_t bssid[6];
  uint8_t channel;
  bool valid;
};

// ------------------------- WiFi helpers -------------------------
esp_err_t load_static_ip_info(StaticIP *info) {
  nvs_handle_t nvs;
  if (nvs_open("storage", NVS_READONLY, &nvs) != ESP_OK)
    return ESP_FAIL;
  size_t size = sizeof(StaticIP);
  esp_err_t err = nvs_get_blob(nvs, "static_ip_blob", info, &size);
  nvs_close(nvs);
  return err;
}

esp_err_t save_static_ip_info(StaticIP *info) {
  nvs_handle_t nvs;
  if (nvs_open("storage", NVS_READWRITE, &nvs) != ESP_OK)
    return ESP_FAIL;
  nvs_set_blob(nvs, "static_ip_blob", info, sizeof(StaticIP));
  nvs_commit(nvs);
  nvs_close(nvs);
  return ESP_OK;
}

esp_err_t clear_static_ip() {
  nvs_handle_t nvs;
  if (nvs_open("storage", NVS_READWRITE, &nvs) != ESP_OK)
    return ESP_FAIL;
  nvs_erase_key(nvs, "static_ip_blob");
  nvs_commit(nvs);
  nvs_close(nvs);
  return ESP_OK;
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data) {
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
    ESP_LOGI(TAG, "WiFi Started. Connecting...");
    rtcWifiLastStatus = 3;
    esp_wifi_connect();
  } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED) {
    ESP_LOGI(TAG, "WiFi Connected to AP. Waiting for IP...");
    rtcWifiLastStatus = 4;
  } else if (event_base == WIFI_EVENT &&
             event_id == WIFI_EVENT_STA_DISCONNECTED) {
    wifi_event_sta_disconnected_t *disconn =
        (wifi_event_sta_disconnected_t *)event_data;
    ESP_LOGW(TAG, "WiFi Disconnected. Reason: %d", disconn->reason);
    rtcWifiLastStatus = 2;

    // Aggressively clear learned IP/BSSID on failure to ensure next boot
    // performs full scan/DHCP 201=NO_AP_FOUND, 202=AUTH_FAIL,
    // 15=4WAY_HANDSHAKE_TIMEOUT, 204=HANDSHAKE_TIMEOUT
    if (disconn->reason == 201 || disconn->reason == 202 ||
        disconn->reason == 15 || disconn->reason == 204 ||
        disconn->reason == WIFI_REASON_CONNECTION_FAIL) {
      ESP_LOGE(TAG,
               "Connection Failed (Reason %d). Clearing Static IP cache to "
               "force fresh DHCP.",
               disconn->reason);
      clear_static_ip();
    }

    xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
  } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
    rtcWifiLastStatus = 1;

    // Capture Static IP & BSSID - Always overwrite to ensure we have the latest
    // correct info This allows the router to change our IP via DHCP reservation
    // and we will respect it on next boot
    wifi_ap_record_t ap_info;
    esp_wifi_sta_get_ap_info(&ap_info);

    StaticIP current = {};
    current.ip = event->ip_info.ip;
    current.gw = event->ip_info.gw;
    current.netmask = event->ip_info.netmask;
    memcpy(current.bssid, ap_info.bssid, 6);
    current.channel = ap_info.primary;
    current.valid = true;

    save_static_ip_info(&current);
    ESP_LOGI(TAG,
             "Learned Fast-Connect Details Updated (IP: " IPSTR ", Ch: %d).",
             IP2STR(&current.ip), current.channel);

    xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
  }
}

// ------------------------- NVS CREDENTIALS -------------------------
esp_err_t save_wifi_credentials(const char *ssid, const char *pass) {
  nvs_handle_t nvs;
  if (nvs_open("storage", NVS_READWRITE, &nvs) != ESP_OK)
    return ESP_FAIL;
  nvs_set_str(nvs, "wifi_ssid", ssid);
  nvs_set_str(nvs, "wifi_pass", pass);
  nvs_commit(nvs);
  nvs_close(nvs);
  return ESP_OK;
}

esp_err_t load_wifi_credentials(char *ssid, char *pass) {
  nvs_handle_t nvs;
  if (nvs_open("storage", NVS_READONLY, &nvs) != ESP_OK)
    return ESP_FAIL;
  size_t s_len = 32, p_len = 64;
  nvs_get_str(nvs, "wifi_ssid", ssid, &s_len);
  nvs_get_str(nvs, "wifi_pass", pass, &p_len);
  nvs_close(nvs);
  return (strlen(ssid) > 0) ? ESP_OK : ESP_FAIL;
}

// ------------------------- WiFi CONFIG PORTAL -------------------------
static httpd_handle_t server = NULL;

static esp_err_t config_get_handler(httpd_req_t *req) {
  const char *resp =
      "<!DOCTYPE html><html><head>"
      "<meta name='viewport' content='width=device-width, initial-scale=1'>"
      "<title>M5-DoorSensor Config</title>"
      "<style>"
      "body { font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; "
      "background: linear-gradient(135deg, #1a1a2e 0%, #16213e 100%); "
      "color: #fff; display: flex; justify-content: center; align-items: "
      "center; height: 100vh; margin: 0; }"
      ".card { background: rgba(255, 255, 255, 0.05); backdrop-filter: "
      "blur(10px); padding: 40px; border-radius: 20px; box-shadow: 0 8px 32px "
      "0 rgba(0, 0, 0, 0.37); border: 1px solid rgba(255, 255, 255, 0.1); "
      "width: 320px; text-align: center; }"
      "h1 { margin-bottom: 30px; font-weight: 300; letter-spacing: 2px; "
      "color: #00d2ff; }"
      "input { width: calc(100% - 24px); padding: 12px; margin: 12px 0; "
      "border: none; border-radius: 8px; background: rgba(255, 255, 255, "
      "0.1); color: #fff; outline: none; transition: 0.3s; }"
      "input:focus { background: rgba(255, 255, 255, 0.2); box-shadow: 0 0 "
      "8px rgba(0, 210, 255, 0.5); }"
      "input[type='submit'] { width: 100%; background: #00d2ff; color: "
      "#1a1a2e; "
      "font-weight: bold; cursor: pointer; margin-top: 20px; transition: 0.3s; "
      "border: none; }"
      "input[type='submit']:hover { background: #0095b3; transform: "
      "translateY(-2px); }"
      "p { font-size: 0.8em; color: rgba(255, 255, 255, 0.5); margin-top: "
      "20px; }"
      "</style></head><body>"
      "<div class='card'>"
      "<h1>SENSITIVE CFG</h1>"
      "<form action='/save' method='POST'>"
      "WiFi SSID<input type='text' name='s' placeholder='Network name' "
      "required>"
      "Password<input type='password' name='p' placeholder='WiFi password'>"
      "<input type='submit' value='CONNECT DEVICE'>"
      "</form>"
      "<p>Device will restart after saving</p>"
      "</div>"
      "</body></html>";
  httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

static esp_err_t save_post_handler(httpd_req_t *req) {
  char buf[128] = {0};
  int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
  if (ret <= 0)
    return ESP_FAIL;

  char ssid[32] = {0}, pass[64] = {0};
  // Simplistic parsing of s=xxx&p=yyy
  char *s_ptr = strstr(buf, "s=");
  char *p_ptr = strstr(buf, "&p=");
  if (s_ptr && p_ptr) {
    *p_ptr = '\0';
    strncpy(ssid, s_ptr + 2, 31);
    strncpy(pass, p_ptr + 3, 63);
    // Decode basic url encoding (space only for simplicity)
    for (int i = 0; ssid[i]; i++)
      if (ssid[i] == '+')
        ssid[i] = ' ';
    for (int i = 0; pass[i]; i++)
      if (pass[i] == '+')
        pass[i] = ' ';

    save_wifi_credentials(ssid, pass);
    clear_static_ip(); // Clear static IP to trigger re-learn on new network
    httpd_resp_send(req, "Saved! Device will restart.", HTTPD_RESP_USE_STRLEN);
    vTaskDelay(pdMS_TO_TICKS(2000));
    esp_restart();
  } else {
    httpd_resp_send(req, "Error: Invalid Data", HTTPD_RESP_USE_STRLEN);
  }
  return ESP_OK;
}

void startConfigPortal() {
  ESP_LOGI(TAG, "Starting Config Portal...");
  // Unregister existing handlers using the global event handler unregister
  esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID,
                               &wifi_event_handler);
  esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP,
                               &wifi_event_handler);
  esp_wifi_stop();

  wifi_config_t ap_config = {};
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  snprintf((char *)ap_config.ap.ssid, 32, "M5-DoorSensor-%02X%02X", mac[4],
           mac[5]);
  ap_config.ap.channel = 1;
  ap_config.ap.max_connection = 4;
  ap_config.ap.authmode = WIFI_AUTH_WPA2_PSK;
  strncpy((char *)ap_config.ap.password, "12345678", 64);

  esp_wifi_set_mode(WIFI_MODE_AP);
  esp_wifi_set_config(WIFI_IF_AP, &ap_config);
  esp_wifi_start();

  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  if (httpd_start(&server, &config) == ESP_OK) {
    httpd_uri_t uri_get = {.uri = "/",
                           .method = HTTP_GET,
                           .handler = config_get_handler,
                           .user_ctx = NULL};
    httpd_register_uri_handler(server, &uri_get);
    httpd_uri_t uri_post = {.uri = "/save",
                            .method = HTTP_POST,
                            .handler = save_post_handler,
                            .user_ctx = NULL};
    httpd_register_uri_handler(server, &uri_post);
  }
}

void initWifiSystem() {
  s_wifi_event_group = xEventGroupCreate();
  esp_netif_init();
  esp_event_loop_create_default();
  sta_netif = esp_netif_create_default_wifi_sta();
  esp_netif_create_default_wifi_ap(); // Added for config portal
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  esp_wifi_init(&cfg);
  esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                      &wifi_event_handler, NULL, NULL);
  esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                      &wifi_event_handler, NULL, NULL);

  // Apply Static IP if valid
  StaticIP s_ip = {};
  bool has_static = (load_static_ip_info(&s_ip) == ESP_OK && s_ip.valid);
  if (has_static) {
    esp_netif_dhcpc_stop(sta_netif);
    esp_netif_ip_info_t ip_info;
    ip_info.ip = s_ip.ip;
    ip_info.gw = s_ip.gw;
    ip_info.netmask = s_ip.netmask;
    esp_netif_set_ip_info(sta_netif, &ip_info);
    ESP_LOGI(TAG, "Static IP Applied: " IPSTR, IP2STR(&s_ip.ip));
  }

  // Load Credentials
  wifi_config_t wifi_config = {};
  char saved_ssid[32] = {0}, saved_pass[64] = {0};
  if (load_wifi_credentials(saved_ssid, saved_pass) == ESP_OK) {
    strncpy((char *)wifi_config.sta.ssid, saved_ssid, 32);
    strncpy((char *)wifi_config.sta.password, saved_pass, 64);
    strncpy(rtcCurrentSSID, saved_ssid, 32); // Store for UI
    ESP_LOGI(TAG, "Loaded WiFi from NVS: %s", saved_ssid);
  } else {
    strncpy((char *)wifi_config.sta.ssid, CONFIG_WIFI_SSID, 32);
    strncpy((char *)wifi_config.sta.password, CONFIG_WIFI_PASSWORD, 64);
    strncpy(rtcCurrentSSID, CONFIG_WIFI_SSID, 32); // Store for UI
    ESP_LOGI(TAG, "Using Default WiFi from Kconfig");
  }

  // Fast Connect Config (Pinning BSSID/Channel)
  if (has_static) {
    wifi_config.sta.bssid_set = true;
    memcpy(wifi_config.sta.bssid, s_ip.bssid, 6);
    wifi_config.sta.channel = s_ip.channel;
    wifi_config.sta.scan_method = WIFI_FAST_SCAN;
    ESP_LOGI(TAG, "Fast-Connect Configured (BSSID Locked)");
  }

  esp_wifi_set_mode(WIFI_MODE_STA);
  esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
  esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
}

// ------------------------- ALERTING SYSTEM -------------------------
void syncClock() {
  esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
  esp_sntp_setservername(0, "pool.ntp.org");
  esp_sntp_init();
  int retry = 0;
  while (esp_sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++retry < 10)
    vTaskDelay(pdMS_TO_TICKS(500));
}

void sendAlert(const char *eventStr) {
  ESP_LOGI(TAG, "Attempting to send Alert: %s (Armed=%d)", eventStr, rtcArmed);

  // If Disarmed and NOT a Low Battery alert, skip.
  if (!rtcArmed && strcmp(eventStr, "BATTERY_LOW") != 0) {
    ESP_LOGI(TAG, "Event Skipped (Disarmed)");
    return;
  }

  rtcWifiLastStatus = 3; // Connecting... status
  xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT);
  esp_wifi_start();
  esp_wifi_set_ps(WIFI_PS_NONE); // Force high performance

  if (xEventGroupWaitBits(s_wifi_event_group,
                          WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE,
                          pdMS_TO_TICKS(12000)) & // Increased to 12s
      WIFI_CONNECTED_BIT) {
    rtcWifiLastStatus = 1; // Success
    // Removed syncClock() here to save several seconds of latency

    // MQTT
    esp_mqtt_client_config_t mqtt_cfg = {};
    mqtt_cfg.broker.address.uri = "mqtt://" CONFIG_MQTT_HOST;
    mqtt_cfg.broker.address.port = CONFIG_MQTT_PORT;
    mqtt_cfg.credentials.username = CONFIG_MQTT_USERNAME;
    mqtt_cfg.credentials.authentication.password = CONFIG_MQTT_PASSWORD;
    mqtt_cfg.session.keepalive = 10;

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_start(client);

    // Instead of waiting, we use a small polling check or just Fire & Forget
    // In low-latency scenarios, we publish directly if connection is fast
    vTaskDelay(pdMS_TO_TICKS(500)); // Reduced from 1500ms

    // Format Time (RTC)
    char timeStr[32] = "N/A";
    if (rtcCalibrationTime > 0) {
      struct tm timeinfo;
      localtime_r(&rtcCalibrationTime, &timeinfo);
      strftime(timeStr, sizeof(timeStr), "%Y-%m-%d %H:%M:%S", &timeinfo);
    }

    char payload[200];
    snprintf(payload, sizeof(payload),
             "{\"event\": \"%s\", \"battery\": %ld, \"calibrated_at\": \"%s\", "
             "\"boot_count\": %lu}",
             eventStr, getBatteryVoltageMv(), timeStr, rtcBootCounter);

    esp_mqtt_client_publish(client, "m5door/tony/state", payload, 0, 1, 0);

// Telegram
#if CONFIG_TELEGRAM_ENABLED
    char url[256];
    char msg[128];
    snprintf(url, sizeof(url), "https://api.telegram.org/bot%s/sendMessage",
             CONFIG_TELEGRAM_BOT_TOKEN);
    snprintf(msg, sizeof(msg), "Alert: %s\nBatt: %ldmV\nCalibrated: %s",
             eventStr, getBatteryVoltageMv(), timeStr);

    // JSON Escape (Simple) - in production use cJSON
    char post_data[512];
    snprintf(post_data, sizeof(post_data),
             "{\"chat_id\": \"%s\", \"text\": \"%s\"}", CONFIG_TELEGRAM_CHAT_ID,
             msg);

    esp_http_client_config_t http_cfg = {};
    http_cfg.url = url;
    http_cfg.method = HTTP_METHOD_POST;
    esp_http_client_handle_t hc = esp_http_client_init(&http_cfg);
    esp_http_client_set_header(hc, "Content-Type", "application/json");
    esp_http_client_set_post_field(hc, post_data, strlen(post_data));
    esp_http_client_perform(hc);
    esp_http_client_cleanup(hc);
#endif

    vTaskDelay(pdMS_TO_TICKS(500));
    esp_mqtt_client_destroy(client);
  } else {
    rtcWifiLastStatus = 2; // Failed
  }
  esp_wifi_stop();
}

// ------------------------- DISPLAY HELPERS -------------------------
void wakeDisplay() {
  if (panel_handle) {
    esp_lcd_panel_disp_on_off(panel_handle, true);
    vTaskDelay(pdMS_TO_TICKS(10)); // Allow TCON to wake
    setBacklight(true);
  }
}

void sleepDisplay() {
  if (panel_handle) {
    setBacklight(false);
    vTaskDelay(pdMS_TO_TICKS(10));
    esp_lcd_panel_disp_on_off(panel_handle, false); // ST7789 Sleep
  }
}

// [Duplicate RTC Vars Removed]
extern "C" void app_main() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  gpio_reset_pin(GPIO_NUM_4);
  gpio_set_direction(GPIO_NUM_4, GPIO_MODE_OUTPUT);
  gpio_set_level(GPIO_NUM_4, 1);
  gpio_hold_en(GPIO_NUM_4);

  // Initialize NVS (Critical for WiFi)
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    nvs_flash_erase();
    nvs_flash_init();
  }

  // Buttons Config
  // Note: GPIO 37/39 are Input-Only and have NO internal pullups/downs.
  // M5StickC Plus 2 has external 10k pullups.
  gpio_config_t btn_conf = {};
  btn_conf.pin_bit_mask = (1ULL << BTN_A_PIN) | (1ULL << BTN_B_PIN);
  btn_conf.mode = GPIO_MODE_INPUT;
  btn_conf.pull_up_en = GPIO_PULLUP_DISABLE; // External Pullups used
  btn_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  gpio_config(&btn_conf);

  initADC();
  initWifiSystem();

  i2c_config_t conf = {};
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = I2C_MASTER_SDA_IO;
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf.scl_io_num = I2C_MASTER_SCL_IO;
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf.master.clk_speed = 400000;
  i2c_param_config(I2C_MASTER_NUM, &conf);
  i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
  initMPU6886();

  initDisplay();
  // Fix Offset for M5StickC Plus 2 (135x240) handled in initDisplay

  // BOOT SCREEN
  esp_lcd_panel_disp_on_off(panel_handle, true);
  setBacklight(true);
  drawBootScreen();

  sleepDisplay();

  ESP_LOGI(TAG, "System Start. RefZ=%.2f", rtcRefZ);

  // Boot Check
  AccelData acc = mpu6886_get_accel();
  int initial_state = (fabs((double)acc.z - (double)rtcRefZ) < 0.2f) ? 1 : 0;
  rtcLastDoorState = initial_state;

  // Power & Sleep Config
  esp_pm_config_t pm_config = {
      .max_freq_mhz = 80, .min_freq_mhz = 80, .light_sleep_enable = true};
  esp_pm_configure(&pm_config);

  gpio_wakeup_enable(BTN_A_PIN, GPIO_INTR_LOW_LEVEL);
  esp_sleep_enable_gpio_wakeup();

  // Update Heartbeat on Boot if 0
  time_t now;
  time(&now);
  if (rtcLastHeartbeat == 0)
    rtcLastHeartbeat = now;

  while (1) {
    esp_sleep_enable_timer_wakeup(POLL_INTERVAL_US);
    esp_sleep_enable_gpio_wakeup();
    esp_light_sleep_start();

    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();

    if (cause == ESP_SLEEP_WAKEUP_TIMER) {
      // 1. Check Door
      acc = mpu6886_get_accel();
      bool closed =
          (fabs((double)acc.z - (double)rtcRefZ) < 0.2f); // Cast for safety
      int state = closed ? 1 : 0;
      if (state != rtcLastDoorState) {
        rtcLastDoorState = state;
        sendAlert(closed ? "CLOSED" : "OPEN");
      }

      // 2. Check Battery
      uint32_t batt = getBatteryVoltageMv();
      if (batt < 3550 && !rtcLowBattReported) {
        sendAlert("BATTERY_LOW");
        rtcLowBattReported = true;
      } else if (batt > 3650) {
        rtcLowBattReported = false;
      }

      // 3. Check Heartbeat (Every 24h = 86400s)
      // We use simple counter estimation if time() isn't reliable in light
      // sleep without SNTP But ESP32 RTC maintains time.
      time(&now);
      if (difftime(now, rtcLastHeartbeat) > 86400) {
        sendAlert("HEARTBEAT");
        rtcLastHeartbeat = now;
      }
    } else if (cause == ESP_SLEEP_WAKEUP_GPIO) {
      wakeDisplay();
      while (gpio_get_level(BTN_A_PIN) == 0)
        vTaskDelay(pdMS_TO_TICKS(10));
      vTaskDelay(pdMS_TO_TICKS(100));

      uint32_t last_interaction = xTaskGetTickCount();
      bool display_active = true;

      while (display_active) {
        if (gpio_get_level(BTN_A_PIN) == 0) {
          uint32_t press_start = xTaskGetTickCount();
          while (gpio_get_level(BTN_A_PIN) == 0)
            vTaskDelay(10);
          if ((xTaskGetTickCount() - press_start) > pdMS_TO_TICKS(1000)) {
            rtcArmed = !rtcArmed;
            fillScreen(0xFFFF);
            vTaskDelay(pdMS_TO_TICKS(200));
          } else {
            display_active = false;
            break;
          }
        }
        if (gpio_get_level(BTN_B_PIN) == 0) {
          last_interaction = xTaskGetTickCount();
          uint32_t b_press_start = xTaskGetTickCount();
          bool long_long_press = false;
          while (gpio_get_level(BTN_B_PIN) == 0) {
            if ((xTaskGetTickCount() - b_press_start) > pdMS_TO_TICKS(5000)) {
              long_long_press = true;
              break;
            }
            vTaskDelay(pdMS_TO_TICKS(50));
          }

          if (long_long_press) {
            // Enter WiFi Config Mode
            fillRect(0, 0, 240, 135, 0x001F); // Blue Screen
            drawString(20, 20, "WIFI CONFIG MODE", 0xFFFF, 2);
            drawString(20, 50, "Connect to AP:", 0xFFFF, 1);

            uint8_t mac[6];
            esp_read_mac(mac, ESP_MAC_WIFI_STA);
            char ap_name[32];
            snprintf(ap_name, 32, "M5-DoorSensor-%02X%02X", mac[4], mac[5]);
            drawString(20, 70, ap_name, 0x07FF, 2);
            drawString(20, 100, "IP: 192.168.4.1", 0xFFFF, 2);

            startConfigPortal();
            // Loop forever in config mode until restart
            while (1)
              vTaskDelay(pdMS_TO_TICKS(1000));
          } else if ((xTaskGetTickCount() - b_press_start) >
                     pdMS_TO_TICKS(1500)) {
            AccelData c = mpu6886_get_accel();
            rtcRefZ = c.z;
            rtcRefX = c.x;
            rtcRefY = c.y;
            time(&rtcCalibrationTime); // Update Timestamp
            fillScreen(0xFFFF);
            vTaskDelay(pdMS_TO_TICKS(500));
          }
        }

        bool closed = (fabs(mpu6886_get_accel().z - rtcRefZ) < 0.2f);

        // Optimize: Update UI only if state changed OR 1s passed
        static int last_ui_state = -1;
        static uint32_t last_ui_time = 0;
        uint32_t now = xTaskGetTickCount();
        int current_state = (closed ? 1 : 0) | (rtcArmed ? 2 : 0);

        if (current_state != last_ui_state ||
            (now - last_ui_time) > pdMS_TO_TICKS(1000)) {
          drawUI(closed, getBatteryVoltageMv(), now);
          last_ui_state = current_state;
          last_ui_time = now;
        }

        if ((xTaskGetTickCount() - last_interaction) > pdMS_TO_TICKS(30000))
          display_active = false;
        vTaskDelay(pdMS_TO_TICKS(100)); // Poll buttons faster (100ms)
      }
      sleepDisplay();
      while (gpio_get_level(BTN_A_PIN) == 0)
        vTaskDelay(10);
    }
  }
}
