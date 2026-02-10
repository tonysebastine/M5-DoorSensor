#pragma once

// Wi-Fi
static const char* WIFI_SSID = "w3infotech";
static const char* WIFI_PASSWORD = "w3@infoch#345";

// MQTT
static const char* MQTT_HOST = "192.168.1.100";
static const uint16_t MQTT_PORT = 1883;
static const char* MQTT_USERNAME = "";
static const char* MQTT_PASSWORD = "";
static const char* MQTT_CLIENT_ID = "m5-door-sensor";

// Optional direct Telegram alerts from device (can be disabled)
static const bool TELEGRAM_ENABLED = false;
static const char* TELEGRAM_BOT_TOKEN = "";
static const char* TELEGRAM_CHAT_ID = "";

// Static IP (optional)
static const bool USE_STATIC_IP = true;
static const IPAddress STATIC_IP_ADDRESS(192, 168, 1, 100);
static const IPAddress STATIC_GATEWAY(192, 168, 1, 1);
static const IPAddress STATIC_SUBNET(255, 255, 255, 0);
static const IPAddress STATIC_DNS(8, 8, 8, 8); // Google DNS

