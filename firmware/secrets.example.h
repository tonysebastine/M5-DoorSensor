#pragma once

// Wi-Fi
static const char* WIFI_SSID = "YOUR_WIFI_SSID";
static const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";

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

