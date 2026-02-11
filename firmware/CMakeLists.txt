cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(m5_doorsensor)

# --- Secrets as Compile Definitions ---
# These are passed from GitHub Secrets via CMake cache variables.
# They are then defined as preprocessor macros for the main component.
# WIFI
if(DEFINED CONFIG_WIFI_SSID)
    add_compile_definitions("WIFI_SSID=\"${CONFIG_WIFI_SSID}\"")
endif()
if(DEFINED CONFIG_WIFI_PASSWORD)
    add_compile_definitions("WIFI_PASSWORD=\"${CONFIG_WIFI_PASSWORD}\"")
endif()

# MQTT
if(DEFINED CONFIG_MQTT_HOST)
    add_compile_definitions("MQTT_HOST=\"${CONFIG_MQTT_HOST}\"")
endif()
if(DEFINED CONFIG_MQTT_PORT)
    add_compile_definitions("MQTT_PORT=${CONFIG_MQTT_PORT}")
endif()
if(DEFINED CONFIG_MQTT_USERNAME)
    add_compile_definitions("MQTT_USERNAME=\"${CONFIG_MQTT_USERNAME}\"")
endif()
if(DEFINED CONFIG_MQTT_PASSWORD)
    add_compile_definitions("MQTT_PASSWORD=\"${CONFIG_MQTT_PASSWORD}\"")
endif()
# MQTT_CLIENT_ID is hardcoded in the C++ for now, but could also be a secret.
# target_compile_definitions(${PROJECT_NAME} PRIVATE "MQTT_CLIENT_ID=\"${CONFIG_MQTT_CLIENT_ID}\"")

# Telegram
if(DEFINED CONFIG_TELEGRAM_ENABLED)
    add_compile_definitions("TELEGRAM_ENABLED=${CONFIG_TELEGRAM_ENABLED}")
endif()
if(DEFINED CONFIG_TELEGRAM_BOT_TOKEN)
    add_compile_definitions("TELEGRAM_BOT_TOKEN=\"${CONFIG_TELEGRAM_BOT_TOKEN}\"")
endif()
if(DEFINED CONFIG_TELEGRAM_CHAT_ID)
    add_compile_definitions("TELEGRAM_CHAT_ID=\"${CONFIG_TELEGRAM_CHAT_ID}\"")
endif()

# Static IP
if(DEFINED CONFIG_USE_STATIC_IP)
    add_compile_definitions("USE_STATIC_IP=${CONFIG_USE_STATIC_IP}")
endif()
if(DEFINED CONFIG_STATIC_IP_ADDRESS_OCTETS)
    add_compile_definitions("STATIC_IP_ADDRESS=\"${CONFIG_STATIC_IP_ADDRESS_OCTETS}\"")
endif()
if(DEFINED CONFIG_STATIC_GATEWAY_OCTETS)
    add_compile_definitions("STATIC_GATEWAY=\"${CONFIG_STATIC_GATEWAY_OCTETS}\"")
endif()
if(DEFINED CONFIG_STATIC_SUBNET_OCTETS)
    add_compile_definitions("STATIC_SUBNET=\"${CONFIG_STATIC_SUBNET_OCTETS}\"")
endif()
if(DEFINED CONFIG_STATIC_DNS_OCTETS)
    add_compile_definitions("STATIC_DNS=\"${CONFIG_STATIC_DNS_OCTETS}\"")
endif()