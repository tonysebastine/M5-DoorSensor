idf_component_register(SRCS "door_sensor.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES "esp_event" "nvs_flash" "esp_system" "esp_log" "mqtt" "cjson" "esp_http_client" "driver" "mbedtls")

# Additional components for M5Unified functionality if they become ESP-IDF components
# For now, we'll assume a more direct driver approach or that specific M5-related components
# will be added later or integrated into this component directly.
