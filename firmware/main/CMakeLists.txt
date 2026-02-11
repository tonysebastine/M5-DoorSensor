# Added to trigger a new build
idf_component_register(SRCS "door_sensor.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES "esp_http_client" "mbedtls" "esp_wifi" "esp_event" "nvs_flash" "esp_netif" "lwip" "esp_adc" "esp_driver_gpio" "driver")

# Additional components for M5Unified functionality if they become ESP-IDF components
# For now, we'll assume a more direct driver approach or that specific M5-related components
# will be added later or integrated into this component directly.
