idf_component_register(SRCS "door_sensor.cpp"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES 
                            esp_wifi 
                            esp_event 
                            nvs_flash 
                            esp_netif 
                            espressif__mqtt
                            json
                            driver
                            esp_adc
                            esp_http_client
                            esp_lcd
                            esp_pm
                            esp_http_server)