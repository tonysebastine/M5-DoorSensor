idf_component_register(SRCS "door_sensor.cpp"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES 
                        esp_wifi 
                        esp_event 
                        nvs_flash 
                        esp_netif 
                        mqtt
                        cjson
                        espp__logger
                        espp__bmi270
                        espp__base_peripheral
                        espp__math
                        espp__format
                        esp_http_client
                        mbedtls
                        lwip
                        esp_adc
                        esp_driver_gpio
                        driver)