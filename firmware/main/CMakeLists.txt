idf_component_register(SRCS "door_sensor.cpp"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES 
                            esp_wifi 
                            esp_event 
                            nvs_flash 
                            esp_netif 
                            espressif__mqtt
                            cjson
                            espp__logger
                            espp__bmi270
                            espp__base_peripheral
                            espp__math
                            esp_http_client
                            espp__format)