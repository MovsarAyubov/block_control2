idf_component_register(SRCS "modbus_slave.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_driver_uart nvs_flash esp_timer)
