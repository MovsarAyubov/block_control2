idf_component_register(SRCS "modbus_slave.c"
                       INCLUDE_DIRS "include"
                       REQUIRES driver nvs_flash esp_timer)
