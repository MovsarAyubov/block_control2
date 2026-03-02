idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS ""
                       REQUIRES rh_sensor rll400 max31865 valve_3way modbus_slave ds3231 nvs_flash)
