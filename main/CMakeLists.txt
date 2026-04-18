idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS ""
                       REQUIRES rh_sensor hc595_outputs max31865 valve_3way modbus_slave
                                ds3231 nvs_flash bt_ascii_control)
