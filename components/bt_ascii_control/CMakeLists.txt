idf_component_register(SRCS "bt_ascii_control.c"
                       INCLUDE_DIRS "include"
                       REQUIRES bt nvs_flash modbus_slave)
