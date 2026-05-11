idf_component_register(SRCS "curtain_controller.c"
                       INCLUDE_DIRS "include"
                       REQUIRES ads1115 hc595_outputs esp_timer)
