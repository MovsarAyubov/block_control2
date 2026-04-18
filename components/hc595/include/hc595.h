#pragma once

#include "driver/gpio.h"
#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  gpio_num_t data_gpio_num;   /*!< SER / DS */
  gpio_num_t clock_gpio_num;  /*!< SHCP / SRCLK */
  gpio_num_t latch_gpio_num;  /*!< STCP / RCLK */
  uint8_t initial_state;      /*!< Initial output latch state */
} hc595_config_t;

typedef struct hc595_ctx_t *hc595_handle_t;

esp_err_t hc595_init(const hc595_config_t *config, hc595_handle_t *ret_handle);
esp_err_t hc595_write_byte(hc595_handle_t handle, uint8_t value);
esp_err_t hc595_write_masked(hc595_handle_t handle, uint8_t mask,
                             uint8_t value);
esp_err_t hc595_set_bit(hc595_handle_t handle, uint8_t bit_index, bool level);
uint8_t hc595_get_state(hc595_handle_t handle);
esp_err_t hc595_del(hc595_handle_t handle);

#ifdef __cplusplus
}
#endif
