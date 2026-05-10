#pragma once

#include "driver/gpio.h"
#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  gpio_num_t data_gpio_num;
  gpio_num_t clock_gpio_num;
  gpio_num_t latch_gpio_num;
  uint8_t chip_count;
  uint8_t relay1_bit_index;
  uint8_t relay2_bit_index;
  uint8_t valve_open_bit_index;
  uint8_t valve_close_bit_index;
  uint32_t initial_state;
} hc595_outputs_config_t;

typedef struct hc595_outputs_ctx_t *hc595_outputs_handle_t;

typedef enum {
  HC595_OUTPUTS_VALVE_STOPPED = 0,
  HC595_OUTPUTS_VALVE_OPENING,
  HC595_OUTPUTS_VALVE_CLOSING,
} hc595_outputs_valve_state_t;

esp_err_t hc595_outputs_init(const hc595_outputs_config_t *config,
                             hc595_outputs_handle_t *ret_handle);
esp_err_t hc595_outputs_set_light_relays(hc595_outputs_handle_t handle,
                                         bool relay1_on, bool relay2_on);
esp_err_t hc595_outputs_set_valve_state(
    hc595_outputs_handle_t handle, hc595_outputs_valve_state_t valve_state);
uint8_t hc595_outputs_get_raw_state(hc595_outputs_handle_t handle);
uint32_t hc595_outputs_get_state(hc595_outputs_handle_t handle);
esp_err_t hc595_outputs_write_masked(hc595_outputs_handle_t handle,
                                     uint32_t mask, uint32_t value);
esp_err_t hc595_outputs_del(hc595_outputs_handle_t handle);

#ifdef __cplusplus
}
#endif
