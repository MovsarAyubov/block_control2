#include "hc595_outputs.h"

#include "hc595.h"
#include <stdlib.h>

struct hc595_outputs_ctx_t {
  hc595_outputs_config_t cfg;
  hc595_handle_t shift_reg;
  uint8_t relay_mask;
  uint8_t valve_mask;
};

static bool hc595_outputs_bit_valid(uint8_t bit_index) { return bit_index < 8U; }

static uint8_t hc595_outputs_bit_mask(uint8_t bit_index) {
  return (uint8_t)(1U << bit_index);
}

esp_err_t hc595_outputs_init(const hc595_outputs_config_t *config,
                             hc595_outputs_handle_t *ret_handle) {
  if (config == NULL || ret_handle == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  if (!hc595_outputs_bit_valid(config->relay1_bit_index) ||
      !hc595_outputs_bit_valid(config->relay2_bit_index) ||
      !hc595_outputs_bit_valid(config->valve_open_bit_index) ||
      !hc595_outputs_bit_valid(config->valve_close_bit_index)) {
    return ESP_ERR_INVALID_ARG;
  }

  const uint8_t relay1_mask = hc595_outputs_bit_mask(config->relay1_bit_index);
  const uint8_t relay2_mask = hc595_outputs_bit_mask(config->relay2_bit_index);
  const uint8_t valve_open_mask =
      hc595_outputs_bit_mask(config->valve_open_bit_index);
  const uint8_t valve_close_mask =
      hc595_outputs_bit_mask(config->valve_close_bit_index);

  if ((relay1_mask & relay2_mask) != 0U || (relay1_mask & valve_open_mask) != 0U ||
      (relay1_mask & valve_close_mask) != 0U ||
      (relay2_mask & valve_open_mask) != 0U ||
      (relay2_mask & valve_close_mask) != 0U ||
      (valve_open_mask & valve_close_mask) != 0U) {
    return ESP_ERR_INVALID_ARG;
  }

  struct hc595_outputs_ctx_t *ctx =
      (struct hc595_outputs_ctx_t *)calloc(1, sizeof(struct hc595_outputs_ctx_t));
  if (ctx == NULL) {
    return ESP_ERR_NO_MEM;
  }

  ctx->cfg = *config;
  ctx->relay_mask = (uint8_t)(relay1_mask | relay2_mask);
  ctx->valve_mask = (uint8_t)(valve_open_mask | valve_close_mask);

  hc595_config_t low_level_cfg = {
      .data_gpio_num = config->data_gpio_num,
      .clock_gpio_num = config->clock_gpio_num,
      .latch_gpio_num = config->latch_gpio_num,
      .initial_state = config->initial_state,
  };
  esp_err_t err = hc595_init(&low_level_cfg, &ctx->shift_reg);
  if (err != ESP_OK) {
    free(ctx);
    return err;
  }

  *ret_handle = ctx;
  return ESP_OK;
}

esp_err_t hc595_outputs_set_light_relays(hc595_outputs_handle_t handle,
                                         bool relay1_on, bool relay2_on) {
  if (handle == NULL || handle->shift_reg == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  uint8_t value = 0U;
  if (relay1_on) {
    value |= hc595_outputs_bit_mask(handle->cfg.relay1_bit_index);
  }
  if (relay2_on) {
    value |= hc595_outputs_bit_mask(handle->cfg.relay2_bit_index);
  }

  return hc595_write_masked(handle->shift_reg, handle->relay_mask, value);
}

esp_err_t hc595_outputs_set_valve_state(
    hc595_outputs_handle_t handle, hc595_outputs_valve_state_t valve_state) {
  if (handle == NULL || handle->shift_reg == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  uint8_t value = 0U;
  if (valve_state == HC595_OUTPUTS_VALVE_OPENING) {
    value |= hc595_outputs_bit_mask(handle->cfg.valve_open_bit_index);
  } else if (valve_state == HC595_OUTPUTS_VALVE_CLOSING) {
    value |= hc595_outputs_bit_mask(handle->cfg.valve_close_bit_index);
  }

  return hc595_write_masked(handle->shift_reg, handle->valve_mask, value);
}

uint8_t hc595_outputs_get_raw_state(hc595_outputs_handle_t handle) {
  if (handle == NULL || handle->shift_reg == NULL) {
    return 0U;
  }
  return hc595_get_state(handle->shift_reg);
}

esp_err_t hc595_outputs_del(hc595_outputs_handle_t handle) {
  if (handle != NULL) {
    if (handle->shift_reg != NULL) {
      (void)hc595_del(handle->shift_reg);
    }
    free(handle);
  }
  return ESP_OK;
}
