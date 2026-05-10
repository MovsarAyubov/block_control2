#include "hc595.h"

#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include <stdlib.h>

struct hc595_ctx_t {
  hc595_config_t cfg;
  uint32_t state;
  portMUX_TYPE lock;
};

#define HC595_MAX_CHIP_COUNT 3U

static bool hc595_gpio_valid(gpio_num_t gpio_num) {
  return gpio_num >= 0 && gpio_num < GPIO_NUM_MAX;
}

static uint8_t hc595_effective_chip_count(const hc595_config_t *cfg) {
  if (cfg == NULL || cfg->chip_count == 0U) {
    return 1U;
  }
  return cfg->chip_count;
}

static uint32_t hc595_state_mask(uint8_t chip_count) {
  const uint8_t bit_count = (uint8_t)(chip_count * 8U);
  if (bit_count >= 32U) {
    return UINT32_MAX;
  }
  return (1UL << bit_count) - 1UL;
}

static void hc595_shift_out_locked(struct hc595_ctx_t *ctx, uint32_t value) {
  const uint8_t chip_count = hc595_effective_chip_count(&ctx->cfg);
  for (int byte = (int)chip_count - 1; byte >= 0; --byte) {
    const uint8_t byte_value = (uint8_t)((value >> ((uint8_t)byte * 8U)) & 0xFFU);
    for (int bit = 7; bit >= 0; --bit) {
      gpio_set_level(ctx->cfg.clock_gpio_num, 0);
      gpio_set_level(ctx->cfg.data_gpio_num, (byte_value >> bit) & 0x01U);
      gpio_set_level(ctx->cfg.clock_gpio_num, 1);
    }
  }
  gpio_set_level(ctx->cfg.latch_gpio_num, 0);
  gpio_set_level(ctx->cfg.latch_gpio_num, 1);
}

esp_err_t hc595_init(const hc595_config_t *config, hc595_handle_t *ret_handle) {
  if (config == NULL || ret_handle == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  if (!hc595_gpio_valid(config->data_gpio_num) ||
      !hc595_gpio_valid(config->clock_gpio_num) ||
      !hc595_gpio_valid(config->latch_gpio_num)) {
    return ESP_ERR_INVALID_ARG;
  }
  const uint8_t chip_count = hc595_effective_chip_count(config);
  if (chip_count > HC595_MAX_CHIP_COUNT) {
    return ESP_ERR_INVALID_ARG;
  }
  if (config->data_gpio_num == config->clock_gpio_num ||
      config->data_gpio_num == config->latch_gpio_num ||
      config->clock_gpio_num == config->latch_gpio_num) {
    return ESP_ERR_INVALID_ARG;
  }

  struct hc595_ctx_t *ctx =
      (struct hc595_ctx_t *)calloc(1, sizeof(struct hc595_ctx_t));
  if (ctx == NULL) {
    return ESP_ERR_NO_MEM;
  }

  ctx->cfg = *config;
  ctx->cfg.chip_count = chip_count;
  ctx->state = config->initial_state & hc595_state_mask(chip_count);
  ctx->lock = (portMUX_TYPE)portMUX_INITIALIZER_UNLOCKED;

  gpio_config_t io_conf = {
      .pin_bit_mask = (1ULL << config->data_gpio_num) |
                      (1ULL << config->clock_gpio_num) |
                      (1ULL << config->latch_gpio_num),
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };

  esp_err_t err = gpio_config(&io_conf);
  if (err != ESP_OK) {
    free(ctx);
    return err;
  }

  gpio_set_level(config->data_gpio_num, 0);
  gpio_set_level(config->clock_gpio_num, 0);
  gpio_set_level(config->latch_gpio_num, 0);

  taskENTER_CRITICAL(&ctx->lock);
  hc595_shift_out_locked(ctx, ctx->state);
  taskEXIT_CRITICAL(&ctx->lock);

  *ret_handle = ctx;
  return ESP_OK;
}

esp_err_t hc595_write_state(hc595_handle_t handle, uint32_t value) {
  if (handle == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  taskENTER_CRITICAL(&handle->lock);
  handle->state = value & hc595_state_mask(hc595_effective_chip_count(&handle->cfg));
  hc595_shift_out_locked(handle, handle->state);
  taskEXIT_CRITICAL(&handle->lock);
  return ESP_OK;
}

esp_err_t hc595_write_byte(hc595_handle_t handle, uint8_t value) {
  return hc595_write_state(handle, (uint32_t)value);
}

esp_err_t hc595_write_masked(hc595_handle_t handle, uint32_t mask,
                             uint32_t value) {
  if (handle == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  taskENTER_CRITICAL(&handle->lock);
  const uint32_t valid_mask =
      hc595_state_mask(hc595_effective_chip_count(&handle->cfg));
  mask &= valid_mask;
  handle->state = ((handle->state & ~mask) | (value & mask)) & valid_mask;
  hc595_shift_out_locked(handle, handle->state);
  taskEXIT_CRITICAL(&handle->lock);
  return ESP_OK;
}

esp_err_t hc595_set_bit(hc595_handle_t handle, uint8_t bit_index, bool level) {
  if (handle == NULL ||
      bit_index >= (uint8_t)(hc595_effective_chip_count(&handle->cfg) * 8U)) {
    return ESP_ERR_INVALID_ARG;
  }

  const uint32_t mask = (1UL << bit_index);
  return hc595_write_masked(handle, mask, level ? mask : 0U);
}

uint32_t hc595_get_state(hc595_handle_t handle) {
  if (handle == NULL) {
    return 0U;
  }

  uint32_t value = 0U;
  taskENTER_CRITICAL(&handle->lock);
  value = handle->state;
  taskEXIT_CRITICAL(&handle->lock);
  return value;
}

esp_err_t hc595_del(hc595_handle_t handle) {
  if (handle != NULL) {
    (void)hc595_write_state(handle, 0U);
    free(handle);
  }
  return ESP_OK;
}
