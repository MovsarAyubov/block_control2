#include "rll400.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <string.h>

static const char *TAG = "rll400";

typedef struct rll400_context_t {
  rll400_config_t config;
  ads1115_handle_t ads_handle;
  float target_percent;
  bool is_active;
} rll400_context_t;

esp_err_t rll400_init(const rll400_config_t *config,
                      rll400_handle_t *ret_handle) {
  ESP_RETURN_ON_FALSE(config && ret_handle, ESP_ERR_INVALID_ARG, TAG,
                      "Invalid args");

  rll400_context_t *ctx = calloc(1, sizeof(rll400_context_t));
  if (!ctx)
    return ESP_ERR_NO_MEM;

  memcpy(&ctx->config, config, sizeof(rll400_config_t));
  ctx->target_percent = 0.0f; // Default safely to 0? Or maybe read current?

  // Init GPIOs
  gpio_config_t io_conf = {.pin_bit_mask = (1ULL << config->pin_open) |
                                           (1ULL << config->pin_close),
                           .mode = GPIO_MODE_OUTPUT,
                           .pull_up_en = 0,
                           .pull_down_en = 0,
                           .intr_type = GPIO_INTR_DISABLE};
  gpio_config(&io_conf);

  // Ensure motors off
  gpio_set_level(config->pin_open, 0);
  gpio_set_level(config->pin_close, 0);

  // Init ADS1115
  ads1115_config_t ads_cfg = {.i2c_port = config->i2c_port,
                              .i2c_addr = config->ads_addr};
  esp_err_t err = ads1115_init(&ads_cfg, &ctx->ads_handle);
  if (err != ESP_OK) {
    free(ctx);
    return err;
  }

  *ret_handle = ctx;
  ESP_LOGI(TAG, "RLL400 Initialized. Shunt: %.2f Ohm",
           config->shunt_resistor_ohm);
  return ESP_OK;
}

esp_err_t rll400_set_target(rll400_handle_t handle, float target_percent) {
  ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
  rll400_context_t *ctx = handle;

  if (target_percent < 0.0f)
    target_percent = 0.0f;
  if (target_percent > 100.0f)
    target_percent = 100.0f;

  ctx->target_percent = target_percent;
  ctx->is_active = true;
  return ESP_OK;
}

esp_err_t rll400_stop(rll400_handle_t handle) {
  ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
  rll400_context_t *ctx = handle;

  ctx->is_active = false;
  gpio_set_level(ctx->config.pin_open, 0);
  gpio_set_level(ctx->config.pin_close, 0);
  return ESP_OK;
}

esp_err_t rll400_get_status(rll400_handle_t handle, float *out_position_percent,
                            float *out_current_ma) {
  ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
  rll400_context_t *ctx = handle;

  float voltage_mv = 0;
  // A2-A3 Differential
  ESP_RETURN_ON_ERROR(
      ads1115_read_voltage_differential(ctx->ads_handle, 2, 3, &voltage_mv),
      TAG, "Read ADC failed (Diff A2-A3)");

  // I = U / R
  // mA = mV / Ohm
  float current_ma = 0;
  if (ctx->config.shunt_resistor_ohm > 0) {
    current_ma = voltage_mv / ctx->config.shunt_resistor_ohm;
  }

  // 4mA -> 0%, 20mA -> 100%
  float pct = (current_ma - 4.0f) / (20.0f - 4.0f) * 100.0f;
  if (pct < 0)
    pct = 0;
  if (pct > 100)
    pct = 100;

  if (out_position_percent)
    *out_position_percent = pct;
  if (out_current_ma)
    *out_current_ma = current_ma;
  return ESP_OK;
}

esp_err_t rll400_process(rll400_handle_t handle) {
  ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
  rll400_context_t *ctx = handle;

  if (!ctx->is_active) {
    // Ensure stopped
    gpio_set_level(ctx->config.pin_open, 0);
    gpio_set_level(ctx->config.pin_close, 0);
    return ESP_OK;
  }

  float current_pos = 0;
  float current_ma = 0;
  // We ignore error here to not stop logic loop, but maybe log it
  if (rll400_get_status(handle, &current_pos, &current_ma) != ESP_OK) {
    ESP_LOGE(TAG, "Failed to get status check wiring");
    gpio_set_level(ctx->config.pin_open, 0);
    gpio_set_level(ctx->config.pin_close, 0);
    return ESP_FAIL;
  }

  float err = ctx->target_percent - current_pos;
  float hyst = ctx->config.hysteresis_percent;

  if (fabs(err) <= hyst) {
    // Reached target
    gpio_set_level(ctx->config.pin_open, 0);
    gpio_set_level(ctx->config.pin_close, 0);
    // We can optionally set is_active = false here if we want "one-shot" move
    // But for holding position we might keep it active.
    // For simplicity, let's keep checking.
  } else if (err > hyst) {
    // Target > Current -> MOVE OPEN (Assuming Open increases %)
    gpio_set_level(ctx->config.pin_close, 0); // Interlock
    gpio_set_level(ctx->config.pin_open, 1);
  } else if (err < -hyst) {
    // Target < Current -> MOVE CLOSE
    gpio_set_level(ctx->config.pin_open, 0); // Interlock
    gpio_set_level(ctx->config.pin_close, 1);
  }

  return ESP_OK;
}

esp_err_t rll400_del(rll400_handle_t handle) {
  if (handle) {
    rll400_context_t *ctx = handle;
    rll400_stop(handle);
    if (ctx->ads_handle) {
      ads1115_del(ctx->ads_handle);
    }
    free(ctx);
  }
  return ESP_OK;
}
