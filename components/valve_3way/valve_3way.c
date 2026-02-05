#include "valve_3way.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include <math.h>


static const char *TAG = "VALVE_3WAY";

typedef enum {
  VALVE_STATE_UNKNOWN = 0,
  VALVE_STATE_OPEN,
  VALVE_STATE_CLOSED
} valve_state_t;

struct valve_3way_ctx_t {
  valve_3way_config_t cfg;
  valve_state_t current_state;
};

esp_err_t valve_3way_init(const valve_3way_config_t *config,
                          valve_3way_handle_t *ret_handle) {
  if (config == NULL || ret_handle == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  // Allocate memory for context
  struct valve_3way_ctx_t *ctx = (struct valve_3way_ctx_t *)heap_caps_malloc(
      sizeof(struct valve_3way_ctx_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (ctx == NULL) {
    return ESP_ERR_NO_MEM;
  }

  ctx->cfg = *config;
  ctx->current_state = VALVE_STATE_UNKNOWN;

  // Configure GPIOs
  gpio_config_t io_conf = {};
  io_conf.intr_type = GPIO_INTR_DISABLE;
  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.pin_bit_mask =
      (1ULL << ctx->cfg.gpio_open_num) | (1ULL << ctx->cfg.gpio_close_num);
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  esp_err_t err = gpio_config(&io_conf);

  if (err != ESP_OK) {
    free(ctx);
    return err;
  }

  // Default State: Both OFF (Safe state)
  gpio_set_level(ctx->cfg.gpio_open_num, 0);
  gpio_set_level(ctx->cfg.gpio_close_num, 0);

  *ret_handle = ctx;
  ESP_LOGI(TAG,
           "Initialized. Open Pin: %d, Close Pin: %d. Thresholds: Open <= "
           "%.1f, Close >= %.1f",
           ctx->cfg.gpio_open_num, ctx->cfg.gpio_close_num,
           ctx->cfg.temp_open_threshold, ctx->cfg.temp_close_threshold);

  return ESP_OK;
}

esp_err_t valve_3way_process(valve_3way_handle_t handle, float current_temp_c) {
  if (handle == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  // Check Open Condition
  if (current_temp_c <= handle->cfg.temp_open_threshold) {
    if (handle->current_state != VALVE_STATE_OPEN) {
      ESP_LOGW(TAG, "Temp %.2f <= %.1f -> OPENING valve", current_temp_c,
               handle->cfg.temp_open_threshold);

      // Interlock: Ensure CLOSE is 0 before setting OPEN to 1
      gpio_set_level(handle->cfg.gpio_close_num, 0);

      // Small delay could be added here if hardware requires it, but for
      // relay/solenoid usually microsecond execution is fine as long as we
      // don't energize both coil sides simultaneously at high power.

      gpio_set_level(handle->cfg.gpio_open_num, 1);

      handle->current_state = VALVE_STATE_OPEN;
    }
  }
  // Check Close Condition
  else if (current_temp_c >= handle->cfg.temp_close_threshold) {
    if (handle->current_state != VALVE_STATE_CLOSED) {
      ESP_LOGW(TAG, "Temp %.2f >= %.1f -> CLOSING valve", current_temp_c,
               handle->cfg.temp_close_threshold);

      // Interlock: Ensure OPEN is 0 before setting CLOSE to 1
      gpio_set_level(handle->cfg.gpio_open_num, 0);

      gpio_set_level(handle->cfg.gpio_close_num, 1);

      handle->current_state = VALVE_STATE_CLOSED;
    }
  }
  // Hysteresis Band (Deadzone)
  else {
    // Do nothing, maintain current state
    // Optional: Log status periodically if needed, but keeping logs clean for
    // now.
  }

  return ESP_OK;
}

esp_err_t valve_3way_del(valve_3way_handle_t handle) {
  if (handle != NULL) {
    // Safe state: turn off both
    gpio_set_level(handle->cfg.gpio_open_num, 0);
    gpio_set_level(handle->cfg.gpio_close_num, 0);

    free(handle);
  }
  return ESP_OK;
}
