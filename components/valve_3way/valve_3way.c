#include "valve_3way.h"

#include "esp_log.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

static const char *TAG = "VALVE_3WAY";
static const char *VALVE_3WAY_DEFAULT_NAME = "valve_3way";

struct valve_3way_ctx_t {
  valve_3way_config_t cfg;
  valve_3way_state_t state;
  float setpoint_c;
  float actual_temp_c;
};

static bool valve_3way_gpio_is_connected(gpio_num_t gpio_num) {
  return gpio_num >= 0 && gpio_num < GPIO_NUM_MAX;
}

static bool valve_3way_outputs_enabled(const valve_3way_handle_t handle) {
  if (handle == NULL) {
    return false;
  }
  return valve_3way_gpio_is_connected(handle->cfg.gpio_open_num) &&
         valve_3way_gpio_is_connected(handle->cfg.gpio_close_num);
}

static const char *valve_3way_name(const valve_3way_handle_t handle) {
  if (handle == NULL || handle->cfg.name == NULL || handle->cfg.name[0] == '\0') {
    return VALVE_3WAY_DEFAULT_NAME;
  }
  return handle->cfg.name;
}

static const char *valve_3way_state_name(valve_3way_state_t state) {
  switch (state) {
  case VALVE_3WAY_STATE_OPENING:
    return "OPENING";
  case VALVE_3WAY_STATE_CLOSING:
    return "CLOSING";
  case VALVE_3WAY_STATE_STOPPED:
  default:
    return "STOPPED";
  }
}

static esp_err_t valve_3way_apply_state(valve_3way_handle_t handle,
                                        valve_3way_state_t next_state) {
  if (handle == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  const bool open_on = (next_state == VALVE_3WAY_STATE_OPENING);
  const bool close_on = (next_state == VALVE_3WAY_STATE_CLOSING);
  if (open_on && close_on) {
    return ESP_ERR_INVALID_STATE;
  }

  if (valve_3way_outputs_enabled(handle)) {
    gpio_set_level(handle->cfg.gpio_open_num, 0);
    gpio_set_level(handle->cfg.gpio_close_num, 0);

    if (open_on) {
      gpio_set_level(handle->cfg.gpio_open_num, 1);
    } else if (close_on) {
      gpio_set_level(handle->cfg.gpio_close_num, 1);
    }
  }

  if (handle->state != next_state) {
    ESP_LOGI(TAG,
             "[%s] state %s -> %s | sp=%.2fC actual=%.2fC hyst=%.2fC",
             valve_3way_name(handle), valve_3way_state_name(handle->state),
             valve_3way_state_name(next_state), handle->setpoint_c,
             handle->actual_temp_c, handle->cfg.hysteresis_c);
  }

  handle->state = next_state;
  return ESP_OK;
}

static valve_3way_state_t valve_3way_decide_state(valve_3way_handle_t handle) {
  if (handle == NULL) {
    return VALVE_3WAY_STATE_STOPPED;
  }

  const float delta_c = handle->actual_temp_c - handle->setpoint_c;
  const float hysteresis_c = handle->cfg.hysteresis_c;

  if (delta_c > hysteresis_c) {
    return VALVE_3WAY_STATE_CLOSING;
  }
  if (delta_c < -hysteresis_c) {
    return VALVE_3WAY_STATE_OPENING;
  }
  return VALVE_3WAY_STATE_STOPPED;
}

esp_err_t valve_3way_init(const valve_3way_config_t *config,
                          valve_3way_handle_t *ret_handle) {
  if (config == NULL || ret_handle == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  const bool open_connected = valve_3way_gpio_is_connected(config->gpio_open_num);
  const bool close_connected =
      valve_3way_gpio_is_connected(config->gpio_close_num);

  if (open_connected != close_connected) {
    return ESP_ERR_INVALID_ARG;
  }
  if (open_connected && config->gpio_open_num == config->gpio_close_num) {
    return ESP_ERR_INVALID_ARG;
  }
  if (!isfinite(config->hysteresis_c) || config->hysteresis_c < 0.0f) {
    return ESP_ERR_INVALID_ARG;
  }
  if (!isfinite(config->initial_setpoint_c) ||
      !isfinite(config->initial_actual_temp_c)) {
    return ESP_ERR_INVALID_ARG;
  }

  struct valve_3way_ctx_t *ctx =
      (struct valve_3way_ctx_t *)calloc(1, sizeof(struct valve_3way_ctx_t));
  if (ctx == NULL) {
    return ESP_ERR_NO_MEM;
  }

  memcpy(&ctx->cfg, config, sizeof(*config));
  ctx->state = VALVE_3WAY_STATE_STOPPED;
  ctx->setpoint_c = config->initial_setpoint_c;
  ctx->actual_temp_c = config->initial_actual_temp_c;

  if (open_connected) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << config->gpio_open_num) |
                        (1ULL << config->gpio_close_num),
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
  }

  esp_err_t err = valve_3way_stop(ctx);
  if (err != ESP_OK) {
    free(ctx);
    return err;
  }

  *ret_handle = ctx;

  if (open_connected) {
    ESP_LOGI(TAG, "[%s] initialized: open=%d close=%d hyst=%.2fC",
             valve_3way_name(ctx), ctx->cfg.gpio_open_num,
             ctx->cfg.gpio_close_num, ctx->cfg.hysteresis_c);
  } else {
    ESP_LOGW(TAG, "[%s] initialized without GPIOs, outputs disabled",
             valve_3way_name(ctx));
  }

  return ESP_OK;
}

esp_err_t valve_3way_set_setpoint_c(valve_3way_handle_t handle,
                                    float setpoint_c) {
  if (handle == NULL || !isfinite(setpoint_c)) {
    return ESP_ERR_INVALID_ARG;
  }
  handle->setpoint_c = setpoint_c;
  return ESP_OK;
}

esp_err_t valve_3way_set_actual_temp_c(valve_3way_handle_t handle,
                                       float actual_temp_c) {
  if (handle == NULL || !isfinite(actual_temp_c)) {
    return ESP_ERR_INVALID_ARG;
  }
  handle->actual_temp_c = actual_temp_c;
  return ESP_OK;
}

esp_err_t valve_3way_set_hysteresis_c(valve_3way_handle_t handle,
                                      float hysteresis_c) {
  if (handle == NULL || !isfinite(hysteresis_c) || hysteresis_c < 0.0f) {
    return ESP_ERR_INVALID_ARG;
  }
  handle->cfg.hysteresis_c = hysteresis_c;
  return ESP_OK;
}

esp_err_t valve_3way_process(valve_3way_handle_t handle) {
  if (handle == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  return valve_3way_apply_state(handle, valve_3way_decide_state(handle));
}

esp_err_t valve_3way_process_temperatures(valve_3way_handle_t handle,
                                          float setpoint_c,
                                          float actual_temp_c) {
  esp_err_t err = valve_3way_set_setpoint_c(handle, setpoint_c);
  if (err != ESP_OK) {
    return err;
  }

  err = valve_3way_set_actual_temp_c(handle, actual_temp_c);
  if (err != ESP_OK) {
    return err;
  }

  return valve_3way_process(handle);
}

esp_err_t valve_3way_open(valve_3way_handle_t handle) {
  return valve_3way_apply_state(handle, VALVE_3WAY_STATE_OPENING);
}

esp_err_t valve_3way_close(valve_3way_handle_t handle) {
  return valve_3way_apply_state(handle, VALVE_3WAY_STATE_CLOSING);
}

esp_err_t valve_3way_stop(valve_3way_handle_t handle) {
  return valve_3way_apply_state(handle, VALVE_3WAY_STATE_STOPPED);
}

valve_3way_state_t valve_3way_get_state(valve_3way_handle_t handle) {
  if (handle == NULL) {
    return VALVE_3WAY_STATE_STOPPED;
  }
  return handle->state;
}

float valve_3way_get_setpoint_c(valve_3way_handle_t handle) {
  if (handle == NULL) {
    return 0.0f;
  }
  return handle->setpoint_c;
}

float valve_3way_get_actual_temp_c(valve_3way_handle_t handle) {
  if (handle == NULL) {
    return 0.0f;
  }
  return handle->actual_temp_c;
}

esp_err_t valve_3way_del(valve_3way_handle_t handle) {
  if (handle != NULL) {
    (void)valve_3way_stop(handle);
    free(handle);
  }
  return ESP_OK;
}
