#include "rll400.h"

#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

static const char *TAG = "rll400";
static const char *RLL400_DEFAULT_NAME = "rll400";
#define RLL400_STATUS_LOG_PERIOD_MS 5000U
#define RLL400_TEMP_DISABLE_FAULTS 1

struct rll400_context_t {
  rll400_config_t config;
  ads1115_handle_t ads_handle;
  float target_percent;
  float current_position_percent;
  float current_voltage_mv;
  float current_ma;
  float last_valid_position_percent;
  float last_motion_position_percent;
  uint32_t boot_started_ms;
  uint32_t last_motion_ms;
  uint32_t last_status_log_ms;
  rll400_state_t state;
  rll400_fault_code_t fault_code;
  bool position_valid;
  bool has_last_valid_position;
  bool control_active;
  bool local_manual_active;
};

static bool rll400_gpio_is_connected(gpio_num_t gpio_num) {
  return gpio_num >= 0 && gpio_num < GPIO_NUM_MAX;
}

static bool rll400_outputs_enabled(const struct rll400_context_t *ctx) {
  if (ctx == NULL) {
    return false;
  }
  return rll400_gpio_is_connected(ctx->config.pin_open) &&
         rll400_gpio_is_connected(ctx->config.pin_close);
}

static bool rll400_local_manual_input_connected(const struct rll400_context_t *ctx) {
  if (ctx == NULL) {
    return false;
  }
  return rll400_gpio_is_connected(ctx->config.pin_local_manual);
}

static uint32_t rll400_now_ms(void) {
  return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

static float rll400_clamp_percent(float value) {
  if (value < 0.0f) {
    return 0.0f;
  }
  if (value > 100.0f) {
    return 100.0f;
  }
  return value;
}

static const char *rll400_name(const struct rll400_context_t *ctx) {
  if (ctx == NULL || ctx->config.name == NULL || ctx->config.name[0] == '\0') {
    return RLL400_DEFAULT_NAME;
  }
  return ctx->config.name;
}

static const char *rll400_state_name(rll400_state_t state) {
  switch (state) {
  case RLL400_STATE_BOOT_WAIT_POSITION:
    return "BOOT_WAIT_POSITION";
  case RLL400_STATE_HOLDING:
    return "HOLDING";
  case RLL400_STATE_MOVING_OPEN:
    return "MOVING_OPEN";
  case RLL400_STATE_MOVING_CLOSE:
    return "MOVING_CLOSE";
  case RLL400_STATE_LOCAL_MANUAL:
    return "LOCAL_MANUAL";
  case RLL400_STATE_FAULT_BLOCKED:
  default:
    return "FAULT_BLOCKED";
  }
}

static const char *rll400_fault_name(rll400_fault_code_t fault_code) {
  switch (fault_code) {
  case RLL400_FAULT_NONE:
    return "NONE";
  case RLL400_FAULT_NO_MOTION:
    return "NO_MOTION";
  case RLL400_FAULT_ENCODER:
    return "ENCODER";
  case RLL400_FAULT_BOOT_NO_POSITION:
    return "BOOT_NO_POSITION";
  default:
    return "UNKNOWN";
  }
}

static bool rll400_valid_diff_pair(uint8_t channel_pos, uint8_t channel_neg) {
  return (channel_pos == 0U && channel_neg == 1U) ||
         (channel_pos == 0U && channel_neg == 3U) ||
         (channel_pos == 1U && channel_neg == 3U) ||
         (channel_pos == 2U && channel_neg == 3U);
}

static void rll400_write_outputs(struct rll400_context_t *ctx, bool open_on,
                                 bool close_on) {
  if (ctx == NULL || !rll400_outputs_enabled(ctx)) {
    return;
  }

  gpio_set_level(ctx->config.pin_open, 0);
  gpio_set_level(ctx->config.pin_close, 0);

  if (open_on) {
    gpio_set_level(ctx->config.pin_open, 1);
  } else if (close_on) {
    gpio_set_level(ctx->config.pin_close, 1);
  }
}

static void rll400_apply_state(struct rll400_context_t *ctx,
                               rll400_state_t next_state) {
  if (ctx == NULL) {
    return;
  }

  switch (next_state) {
  case RLL400_STATE_MOVING_OPEN:
    rll400_write_outputs(ctx, true, false);
    break;
  case RLL400_STATE_MOVING_CLOSE:
    rll400_write_outputs(ctx, false, true);
    break;
  case RLL400_STATE_BOOT_WAIT_POSITION:
  case RLL400_STATE_HOLDING:
  case RLL400_STATE_LOCAL_MANUAL:
  case RLL400_STATE_FAULT_BLOCKED:
  default:
    rll400_write_outputs(ctx, false, false);
    break;
  }

  if (ctx->state != next_state) {
    ESP_LOGI(TAG,
             "[%s] state %s -> %s | target=%.1f%% pos=%.1f%% current=%.2fmA "
             "fault=%s",
             rll400_name(ctx), rll400_state_name(ctx->state),
             rll400_state_name(next_state), ctx->target_percent,
             ctx->has_last_valid_position ? ctx->last_valid_position_percent : 0.0f,
             ctx->current_ma, rll400_fault_name(ctx->fault_code));
  }

  ctx->state = next_state;
}

static void rll400_set_fault(struct rll400_context_t *ctx,
                             rll400_fault_code_t fault_code) {
  if (ctx == NULL || fault_code == RLL400_FAULT_NONE) {
    return;
  }

#if RLL400_TEMP_DISABLE_FAULTS
  ESP_LOGW(TAG, "[%s] fault suppressed -> %s", rll400_name(ctx),
           rll400_fault_name(fault_code));
  return;
#endif

  if (ctx->fault_code != fault_code) {
    ESP_LOGE(TAG, "[%s] fault -> %s", rll400_name(ctx),
             rll400_fault_name(fault_code));
  }
  ctx->fault_code = fault_code;
  rll400_apply_state(ctx, RLL400_STATE_FAULT_BLOCKED);
}

static bool rll400_read_local_manual(const struct rll400_context_t *ctx) {
  if (!rll400_local_manual_input_connected(ctx)) {
    return false;
  }
  const int level = gpio_get_level(ctx->config.pin_local_manual);
  if (ctx->config.local_manual_active_high) {
    return level != 0;
  }
  return level == 0;
}

static esp_err_t rll400_sample_encoder(struct rll400_context_t *ctx,
                                       float *out_position_percent,
                                       float *out_current_ma, bool *out_valid) {
  if (ctx == NULL || out_valid == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  float voltage_mv = 0.0f;
  esp_err_t err = ads1115_read_voltage_differential(
      ctx->ads_handle, ctx->config.ads_channel_pos, ctx->config.ads_channel_neg,
      &voltage_mv);
  ctx->current_voltage_mv = voltage_mv;
  if (err != ESP_OK) {
    *out_valid = false;
    return err;
  }

  float current_ma = 0.0f;
  if (ctx->config.shunt_resistor_ohm > 0.0f) {
    current_ma = voltage_mv / ctx->config.shunt_resistor_ohm;
  }

  const bool valid = isfinite(current_ma) &&
                     current_ma >= ctx->config.encoder_min_ma &&
                     current_ma <= ctx->config.encoder_max_ma;
  float percent = 0.0f;
  if (valid) {
    percent = ((current_ma - 4.0f) / 16.0f) * 100.0f;
    percent = rll400_clamp_percent(percent);
  }

  if (out_position_percent != NULL) {
    *out_position_percent = percent;
  }
  if (out_current_ma != NULL) {
    *out_current_ma = current_ma;
  }
  *out_valid = valid;
  return ESP_OK;
}

static void rll400_update_motion_reference(struct rll400_context_t *ctx,
                                           float current_position_percent,
                                           uint32_t now_ms) {
  if (ctx == NULL) {
    return;
  }
  ctx->last_motion_position_percent = current_position_percent;
  ctx->last_motion_ms = now_ms;
}

static void rll400_maybe_log_status(struct rll400_context_t *ctx,
                                    uint32_t now_ms) {
  if (ctx == NULL) {
    return;
  }

  if (ctx->last_status_log_ms != 0U &&
      (now_ms - ctx->last_status_log_ms) < RLL400_STATUS_LOG_PERIOD_MS) {
    return;
  }

  ctx->last_status_log_ms = now_ms;

  const float position_percent =
      ctx->position_valid ? ctx->current_position_percent
                          : (ctx->has_last_valid_position ? ctx->last_valid_position_percent
                                                          : 0.0f);

  ESP_LOGI(TAG,
           "[%s] status state=%s target=%.1f%% pos=%.1f%% voltage=%.1fmV "
           "current=%.2fmA "
           "valid=%u manual=%u outputs=%u control=%u fault=%s",
           rll400_name(ctx), rll400_state_name(ctx->state), ctx->target_percent,
           position_percent, ctx->current_voltage_mv, ctx->current_ma,
           ctx->position_valid ? 1U : 0U, ctx->local_manual_active ? 1U : 0U,
           rll400_outputs_enabled(ctx) ? 1U : 0U, ctx->control_active ? 1U : 0U,
           rll400_fault_name(ctx->fault_code));
}

esp_err_t rll400_init(const rll400_config_t *config,
                      rll400_handle_t *ret_handle) {
  ESP_RETURN_ON_FALSE(config != NULL && ret_handle != NULL, ESP_ERR_INVALID_ARG,
                      TAG, "Invalid args");
  ESP_RETURN_ON_FALSE(config->shunt_resistor_ohm > 0.0f, ESP_ERR_INVALID_ARG,
                      TAG, "Invalid shunt resistor");
  ESP_RETURN_ON_FALSE(rll400_valid_diff_pair(config->ads_channel_pos,
                                             config->ads_channel_neg),
                      ESP_ERR_INVALID_ARG, TAG, "Unsupported ADS diff pair");

  const bool open_connected = rll400_gpio_is_connected(config->pin_open);
  const bool close_connected = rll400_gpio_is_connected(config->pin_close);
  ESP_RETURN_ON_FALSE(open_connected == close_connected, ESP_ERR_INVALID_ARG, TAG,
                      "OPEN/CLOSE pins must be both connected or both disconnected");
  ESP_RETURN_ON_FALSE(!open_connected || config->pin_open != config->pin_close,
                      ESP_ERR_INVALID_ARG, TAG, "OPEN/CLOSE pins must differ");

  struct rll400_context_t *ctx =
      (struct rll400_context_t *)calloc(1, sizeof(struct rll400_context_t));
  if (ctx == NULL) {
    return ESP_ERR_NO_MEM;
  }

  memcpy(&ctx->config, config, sizeof(*config));
  ctx->target_percent = 0.0f;
  ctx->boot_started_ms = rll400_now_ms();
  ctx->state = RLL400_STATE_BOOT_WAIT_POSITION;

  if (open_connected) {
    gpio_config_t io_conf = {
        .pin_bit_mask =
            (1ULL << (uint32_t)config->pin_open) | (1ULL << (uint32_t)config->pin_close),
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

  if (rll400_local_manual_input_connected(ctx)) {
    gpio_config_t manual_conf = {
        .pin_bit_mask = (1ULL << (uint32_t)config->pin_local_manual),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&manual_conf);
    if (err != ESP_OK) {
      free(ctx);
      return err;
    }
  }

  rll400_write_outputs(ctx, false, false);

  ads1115_config_t ads_cfg = {
      .i2c_port = config->i2c_port,
      .i2c_addr = config->ads_addr,
  };
  esp_err_t err = ads1115_init(&ads_cfg, &ctx->ads_handle);
  if (err != ESP_OK) {
    free(ctx);
    return err;
  }

  *ret_handle = ctx;

  if (open_connected) {
    ESP_LOGI(TAG,
             "[%s] initialized: ads=0x%02X diff=%u-%u open=%d close=%d "
             "manual=%d",
             rll400_name(ctx), (unsigned)config->ads_addr,
             (unsigned)config->ads_channel_pos, (unsigned)config->ads_channel_neg,
             config->pin_open, config->pin_close, config->pin_local_manual);
  } else {
    ESP_LOGW(TAG,
             "[%s] initialized without motor GPIOs, outputs disabled for now",
             rll400_name(ctx));
  }
#if RLL400_TEMP_DISABLE_FAULTS
  ESP_LOGW(TAG, "[%s] temporary mode: fault protections disabled",
           rll400_name(ctx));
#endif
  return ESP_OK;
}

esp_err_t rll400_set_target(rll400_handle_t handle, float target_percent) {
  ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
  struct rll400_context_t *ctx = handle;

  ctx->target_percent = rll400_clamp_percent(target_percent);
  ctx->control_active = true;
  return ESP_OK;
}

esp_err_t rll400_set_target_hysteresis_percent(rll400_handle_t handle,
                                               float hysteresis_percent) {
  ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
  if (!isfinite(hysteresis_percent) || hysteresis_percent < 0.0f) {
    return ESP_ERR_INVALID_ARG;
  }
  struct rll400_context_t *ctx = handle;
  ctx->config.target_hysteresis_percent = hysteresis_percent;
  return ESP_OK;
}

esp_err_t rll400_set_motion_fault_config(rll400_handle_t handle,
                                         float motion_delta_percent,
                                         uint32_t no_motion_timeout_ms) {
  ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
  if (!isfinite(motion_delta_percent) || motion_delta_percent < 0.0f) {
    return ESP_ERR_INVALID_ARG;
  }
  struct rll400_context_t *ctx = handle;
  ctx->config.motion_delta_percent = motion_delta_percent;
  ctx->config.no_motion_timeout_ms = no_motion_timeout_ms;
  return ESP_OK;
}

esp_err_t rll400_stop(rll400_handle_t handle) {
  ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
  struct rll400_context_t *ctx = handle;

  ctx->control_active = false;
  rll400_apply_state(ctx, RLL400_STATE_HOLDING);
  return ESP_OK;
}

esp_err_t rll400_reset_fault(rll400_handle_t handle) {
  ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
  struct rll400_context_t *ctx = handle;

  if (ctx->fault_code == RLL400_FAULT_NONE) {
    return ESP_OK;
  }

  ESP_LOGI(TAG, "[%s] fault reset", rll400_name(ctx));
  ctx->fault_code = RLL400_FAULT_NONE;
  ctx->boot_started_ms = rll400_now_ms();
  if (ctx->position_valid) {
    rll400_update_motion_reference(ctx, ctx->current_position_percent,
                                   ctx->boot_started_ms);
  }
  rll400_apply_state(ctx, ctx->local_manual_active ? RLL400_STATE_LOCAL_MANUAL
                                                   : RLL400_STATE_HOLDING);
  return ESP_OK;
}

esp_err_t rll400_get_status(rll400_handle_t handle, float *out_position_percent,
                            float *out_current_ma) {
  ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
  struct rll400_context_t *ctx = handle;

  if (out_position_percent != NULL) {
    if (ctx->position_valid) {
      *out_position_percent = ctx->current_position_percent;
    } else if (ctx->has_last_valid_position) {
      *out_position_percent = ctx->last_valid_position_percent;
    } else {
      *out_position_percent = 0.0f;
    }
  }
  if (out_current_ma != NULL) {
    *out_current_ma = ctx->current_ma;
  }

  return ctx->position_valid ? ESP_OK : ESP_ERR_INVALID_STATE;
}

esp_err_t rll400_get_runtime_status(rll400_handle_t handle,
                                    rll400_status_t *out_status) {
  ESP_RETURN_ON_FALSE(handle != NULL && out_status != NULL, ESP_ERR_INVALID_ARG,
                      TAG, "Invalid args");
  struct rll400_context_t *ctx = handle;

  memset(out_status, 0, sizeof(*out_status));
  out_status->position_percent =
      ctx->position_valid ? ctx->current_position_percent
                          : (ctx->has_last_valid_position ? ctx->last_valid_position_percent
                                                          : 0.0f);
  out_status->target_percent = ctx->target_percent;
  out_status->current_ma = ctx->current_ma;
  out_status->state = ctx->state;
  out_status->fault_code = ctx->fault_code;
  out_status->position_valid = ctx->position_valid;
  out_status->at_target =
      ctx->position_valid &&
      fabsf(ctx->target_percent - ctx->current_position_percent) <=
          ctx->config.target_hysteresis_percent;
  out_status->blocked = (ctx->fault_code != RLL400_FAULT_NONE);
  out_status->local_manual_active = ctx->local_manual_active;
  out_status->outputs_enabled = rll400_outputs_enabled(ctx);
  out_status->control_active = ctx->control_active;
  return ESP_OK;
}

esp_err_t rll400_process(rll400_handle_t handle) {
  ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
  struct rll400_context_t *ctx = handle;
  const uint32_t now_ms = rll400_now_ms();

  ctx->local_manual_active = rll400_read_local_manual(ctx);
  if (ctx->local_manual_active) {
    rll400_apply_state(ctx, RLL400_STATE_LOCAL_MANUAL);
    rll400_maybe_log_status(ctx, now_ms);
    return ESP_OK;
  }

  float current_position_percent = 0.0f;
  float current_ma = 0.0f;
  bool encoder_valid = false;
  esp_err_t sample_err =
      rll400_sample_encoder(ctx, &current_position_percent, &current_ma, &encoder_valid);

  ctx->current_ma = current_ma;
  ctx->position_valid = (sample_err == ESP_OK) && encoder_valid;
  if (ctx->position_valid) {
    ctx->current_position_percent = current_position_percent;
    ctx->last_valid_position_percent = current_position_percent;
    ctx->has_last_valid_position = true;
    if (ctx->last_motion_ms == 0U) {
      rll400_update_motion_reference(ctx, current_position_percent, now_ms);
    }
  }

  if (!ctx->position_valid) {
    const bool boot_timeout_elapsed =
        (now_ms - ctx->boot_started_ms) >= ctx->config.boot_wait_position_timeout_ms;
    if (!ctx->has_last_valid_position && !boot_timeout_elapsed) {
      rll400_apply_state(ctx, RLL400_STATE_BOOT_WAIT_POSITION);
      rll400_maybe_log_status(ctx, now_ms);
      return ESP_OK;
    }

#if RLL400_TEMP_DISABLE_FAULTS
    if (!ctx->has_last_valid_position) {
      rll400_apply_state(ctx, RLL400_STATE_HOLDING);
      rll400_maybe_log_status(ctx, now_ms);
      return sample_err == ESP_OK ? ESP_ERR_INVALID_STATE : sample_err;
    }

    current_position_percent = ctx->last_valid_position_percent;
#else
    if (!ctx->has_last_valid_position) {
      rll400_set_fault(ctx, RLL400_FAULT_BOOT_NO_POSITION);
    } else {
      rll400_set_fault(ctx, RLL400_FAULT_ENCODER);
    }
    rll400_maybe_log_status(ctx, now_ms);
    return sample_err == ESP_OK ? ESP_ERR_INVALID_STATE : sample_err;
#endif
  }

  if (ctx->fault_code != RLL400_FAULT_NONE) {
    rll400_apply_state(ctx, RLL400_STATE_FAULT_BLOCKED);
    rll400_maybe_log_status(ctx, now_ms);
    return ESP_OK;
  }

  if (!ctx->control_active) {
    rll400_apply_state(ctx, RLL400_STATE_HOLDING);
    rll400_maybe_log_status(ctx, now_ms);
    return ESP_OK;
  }

  const float error_percent = ctx->target_percent - current_position_percent;
  const float hysteresis = ctx->config.target_hysteresis_percent;
  if (fabsf(error_percent) <= hysteresis) {
    rll400_update_motion_reference(ctx, current_position_percent, now_ms);
    rll400_apply_state(ctx, RLL400_STATE_HOLDING);
    rll400_maybe_log_status(ctx, now_ms);
    return ESP_OK;
  }

  const rll400_state_t desired_state =
      (error_percent > 0.0f) ? RLL400_STATE_MOVING_OPEN : RLL400_STATE_MOVING_CLOSE;

  if (ctx->state != desired_state) {
    rll400_update_motion_reference(ctx, current_position_percent, now_ms);
  } else if (fabsf(current_position_percent - ctx->last_motion_position_percent) >=
             ctx->config.motion_delta_percent) {
    rll400_update_motion_reference(ctx, current_position_percent, now_ms);
#if !RLL400_TEMP_DISABLE_FAULTS
  } else if (rll400_outputs_enabled(ctx) && ctx->config.no_motion_timeout_ms > 0U &&
             (now_ms - ctx->last_motion_ms) >= ctx->config.no_motion_timeout_ms) {
    rll400_set_fault(ctx, RLL400_FAULT_NO_MOTION);
    rll400_maybe_log_status(ctx, now_ms);
    return ESP_OK;
#endif
  }

  rll400_apply_state(ctx, desired_state);
  rll400_maybe_log_status(ctx, now_ms);
  return ESP_OK;
}

esp_err_t rll400_del(rll400_handle_t handle) {
  if (handle != NULL) {
    struct rll400_context_t *ctx = handle;
    (void)rll400_stop(handle);
    if (ctx->ads_handle != NULL) {
      ads1115_del(ctx->ads_handle);
    }
    free(ctx);
  }
  return ESP_OK;
}
