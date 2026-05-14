#include "curtain_controller.h"
#include "ads1115.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

static const char *TAG = "curtain_controller";
#define CURTAIN_CONTROLLER_LOG_PERIOD_US (5000000LL)

typedef enum {
  CURTAIN_OUTPUT_STOP = 0,
  CURTAIN_OUTPUT_OPEN,
  CURTAIN_OUTPUT_CLOSE,
} curtain_output_cmd_t;

struct curtain_controller_ctx_t {
  curtain_controller_config_t config;
  ads1115_handle_t ads_handle;
  curtain_controller_status_t status;
  curtain_output_cmd_t output_cmd;
  bool radiation_active;
  bool cold_active;
  bool heat_active;
  bool humidity_low_active;
  bool humidity_high_active;
  bool motion_ref_valid;
  float motion_ref_position_percent;
  int64_t motion_ref_time_us;
  int64_t boot_time_us;
  int64_t last_status_log_us;
};

static float clampf_local(float value, float min_value, float max_value) {
  if (value < min_value) {
    return min_value;
  }
  if (value > max_value) {
    return max_value;
  }
  return value;
}

static float clamp_target_percent(const curtain_controller_settings_t *settings,
                                  float target_percent) {
  if (target_percent <= 0.0f) {
    return 0.0f;
  }
  return clampf_local(target_percent, settings->min_position_percent,
                      settings->max_position_percent);
}

static bool value_is_valid(float value) {
  return isfinite(value);
}

static bool hhmm_valid(uint16_t hhmm) {
  const uint16_t hour = (uint16_t)(hhmm / 100U);
  const uint16_t minute = (uint16_t)(hhmm % 100U);
  return hour < 24U && minute < 60U;
}

static uint16_t hhmm_to_minutes(uint16_t hhmm) {
  return (uint16_t)(((hhmm / 100U) * 60U) + (hhmm % 100U));
}

static bool schedule_active(const curtain_controller_settings_t *settings,
                            const curtain_controller_inputs_t *inputs) {
  if (settings == NULL || inputs == NULL || !inputs->time_valid ||
      inputs->hour > 23U || inputs->minute > 59U ||
      !hhmm_valid(settings->schedule_start_hhmm) ||
      !hhmm_valid(settings->schedule_end_hhmm)) {
    return false;
  }

  const uint16_t start = hhmm_to_minutes(settings->schedule_start_hhmm);
  const uint16_t end = hhmm_to_minutes(settings->schedule_end_hhmm);
  const uint16_t now = (uint16_t)(inputs->hour * 60U + inputs->minute);

  if (start == end) {
    return true;
  }
  if (start < end) {
    return now >= start && now < end;
  }
  return now >= start || now < end;
}

static void set_fault(curtain_controller_handle_t handle,
                      curtain_controller_fault_code_t fault_code) {
  if (handle == NULL || handle->status.fault_code != CURTAIN_CONTROLLER_FAULT_NONE) {
    return;
  }
  handle->status.fault_code = fault_code;
}

static esp_err_t apply_output(curtain_controller_handle_t handle,
                              curtain_output_cmd_t cmd) {
  ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "invalid handle");
  ESP_RETURN_ON_FALSE(handle->config.outputs != NULL, ESP_ERR_INVALID_STATE, TAG,
                      "outputs not initialized");

  uint32_t mask = (1UL << handle->config.open_bit_index) |
                  (1UL << handle->config.close_bit_index);
  uint32_t value = 0U;
  if (cmd == CURTAIN_OUTPUT_OPEN) {
    value = (1UL << handle->config.open_bit_index);
  } else if (cmd == CURTAIN_OUTPUT_CLOSE) {
    value = (1UL << handle->config.close_bit_index);
  }

  esp_err_t err =
      hc595_outputs_write_masked(handle->config.outputs, mask, value);
  if (err == ESP_OK) {
    handle->output_cmd = cmd;
    handle->status.output_open = (cmd == CURTAIN_OUTPUT_OPEN);
    handle->status.output_close = (cmd == CURTAIN_OUTPUT_CLOSE);
  } else {
    set_fault(handle, CURTAIN_CONTROLLER_FAULT_OUTPUT);
  }
  return err;
}

static esp_err_t read_position(curtain_controller_handle_t handle) {
  ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "invalid handle");

  float voltage_mv = 0.0f;
  esp_err_t err = ads1115_read_voltage_differential(
      handle->ads_handle, handle->config.ads_channel_pos,
      handle->config.ads_channel_neg, &voltage_mv);
  if (err != ESP_OK) {
    handle->status.position_valid = false;
    handle->status.current_ma = 0.0f;
    set_fault(handle, CURTAIN_CONTROLLER_FAULT_ENCODER);
    return err;
  }

  const float current_ma = voltage_mv / handle->config.shunt_resistor_ohm;
  handle->status.current_ma = current_ma;

  const bool valid = value_is_valid(current_ma) &&
                     current_ma >= handle->config.encoder_min_ma &&
                     current_ma <= handle->config.encoder_max_ma;
  handle->status.position_valid = valid;
  if (!valid) {
    const int64_t elapsed_ms =
        (esp_timer_get_time() - handle->boot_time_us) / 1000LL;
    if ((uint32_t)elapsed_ms >= handle->config.boot_wait_position_timeout_ms) {
      set_fault(handle, CURTAIN_CONTROLLER_FAULT_ENCODER);
    }
    return ESP_ERR_INVALID_RESPONSE;
  }

  const float position =
      ((current_ma - 4.0f) / (20.0f - 4.0f)) * 100.0f;
  handle->status.position_percent = clampf_local(position, 0.0f, 100.0f);
  return ESP_OK;
}

static void update_latched_rule(bool *active, bool can_evaluate,
                                bool activate, bool release) {
  if (active == NULL) {
    return;
  }
  if (!can_evaluate) {
    *active = false;
    return;
  }
  if (*active) {
    if (release) {
      *active = false;
    }
  } else if (activate) {
    *active = true;
  }
}

static float calculate_auto_target(curtain_controller_handle_t handle,
                                   const curtain_controller_settings_t *settings,
                                   const curtain_controller_inputs_t *inputs,
                                   uint16_t *reason_bits,
                                   float *base_target_percent) {
  const bool in_schedule = schedule_active(settings, inputs);
  float target = settings->min_position_percent;

  if (reason_bits != NULL) {
    if (in_schedule) {
      *reason_bits |= CURTAIN_CONTROLLER_REASON_SCHEDULE_ACTIVE;
    } else {
      *reason_bits |= CURTAIN_CONTROLLER_REASON_OUTSIDE_SCHEDULE;
    }
    if (!inputs->time_valid) {
      *reason_bits |= CURTAIN_CONTROLLER_REASON_TIME_FAULT;
    }
  }

  if (in_schedule && settings->radiation_step_wm2 > 0U &&
      settings->radiation_step_percent > 0.0f) {
    const bool can_eval_rad = inputs->radiation_valid &&
                              value_is_valid(inputs->radiation_wm2);
    update_latched_rule(
        &handle->radiation_active, can_eval_rad,
        inputs->radiation_wm2 >= (float)settings->radiation_threshold_wm2,
        inputs->radiation_wm2 <=
            (float)((settings->radiation_threshold_wm2 >
                     settings->radiation_hysteresis_wm2)
                        ? (settings->radiation_threshold_wm2 -
                           settings->radiation_hysteresis_wm2)
                        : 0U));

    if (can_eval_rad && handle->radiation_active) {
      const float above_threshold =
          fmaxf(inputs->radiation_wm2 -
                    (float)settings->radiation_threshold_wm2,
                0.0f);
      const uint16_t step_count =
          (uint16_t)(floorf(above_threshold /
                            (float)settings->radiation_step_wm2) +
                     1.0f);
      target = settings->min_position_percent +
               ((float)step_count * settings->radiation_step_percent);
      if (reason_bits != NULL) {
        *reason_bits |= CURTAIN_CONTROLLER_REASON_RADIATION_ACTIVE;
      }
    } else if (reason_bits != NULL && !can_eval_rad) {
      *reason_bits |= CURTAIN_CONTROLLER_REASON_RADIATION_FAULT;
    }
  } else {
    handle->radiation_active = false;
  }

  target = clamp_target_percent(settings, target);
  if (base_target_percent != NULL) {
    *base_target_percent = target;
  }

  const bool can_eval_temp = inputs->air_temp_valid &&
                             inputs->temp_setpoint_valid &&
                             value_is_valid(inputs->air_temp_c) &&
                             value_is_valid(inputs->temp_setpoint_c);
  update_latched_rule(
      &handle->cold_active, can_eval_temp,
      inputs->air_temp_c <= inputs->temp_setpoint_c - settings->cold_delta_c -
                                settings->cold_hysteresis_c,
      inputs->air_temp_c >= inputs->temp_setpoint_c - settings->cold_delta_c);
  update_latched_rule(
      &handle->heat_active, can_eval_temp,
      inputs->air_temp_c >= inputs->temp_setpoint_c + settings->heat_delta_c +
                                settings->heat_hysteresis_c,
      inputs->air_temp_c <= inputs->temp_setpoint_c + settings->heat_delta_c);

  const bool can_eval_humidity = inputs->humidity_valid &&
                                 value_is_valid(inputs->humidity_percent);
  float humidity_low_threshold = settings->humidity_low_threshold_percent;
  float humidity_low_release = settings->humidity_low_threshold_percent +
                               settings->humidity_low_hysteresis_percent;
  float humidity_high_threshold = settings->humidity_high_threshold_percent;
  float humidity_high_release = settings->humidity_high_threshold_percent -
                                settings->humidity_high_hysteresis_percent;
  if (settings->humidity_setpoint_valid &&
      value_is_valid(settings->humidity_setpoint_percent)) {
    const float target =
        clampf_local(settings->humidity_setpoint_percent, 0.0f, 100.0f);
    humidity_low_threshold =
        clampf_local(target - settings->humidity_low_threshold_percent -
                         settings->humidity_low_hysteresis_percent,
                     0.0f, 100.0f);
    humidity_low_release =
        clampf_local(target - settings->humidity_low_threshold_percent, 0.0f,
                     100.0f);
    humidity_high_threshold =
        clampf_local(target + settings->humidity_high_threshold_percent +
                         settings->humidity_high_hysteresis_percent,
                     0.0f, 100.0f);
    humidity_high_release =
        clampf_local(target + settings->humidity_high_threshold_percent, 0.0f,
                     100.0f);
  }
  update_latched_rule(
      &handle->humidity_low_active, can_eval_humidity,
      inputs->humidity_percent <= humidity_low_threshold,
      inputs->humidity_percent >= humidity_low_release);
  update_latched_rule(
      &handle->humidity_high_active, can_eval_humidity,
      inputs->humidity_percent >= humidity_high_threshold,
      inputs->humidity_percent <= humidity_high_release);

  if (can_eval_temp) {
    if (handle->heat_active) {
      if (reason_bits != NULL) {
        *reason_bits |= CURTAIN_CONTROLLER_REASON_HEAT_OPEN;
      }
    }
    if (handle->cold_active) {
      if (reason_bits != NULL) {
        *reason_bits |= CURTAIN_CONTROLLER_REASON_COLD_CLOSE;
      }
    }
  } else if (reason_bits != NULL) {
    *reason_bits |= CURTAIN_CONTROLLER_REASON_TEMP_SENSOR_FAULT;
  }

  if (can_eval_humidity) {
    if (handle->humidity_low_active) {
      if (reason_bits != NULL) {
        *reason_bits |= CURTAIN_CONTROLLER_REASON_HUMIDITY_LOW_OPEN;
      }
    }
    if (handle->humidity_high_active) {
      if (reason_bits != NULL) {
        *reason_bits |= CURTAIN_CONTROLLER_REASON_HUMIDITY_HIGH_CLOSE;
      }
    }
  } else if (reason_bits != NULL) {
    *reason_bits |= CURTAIN_CONTROLLER_REASON_HUM_SENSOR_FAULT;
  }

  if (handle->humidity_low_active) {
    target = settings->humidity_low_target_percent;
  }
  if (handle->humidity_high_active) {
    target = settings->humidity_high_target_percent;
  }
  if (handle->cold_active) {
    target = settings->cold_target_percent;
  }
  if (handle->heat_active) {
    target = settings->heat_target_percent;
  }

  return clamp_target_percent(settings, target);
}

static void update_motion_fault(curtain_controller_handle_t handle,
                                curtain_output_cmd_t requested_cmd) {
  if (handle == NULL || requested_cmd == CURTAIN_OUTPUT_STOP ||
      !handle->status.position_valid ||
      handle->status.fault_code != CURTAIN_CONTROLLER_FAULT_NONE ||
      handle->config.no_motion_timeout_ms == 0U) {
    handle->motion_ref_valid = false;
    return;
  }

  const int64_t now_us = esp_timer_get_time();
  if (!handle->motion_ref_valid || requested_cmd != handle->output_cmd) {
    handle->motion_ref_valid = true;
    handle->motion_ref_position_percent = handle->status.position_percent;
    handle->motion_ref_time_us = now_us;
    return;
  }

  const float delta =
      fabsf(handle->status.position_percent - handle->motion_ref_position_percent);
  if (delta >= handle->config.motion_delta_percent) {
    handle->motion_ref_position_percent = handle->status.position_percent;
    handle->motion_ref_time_us = now_us;
    return;
  }

  const uint32_t elapsed_ms =
      (uint32_t)((now_us - handle->motion_ref_time_us) / 1000LL);
  if (elapsed_ms >= handle->config.no_motion_timeout_ms) {
    set_fault(handle, CURTAIN_CONTROLLER_FAULT_NO_MOTION);
  }
}

static void rebuild_status_bits(curtain_controller_handle_t handle,
                                curtain_controller_mode_t mode) {
  uint16_t status_bits = 0U;
  uint16_t position_status_bits = 0U;

  if (mode == CURTAIN_CONTROLLER_MODE_AUTO) {
    status_bits |= CURTAIN_CONTROLLER_STATUS_AUTO_MODE;
  } else if (mode == CURTAIN_CONTROLLER_MODE_MANUAL) {
    status_bits |= CURTAIN_CONTROLLER_STATUS_MANUAL_MODE;
  } else {
    status_bits |= CURTAIN_CONTROLLER_STATUS_OFF_MODE;
  }

  if (handle->status.position_valid) {
    status_bits |= CURTAIN_CONTROLLER_STATUS_POSITION_VALID;
    position_status_bits |= CURTAIN_CONTROLLER_POS_STATUS_VALID;
  }
  if (handle->status.moving_open) {
    status_bits |= CURTAIN_CONTROLLER_STATUS_MOVING_OPEN;
    position_status_bits |= CURTAIN_CONTROLLER_POS_STATUS_MOVING_OPEN;
  }
  if (handle->status.moving_close) {
    status_bits |= CURTAIN_CONTROLLER_STATUS_MOVING_CLOSE;
    position_status_bits |= CURTAIN_CONTROLLER_POS_STATUS_MOVING_CLOSE;
  }
  if (handle->status.at_target) {
    status_bits |= CURTAIN_CONTROLLER_STATUS_AT_TARGET;
    position_status_bits |= CURTAIN_CONTROLLER_POS_STATUS_AT_TARGET;
  }
  if (handle->status.output_open) {
    status_bits |= CURTAIN_CONTROLLER_STATUS_OUTPUT_OPEN;
  }
  if (handle->status.output_close) {
    status_bits |= CURTAIN_CONTROLLER_STATUS_OUTPUT_CLOSE;
  }
  if (handle->status.output_open || handle->status.output_close) {
    status_bits |= CURTAIN_CONTROLLER_STATUS_ENABLED;
  }
  if (handle->status.fault_code != CURTAIN_CONTROLLER_FAULT_NONE) {
    status_bits |= CURTAIN_CONTROLLER_STATUS_FAULT;
    if (handle->status.fault_code == CURTAIN_CONTROLLER_FAULT_ENCODER) {
      position_status_bits |= CURTAIN_CONTROLLER_POS_STATUS_FAULT_ENCODER;
    } else if (handle->status.fault_code == CURTAIN_CONTROLLER_FAULT_NO_MOTION) {
      position_status_bits |= CURTAIN_CONTROLLER_POS_STATUS_FAULT_NO_MOTION;
    }
  }

  handle->status.status_bits = status_bits;
  handle->status.position_status_bits = position_status_bits;
}

esp_err_t curtain_controller_init(const curtain_controller_config_t *config,
                                  curtain_controller_handle_t *ret_handle) {
  ESP_RETURN_ON_FALSE(config != NULL && ret_handle != NULL, ESP_ERR_INVALID_ARG,
                      TAG, "invalid args");
  ESP_RETURN_ON_FALSE(config->outputs != NULL, ESP_ERR_INVALID_ARG, TAG,
                      "outputs handle required");
  ESP_RETURN_ON_FALSE(config->shunt_resistor_ohm > 0.0f, ESP_ERR_INVALID_ARG,
                      TAG, "invalid shunt");
  ESP_RETURN_ON_FALSE(config->open_bit_index != config->close_bit_index,
                      ESP_ERR_INVALID_ARG, TAG, "invalid output bit map");

  curtain_controller_handle_t handle = calloc(1, sizeof(*handle));
  if (handle == NULL) {
    return ESP_ERR_NO_MEM;
  }

  memcpy(&handle->config, config, sizeof(handle->config));
  if (handle->config.encoder_min_ma <= 0.0f) {
    handle->config.encoder_min_ma = 3.6f;
  }
  if (handle->config.encoder_max_ma <= handle->config.encoder_min_ma) {
    handle->config.encoder_max_ma = 20.5f;
  }
  if (handle->config.motion_delta_percent <= 0.0f) {
    handle->config.motion_delta_percent = 0.5f;
  }
  if (handle->config.boot_wait_position_timeout_ms == 0U) {
    handle->config.boot_wait_position_timeout_ms = 1000U;
  }

  ads1115_config_t ads_config = {
      .i2c_port = config->i2c_port,
      .i2c_addr = config->ads_addr,
  };
  esp_err_t err = ads1115_init(&ads_config, &handle->ads_handle);
  if (err != ESP_OK) {
    free(handle);
    return err;
  }

  handle->boot_time_us = esp_timer_get_time();
  err = apply_output(handle, CURTAIN_OUTPUT_STOP);
  if (err != ESP_OK) {
    ads1115_del(handle->ads_handle);
    free(handle);
    return err;
  }

  *ret_handle = handle;
  ESP_LOGI(TAG, "%s initialized on ADS 0x%02X diff %u-%u, outputs %u/%u",
           config->name ? config->name : "curtain", config->ads_addr,
           (unsigned)config->ads_channel_pos, (unsigned)config->ads_channel_neg,
           (unsigned)config->open_bit_index, (unsigned)config->close_bit_index);
  return ESP_OK;
}

esp_err_t curtain_controller_process(
    curtain_controller_handle_t handle,
    const curtain_controller_settings_t *settings,
    const curtain_controller_inputs_t *inputs) {
  ESP_RETURN_ON_FALSE(handle != NULL && settings != NULL && inputs != NULL,
                      ESP_ERR_INVALID_ARG, TAG, "invalid args");

  curtain_controller_settings_t cfg = *settings;
  cfg.position_hysteresis_percent =
      clampf_local(cfg.position_hysteresis_percent, 0.1f, 20.0f);
  cfg.min_position_percent = clampf_local(cfg.min_position_percent, 0.0f, 100.0f);
  cfg.max_position_percent =
      clampf_local(cfg.max_position_percent, cfg.min_position_percent, 100.0f);

  (void)read_position(handle);

  handle->status.reason_bits = 0U;
  handle->status.moving_open = false;
  handle->status.moving_close = false;
  handle->status.at_target = false;

  curtain_output_cmd_t requested_cmd = CURTAIN_OUTPUT_STOP;
  if (handle->status.fault_code != CURTAIN_CONTROLLER_FAULT_NONE ||
      !handle->status.position_valid || cfg.mode == CURTAIN_CONTROLLER_MODE_OFF) {
    handle->status.target_percent = handle->status.position_percent;
    handle->status.base_target_percent = handle->status.position_percent;
  } else {
    if (cfg.mode == CURTAIN_CONTROLLER_MODE_MANUAL) {
      handle->status.reason_bits |= CURTAIN_CONTROLLER_REASON_MANUAL;
      handle->status.base_target_percent =
          clampf_local(cfg.manual_target_percent, cfg.min_position_percent,
                       cfg.max_position_percent);
      handle->status.target_percent = handle->status.base_target_percent;
    } else {
      handle->status.target_percent =
          calculate_auto_target(handle, &cfg, inputs, &handle->status.reason_bits,
                                &handle->status.base_target_percent);
    }

    if (handle->status.position_percent <
        handle->status.target_percent - cfg.position_hysteresis_percent) {
      requested_cmd = CURTAIN_OUTPUT_OPEN;
      handle->status.moving_open = true;
    } else if (handle->status.position_percent >
               handle->status.target_percent + cfg.position_hysteresis_percent) {
      requested_cmd = CURTAIN_OUTPUT_CLOSE;
      handle->status.moving_close = true;
    } else {
      handle->status.at_target = true;
    }
  }

  update_motion_fault(handle, requested_cmd);
  if (handle->status.fault_code != CURTAIN_CONTROLLER_FAULT_NONE) {
    requested_cmd = CURTAIN_OUTPUT_STOP;
    handle->status.moving_open = false;
    handle->status.moving_close = false;
  }

  esp_err_t err = apply_output(handle, requested_cmd);
  rebuild_status_bits(handle, cfg.mode);

  const int64_t now_us = esp_timer_get_time();
  if (handle->last_status_log_us == 0 ||
      now_us - handle->last_status_log_us >= CURTAIN_CONTROLLER_LOG_PERIOD_US) {
    ESP_LOGI(TAG,
             "Curtain mode=%u pos=%.1f%% target=%.1f%% current=%.1fmA out=%u fault=%u reason=0x%X motion=%.1f%% nomotion_ms=%lu",
             (unsigned)cfg.mode, handle->status.position_percent,
             handle->status.target_percent, handle->status.current_ma,
             (unsigned)requested_cmd, (unsigned)handle->status.fault_code,
             (unsigned)handle->status.reason_bits,
             handle->config.motion_delta_percent,
             (unsigned long)handle->config.no_motion_timeout_ms);
    handle->last_status_log_us = now_us;
  }
  return err;
}

esp_err_t curtain_controller_get_status(
    curtain_controller_handle_t handle, curtain_controller_status_t *out_status) {
  ESP_RETURN_ON_FALSE(handle != NULL && out_status != NULL, ESP_ERR_INVALID_ARG,
                      TAG, "invalid args");
  *out_status = handle->status;
  return ESP_OK;
}

esp_err_t curtain_controller_set_motion_fault_config(
    curtain_controller_handle_t handle, float motion_delta_percent,
    uint32_t no_motion_timeout_ms) {
  ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG,
                      "invalid handle");
  if (!isfinite(motion_delta_percent) || motion_delta_percent < 0.0f) {
    return ESP_ERR_INVALID_ARG;
  }

  handle->config.motion_delta_percent = motion_delta_percent;
  handle->config.no_motion_timeout_ms = no_motion_timeout_ms;
  if (no_motion_timeout_ms == 0U &&
      handle->status.fault_code == CURTAIN_CONTROLLER_FAULT_NO_MOTION) {
    handle->status.fault_code = CURTAIN_CONTROLLER_FAULT_NONE;
    handle->motion_ref_valid = false;
  }
  return ESP_OK;
}

esp_err_t curtain_controller_stop(curtain_controller_handle_t handle) {
  ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "invalid handle");
  esp_err_t err = apply_output(handle, CURTAIN_OUTPUT_STOP);
  handle->status.moving_open = false;
  handle->status.moving_close = false;
  rebuild_status_bits(handle, CURTAIN_CONTROLLER_MODE_OFF);
  return err;
}

esp_err_t curtain_controller_reset_fault(curtain_controller_handle_t handle) {
  ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "invalid handle");
  handle->status.fault_code = CURTAIN_CONTROLLER_FAULT_NONE;
  handle->motion_ref_valid = false;
  handle->boot_time_us = esp_timer_get_time();
  return apply_output(handle, CURTAIN_OUTPUT_STOP);
}

esp_err_t curtain_controller_del(curtain_controller_handle_t handle) {
  if (handle == NULL) {
    return ESP_OK;
  }
  (void)apply_output(handle, CURTAIN_OUTPUT_STOP);
  ads1115_del(handle->ads_handle);
  free(handle);
  return ESP_OK;
}
