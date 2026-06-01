#include "co2_controller.h"

#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

static const char *TAG = "co2_controller";

struct co2_controller_ctx_t {
  co2_controller_config_t config;
  co2_controller_status_t status;
  bool dosing_active;
  int64_t dosing_started_us;
  int64_t last_dosing_stop_us;
  uint16_t dosing_start_ppm;
};

static bool hhmm_valid(uint16_t hhmm) {
  return (hhmm / 100U) < 24U && (hhmm % 100U) < 60U;
}

static uint16_t hhmm_to_minutes(uint16_t hhmm) {
  return (uint16_t)(((hhmm / 100U) * 60U) + (hhmm % 100U));
}

static bool schedule_active(const co2_controller_settings_t *settings,
                            const co2_controller_inputs_t *inputs) {
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

static uint16_t clamp_u16(uint16_t value, uint16_t min_value,
                          uint16_t max_value) {
  if (value < min_value) {
    return min_value;
  }
  if (value > max_value) {
    return max_value;
  }
  return value;
}

static uint16_t sanitize_target(uint16_t target_ppm, uint16_t max_safe_ppm) {
  const uint16_t max_target = max_safe_ppm > 0U ? max_safe_ppm : 1200U;
  return clamp_u16(target_ppm, 350U, max_target);
}

static float max_window_open_percent(const co2_controller_inputs_t *inputs) {
  if (inputs == NULL || !inputs->ventilation_valid) {
    return 100.0f;
  }
  float value = 0.0f;
  if (inputs->window_a_valid && isfinite(inputs->window_a_percent)) {
    value = fmaxf(value, inputs->window_a_percent);
  }
  if (inputs->window_b_valid && isfinite(inputs->window_b_percent)) {
    value = fmaxf(value, inputs->window_b_percent);
  }
  return fminf(fmaxf(value, 0.0f), 100.0f);
}

static void set_fault(co2_controller_status_t *status,
                      co2_controller_fault_code_t fault_code) {
  if (status == NULL || status->fault_code != CO2_CONTROLLER_FAULT_NONE) {
    return;
  }
  status->fault_code = fault_code;
}

esp_err_t co2_controller_init(const co2_controller_config_t *config,
                              co2_controller_handle_t *ret_handle) {
  ESP_RETURN_ON_FALSE(config != NULL && ret_handle != NULL, ESP_ERR_INVALID_ARG,
                      TAG, "invalid args");

  co2_controller_handle_t handle =
      (co2_controller_handle_t)calloc(1, sizeof(*handle));
  ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_NO_MEM, TAG, "no memory");

  handle->config = *config;
  handle->last_dosing_stop_us = esp_timer_get_time();
  *ret_handle = handle;
  return ESP_OK;
}

esp_err_t co2_controller_process(co2_controller_handle_t handle,
                                 const co2_controller_settings_t *settings,
                                 const co2_controller_inputs_t *inputs) {
  ESP_RETURN_ON_FALSE(handle != NULL && settings != NULL && inputs != NULL,
                      ESP_ERR_INVALID_ARG, TAG, "invalid args");

  co2_controller_status_t status = handle->status;
  status.target_ppm = 0U;
  status.effective_target_ppm = 0U;
  status.measured_ppm = inputs->co2_ppm;
  status.valve_open = false;
  status.mixing_requested = false;
  status.status_bits = 0U;
  status.reason_bits = 0U;
  status.active_protection_bits = 0U;

  const int64_t now_us = esp_timer_get_time();
  if (handle->dosing_started_us > 0) {
    status.dosing_elapsed_s =
        (uint16_t)((now_us - handle->dosing_started_us) / 1000000LL);
  } else {
    status.dosing_elapsed_s = 0U;
  }
  if (handle->last_dosing_stop_us > 0) {
    status.pause_elapsed_s =
        (uint16_t)((now_us - handle->last_dosing_stop_us) / 1000000LL);
  } else {
    status.pause_elapsed_s = UINT16_MAX;
  }

  if (settings->mode == CO2_CONTROLLER_MODE_OFF) {
    status.status_bits |= CO2_CONTROLLER_STATUS_OFF_MODE;
    if (handle->dosing_active) {
      handle->last_dosing_stop_us = now_us;
    }
    handle->dosing_active = false;
    handle->dosing_started_us = 0;
    handle->status = status;
    return ESP_OK;
  }

  if (settings->mode == CO2_CONTROLLER_MODE_MANUAL) {
    status.status_bits |= CO2_CONTROLLER_STATUS_MANUAL_MODE;
    status.valve_open = settings->manual_valve_open &&
                        status.fault_code == CO2_CONTROLLER_FAULT_NONE;
    status.mixing_requested = settings->manual_fan_on || status.valve_open;
    if (status.valve_open) {
      status.status_bits |= CO2_CONTROLLER_STATUS_ENABLED |
                            CO2_CONTROLLER_STATUS_VALVE_OPEN;
    }
    if (status.mixing_requested) {
      status.status_bits |= CO2_CONTROLLER_STATUS_MIX_REQUEST;
    }
    if (handle->dosing_active && !status.valve_open) {
      handle->last_dosing_stop_us = now_us;
    }
    if (!handle->dosing_active && status.valve_open) {
      handle->dosing_started_us = now_us;
      handle->dosing_start_ppm = inputs->co2_ppm;
    }
    handle->dosing_active = status.valve_open;
    handle->status = status;
    return ESP_OK;
  }

  status.status_bits |= CO2_CONTROLLER_STATUS_AUTO_MODE;
  if (inputs->co2_valid) {
    status.status_bits |= CO2_CONTROLLER_STATUS_SENSOR_VALID;
  }

  bool allowed = true;
  const bool in_schedule = schedule_active(settings, inputs);
  if (in_schedule) {
    status.reason_bits |= CO2_CONTROLLER_REASON_SCHEDULE_ACTIVE;
  } else {
    status.reason_bits |= CO2_CONTROLLER_REASON_OUTSIDE_SCHEDULE;
    status.active_protection_bits |= CO2_CONTROLLER_PROTECTION_TIME_OR_NIGHT;
    allowed = false;
  }
  if (!inputs->time_valid) {
    status.reason_bits |= CO2_CONTROLLER_REASON_TIME_FAULT;
  }

  if (inputs->external_safety_alarm) {
    status.reason_bits |= CO2_CONTROLLER_REASON_EXTERNAL_ALARM;
    status.active_protection_bits |= CO2_CONTROLLER_PROTECTION_EXTERNAL_ALARM;
    set_fault(&status, CO2_CONTROLLER_FAULT_EXTERNAL_SAFETY);
    allowed = false;
  }

  if (!inputs->co2_valid) {
    status.reason_bits |= CO2_CONTROLLER_REASON_SENSOR_FAULT;
    status.active_protection_bits |= CO2_CONTROLLER_PROTECTION_SENSOR_FAULT;
    set_fault(&status, CO2_CONTROLLER_FAULT_SENSOR);
    allowed = false;
  } else if (settings->max_safe_ppm > 0U &&
             inputs->co2_ppm >= settings->max_safe_ppm) {
    status.active_protection_bits |= CO2_CONTROLLER_PROTECTION_OVERRANGE;
    set_fault(&status, CO2_CONTROLLER_FAULT_OVERRANGE);
    allowed = false;
  }

  uint16_t target_ppm = settings->low_light_target_ppm;
  if (inputs->radiation_valid && isfinite(inputs->radiation_wm2)) {
    if (inputs->radiation_wm2 >= (float)settings->high_light_threshold_wm2) {
      target_ppm = settings->high_light_target_ppm;
      status.reason_bits |= CO2_CONTROLLER_REASON_LIGHT_HIGH;
    } else if (inputs->radiation_wm2 >=
               (float)settings->mid_light_threshold_wm2) {
      target_ppm = settings->mid_light_target_ppm;
      status.reason_bits |= CO2_CONTROLLER_REASON_LIGHT_MID;
    } else if (inputs->radiation_wm2 >=
               (float)settings->low_light_threshold_wm2) {
      target_ppm = settings->low_light_target_ppm;
      status.reason_bits |= CO2_CONTROLLER_REASON_LIGHT_LOW;
    } else {
      target_ppm = 420U;
      status.reason_bits |= CO2_CONTROLLER_REASON_LIGHT_LOW;
      allowed = false;
    }
  } else {
    status.reason_bits |= CO2_CONTROLLER_REASON_LIGHT_FAULT;
    target_ppm = 420U;
    allowed = false;
  }

  const float vent_percent = max_window_open_percent(inputs);
  if (vent_percent >= (float)settings->ventilation_cutoff_percent) {
    status.reason_bits |= CO2_CONTROLLER_REASON_VENT_BLOCKED;
    status.active_protection_bits |= CO2_CONTROLLER_PROTECTION_VENT_CUTOFF;
    allowed = false;
  } else if (vent_percent >= (float)settings->ventilation_limit_high_percent) {
    status.reason_bits |= CO2_CONTROLLER_REASON_VENT_LIMITED;
    target_ppm = (uint16_t)fminf((float)target_ppm, 500.0f);
  } else if (vent_percent >= (float)settings->ventilation_limit_low_percent) {
    status.reason_bits |= CO2_CONTROLLER_REASON_VENT_LIMITED;
    target_ppm = (uint16_t)fminf((float)target_ppm, 650.0f);
  }

  if (inputs->air_temp_valid && inputs->air_temp_target_valid &&
      isfinite(inputs->air_temp_c) && isfinite(inputs->air_temp_target_c)) {
    const float delta = inputs->air_temp_c - inputs->air_temp_target_c;
    if (delta >= settings->temp_critical_delta_c) {
      status.reason_bits |= CO2_CONTROLLER_REASON_TEMP_CRITICAL;
      status.active_protection_bits |= CO2_CONTROLLER_PROTECTION_TEMP_CRITICAL;
      allowed = false;
    } else if (delta >= settings->temp_high_delta_c) {
      status.reason_bits |= CO2_CONTROLLER_REASON_TEMP_HIGH;
      status.active_protection_bits |= CO2_CONTROLLER_PROTECTION_TEMP_HIGH;
      target_ppm = (uint16_t)fminf((float)target_ppm, 500.0f);
    }
  }

  if (inputs->humidity_valid && inputs->humidity_target_valid &&
      isfinite(inputs->humidity_percent) &&
      isfinite(inputs->humidity_target_percent) &&
      inputs->humidity_percent >=
          inputs->humidity_target_percent + settings->humidity_high_delta_percent) {
    status.reason_bits |= CO2_CONTROLLER_REASON_HUMIDITY_HIGH;
    status.active_protection_bits |= CO2_CONTROLLER_PROTECTION_HUMIDITY_HIGH;
    allowed = false;
  }

  target_ppm = sanitize_target(target_ppm, settings->max_safe_ppm);
  status.target_ppm = target_ppm;
  status.effective_target_ppm = allowed ? target_ppm : 0U;

  const uint16_t hyst = settings->dosing_hysteresis_ppm > 0U
                            ? settings->dosing_hysteresis_ppm
                            : 50U;
  const uint16_t open_below =
      target_ppm > hyst ? (uint16_t)(target_ppm - hyst) : 0U;
  const uint16_t close_above = (uint16_t)(target_ppm + hyst);

  bool request_dosing = allowed &&
                        status.fault_code == CO2_CONTROLLER_FAULT_NONE &&
                        inputs->co2_valid && inputs->co2_ppm <= open_below;
  if (handle->dosing_active && allowed && inputs->co2_valid &&
      inputs->co2_ppm < close_above &&
      status.fault_code == CO2_CONTROLLER_FAULT_NONE) {
    request_dosing = true;
  }

  if (!handle->dosing_active && request_dosing &&
      status.pause_elapsed_s < settings->min_pause_time_s) {
    status.active_protection_bits |= CO2_CONTROLLER_PROTECTION_MIN_PAUSE;
    status.status_bits |= CO2_CONTROLLER_STATUS_PAUSE_HOLD;
    request_dosing = false;
  }

  if (handle->dosing_active) {
    if (settings->max_dosing_time_s > 0U &&
        status.dosing_elapsed_s >= settings->max_dosing_time_s) {
      status.active_protection_bits |= CO2_CONTROLLER_PROTECTION_MAX_DOSING;
      request_dosing = false;
    }
    if (settings->no_rise_check_time_s > 0U &&
        status.dosing_elapsed_s >= settings->no_rise_check_time_s &&
        inputs->co2_ppm <
            (uint16_t)(handle->dosing_start_ppm +
                       settings->no_rise_min_delta_ppm)) {
      status.active_protection_bits |= CO2_CONTROLLER_PROTECTION_NO_RISE;
      set_fault(&status, CO2_CONTROLLER_FAULT_NO_RISE);
      request_dosing = false;
    }
  }

  if (request_dosing && !handle->dosing_active) {
    handle->dosing_active = true;
    handle->dosing_started_us = now_us;
    handle->dosing_start_ppm = inputs->co2_ppm;
    status.dosing_elapsed_s = 0U;
  } else if (!request_dosing && handle->dosing_active) {
    handle->dosing_active = false;
    handle->dosing_started_us = 0;
    handle->last_dosing_stop_us = now_us;
    status.pause_elapsed_s = 0U;
  }

  status.valve_open = handle->dosing_active;
  status.mixing_requested = handle->dosing_active;
  if (status.valve_open) {
    status.status_bits |= CO2_CONTROLLER_STATUS_ENABLED |
                          CO2_CONTROLLER_STATUS_VALVE_OPEN;
  }
  if (status.mixing_requested) {
    status.status_bits |= CO2_CONTROLLER_STATUS_MIX_REQUEST;
  }
  if (status.fault_code != CO2_CONTROLLER_FAULT_NONE) {
    status.status_bits |= CO2_CONTROLLER_STATUS_FAULT;
  }
  if (handle->dosing_active && settings->max_dosing_time_s > 0U) {
    status.status_bits |= CO2_CONTROLLER_STATUS_DOSING_HOLD;
  }

  handle->status = status;
  return ESP_OK;
}

esp_err_t co2_controller_get_status(co2_controller_handle_t handle,
                                    co2_controller_status_t *out_status) {
  ESP_RETURN_ON_FALSE(handle != NULL && out_status != NULL, ESP_ERR_INVALID_ARG,
                      TAG, "invalid args");
  *out_status = handle->status;
  return ESP_OK;
}

esp_err_t co2_controller_reset_fault(co2_controller_handle_t handle) {
  ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "invalid handle");
  handle->status.fault_code = CO2_CONTROLLER_FAULT_NONE;
  return ESP_OK;
}

esp_err_t co2_controller_del(co2_controller_handle_t handle) {
  free(handle);
  return ESP_OK;
}
