#include "circulation_controller.h"

#include "esp_check.h"
#include "esp_timer.h"
#include <math.h>
#include <stdlib.h>

static const char *TAG = "circulation_controller";

struct circulation_controller_ctx_t {
  circulation_controller_config_t config;
  circulation_controller_status_t status;
  bool cycle_on;
  int64_t cycle_phase_started_us;
  int64_t output_changed_us;
};

static bool hhmm_valid(uint16_t hhmm) {
  return (hhmm / 100U) < 24U && (hhmm % 100U) < 60U;
}

static uint16_t hhmm_to_minutes(uint16_t hhmm) {
  return (uint16_t)(((hhmm / 100U) * 60U) + (hhmm % 100U));
}

static bool schedule_active(const circulation_controller_settings_t *settings,
                            const circulation_controller_inputs_t *inputs) {
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

static float max_window_open_percent(
    const circulation_controller_inputs_t *inputs) {
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

static bool cycle_output(circulation_controller_handle_t handle,
                         uint16_t on_s, uint16_t off_s, int64_t now_us,
                         uint16_t *elapsed_s) {
  if (on_s == 0U) {
    if (elapsed_s != NULL) {
      *elapsed_s = 0U;
    }
    return false;
  }
  if (off_s == 0U) {
    if (elapsed_s != NULL) {
      *elapsed_s = 0U;
    }
    return true;
  }
  if (handle->cycle_phase_started_us == 0) {
    handle->cycle_phase_started_us = now_us;
    handle->cycle_on = true;
  }

  const uint16_t phase_elapsed_s =
      (uint16_t)((now_us - handle->cycle_phase_started_us) / 1000000LL);
  const uint16_t phase_limit_s = handle->cycle_on ? on_s : off_s;
  if (phase_elapsed_s >= phase_limit_s) {
    handle->cycle_on = !handle->cycle_on;
    handle->cycle_phase_started_us = now_us;
    if (elapsed_s != NULL) {
      *elapsed_s = 0U;
    }
    return handle->cycle_on;
  }

  if (elapsed_s != NULL) {
    *elapsed_s = phase_elapsed_s;
  }
  return handle->cycle_on;
}

esp_err_t circulation_controller_init(
    const circulation_controller_config_t *config,
    circulation_controller_handle_t *ret_handle) {
  ESP_RETURN_ON_FALSE(config != NULL && ret_handle != NULL, ESP_ERR_INVALID_ARG,
                      TAG, "invalid args");
  circulation_controller_handle_t handle =
      (circulation_controller_handle_t)calloc(1, sizeof(*handle));
  ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_NO_MEM, TAG, "no memory");
  handle->config = *config;
  handle->cycle_on = true;
  handle->cycle_phase_started_us = esp_timer_get_time();
  handle->output_changed_us = handle->cycle_phase_started_us - 3600000000LL;
  *ret_handle = handle;
  return ESP_OK;
}

esp_err_t circulation_controller_process(
    circulation_controller_handle_t handle,
    const circulation_controller_settings_t *settings,
    const circulation_controller_inputs_t *inputs) {
  ESP_RETURN_ON_FALSE(handle != NULL && settings != NULL && inputs != NULL,
                      ESP_ERR_INVALID_ARG, TAG, "invalid args");

  const int64_t now_us = esp_timer_get_time();
  circulation_controller_status_t status = {0};
  if (inputs->time_valid) {
    status.status_bits |= CIRCULATION_CONTROLLER_STATUS_TIME_VALID;
  }

  const uint16_t available_mask =
      settings->available_fan_mask != 0U ? settings->available_fan_mask
                                         : handle->config.fan_mask;

  uint16_t requested = 0U;
  const bool force_off = settings->mode == CIRCULATION_CONTROLLER_MODE_OFF;
  if (settings->mode == CIRCULATION_CONTROLLER_MODE_OFF) {
    status.status_bits |= CIRCULATION_CONTROLLER_STATUS_OFF_MODE;
  } else if (settings->mode == CIRCULATION_CONTROLLER_MODE_MANUAL) {
    status.status_bits |= CIRCULATION_CONTROLLER_STATUS_MANUAL_MODE;
    status.reason_bits |= CIRCULATION_CONTROLLER_REASON_MANUAL;
    requested = settings->manual_fan_mask;
  } else {
    status.status_bits |= CIRCULATION_CONTROLLER_STATUS_AUTO_MODE;

    const bool in_schedule = schedule_active(settings, inputs);
    if (!inputs->time_valid) {
      status.reason_bits |= CIRCULATION_CONTROLLER_REASON_TIME_FAULT;
    }
    if (!in_schedule) {
      status.reason_bits |= CIRCULATION_CONTROLLER_REASON_OUTSIDE_SCHEDULE;
    }

    bool continuous = false;
    uint16_t cycle_mask = 0U;
    uint16_t cycle_on_s = 0U;
    uint16_t cycle_off_s = 0U;

    if (inputs->co2_dosing_active) {
      requested |= settings->co2_fan_mask;
      continuous = true;
      status.reason_bits |= CIRCULATION_CONTROLLER_REASON_CO2_DOSING;
    }
    if (inputs->heating_active) {
      requested |= settings->heating_fan_mask;
      continuous = true;
      status.reason_bits |= CIRCULATION_CONTROLLER_REASON_HEATING;
    }

    const bool humidity_valid = inputs->humidity_valid &&
                                inputs->humidity_target_valid &&
                                isfinite(inputs->humidity_percent) &&
                                isfinite(inputs->humidity_target_percent);
    const bool humidity_high =
        humidity_valid &&
        inputs->humidity_percent >=
            inputs->humidity_target_percent +
                ((float)settings->humidity_high_delta_percent / 10.0f);
    if (humidity_high) {
      cycle_mask |= settings->humidity_fan_mask;
      cycle_on_s = settings->humidity_cycle_on_s;
      cycle_off_s = settings->humidity_cycle_off_s;
      status.reason_bits |= CIRCULATION_CONTROLLER_REASON_HUMIDITY_HIGH;
    } else if (!humidity_valid) {
      status.reason_bits |= CIRCULATION_CONTROLLER_REASON_HUMIDITY_FAULT;
    }

    if (!continuous && cycle_mask == 0U) {
      if (in_schedule) {
        cycle_mask = settings->day_fan_mask;
        cycle_on_s = settings->day_cycle_on_s;
        cycle_off_s = settings->day_cycle_off_s;
        status.reason_bits |= CIRCULATION_CONTROLLER_REASON_DAY_BASE;
      } else {
        cycle_mask = settings->night_fan_mask;
        cycle_on_s = settings->night_cycle_on_s;
        cycle_off_s = settings->night_cycle_off_s;
        status.reason_bits |= CIRCULATION_CONTROLLER_REASON_NIGHT_CYCLE;
      }
    }

    if (cycle_mask != 0U) {
      const bool cycle_on =
          cycle_output(handle, cycle_on_s, cycle_off_s, now_us,
                       &status.cycle_elapsed_s);
      if (cycle_on) {
        requested |= cycle_mask;
        status.status_bits |= CIRCULATION_CONTROLLER_STATUS_CYCLE_ON;
      }
    } else {
      handle->cycle_phase_started_us = now_us;
      handle->cycle_on = true;
    }

    const float vent_percent = max_window_open_percent(inputs);
    if (!inputs->ventilation_valid) {
      status.protection_bits |= CIRCULATION_CONTROLLER_PROTECTION_UNKNOWN_VENT;
      requested &= settings->vent_limited_fan_mask;
    } else if (vent_percent >= (float)settings->ventilation_cutoff_percent) {
      status.protection_bits |= CIRCULATION_CONTROLLER_PROTECTION_VENT_CUTOFF;
      requested &= settings->vent_limited_fan_mask;
    } else if (vent_percent >=
               (float)settings->ventilation_limit_percent) {
      status.protection_bits |= CIRCULATION_CONTROLLER_PROTECTION_VENT_LIMITED;
      requested &= settings->vent_limited_fan_mask;
    }
  }

  requested &= available_mask;
  status.requested_fan_mask = requested;

  uint16_t output = requested;
  const bool prev_on = handle->status.output_fan_mask != 0U;
  const bool next_on = output != 0U;
  const uint16_t elapsed_since_change_s =
      (uint16_t)((now_us - handle->output_changed_us) / 1000000LL);
  if (!force_off && prev_on && !next_on &&
      elapsed_since_change_s < settings->min_on_s) {
    output = handle->status.output_fan_mask;
    status.status_bits |= CIRCULATION_CONTROLLER_STATUS_MIN_ON_HOLD;
    status.protection_bits |= CIRCULATION_CONTROLLER_PROTECTION_MIN_ON;
  } else if (!prev_on && next_on &&
             elapsed_since_change_s < settings->min_off_s) {
    output = 0U;
    status.status_bits |= CIRCULATION_CONTROLLER_STATUS_MIN_OFF_HOLD;
    status.protection_bits |= CIRCULATION_CONTROLLER_PROTECTION_MIN_OFF;
  }

  if (output != handle->status.output_fan_mask) {
    handle->output_changed_us = now_us;
  }
  status.output_fan_mask = output;
  if (output != 0U) {
    status.status_bits |= CIRCULATION_CONTROLLER_STATUS_ENABLED;
  }

  handle->status = status;
  return ESP_OK;
}

esp_err_t circulation_controller_get_status(
    circulation_controller_handle_t handle,
    circulation_controller_status_t *out_status) {
  ESP_RETURN_ON_FALSE(handle != NULL && out_status != NULL, ESP_ERR_INVALID_ARG,
                      TAG, "invalid args");
  *out_status = handle->status;
  return ESP_OK;
}

esp_err_t circulation_controller_del(circulation_controller_handle_t handle) {
  free(handle);
  return ESP_OK;
}
