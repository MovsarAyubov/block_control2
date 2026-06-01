#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct circulation_controller_ctx_t *circulation_controller_handle_t;

typedef enum {
  CIRCULATION_CONTROLLER_MODE_AUTO = 0,
  CIRCULATION_CONTROLLER_MODE_OFF = 1,
  CIRCULATION_CONTROLLER_MODE_MANUAL = 2,
} circulation_controller_mode_t;

typedef struct {
  const char *name;
  uint16_t fan_mask;
} circulation_controller_config_t;

typedef struct {
  circulation_controller_mode_t mode;
  uint16_t manual_fan_mask;
  uint16_t available_fan_mask;
  uint16_t schedule_start_hhmm;
  uint16_t schedule_end_hhmm;
  uint16_t co2_fan_mask;
  uint16_t heating_fan_mask;
  uint16_t humidity_fan_mask;
  uint16_t day_fan_mask;
  uint16_t night_fan_mask;
  uint16_t vent_limited_fan_mask;
  uint16_t ventilation_limit_percent;
  uint16_t ventilation_cutoff_percent;
  uint16_t humidity_high_delta_percent;
  uint16_t day_cycle_on_s;
  uint16_t day_cycle_off_s;
  uint16_t night_cycle_on_s;
  uint16_t night_cycle_off_s;
  uint16_t humidity_cycle_on_s;
  uint16_t humidity_cycle_off_s;
  uint16_t min_on_s;
  uint16_t min_off_s;
} circulation_controller_settings_t;

typedef struct {
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
  bool time_valid;
  bool co2_dosing_active;
  bool heating_active;
  float humidity_percent;
  bool humidity_valid;
  float humidity_target_percent;
  bool humidity_target_valid;
  float window_a_percent;
  bool window_a_valid;
  float window_b_percent;
  bool window_b_valid;
  bool ventilation_valid;
  float curtain_percent;
  bool curtain_valid;
} circulation_controller_inputs_t;

typedef struct {
  uint16_t requested_fan_mask;
  uint16_t output_fan_mask;
  uint16_t status_bits;
  uint16_t reason_bits;
  uint16_t protection_bits;
  uint16_t cycle_elapsed_s;
} circulation_controller_status_t;

#define CIRCULATION_CONTROLLER_STATUS_ENABLED (1U << 0)
#define CIRCULATION_CONTROLLER_STATUS_AUTO_MODE (1U << 1)
#define CIRCULATION_CONTROLLER_STATUS_OFF_MODE (1U << 2)
#define CIRCULATION_CONTROLLER_STATUS_MANUAL_MODE (1U << 3)
#define CIRCULATION_CONTROLLER_STATUS_CYCLE_ON (1U << 4)
#define CIRCULATION_CONTROLLER_STATUS_MIN_ON_HOLD (1U << 5)
#define CIRCULATION_CONTROLLER_STATUS_MIN_OFF_HOLD (1U << 6)
#define CIRCULATION_CONTROLLER_STATUS_TIME_VALID (1U << 7)

#define CIRCULATION_CONTROLLER_REASON_CO2_DOSING (1U << 0)
#define CIRCULATION_CONTROLLER_REASON_HEATING (1U << 1)
#define CIRCULATION_CONTROLLER_REASON_HUMIDITY_HIGH (1U << 2)
#define CIRCULATION_CONTROLLER_REASON_DAY_BASE (1U << 3)
#define CIRCULATION_CONTROLLER_REASON_NIGHT_CYCLE (1U << 4)
#define CIRCULATION_CONTROLLER_REASON_MANUAL (1U << 5)
#define CIRCULATION_CONTROLLER_REASON_OUTSIDE_SCHEDULE (1U << 6)
#define CIRCULATION_CONTROLLER_REASON_TIME_FAULT (1U << 7)
#define CIRCULATION_CONTROLLER_REASON_HUMIDITY_FAULT (1U << 8)

#define CIRCULATION_CONTROLLER_PROTECTION_VENT_LIMITED (1U << 0)
#define CIRCULATION_CONTROLLER_PROTECTION_VENT_CUTOFF (1U << 1)
#define CIRCULATION_CONTROLLER_PROTECTION_UNKNOWN_VENT (1U << 2)
#define CIRCULATION_CONTROLLER_PROTECTION_MIN_ON (1U << 3)
#define CIRCULATION_CONTROLLER_PROTECTION_MIN_OFF (1U << 4)

esp_err_t circulation_controller_init(
    const circulation_controller_config_t *config,
    circulation_controller_handle_t *ret_handle);
esp_err_t circulation_controller_process(
    circulation_controller_handle_t handle,
    const circulation_controller_settings_t *settings,
    const circulation_controller_inputs_t *inputs);
esp_err_t circulation_controller_get_status(
    circulation_controller_handle_t handle,
    circulation_controller_status_t *out_status);
esp_err_t circulation_controller_del(circulation_controller_handle_t handle);

#ifdef __cplusplus
}
#endif
