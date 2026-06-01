#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct co2_controller_ctx_t *co2_controller_handle_t;

typedef enum {
  CO2_CONTROLLER_MODE_AUTO = 0,
  CO2_CONTROLLER_MODE_OFF = 1,
  CO2_CONTROLLER_MODE_MANUAL = 2,
} co2_controller_mode_t;

typedef enum {
  CO2_CONTROLLER_FAULT_NONE = 0,
  CO2_CONTROLLER_FAULT_SENSOR = 1,
  CO2_CONTROLLER_FAULT_OVERRANGE = 2,
  CO2_CONTROLLER_FAULT_NO_RISE = 3,
  CO2_CONTROLLER_FAULT_EXTERNAL_SAFETY = 4,
} co2_controller_fault_code_t;

typedef struct {
  const char *name;
} co2_controller_config_t;

typedef struct {
  co2_controller_mode_t mode;
  bool manual_valve_open;
  bool manual_fan_on;
  uint16_t schedule_start_hhmm;
  uint16_t schedule_end_hhmm;
  uint16_t low_light_threshold_wm2;
  uint16_t mid_light_threshold_wm2;
  uint16_t high_light_threshold_wm2;
  uint16_t low_light_target_ppm;
  uint16_t mid_light_target_ppm;
  uint16_t high_light_target_ppm;
  uint16_t ventilation_limit_low_percent;
  uint16_t ventilation_limit_high_percent;
  uint16_t ventilation_cutoff_percent;
  uint16_t dosing_hysteresis_ppm;
  uint16_t max_safe_ppm;
  uint16_t max_dosing_time_s;
  uint16_t min_pause_time_s;
  uint16_t no_rise_check_time_s;
  uint16_t no_rise_min_delta_ppm;
  float temp_high_delta_c;
  float temp_critical_delta_c;
  float humidity_high_delta_percent;
} co2_controller_settings_t;

typedef struct {
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
  bool time_valid;
  uint16_t co2_ppm;
  bool co2_valid;
  float radiation_wm2;
  bool radiation_valid;
  float air_temp_c;
  bool air_temp_valid;
  float air_temp_target_c;
  bool air_temp_target_valid;
  float humidity_percent;
  bool humidity_valid;
  float humidity_target_percent;
  bool humidity_target_valid;
  float window_a_percent;
  bool window_a_valid;
  float window_b_percent;
  bool window_b_valid;
  bool ventilation_valid;
  bool external_safety_alarm;
} co2_controller_inputs_t;

typedef struct {
  uint16_t target_ppm;
  uint16_t effective_target_ppm;
  uint16_t measured_ppm;
  bool valve_open;
  bool mixing_requested;
  uint16_t status_bits;
  uint16_t reason_bits;
  uint16_t active_protection_bits;
  uint16_t dosing_elapsed_s;
  uint16_t pause_elapsed_s;
  co2_controller_fault_code_t fault_code;
} co2_controller_status_t;

#define CO2_CONTROLLER_STATUS_ENABLED (1U << 0)
#define CO2_CONTROLLER_STATUS_AUTO_MODE (1U << 1)
#define CO2_CONTROLLER_STATUS_OFF_MODE (1U << 2)
#define CO2_CONTROLLER_STATUS_MANUAL_MODE (1U << 3)
#define CO2_CONTROLLER_STATUS_VALVE_OPEN (1U << 4)
#define CO2_CONTROLLER_STATUS_MIX_REQUEST (1U << 5)
#define CO2_CONTROLLER_STATUS_SENSOR_VALID (1U << 6)
#define CO2_CONTROLLER_STATUS_FAULT (1U << 7)
#define CO2_CONTROLLER_STATUS_DOSING_HOLD (1U << 8)
#define CO2_CONTROLLER_STATUS_PAUSE_HOLD (1U << 9)

#define CO2_CONTROLLER_REASON_SCHEDULE_ACTIVE (1U << 0)
#define CO2_CONTROLLER_REASON_OUTSIDE_SCHEDULE (1U << 1)
#define CO2_CONTROLLER_REASON_LIGHT_LOW (1U << 2)
#define CO2_CONTROLLER_REASON_LIGHT_MID (1U << 3)
#define CO2_CONTROLLER_REASON_LIGHT_HIGH (1U << 4)
#define CO2_CONTROLLER_REASON_VENT_LIMITED (1U << 5)
#define CO2_CONTROLLER_REASON_VENT_BLOCKED (1U << 6)
#define CO2_CONTROLLER_REASON_TEMP_HIGH (1U << 7)
#define CO2_CONTROLLER_REASON_TEMP_CRITICAL (1U << 8)
#define CO2_CONTROLLER_REASON_HUMIDITY_HIGH (1U << 9)
#define CO2_CONTROLLER_REASON_SENSOR_FAULT (1U << 10)
#define CO2_CONTROLLER_REASON_LIGHT_FAULT (1U << 11)
#define CO2_CONTROLLER_REASON_TIME_FAULT (1U << 12)
#define CO2_CONTROLLER_REASON_EXTERNAL_ALARM (1U << 13)

#define CO2_CONTROLLER_PROTECTION_EXTERNAL_ALARM (1U << 0)
#define CO2_CONTROLLER_PROTECTION_SENSOR_FAULT (1U << 1)
#define CO2_CONTROLLER_PROTECTION_OVERRANGE (1U << 2)
#define CO2_CONTROLLER_PROTECTION_TIME_OR_NIGHT (1U << 3)
#define CO2_CONTROLLER_PROTECTION_TEMP_CRITICAL (1U << 4)
#define CO2_CONTROLLER_PROTECTION_TEMP_HIGH (1U << 5)
#define CO2_CONTROLLER_PROTECTION_HUMIDITY_HIGH (1U << 6)
#define CO2_CONTROLLER_PROTECTION_VENT_CUTOFF (1U << 7)
#define CO2_CONTROLLER_PROTECTION_MAX_DOSING (1U << 8)
#define CO2_CONTROLLER_PROTECTION_MIN_PAUSE (1U << 9)
#define CO2_CONTROLLER_PROTECTION_NO_RISE (1U << 10)

esp_err_t co2_controller_init(const co2_controller_config_t *config,
                              co2_controller_handle_t *ret_handle);
esp_err_t co2_controller_process(co2_controller_handle_t handle,
                                 const co2_controller_settings_t *settings,
                                 const co2_controller_inputs_t *inputs);
esp_err_t co2_controller_get_status(co2_controller_handle_t handle,
                                    co2_controller_status_t *out_status);
esp_err_t co2_controller_reset_fault(co2_controller_handle_t handle);
esp_err_t co2_controller_del(co2_controller_handle_t handle);

#ifdef __cplusplus
}
#endif
