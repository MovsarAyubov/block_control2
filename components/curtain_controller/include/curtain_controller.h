#pragma once

#include "driver/i2c.h"
#include "esp_err.h"
#include "hc595_outputs.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct curtain_controller_ctx_t *curtain_controller_handle_t;

typedef enum {
  CURTAIN_CONTROLLER_MODE_AUTO = 0,
  CURTAIN_CONTROLLER_MODE_MANUAL = 1,
  CURTAIN_CONTROLLER_MODE_OFF = 2,
} curtain_controller_mode_t;

typedef enum {
  CURTAIN_CONTROLLER_FAULT_NONE = 0,
  CURTAIN_CONTROLLER_FAULT_ENCODER = 1,
  CURTAIN_CONTROLLER_FAULT_NO_MOTION = 2,
  CURTAIN_CONTROLLER_FAULT_OUTPUT = 3,
} curtain_controller_fault_code_t;

typedef struct {
  const char *name;
  i2c_port_t i2c_port;
  uint8_t ads_addr;
  uint8_t ads_channel_pos;
  uint8_t ads_channel_neg;
  float shunt_resistor_ohm;
  float encoder_min_ma;
  float encoder_max_ma;
  uint32_t boot_wait_position_timeout_ms;
  float motion_delta_percent;
  uint32_t no_motion_timeout_ms;
  hc595_outputs_handle_t outputs;
  uint8_t open_bit_index;
  uint8_t close_bit_index;
} curtain_controller_config_t;

typedef struct {
  curtain_controller_mode_t mode;
  uint16_t schedule_start_hhmm;
  uint16_t schedule_end_hhmm;
  float manual_target_percent;
  float outside_target_percent;
  float position_hysteresis_percent;
  float min_position_percent;
  float max_position_percent;
  uint16_t radiation_threshold_wm2;
  uint16_t radiation_step_wm2;
  float radiation_step_percent;
  uint16_t radiation_hysteresis_wm2;
  float cold_delta_c;
  float cold_hysteresis_c;
  float cold_target_percent;
  float heat_delta_c;
  float heat_hysteresis_c;
  float heat_target_percent;
  float humidity_low_threshold_percent;
  float humidity_low_hysteresis_percent;
  float humidity_low_target_percent;
  float humidity_high_threshold_percent;
  float humidity_high_hysteresis_percent;
  float humidity_high_target_percent;
  float humidity_setpoint_percent;
  bool humidity_setpoint_valid;
} curtain_controller_settings_t;

typedef struct {
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
  bool time_valid;
  float air_temp_c;
  bool air_temp_valid;
  float humidity_percent;
  bool humidity_valid;
  float radiation_wm2;
  bool radiation_valid;
  float temp_setpoint_c;
  bool temp_setpoint_valid;
} curtain_controller_inputs_t;

typedef struct {
  float position_percent;
  float target_percent;
  float base_target_percent;
  float current_ma;
  bool position_valid;
  bool at_target;
  bool moving_open;
  bool moving_close;
  bool output_open;
  bool output_close;
  uint16_t status_bits;
  uint16_t reason_bits;
  uint16_t position_status_bits;
  curtain_controller_fault_code_t fault_code;
} curtain_controller_status_t;

#define CURTAIN_CONTROLLER_STATUS_ENABLED (1U << 0)
#define CURTAIN_CONTROLLER_STATUS_MANUAL_MODE (1U << 1)
#define CURTAIN_CONTROLLER_STATUS_AUTO_MODE (1U << 2)
#define CURTAIN_CONTROLLER_STATUS_OFF_MODE (1U << 3)
#define CURTAIN_CONTROLLER_STATUS_POSITION_VALID (1U << 4)
#define CURTAIN_CONTROLLER_STATUS_MOVING_OPEN (1U << 5)
#define CURTAIN_CONTROLLER_STATUS_MOVING_CLOSE (1U << 6)
#define CURTAIN_CONTROLLER_STATUS_AT_TARGET (1U << 7)
#define CURTAIN_CONTROLLER_STATUS_FAULT (1U << 8)
#define CURTAIN_CONTROLLER_STATUS_OUTPUT_OPEN (1U << 9)
#define CURTAIN_CONTROLLER_STATUS_OUTPUT_CLOSE (1U << 10)

#define CURTAIN_CONTROLLER_REASON_SCHEDULE_ACTIVE (1U << 0)
#define CURTAIN_CONTROLLER_REASON_RADIATION_ACTIVE (1U << 1)
#define CURTAIN_CONTROLLER_REASON_COLD_CLOSE (1U << 2)
#define CURTAIN_CONTROLLER_REASON_HEAT_OPEN (1U << 3)
#define CURTAIN_CONTROLLER_REASON_HUMIDITY_LOW_OPEN (1U << 4)
#define CURTAIN_CONTROLLER_REASON_HUMIDITY_HIGH_CLOSE (1U << 5)
#define CURTAIN_CONTROLLER_REASON_MANUAL (1U << 6)
#define CURTAIN_CONTROLLER_REASON_OUTSIDE_SCHEDULE (1U << 7)
#define CURTAIN_CONTROLLER_REASON_TEMP_SENSOR_FAULT (1U << 8)
#define CURTAIN_CONTROLLER_REASON_HUM_SENSOR_FAULT (1U << 9)
#define CURTAIN_CONTROLLER_REASON_RADIATION_FAULT (1U << 10)
#define CURTAIN_CONTROLLER_REASON_TIME_FAULT (1U << 11)

#define CURTAIN_CONTROLLER_POS_STATUS_VALID (1U << 0)
#define CURTAIN_CONTROLLER_POS_STATUS_AT_TARGET (1U << 1)
#define CURTAIN_CONTROLLER_POS_STATUS_MOVING_OPEN (1U << 2)
#define CURTAIN_CONTROLLER_POS_STATUS_MOVING_CLOSE (1U << 3)
#define CURTAIN_CONTROLLER_POS_STATUS_FAULT_ENCODER (1U << 4)
#define CURTAIN_CONTROLLER_POS_STATUS_FAULT_NO_MOTION (1U << 5)

esp_err_t curtain_controller_init(const curtain_controller_config_t *config,
                                  curtain_controller_handle_t *ret_handle);
esp_err_t curtain_controller_process(
    curtain_controller_handle_t handle,
    const curtain_controller_settings_t *settings,
    const curtain_controller_inputs_t *inputs);
esp_err_t curtain_controller_get_status(
    curtain_controller_handle_t handle, curtain_controller_status_t *out_status);
esp_err_t curtain_controller_set_motion_fault_config(
    curtain_controller_handle_t handle, float motion_delta_percent,
    uint32_t no_motion_timeout_ms);
esp_err_t curtain_controller_stop(curtain_controller_handle_t handle);
esp_err_t curtain_controller_reset_fault(curtain_controller_handle_t handle);
esp_err_t curtain_controller_del(curtain_controller_handle_t handle);

#ifdef __cplusplus
}
#endif
