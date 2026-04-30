#pragma once

#include "ads1115.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  RLL400_STATE_BOOT_WAIT_POSITION = 0,
  RLL400_STATE_HOLDING,
  RLL400_STATE_MOVING_OPEN,
  RLL400_STATE_MOVING_CLOSE,
  RLL400_STATE_LOCAL_MANUAL,
  RLL400_STATE_FAULT_BLOCKED,
} rll400_state_t;

typedef enum {
  RLL400_FAULT_NONE = 0,
  RLL400_FAULT_NO_MOTION = 1,
  RLL400_FAULT_ENCODER = 2,
  RLL400_FAULT_BOOT_NO_POSITION = 3,
} rll400_fault_code_t;

typedef struct {
  const char *name;

  i2c_port_t i2c_port;
  uint8_t ads_addr;
  uint8_t ads_channel_pos;
  uint8_t ads_channel_neg;
  float shunt_resistor_ohm;

  gpio_num_t pin_open;
  gpio_num_t pin_close;
  gpio_num_t pin_local_manual;
  bool local_manual_active_high;

  float target_hysteresis_percent;
  float motion_delta_percent;
  uint32_t no_motion_timeout_ms;
  float encoder_min_ma;
  float encoder_max_ma;
  uint32_t boot_wait_position_timeout_ms;
} rll400_config_t;

typedef struct {
  float position_percent;
  float target_percent;
  float current_ma;
  rll400_state_t state;
  rll400_fault_code_t fault_code;
  bool position_valid;
  bool at_target;
  bool blocked;
  bool local_manual_active;
  bool outputs_enabled;
  bool control_active;
} rll400_status_t;

typedef struct rll400_context_t *rll400_handle_t;

esp_err_t rll400_init(const rll400_config_t *config,
                      rll400_handle_t *ret_handle);

esp_err_t rll400_set_target(rll400_handle_t handle, float target_percent);

esp_err_t rll400_set_target_hysteresis_percent(rll400_handle_t handle,
                                               float hysteresis_percent);

esp_err_t rll400_set_motion_fault_config(rll400_handle_t handle,
                                         float motion_delta_percent,
                                         uint32_t no_motion_timeout_ms);

esp_err_t rll400_get_status(rll400_handle_t handle, float *out_position_percent,
                            float *out_current_ma);

esp_err_t rll400_get_runtime_status(rll400_handle_t handle,
                                    rll400_status_t *out_status);

esp_err_t rll400_process(rll400_handle_t handle);

esp_err_t rll400_stop(rll400_handle_t handle);

esp_err_t rll400_reset_fault(rll400_handle_t handle);

esp_err_t rll400_del(rll400_handle_t handle);

#ifdef __cplusplus
}
#endif
