#include "bt_ascii_control.h"
#include "co2_controller.h"
#include "circulation_controller.h"
#include "ads1115.h"
#include "curtain_controller.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "ds3231.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hc595_outputs.h"
#include "max31865.h"
#include "modbus_slave.h"
#include "nvs_flash.h"
#include "rh_sensor.h"
#include "rll400.h"
#include "valve_3way.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

static const char *TAG = "APP";
#define APP_LOG_WINDOWS_ONLY false

// I2C configuration
#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_NUM 0
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0

// RH sensor config
#define R1 5220.0f
#define R2 10000.0f
#define ADC_OFFSET_MV 0

// MAX31865 config (HSPI)
#define MAX31865_HOST SPI2_HOST
#define MAX31865_MISO 12
#define MAX31865_MOSI 14
#define MAX31865_CLK 13
#define MAX31865_AIR_CS 15
#define MAX31865_WATER_RAIL_CS 5
#define MAX31865_WATER_UPPER_CS 16
#define MAX31865_WATER_UNDERTRAY_CS 17
// GPIO0 is a boot strap pin; change this if final hardware provides a safer CS.
#define MAX31865_WATER_GROW_CS 0
#define MAX31865_RREF 1999.0f
#define MAX31865_R0 500.0f

// Heating control config
#define HEATING_CONTOUR_COUNT 4U
#define HEATING_HC595_CHIP_COUNT 3U
#define VALVE_TEMPERATURE_HYSTERESIS_C 2.5f
#define HEATING_SAFE_CLOSE_MS 120000U

// 74HC595 control bus
#define HC595_DATA_GPIO GPIO_NUM_2
#define HC595_CLOCK_GPIO GPIO_NUM_4
#define HC595_LATCH_GPIO GPIO_NUM_18

// 74HC595 output mapping
#define HC595_BIT_LIGHT_RELAY_1 0U
#define HC595_BIT_LIGHT_RELAY_2 1U
#define HC595_BIT_HEATING_RAIL_VALVE_OPEN 2U
#define HC595_BIT_HEATING_RAIL_VALVE_CLOSE 3U
#define HC595_BIT_HEATING_UPPER_VALVE_OPEN 4U
#define HC595_BIT_HEATING_UPPER_VALVE_CLOSE 5U
#define HC595_BIT_HEATING_UNDERTRAY_VALVE_OPEN 6U
#define HC595_BIT_HEATING_UNDERTRAY_VALVE_CLOSE 7U
#define HC595_BIT_HEATING_GROW_VALVE_OPEN 8U
#define HC595_BIT_HEATING_GROW_VALVE_CLOSE 9U
#define HC595_BIT_HEATING_RAIL_PUMP 10U
#define HC595_BIT_HEATING_UPPER_PUMP 11U
#define HC595_BIT_HEATING_UNDERTRAY_PUMP 12U
#define HC595_BIT_HEATING_GROW_PUMP 13U
#define HC595_BIT_CURTAIN_OPEN 14U
#define HC595_BIT_CURTAIN_CLOSE 15U
#define HC595_BIT_CO2_VALVE 16U
#define HC595_BIT_CIRC_FAN_A 17U
#define HC595_BIT_CIRC_FAN_B 18U

// Window motor config
#define WINDOW_A_NAME "window_a"
#define WINDOW_B_NAME "window_b"
#define WINDOW_SENSOR_SHUNT_OHM 109.3f
#define WINDOW_ENCODER_MIN_MA 3.6f
#define WINDOW_ENCODER_MAX_MA 20.5f
#define WINDOW_BOOT_WAIT_POSITION_TIMEOUT_MS 1000U

// Default window motor GPIO map chosen to avoid conflicts with I2C, SPI, RS485,
// and the 74HC595 latch bus. Adjust here if the final cabinet wiring differs.
#define WINDOW_A_PIN_OPEN GPIO_NUM_32
#define WINDOW_A_PIN_CLOSE GPIO_NUM_33
#define WINDOW_A_PIN_LOCAL_MANUAL GPIO_NUM_NC
#define WINDOW_B_PIN_OPEN GPIO_NUM_19
#define WINDOW_B_PIN_CLOSE GPIO_NUM_23
#define WINDOW_B_PIN_LOCAL_MANUAL GPIO_NUM_NC
#define WINDOW_LOCAL_MANUAL_ACTIVE_HIGH true

// RH sensor and curtain positioner share ADS1115 at 0x48.
// Window positioners share ADS1115 at 0x49.
#define WINDOW_A_ADS_ADDR ADS1115_ADDR_VDD
#define WINDOW_A_ADS_CH_POS 0U
#define WINDOW_A_ADS_CH_NEG 1U
#define WINDOW_B_ADS_ADDR ADS1115_ADDR_VDD
#define WINDOW_B_ADS_CH_POS 2U
#define WINDOW_B_ADS_CH_NEG 3U

// Curtain positioner uses ADS1115 0x48 differential input AIN2-AIN3.
#define CURTAIN_NAME "curtain"
#define CURTAIN_ADS_ADDR ADS1115_ADDR_GND
#define CURTAIN_ADS_CH_POS 2U
#define CURTAIN_ADS_CH_NEG 3U
#define CURTAIN_SENSOR_SHUNT_OHM 109.3f
#define CURTAIN_ENCODER_MIN_MA 3.6f
#define CURTAIN_ENCODER_MAX_MA 20.5f
#define CURTAIN_BOOT_WAIT_POSITION_TIMEOUT_MS 1000U
#define CURTAIN_MOTION_DELTA_DEFAULT_PERCENT 0.5f
#define CURTAIN_NO_MOTION_DEFAULT_MS 3000U

// Window logic defaults
#define WINDOWS_WEATHER_STALE_TIMEOUT_DEFAULT_MS 20000U
#define WINDOWS_WEATHER_STALE_SOURCE_AGE_DEFAULT_S 20U
#define WINDOWS_TEMP_SETPOINT_DEFAULT_C 25.0f
#define WINDOWS_SAFE_MIN_DEFAULT_PERCENT 0.0f
#define WINDOWS_WIND_LIMIT_DEFAULT_MS 8.0f
#define WINDOWS_WIND_STORM_DEFAULT_MS 15.0f
#define WINDOWS_WIND_RECOVER_DEFAULT_MS 6.0f
#define WINDOWS_TEMP_STEP_DEFAULT_C 3.0f
#define WINDOWS_TEMP_STEP_HYST_DEFAULT_C 0.5f
#define WINDOWS_TEMP_STEP_TARGET_DEFAULT_PERCENT 20.0f
#define WINDOWS_TEMP_STEP_MAX_INDEX_DEFAULT 5U
#define WINDOWS_TARGET_HYST_DEFAULT_PERCENT 3.0f
#define WINDOWS_MOTION_DELTA_DEFAULT_PERCENT 0.5f
#define WINDOWS_NO_MOTION_DEFAULT_MS 3000U
#define WINDOWS_WIND_HALF_WIDTH_DEFAULT_DEG 45U
#define WINDOWS_HUM_SETPOINT_DEFAULT_PERCENT 80.0f
#define WINDOWS_HUM_STEP_DEFAULT_PERCENT 5.0f
#define WINDOWS_HUM_STEP_HYST_DEFAULT_PERCENT 1.0f
#define WINDOWS_HUM_STEP_TARGET_DEFAULT_PERCENT 20.0f
#define WINDOWS_HUM_STEP_MAX_INDEX_DEFAULT 5U
#define WINDOWS_COLD_CLOSE_DELTA_DEFAULT_C 2.0f
#define WINDOWS_COLD_CLOSE_HYST_DEFAULT_C 0.5f
#define WINDOWS_WIND_SPEED_TARGET_STEP_MS 1.0f
#define WINDOWS_WINDWARD_MIN_DEFAULT_PERCENT 0.0f
#define WINDOWS_WINDWARD_MAX_DEFAULT_PERCENT 0.0f
#define WINDOWS_WINDWARD_REDUCTION_DEFAULT_PERCENT_PER_MS 0.0f
#define WINDOWS_LEEWARD_MIN_DEFAULT_PERCENT 0.0f
#define WINDOWS_LEEWARD_MAX_DEFAULT_PERCENT 100.0f
#define WINDOWS_LEEWARD_REDUCTION_DEFAULT_PERCENT_PER_MS 0.0f
#define WINDOWS_WINDWARD_LAG_DEFAULT_PERCENT 0.0f
#define WINDOWS_RAIN_WINDWARD_DEFAULT_PERCENT 0.0f

#define CONTROL_LOOP_MS 200
#define SENSOR_LOOP_MS 5000
#define RTC_INIT_HOUR 10
#define RTC_INIT_MINUTE 31
#define RTC_INIT_SECOND 0

#define WINDOWS_STATUS_FORCE_SAFE_ACTIVE (1U << 0)
#define WINDOWS_STATUS_WEATHER_STALE (1U << 1)
#define WINDOWS_STATUS_STORM_ACTIVE (1U << 2)
#define WINDOWS_STATUS_WIND_LIMIT_A (1U << 3)
#define WINDOWS_STATUS_WIND_LIMIT_B (1U << 4)
#define WINDOWS_STATUS_TEMP_SENSOR_FAULT (1U << 5)
#define WINDOWS_STATUS_MANUAL_SYSTEM_MODE (1U << 6)
#define WINDOWS_STATUS_MODBUS_AUTONOMOUS (1U << 7)
#define WINDOWS_STATUS_COLD_CLOSE_ACTIVE (1U << 8)
#define WINDOWS_STATUS_RAIN_LIMIT_A (1U << 9)
#define WINDOWS_STATUS_RAIN_LIMIT_B (1U << 10)
#define WINDOWS_STATUS_HUM_SENSOR_FAULT (1U << 11)
#define WINDOWS_STATUS_ALGO_HUMIDITY (1U << 12)

#define WEATHER_STATUS_LAST_ERROR (1U << 15)

#define WINDOWS_PROTECTION_FORCE_SAFE (1U << 0)
#define WINDOWS_PROTECTION_WEATHER_STALE (1U << 1)
#define WINDOWS_PROTECTION_STORM (1U << 2)
#define WINDOWS_PROTECTION_COLD_CLOSE (1U << 3)
#define WINDOWS_PROTECTION_RAIN_A (1U << 4)
#define WINDOWS_PROTECTION_RAIN_B (1U << 5)
#define WINDOWS_PROTECTION_WIND_A (1U << 6)
#define WINDOWS_PROTECTION_WIND_B (1U << 7)
#define WINDOWS_PROTECTION_TEMP_SENSOR_FAULT (1U << 8)
#define WINDOWS_PROTECTION_HUM_SENSOR_FAULT (1U << 9)

#define WINDOW_STATUS_POSITION_VALID (1U << 0)
#define WINDOW_STATUS_MOVING_OPEN (1U << 1)
#define WINDOW_STATUS_MOVING_CLOSE (1U << 2)
#define WINDOW_STATUS_AT_TARGET (1U << 3)
#define WINDOW_STATUS_FAULT_NO_MOTION (1U << 4)
#define WINDOW_STATUS_FAULT_ENCODER (1U << 5)
#define WINDOW_STATUS_LOCAL_MANUAL_ACTIVE (1U << 6)
#define WINDOW_STATUS_BLOCKED (1U << 7)
#define WINDOW_STATUS_OUTPUTS_ENABLED (1U << 8)
#define WINDOW_STATUS_CONTROL_ACTIVE (1U << 9)

#define AIR_TEMP_SENSOR_STATUS_OK 0U
#define AIR_TEMP_SENSOR_STATUS_FAULT 1U

#define HEATING_STATUS_ENABLED (1U << 0)
#define HEATING_STATUS_MANUAL_MODE (1U << 1)
#define HEATING_STATUS_OFF_MODE (1U << 2)
#define HEATING_STATUS_AIR_SENSOR_FAULT (1U << 3)
#define HEATING_STATUS_WATER_SENSOR_FAULT (1U << 4)
#define HEATING_STATUS_MIN_ON_HOLD (1U << 5)
#define HEATING_STATUS_MIN_OFF_HOLD (1U << 6)

#define HEATING_SENSOR_STATUS_AIR_FAULT (1U << 0)
#define HEATING_SENSOR_STATUS_RAIL_WATER_FAULT (1U << 1)
#define HEATING_SENSOR_STATUS_UPPER_WATER_FAULT (1U << 2)
#define HEATING_SENSOR_STATUS_UNDERTRAY_WATER_FAULT (1U << 3)
#define HEATING_SENSOR_STATUS_GROW_WATER_FAULT (1U << 4)

typedef enum {
  HEATING_CONTOUR_RAIL = 0,
  HEATING_CONTOUR_UPPER = 1,
  HEATING_CONTOUR_UNDERTRAY = 2,
  HEATING_CONTOUR_GROW = 3,
} heating_contour_id_t;

typedef struct {
  float rh;
  bool rh_valid;
  float temp_air;
  bool temp_air_valid;
  float water_temp[HEATING_CONTOUR_COUNT];
  bool water_temp_valid[HEATING_CONTOUR_COUNT];
} app_sensor_snapshot_t;

typedef struct {
  const char *name;
  modbus_water_channel_t water_channel;
  uint8_t valve_open_bit;
  uint8_t valve_close_bit;
  uint8_t pump_bit;
  valve_3way_handle_t valve;
  uint32_t inactive_since_ms;
} heating_contour_t;

typedef struct {
  uint8_t active_stage;
  uint16_t active_mask;
  uint16_t pump_mask;
  uint32_t last_stage_change_ms;
  bool initialized;
} heating_runtime_t;

typedef struct {
  float min_percent;
  float max_percent;
  float speed_threshold_ms;
  float reduction_percent_per_ms;
} window_wind_role_settings_t;

typedef struct {
  bool active;
  float reduction_percent;
} window_wind_cap_t;

typedef struct {
  bool initialized;
  float wind_speed_ms;
} window_wind_speed_hold_t;

typedef struct {
  modbus_windows_ctrl_mode_t ctrl_mode;
  modbus_windows_auto_algo_mode_t auto_algo_mode;
  modbus_windows_rain_mode_t rain_mode;
  modbus_windows_weather_stale_policy_t weather_stale_policy;
  bool force_safe_cmd;
  float temp_setpoint_c;
  float safe_min_percent;
  float wind_limit_ms;
  float wind_storm_ms;
  float wind_recover_ms;
  uint16_t window_a_azimuth_deg;
  uint16_t wind_sector_half_width_deg;
  uint32_t weather_stale_timeout_ms;
  uint16_t weather_source_age_limit_s;
  float temp_step_c;
  float temp_step_hyst_c;
  float temp_step_target_percent;
  float humidity_setpoint_percent;
  float humidity_step_percent;
  float humidity_step_hyst_percent;
  float humidity_step_target_percent;
  float cold_close_delta_c;
  float cold_close_hyst_c;
  window_wind_role_settings_t windward;
  window_wind_role_settings_t leeward;
  float windward_lag_percent;
  float rain_windward_percent;
  float target_hyst_percent;
  float motion_delta_percent;
  uint32_t no_motion_timeout_ms;
} window_settings_t;

typedef struct {
  int temp_step_index;
  int humidity_step_index;
  bool cold_close_active;
  bool storm_active;
  bool wind_limit_a_active;
  bool wind_limit_b_active;
  bool effective_target_initialized;
  float last_effective_target_a_percent;
  float last_effective_target_b_percent;
  window_wind_speed_hold_t windward_speed_hold;
  window_wind_speed_hold_t leeward_speed_hold;
  uint16_t last_fault_reset_token_a;
  uint16_t last_fault_reset_token_b;
} window_pair_runtime_t;

static rh_sensor_handle_t rh_handle = NULL;
static hc595_outputs_handle_t s_hc595_outputs = NULL;
static max31865_handle_t s_air_temp_handle = NULL;
static max31865_handle_t s_water_temp_handles[HEATING_CONTOUR_COUNT] = {0};
static heating_contour_t s_heating_contours[HEATING_CONTOUR_COUNT] = {
    [HEATING_CONTOUR_RAIL] =
        {
            .name = "rail",
            .water_channel = MODBUS_WATER_CHANNEL_RAIL,
            .valve_open_bit = HC595_BIT_HEATING_RAIL_VALVE_OPEN,
            .valve_close_bit = HC595_BIT_HEATING_RAIL_VALVE_CLOSE,
            .pump_bit = HC595_BIT_HEATING_RAIL_PUMP,
        },
    [HEATING_CONTOUR_UPPER] =
        {
            .name = "upper",
            .water_channel = MODBUS_WATER_CHANNEL_UPPER,
            .valve_open_bit = HC595_BIT_HEATING_UPPER_VALVE_OPEN,
            .valve_close_bit = HC595_BIT_HEATING_UPPER_VALVE_CLOSE,
            .pump_bit = HC595_BIT_HEATING_UPPER_PUMP,
        },
    [HEATING_CONTOUR_UNDERTRAY] =
        {
            .name = "undertray",
            .water_channel = MODBUS_WATER_CHANNEL_UNDERTRAY,
            .valve_open_bit = HC595_BIT_HEATING_UNDERTRAY_VALVE_OPEN,
            .valve_close_bit = HC595_BIT_HEATING_UNDERTRAY_VALVE_CLOSE,
            .pump_bit = HC595_BIT_HEATING_UNDERTRAY_PUMP,
        },
    [HEATING_CONTOUR_GROW] =
        {
            .name = "grow",
            .water_channel = MODBUS_WATER_CHANNEL_GROW,
            .valve_open_bit = HC595_BIT_HEATING_GROW_VALVE_OPEN,
            .valve_close_bit = HC595_BIT_HEATING_GROW_VALVE_CLOSE,
            .pump_bit = HC595_BIT_HEATING_GROW_PUMP,
        },
};
static ds3231_handle_t rtc_handle = NULL;
static bool s_rtc_available = false;
static rll400_handle_t s_window_a_handle = NULL;
static rll400_handle_t s_window_b_handle = NULL;
static curtain_controller_handle_t s_curtain_handle = NULL;
static co2_controller_handle_t s_co2_handle = NULL;
static circulation_controller_handle_t s_circulation_handle = NULL;
static portMUX_TYPE s_sensor_lock = portMUX_INITIALIZER_UNLOCKED;
static app_sensor_snapshot_t s_sensor_snapshot = {0};
static window_pair_runtime_t s_window_runtime = {0};
static heating_runtime_t s_heating_runtime = {0};

static float clampf_local(float value, float min_value, float max_value) {
  if (value < min_value) {
    return min_value;
  }
  if (value > max_value) {
    return max_value;
  }
  return value;
}

static float sanitize_positive_or_default(float value, float default_value) {
  if (!isfinite(value) || value <= 0.0f) {
    return default_value;
  }
  return value;
}

static float sanitize_nonnegative_or_default(float value, float default_value) {
  if (!isfinite(value) || value < 0.0f) {
    return default_value;
  }
  return value;
}

static float stabilize_effective_target(float requested_percent,
                                        float previous_percent,
                                        float hysteresis_percent,
                                        bool force_update) {
  if (force_update ||
      fabsf(requested_percent - previous_percent) >= hysteresis_percent) {
    return requested_percent;
  }
  return previous_percent;
}

static uint32_t sanitize_timeout_or_default(uint32_t value, uint32_t default_value) {
  return (value == 0U) ? default_value : value;
}

static uint16_t sanitize_u16_or_default(uint16_t value, uint16_t default_value) {
  return (value == 0U) ? default_value : value;
}

static uint16_t derive_step_index_limit(float step_target_percent) {
  const float sanitized_step_target =
      clampf_local(step_target_percent, 0.1f, 100.0f);
  const float raw_limit = ceilf(100.0f / sanitized_step_target);
  return (uint16_t)clampf_local(raw_limit, 1.0f, 1000.0f);
}

static void set_sensor_snapshot(const app_sensor_snapshot_t *snapshot) {
  if (snapshot == NULL) {
    return;
  }
  taskENTER_CRITICAL(&s_sensor_lock);
  s_sensor_snapshot = *snapshot;
  taskEXIT_CRITICAL(&s_sensor_lock);
}

static app_sensor_snapshot_t get_sensor_snapshot(void) {
  app_sensor_snapshot_t snapshot = {0};
  taskENTER_CRITICAL(&s_sensor_lock);
  snapshot = s_sensor_snapshot;
  taskEXIT_CRITICAL(&s_sensor_lock);
  return snapshot;
}

static void apply_sensor_test_overrides(app_sensor_snapshot_t *sensors) {
  if (sensors == NULL) {
    return;
  }

  modbus_sensor_test_override_t override = {0};
  modbus_get_sensor_test_override(&override);

  if (override.air_temp_override_active) {
    sensors->temp_air = override.air_temp_c;
    sensors->temp_air_valid = true;
    ESP_LOGW(TAG, "TEST OVERRIDE: air temp -> %.1f C", sensors->temp_air);
  }

  if (override.rh_override_active) {
    sensors->rh = override.rh_percent;
    sensors->rh_valid = true;
    ESP_LOGW(TAG, "TEST OVERRIDE: RH -> %.1f %%", sensors->rh);
  }
}

static float normalize_angle_deg(float angle_deg) {
  float normalized = fmodf(angle_deg, 360.0f);
  if (normalized < 0.0f) {
    normalized += 360.0f;
  }
  return normalized;
}

static float angular_distance_deg(float lhs_deg, float rhs_deg) {
  float diff = fabsf(normalize_angle_deg(lhs_deg) - normalize_angle_deg(rhs_deg));
  if (diff > 180.0f) {
    diff = 360.0f - diff;
  }
  return diff;
}

static uint16_t build_window_status_bits(const rll400_status_t *status) {
  if (status == NULL) {
    return 0U;
  }

  uint16_t bits = 0U;
  if (status->position_valid) {
    bits |= WINDOW_STATUS_POSITION_VALID;
  }
  if (status->state == RLL400_STATE_MOVING_OPEN) {
    bits |= WINDOW_STATUS_MOVING_OPEN;
  }
  if (status->state == RLL400_STATE_MOVING_CLOSE) {
    bits |= WINDOW_STATUS_MOVING_CLOSE;
  }
  if (status->at_target) {
    bits |= WINDOW_STATUS_AT_TARGET;
  }
  if (status->fault_code == RLL400_FAULT_NO_MOTION) {
    bits |= WINDOW_STATUS_FAULT_NO_MOTION;
  }
  if (status->fault_code == RLL400_FAULT_ENCODER ||
      status->fault_code == RLL400_FAULT_BOOT_NO_POSITION) {
    bits |= WINDOW_STATUS_FAULT_ENCODER;
  }
  if (status->local_manual_active) {
    bits |= WINDOW_STATUS_LOCAL_MANUAL_ACTIVE;
  }
  if (status->blocked) {
    bits |= WINDOW_STATUS_BLOCKED;
  }
  if (status->outputs_enabled) {
    bits |= WINDOW_STATUS_OUTPUTS_ENABLED;
  }
  if (status->control_active) {
    bits |= WINDOW_STATUS_CONTROL_ACTIVE;
  }
  return bits;
}

static void read_window_settings(window_settings_t *settings) {
  if (settings == NULL) {
    return;
  }

  settings->ctrl_mode = modbus_get_windows_ctrl_mode();
  settings->auto_algo_mode = modbus_get_windows_auto_algo_mode();
  settings->rain_mode = modbus_get_windows_rain_mode();
  settings->weather_stale_policy = modbus_get_windows_weather_stale_policy();
  settings->force_safe_cmd = modbus_get_windows_force_safe_cmd();
  settings->temp_setpoint_c = sanitize_positive_or_default(
      modbus_get_windows_temp_setpoint_c(), WINDOWS_TEMP_SETPOINT_DEFAULT_C);
  settings->safe_min_percent = clampf_local(modbus_get_windows_safe_min_percent(),
                                            0.0f, 100.0f);
  settings->wind_limit_ms = sanitize_positive_or_default(
      modbus_get_windows_wind_limit_ms(), WINDOWS_WIND_LIMIT_DEFAULT_MS);
  settings->wind_storm_ms = sanitize_positive_or_default(
      modbus_get_windows_wind_storm_ms(), WINDOWS_WIND_STORM_DEFAULT_MS);
  settings->wind_recover_ms = sanitize_positive_or_default(
      modbus_get_windows_wind_recover_ms(), WINDOWS_WIND_RECOVER_DEFAULT_MS);
  settings->window_a_azimuth_deg = modbus_get_window_a_azimuth_deg();
  settings->wind_sector_half_width_deg =
      modbus_get_windows_wind_sector_half_width_deg();
  if (settings->wind_sector_half_width_deg == 0U) {
    settings->wind_sector_half_width_deg = WINDOWS_WIND_HALF_WIDTH_DEFAULT_DEG;
  }
  settings->weather_stale_timeout_ms = sanitize_timeout_or_default(
      modbus_get_windows_weather_stale_timeout_ms(),
      WINDOWS_WEATHER_STALE_TIMEOUT_DEFAULT_MS);
  settings->weather_source_age_limit_s = sanitize_u16_or_default(
      modbus_get_windows_weather_source_age_limit_s(),
      WINDOWS_WEATHER_STALE_SOURCE_AGE_DEFAULT_S);
  settings->temp_step_c = sanitize_positive_or_default(
      modbus_get_windows_temp_step_c(), WINDOWS_TEMP_STEP_DEFAULT_C);
  settings->temp_step_hyst_c = sanitize_positive_or_default(
      modbus_get_windows_temp_step_hysteresis_c(), WINDOWS_TEMP_STEP_HYST_DEFAULT_C);
  if (settings->temp_step_hyst_c > settings->temp_step_c) {
    settings->temp_step_hyst_c = settings->temp_step_c;
  }
  settings->temp_step_target_percent = sanitize_positive_or_default(
      modbus_get_windows_temp_step_target_increment_percent(),
      WINDOWS_TEMP_STEP_TARGET_DEFAULT_PERCENT);
  settings->humidity_setpoint_percent = clampf_local(
      modbus_get_windows_humidity_setpoint_percent(), 0.0f, 100.0f);
  settings->humidity_step_percent = sanitize_positive_or_default(
      modbus_get_windows_humidity_step_percent(),
      WINDOWS_HUM_STEP_DEFAULT_PERCENT);
  settings->humidity_step_hyst_percent = sanitize_nonnegative_or_default(
      modbus_get_windows_humidity_step_hysteresis_percent(),
      WINDOWS_HUM_STEP_HYST_DEFAULT_PERCENT);
  if (settings->humidity_step_hyst_percent > settings->humidity_step_percent) {
    settings->humidity_step_hyst_percent = settings->humidity_step_percent;
  }
  settings->humidity_step_target_percent = sanitize_positive_or_default(
      modbus_get_windows_humidity_step_target_increment_percent(),
      WINDOWS_HUM_STEP_TARGET_DEFAULT_PERCENT);
  settings->cold_close_delta_c = sanitize_nonnegative_or_default(
      modbus_get_windows_cold_close_delta_c(),
      WINDOWS_COLD_CLOSE_DELTA_DEFAULT_C);
  settings->cold_close_hyst_c = sanitize_nonnegative_or_default(
      modbus_get_windows_cold_close_hysteresis_c(),
      WINDOWS_COLD_CLOSE_HYST_DEFAULT_C);
  settings->windward.min_percent = clampf_local(
      modbus_get_windows_windward_min_percent(), 0.0f, 100.0f);
  settings->windward.max_percent = clampf_local(
      modbus_get_windows_windward_max_percent(), 0.0f, 100.0f);
  if (settings->windward.max_percent < settings->windward.min_percent) {
    settings->windward.max_percent = settings->windward.min_percent;
  }
  settings->windward.speed_threshold_ms = clampf_local(
      sanitize_nonnegative_or_default(
          modbus_get_windows_windward_speed_threshold_ms(),
          WINDOWS_WIND_LIMIT_DEFAULT_MS),
      0.0f, 100.0f);
  settings->windward.reduction_percent_per_ms = clampf_local(
      sanitize_nonnegative_or_default(
          modbus_get_windows_windward_reduction_percent_per_ms(),
          WINDOWS_WINDWARD_REDUCTION_DEFAULT_PERCENT_PER_MS),
      0.0f, 100.0f);
  settings->leeward.min_percent = clampf_local(
      modbus_get_windows_leeward_min_percent(), 0.0f, 100.0f);
  settings->leeward.max_percent = clampf_local(
      modbus_get_windows_leeward_max_percent(), 0.0f, 100.0f);
  if (settings->leeward.max_percent < settings->leeward.min_percent) {
    settings->leeward.max_percent = settings->leeward.min_percent;
  }
  settings->leeward.speed_threshold_ms = clampf_local(
      sanitize_nonnegative_or_default(
          modbus_get_windows_leeward_speed_threshold_ms(),
          WINDOWS_WIND_LIMIT_DEFAULT_MS),
      0.0f, 100.0f);
  settings->leeward.reduction_percent_per_ms = clampf_local(
      sanitize_nonnegative_or_default(
          modbus_get_windows_leeward_reduction_percent_per_ms(),
          WINDOWS_LEEWARD_REDUCTION_DEFAULT_PERCENT_PER_MS),
      0.0f, 100.0f);
  const float legacy_wind_limit_ms =
      clampf_local(settings->wind_limit_ms, 0.1f, 100.0f);
  if (fabsf(legacy_wind_limit_ms - WINDOWS_WIND_LIMIT_DEFAULT_MS) > 0.05f) {
    if (fabsf(settings->windward.speed_threshold_ms -
              WINDOWS_WIND_LIMIT_DEFAULT_MS) <= 0.05f) {
      settings->windward.speed_threshold_ms = legacy_wind_limit_ms;
    }
    if (fabsf(settings->leeward.speed_threshold_ms -
              WINDOWS_WIND_LIMIT_DEFAULT_MS) <= 0.05f) {
      settings->leeward.speed_threshold_ms = legacy_wind_limit_ms;
    }
  }
  settings->windward_lag_percent =
      clampf_local(modbus_get_windows_windward_lag_percent(), 0.0f, 100.0f);
  settings->rain_windward_percent = clampf_local(
      modbus_get_windows_rain_windward_percent(), 0.0f, 100.0f);
  settings->target_hyst_percent = sanitize_positive_or_default(
      modbus_get_rll400_target_hysteresis_percent(),
      WINDOWS_TARGET_HYST_DEFAULT_PERCENT);
  settings->motion_delta_percent = sanitize_positive_or_default(
      modbus_get_rll400_motion_delta_percent(),
      WINDOWS_MOTION_DELTA_DEFAULT_PERCENT);
  settings->no_motion_timeout_ms = modbus_get_rll400_no_motion_timeout_ms();

  settings->wind_limit_ms = clampf_local(settings->wind_limit_ms, 0.1f, 100.0f);
  settings->wind_storm_ms = clampf_local(settings->wind_storm_ms, 0.1f, 100.0f);
  settings->wind_recover_ms =
      clampf_local(settings->wind_recover_ms, 0.0f, settings->wind_storm_ms);
  settings->weather_stale_timeout_ms =
      (uint32_t)clampf_local((float)settings->weather_stale_timeout_ms, 1000.0f,
                             60000.0f);
  settings->weather_source_age_limit_s =
      (uint16_t)clampf_local((float)settings->weather_source_age_limit_s, 1.0f,
                             600.0f);
  settings->temp_step_target_percent =
      clampf_local(settings->temp_step_target_percent, 0.1f, 100.0f);
  settings->humidity_step_target_percent =
      clampf_local(settings->humidity_step_target_percent, 0.1f, 100.0f);
  settings->target_hyst_percent =
      clampf_local(settings->target_hyst_percent, 0.1f, 20.0f);
  settings->motion_delta_percent =
      clampf_local(settings->motion_delta_percent, 0.1f, 20.0f);
}

static void apply_window_motor_settings(const window_settings_t *settings) {
  if (settings == NULL) {
    return;
  }
  if (s_window_a_handle != NULL) {
    (void)rll400_set_target_hysteresis_percent(
        s_window_a_handle, settings->target_hyst_percent);
    (void)rll400_set_motion_fault_config(s_window_a_handle,
                                         settings->motion_delta_percent,
                                         settings->no_motion_timeout_ms);
  }
  if (s_window_b_handle != NULL) {
    (void)rll400_set_target_hysteresis_percent(
        s_window_b_handle, settings->target_hyst_percent);
    (void)rll400_set_motion_fault_config(s_window_b_handle,
                                         settings->motion_delta_percent,
                                         settings->no_motion_timeout_ms);
  }
}

static int update_step_index(int *runtime_step_index, float actual_value,
                             float setpoint_value, float step_value,
                             float step_hyst_value, uint16_t max_step_index) {
  if (runtime_step_index == NULL) {
    return 0;
  }

  int step_index = *runtime_step_index;
  if (step_index < 0) {
    step_index = 0;
  }
  if (step_index > (int)max_step_index) {
    step_index = (int)max_step_index;
  }

  const float delta_value = actual_value - setpoint_value;
  while (step_index < (int)max_step_index &&
         delta_value >= ((float)(step_index + 1) * step_value)) {
    ++step_index;
  }
  while (step_index > 0 &&
         delta_value < (((float)step_index * step_value) - step_hyst_value)) {
    --step_index;
  }

  *runtime_step_index = step_index;
  return step_index;
}

static float stabilize_wind_speed_for_target(
    float wind_speed_ms, float threshold_ms, window_wind_speed_hold_t *hold) {
  if (hold == NULL || !isfinite(wind_speed_ms)) {
    return wind_speed_ms;
  }

  if (wind_speed_ms < threshold_ms) {
    hold->initialized = false;
    hold->wind_speed_ms = threshold_ms;
    return wind_speed_ms;
  }

  if (!hold->initialized) {
    hold->initialized = true;
    hold->wind_speed_ms = threshold_ms;
  }

  if (fabsf(wind_speed_ms - hold->wind_speed_ms) >=
      WINDOWS_WIND_SPEED_TARGET_STEP_MS) {
    hold->wind_speed_ms = wind_speed_ms;
  }

  return hold->wind_speed_ms;
}

static window_wind_cap_t
calculate_wind_cap(float wind_speed_ms,
                   const window_wind_role_settings_t *role_settings,
                   window_wind_speed_hold_t *speed_hold) {
  window_wind_cap_t cap = {
      .active = false,
      .reduction_percent = 0.0f,
  };
  if (role_settings == NULL || !isfinite(wind_speed_ms)) {
    return cap;
  }

  const float calculation_wind_speed_ms = stabilize_wind_speed_for_target(
      wind_speed_ms, role_settings->speed_threshold_ms, speed_hold);
  if (calculation_wind_speed_ms < role_settings->speed_threshold_ms) {
    return cap;
  }

  const float excess_ms =
      calculation_wind_speed_ms - role_settings->speed_threshold_ms;
  if (excess_ms <= 0.0f) {
    return cap;
  }

  cap.active = true;
  cap.reduction_percent =
      excess_ms * role_settings->reduction_percent_per_ms;
  return cap;
}

static float apply_leeward_wind_cap(
    float target_percent, const window_wind_role_settings_t *role_settings,
    const window_wind_cap_t *cap) {
  if (role_settings == NULL || cap == NULL || !cap->active) {
    return target_percent;
  }

  return clampf_local(target_percent - cap->reduction_percent,
                      role_settings->min_percent, role_settings->max_percent);
}

static float apply_windward_wind_cap(
    float target_percent, const window_wind_role_settings_t *role_settings,
    const window_wind_cap_t *cap, float leeward_target_percent,
    float lag_percent) {
  if (role_settings == NULL || cap == NULL || !cap->active) {
    return target_percent;
  }

  float max_percent = fminf(role_settings->max_percent,
                            leeward_target_percent - lag_percent);
  max_percent = fmaxf(max_percent, role_settings->min_percent);
  return clampf_local(target_percent - cap->reduction_percent,
                      role_settings->min_percent, max_percent);
}

static float apply_rain_cap(float target_percent, float rain_cap_percent) {
  const float max_percent = clampf_local(rain_cap_percent, 0.0f, 100.0f);
  return clampf_local(fminf(target_percent, max_percent), 0.0f, max_percent);
}

static modbus_windows_windward_side_t
detect_windward_side(const modbus_weather_runtime_t *weather,
                     const window_settings_t *settings, bool weather_stale) {
  if (weather == NULL || settings == NULL || weather_stale) {
    return MODBUS_WINDOWS_WINDWARD_SIDE_NONE;
  }

  const uint16_t window_b_azimuth_deg =
      (uint16_t)((settings->window_a_azimuth_deg + 180U) % 360U);
  const float dist_a =
      angular_distance_deg((float)weather->wind_dir_deg,
                           (float)settings->window_a_azimuth_deg);
  const float dist_b =
      angular_distance_deg((float)weather->wind_dir_deg,
                           (float)window_b_azimuth_deg);
  const float half_width = (float)settings->wind_sector_half_width_deg;

  if (dist_a <= half_width && dist_a <= dist_b) {
    return MODBUS_WINDOWS_WINDWARD_SIDE_A;
  }
  if (dist_b <= half_width && dist_b < dist_a) {
    return MODBUS_WINDOWS_WINDWARD_SIDE_B;
  }
  return MODBUS_WINDOWS_WINDWARD_SIDE_NONE;
}

static void consume_fault_reset_tokens(window_pair_runtime_t *runtime) {
  if (runtime == NULL) {
    return;
  }

  const uint16_t token_a =
      modbus_get_window_fault_reset_token(MODBUS_WINDOW_CHANNEL_A);
  const uint16_t token_b =
      modbus_get_window_fault_reset_token(MODBUS_WINDOW_CHANNEL_B);

  if (token_a != 0U && token_a != runtime->last_fault_reset_token_a &&
      s_window_a_handle != NULL) {
    (void)rll400_reset_fault(s_window_a_handle);
    runtime->last_fault_reset_token_a = token_a;
  }

  if (token_b != 0U && token_b != runtime->last_fault_reset_token_b &&
      s_window_b_handle != NULL) {
    (void)rll400_reset_fault(s_window_b_handle);
    runtime->last_fault_reset_token_b = token_b;
  }
}

static esp_err_t init_hc595_outputs(void) {
  hc595_outputs_config_t cfg = {
      .data_gpio_num = HC595_DATA_GPIO,
      .clock_gpio_num = HC595_CLOCK_GPIO,
      .latch_gpio_num = HC595_LATCH_GPIO,
      .chip_count = HEATING_HC595_CHIP_COUNT,
      .relay1_bit_index = HC595_BIT_LIGHT_RELAY_1,
      .relay2_bit_index = HC595_BIT_LIGHT_RELAY_2,
      .valve_open_bit_index = HC595_BIT_HEATING_RAIL_VALVE_OPEN,
      .valve_close_bit_index = HC595_BIT_HEATING_RAIL_VALVE_CLOSE,
      .initial_state = 0U,
  };
  return hc595_outputs_init(&cfg, &s_hc595_outputs);
}

static esp_err_t apply_light_outputs(bool relay1_on, bool relay2_on) {
  if (s_hc595_outputs == NULL) {
    return ESP_ERR_INVALID_STATE;
  }
  return hc595_outputs_set_light_relays(s_hc595_outputs, relay1_on, relay2_on);
}

static uint32_t heating_output_bit_mask(uint8_t bit_index) {
  return (1UL << bit_index);
}

static uint32_t heating_all_outputs_mask(void) {
  uint32_t mask = 0U;
  for (uint8_t i = 0; i < HEATING_CONTOUR_COUNT; ++i) {
    mask |= heating_output_bit_mask(s_heating_contours[i].valve_open_bit);
    mask |= heating_output_bit_mask(s_heating_contours[i].valve_close_bit);
    mask |= heating_output_bit_mask(s_heating_contours[i].pump_bit);
  }
  return mask;
}

static esp_err_t apply_heating_outputs(uint16_t pump_mask,
                                       uint16_t valve_open_mask,
                                       uint16_t valve_close_mask) {
  if (s_hc595_outputs == NULL) {
    return ESP_ERR_INVALID_STATE;
  }

  uint32_t value = 0U;
  for (uint8_t i = 0; i < HEATING_CONTOUR_COUNT; ++i) {
    const uint16_t contour_mask = (uint16_t)(1U << i);
    if ((pump_mask & contour_mask) != 0U) {
      value |= heating_output_bit_mask(s_heating_contours[i].pump_bit);
    }
    const bool open_on = (valve_open_mask & contour_mask) != 0U;
    const bool close_on = (valve_close_mask & contour_mask) != 0U;
    if (open_on && !close_on) {
      value |= heating_output_bit_mask(s_heating_contours[i].valve_open_bit);
    } else if (close_on && !open_on) {
      value |= heating_output_bit_mask(s_heating_contours[i].valve_close_bit);
    }
  }
  return hc595_outputs_write_masked(s_hc595_outputs, heating_all_outputs_mask(),
                                    value);
}

static uint32_t co2_output_mask(void) {
  return (1UL << HC595_BIT_CO2_VALVE);
}

static esp_err_t apply_co2_outputs(bool valve_open) {
  if (s_hc595_outputs == NULL) {
    return ESP_ERR_INVALID_STATE;
  }
  uint32_t value = 0U;
  if (valve_open) {
    value |= (1UL << HC595_BIT_CO2_VALVE);
  }
  return hc595_outputs_write_masked(s_hc595_outputs, co2_output_mask(), value);
}

static uint32_t circulation_output_mask(void) {
  return (1UL << HC595_BIT_CIRC_FAN_A) | (1UL << HC595_BIT_CIRC_FAN_B);
}

static esp_err_t apply_circulation_outputs(uint16_t fan_mask) {
  if (s_hc595_outputs == NULL) {
    return ESP_ERR_INVALID_STATE;
  }
  uint32_t value = 0U;
  if ((fan_mask & 0x01U) != 0U) {
    value |= (1UL << HC595_BIT_CIRC_FAN_A);
  }
  if ((fan_mask & 0x02U) != 0U) {
    value |= (1UL << HC595_BIT_CIRC_FAN_B);
  }
  return hc595_outputs_write_masked(s_hc595_outputs,
                                    circulation_output_mask(), value);
}

static esp_err_t init_co2_controller(void) {
  co2_controller_config_t cfg = {
      .name = "greenhouse_co2",
  };
  return co2_controller_init(&cfg, &s_co2_handle);
}

static esp_err_t init_circulation_controller(void) {
  circulation_controller_config_t cfg = {
      .name = "greenhouse_circulation",
      .fan_mask = 0x0003U,
  };
  return circulation_controller_init(&cfg, &s_circulation_handle);
}

static esp_err_t init_heating_controllers(void) {
  for (uint8_t i = 0; i < HEATING_CONTOUR_COUNT; ++i) {
    valve_3way_config_t valve_cfg = {
        .name = s_heating_contours[i].name,
        .gpio_open_num = GPIO_NUM_NC,
        .gpio_close_num = GPIO_NUM_NC,
        .hysteresis_c = VALVE_TEMPERATURE_HYSTERESIS_C,
        .initial_setpoint_c = 0.0f,
        .initial_actual_temp_c = 0.0f,
    };
    esp_err_t err =
        valve_3way_init(&valve_cfg, &s_heating_contours[i].valve);
    if (err != ESP_OK) {
      return err;
    }
    s_heating_contours[i].inactive_since_ms = 0U;
  }
  return apply_heating_outputs(0U, 0U, 0U);
}

static uint8_t heating_calculate_auto_stage(const app_sensor_snapshot_t *sensors,
                                            uint16_t *status_bits) {
  if (sensors == NULL || !sensors->temp_air_valid) {
    if (status_bits != NULL) {
      *status_bits |= HEATING_STATUS_AIR_SENSOR_FAULT;
    }
    return 0U;
  }

  const float air_setpoint_c = sanitize_positive_or_default(
      modbus_get_heating_air_setpoint_c(), 20.0f);
  const float air_hyst_c =
      sanitize_nonnegative_or_default(modbus_get_heating_air_hysteresis_c(),
                                      0.5f);
  const float deficit_c = air_setpoint_c - sensors->temp_air;
  uint8_t requested_stage = 0U;

  if (s_heating_runtime.active_stage > 0U &&
      sensors->temp_air < (air_setpoint_c + air_hyst_c)) {
    requested_stage = s_heating_runtime.active_stage;
  }

  for (uint8_t stage = 0; stage < HEATING_CONTOUR_COUNT; ++stage) {
    const float delta_c = sanitize_nonnegative_or_default(
        modbus_get_heating_stage_delta_c(stage), 0.0f);
    if (deficit_c >= delta_c) {
      requested_stage = (uint8_t)(stage + 1U);
    }
  }

  return requested_stage;
}

static uint8_t heating_apply_min_timers(uint8_t requested_stage,
                                        uint16_t *status_bits) {
  const uint32_t now = (uint32_t)(esp_timer_get_time() / 1000ULL);
  if (!s_heating_runtime.initialized) {
    s_heating_runtime.initialized = true;
    s_heating_runtime.last_stage_change_ms = now;
    return requested_stage;
  }

  if (requested_stage == s_heating_runtime.active_stage) {
    return requested_stage;
  }

  const uint32_t elapsed_s =
      (now - s_heating_runtime.last_stage_change_ms) / 1000U;
  if (requested_stage > s_heating_runtime.active_stage) {
    const uint16_t min_off_s = modbus_get_heating_min_off_s();
    if (s_heating_runtime.active_stage == 0U && elapsed_s < min_off_s) {
      if (status_bits != NULL) {
        *status_bits |= HEATING_STATUS_MIN_OFF_HOLD;
      }
      return s_heating_runtime.active_stage;
    }
  } else {
    const uint16_t min_on_s = modbus_get_heating_min_on_s();
    if (s_heating_runtime.active_stage > 0U && elapsed_s < min_on_s) {
      if (status_bits != NULL) {
        *status_bits |= HEATING_STATUS_MIN_ON_HOLD;
      }
      return s_heating_runtime.active_stage;
    }
  }

  s_heating_runtime.last_stage_change_ms = now;
  return requested_stage;
}

static esp_err_t process_heating_controller(const app_sensor_snapshot_t *sensors) {
  if (sensors == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  uint16_t status_bits = 0U;
  uint16_t sensor_status_bits = 0U;
  uint16_t requested_active_mask = 0U;
  uint16_t pump_mask = 0U;
  uint16_t valve_open_mask = 0U;
  uint16_t valve_close_mask = 0U;

  if (!sensors->temp_air_valid) {
    sensor_status_bits |= HEATING_SENSOR_STATUS_AIR_FAULT;
  }
  for (uint8_t i = 0; i < HEATING_CONTOUR_COUNT; ++i) {
    if (!sensors->water_temp_valid[i]) {
      sensor_status_bits |= (uint16_t)(HEATING_SENSOR_STATUS_RAIL_WATER_FAULT
                                       << i);
    }
  }
  if ((sensor_status_bits & ~HEATING_SENSOR_STATUS_AIR_FAULT) != 0U) {
    status_bits |= HEATING_STATUS_WATER_SENSOR_FAULT;
  }

  const modbus_heating_ctrl_mode_t mode = modbus_get_heating_ctrl_mode();
  uint8_t active_stage = 0U;
  if (mode == MODBUS_HEATING_CTRL_MODE_MANUAL) {
    status_bits |= HEATING_STATUS_MANUAL_MODE;
    pump_mask =
        (uint16_t)(modbus_get_heating_manual_pump_mask() &
                   ((1U << HEATING_CONTOUR_COUNT) - 1U));
    valve_open_mask =
        (uint16_t)(modbus_get_heating_manual_valve_open_mask() &
                   ((1U << HEATING_CONTOUR_COUNT) - 1U));
    valve_close_mask =
        (uint16_t)(modbus_get_heating_manual_valve_close_mask() &
                   ((1U << HEATING_CONTOUR_COUNT) - 1U));
    valve_close_mask = (uint16_t)(valve_close_mask & ~valve_open_mask);
    active_stage = 0U;
  } else if (mode == MODBUS_HEATING_CTRL_MODE_OFF) {
    status_bits |= HEATING_STATUS_OFF_MODE;
  } else {
    active_stage = heating_calculate_auto_stage(sensors, &status_bits);
    if (sensors->temp_air_valid) {
      active_stage = heating_apply_min_timers(active_stage, &status_bits);
    }
    for (uint8_t i = 0; i < active_stage && i < HEATING_CONTOUR_COUNT; ++i) {
      requested_active_mask |= (uint16_t)(1U << i);
    }
  }

  const uint32_t now = (uint32_t)(esp_timer_get_time() / 1000ULL);
  if (mode == MODBUS_HEATING_CTRL_MODE_AUTO) {
    for (uint8_t i = 0; i < HEATING_CONTOUR_COUNT; ++i) {
      const uint16_t contour_mask = (uint16_t)(1U << i);
      const bool active = (requested_active_mask & contour_mask) != 0U;
      if (active && sensors->water_temp_valid[i]) {
        pump_mask |= contour_mask;
        const float setpoint_c =
            modbus_get_water_setpoint_c(s_heating_contours[i].water_channel);
        esp_err_t err = valve_3way_process_temperatures(
            s_heating_contours[i].valve, setpoint_c, sensors->water_temp[i]);
        if (err != ESP_OK) {
          return err;
        }
        s_heating_contours[i].inactive_since_ms = 0U;
      } else {
        if (s_heating_contours[i].inactive_since_ms == 0U) {
          s_heating_contours[i].inactive_since_ms = now;
        }
        if ((now - s_heating_contours[i].inactive_since_ms) <
            HEATING_SAFE_CLOSE_MS) {
          (void)valve_3way_close(s_heating_contours[i].valve);
        } else {
          (void)valve_3way_stop(s_heating_contours[i].valve);
        }
      }

      const valve_3way_state_t valve_state =
          valve_3way_get_state(s_heating_contours[i].valve);
      if (valve_state == VALVE_3WAY_STATE_OPENING) {
        valve_open_mask |= contour_mask;
      } else if (valve_state == VALVE_3WAY_STATE_CLOSING) {
        valve_close_mask |= contour_mask;
      }
    }
  } else if (mode != MODBUS_HEATING_CTRL_MODE_MANUAL) {
    for (uint8_t i = 0; i < HEATING_CONTOUR_COUNT; ++i) {
      (void)valve_3way_close(s_heating_contours[i].valve);
      valve_close_mask |= (uint16_t)(1U << i);
    }
  }

  if (pump_mask != 0U) {
    status_bits |= HEATING_STATUS_ENABLED;
  }

  esp_err_t err = apply_heating_outputs(pump_mask, valve_open_mask,
                                        valve_close_mask);
  if (err != ESP_OK) {
    return err;
  }

  s_heating_runtime.active_stage = active_stage;
  s_heating_runtime.active_mask = requested_active_mask;
  s_heating_runtime.pump_mask = pump_mask;
  modbus_set_heating_runtime(status_bits, active_stage, pump_mask,
                             valve_open_mask, valve_close_mask,
                             sensor_status_bits);

  ESP_LOGI(TAG,
           "Heating mode=%u air=%.2fC stage=%u pumps=0x%X open=0x%X close=0x%X "
           "sensors=0x%X",
           (unsigned)mode, sensors->temp_air, (unsigned)active_stage,
           (unsigned)pump_mask, (unsigned)valve_open_mask,
           (unsigned)valve_close_mask, (unsigned)sensor_status_bits);
  return ESP_OK;
}

static esp_err_t init_window_controllers(void) {
  rll400_config_t window_a_cfg = {
      .name = WINDOW_A_NAME,
      .i2c_port = I2C_MASTER_NUM,
      .ads_addr = WINDOW_A_ADS_ADDR,
      .ads_channel_pos = WINDOW_A_ADS_CH_POS,
      .ads_channel_neg = WINDOW_A_ADS_CH_NEG,
      .shunt_resistor_ohm = WINDOW_SENSOR_SHUNT_OHM,
      .pin_open = WINDOW_A_PIN_OPEN,
      .pin_close = WINDOW_A_PIN_CLOSE,
      .pin_local_manual = WINDOW_A_PIN_LOCAL_MANUAL,
      .local_manual_active_high = WINDOW_LOCAL_MANUAL_ACTIVE_HIGH,
      .target_hysteresis_percent = WINDOWS_TARGET_HYST_DEFAULT_PERCENT,
      .motion_delta_percent = WINDOWS_MOTION_DELTA_DEFAULT_PERCENT,
      .no_motion_timeout_ms = WINDOWS_NO_MOTION_DEFAULT_MS,
      .encoder_min_ma = WINDOW_ENCODER_MIN_MA,
      .encoder_max_ma = WINDOW_ENCODER_MAX_MA,
      .boot_wait_position_timeout_ms = WINDOW_BOOT_WAIT_POSITION_TIMEOUT_MS,
  };
  rll400_config_t window_b_cfg = {
      .name = WINDOW_B_NAME,
      .i2c_port = I2C_MASTER_NUM,
      .ads_addr = WINDOW_B_ADS_ADDR,
      .ads_channel_pos = WINDOW_B_ADS_CH_POS,
      .ads_channel_neg = WINDOW_B_ADS_CH_NEG,
      .shunt_resistor_ohm = WINDOW_SENSOR_SHUNT_OHM,
      .pin_open = WINDOW_B_PIN_OPEN,
      .pin_close = WINDOW_B_PIN_CLOSE,
      .pin_local_manual = WINDOW_B_PIN_LOCAL_MANUAL,
      .local_manual_active_high = WINDOW_LOCAL_MANUAL_ACTIVE_HIGH,
      .target_hysteresis_percent = WINDOWS_TARGET_HYST_DEFAULT_PERCENT,
      .motion_delta_percent = WINDOWS_MOTION_DELTA_DEFAULT_PERCENT,
      .no_motion_timeout_ms = WINDOWS_NO_MOTION_DEFAULT_MS,
      .encoder_min_ma = WINDOW_ENCODER_MIN_MA,
      .encoder_max_ma = WINDOW_ENCODER_MAX_MA,
      .boot_wait_position_timeout_ms = WINDOW_BOOT_WAIT_POSITION_TIMEOUT_MS,
  };

  esp_err_t err = rll400_init(&window_a_cfg, &s_window_a_handle);
  if (err != ESP_OK) {
    s_window_a_handle = NULL;
    return err;
  }
  err = rll400_init(&window_b_cfg, &s_window_b_handle);
  if (err != ESP_OK) {
    (void)rll400_del(s_window_a_handle);
    s_window_a_handle = NULL;
    s_window_b_handle = NULL;
    return err;
  }
  return ESP_OK;
}

static esp_err_t init_curtain_controller(void) {
  curtain_controller_config_t cfg = {
      .name = CURTAIN_NAME,
      .i2c_port = I2C_MASTER_NUM,
      .ads_addr = CURTAIN_ADS_ADDR,
      .ads_channel_pos = CURTAIN_ADS_CH_POS,
      .ads_channel_neg = CURTAIN_ADS_CH_NEG,
      .shunt_resistor_ohm = CURTAIN_SENSOR_SHUNT_OHM,
      .encoder_min_ma = CURTAIN_ENCODER_MIN_MA,
      .encoder_max_ma = CURTAIN_ENCODER_MAX_MA,
      .boot_wait_position_timeout_ms = CURTAIN_BOOT_WAIT_POSITION_TIMEOUT_MS,
      .motion_delta_percent = CURTAIN_MOTION_DELTA_DEFAULT_PERCENT,
      .no_motion_timeout_ms = CURTAIN_NO_MOTION_DEFAULT_MS,
      .outputs = s_hc595_outputs,
      .open_bit_index = HC595_BIT_CURTAIN_OPEN,
      .close_bit_index = HC595_BIT_CURTAIN_CLOSE,
  };
  return curtain_controller_init(&cfg, &s_curtain_handle);
}

static bool modbus_rtc_get_time_cb(uint8_t *hour, uint8_t *minute,
                                   uint8_t *second, void *ctx) {
  (void)ctx;
  if (!s_rtc_available || rtc_handle == NULL || hour == NULL || minute == NULL ||
      second == NULL) {
    return false;
  }

  ds3231_time_t rtc_time = {0};
  if (ds3231_get_time(rtc_handle, &rtc_time) != ESP_OK) {
    return false;
  }

  *hour = rtc_time.hour;
  *minute = rtc_time.minute;
  *second = rtc_time.second;
  return true;
}

static bool modbus_rtc_set_time_cb(uint8_t hour, uint8_t minute,
                                   uint8_t second, void *ctx) {
  (void)ctx;
  if (!s_rtc_available || rtc_handle == NULL) {
    return false;
  }

  ds3231_time_t rtc_time = {
      .hour = hour,
      .minute = minute,
      .second = second,
  };
  return ds3231_set_time(rtc_handle, &rtc_time) == ESP_OK;
}

static esp_err_t i2c_master_init(void) {
  i2c_config_t conf = {
      .mode = I2C_MODE_MASTER,
      .sda_io_num = I2C_MASTER_SDA_IO,
      .scl_io_num = I2C_MASTER_SCL_IO,
      .sda_pullup_en = GPIO_PULLUP_ENABLE,
      .scl_pullup_en = GPIO_PULLUP_ENABLE,
      .master.clk_speed = I2C_MASTER_FREQ_HZ,
  };
  i2c_param_config(I2C_MASTER_NUM, &conf);
  return i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE,
                            I2C_MASTER_TX_BUF_DISABLE, 0);
}

static void process_windows(const app_sensor_snapshot_t *sensor_snapshot,
                            float *out_pos_a_percent,
                            float *out_pos_b_percent) {
  if (s_window_a_handle == NULL || s_window_b_handle == NULL) {
    return;
  }

  window_settings_t settings = {0};
  modbus_weather_runtime_t weather = {0};
  read_window_settings(&settings);
  modbus_get_weather_runtime(&weather);
  apply_window_motor_settings(&settings);
  consume_fault_reset_tokens(&s_window_runtime);

  rll400_status_t status_a = {0};
  rll400_status_t status_b = {0};
  (void)rll400_get_runtime_status(s_window_a_handle, &status_a);
  (void)rll400_get_runtime_status(s_window_b_handle, &status_b);

  const modbus_mode_state_t modbus_mode = modbus_get_mode_state();
  const bool modbus_autonomous = (modbus_mode == MODBUS_MODE_AUTONOMOUS);
  const bool system_manual_mode =
      (settings.ctrl_mode == MODBUS_WINDOWS_CTRL_MODE_MANUAL);
  const bool temp_sensor_fault =
      (sensor_snapshot == NULL || !sensor_snapshot->temp_air_valid);
  const bool humidity_algorithm =
      (settings.auto_algo_mode == MODBUS_WINDOWS_AUTO_ALGO_HUMIDITY);
  const bool hum_sensor_fault =
      humidity_algorithm && (sensor_snapshot == NULL || !sensor_snapshot->rh_valid);
  const bool algo_sensor_fault =
      humidity_algorithm ? hum_sensor_fault : temp_sensor_fault;
  const bool freeze_targets = temp_sensor_fault || algo_sensor_fault;

  float base_target_a = status_a.target_percent;
  float base_target_b = status_b.target_percent;

  if (freeze_targets) {
    base_target_a = status_a.target_percent;
    base_target_b = status_b.target_percent;
  } else if (system_manual_mode) {
    base_target_a = modbus_get_window_a_target_percent();
    base_target_b = modbus_get_window_b_target_percent();
  } else if (humidity_algorithm) {
    const uint16_t humidity_step_limit =
        derive_step_index_limit(settings.humidity_step_target_percent);
    const int step_index = update_step_index(
        &s_window_runtime.humidity_step_index, sensor_snapshot->rh,
        settings.humidity_setpoint_percent, settings.humidity_step_percent,
        settings.humidity_step_hyst_percent, humidity_step_limit);
    const float target_percent =
        clampf_local((float)step_index * settings.humidity_step_target_percent,
                     0.0f, 100.0f);
    base_target_a = target_percent;
    base_target_b = target_percent;
  } else {
    const uint16_t temp_step_limit =
        derive_step_index_limit(settings.temp_step_target_percent);
    const int step_index = update_step_index(
        &s_window_runtime.temp_step_index, sensor_snapshot->temp_air,
        settings.temp_setpoint_c, settings.temp_step_c,
        settings.temp_step_hyst_c, temp_step_limit);
    const float target_percent =
        clampf_local((float)step_index * settings.temp_step_target_percent,
                     0.0f, 100.0f);
    base_target_a = target_percent;
    base_target_b = target_percent;
  }

  const bool weather_status_fault =
      (weather.status_bits & WEATHER_STATUS_LAST_ERROR) != 0U;
  const bool weather_stale =
      (!weather.valid || weather.stale || weather_status_fault ||
       weather.rx_age_ms > settings.weather_stale_timeout_ms ||
       weather.source_age_s > settings.weather_source_age_limit_s);
  modbus_windows_windward_side_t windward_side =
      detect_windward_side(&weather, &settings, weather_stale);

  if (!temp_sensor_fault) {
    const float close_at_c =
        settings.temp_setpoint_c - settings.cold_close_delta_c;
    if (s_window_runtime.cold_close_active) {
      s_window_runtime.cold_close_active =
          sensor_snapshot->temp_air <= (close_at_c + settings.cold_close_hyst_c);
    } else if (sensor_snapshot->temp_air <= close_at_c) {
      s_window_runtime.cold_close_active = true;
    }
  } else {
    s_window_runtime.cold_close_active = false;
  }

  if (weather_stale) {
    s_window_runtime.storm_active = false;
    s_window_runtime.wind_limit_a_active = false;
    s_window_runtime.wind_limit_b_active = false;
  } else {
    if (s_window_runtime.storm_active) {
      s_window_runtime.storm_active =
          weather.wind_speed_ms > settings.wind_recover_ms;
    } else if (weather.wind_speed_ms >= settings.wind_storm_ms) {
      s_window_runtime.storm_active = true;
    }

    s_window_runtime.wind_limit_a_active = false;
    s_window_runtime.wind_limit_b_active = false;
  }

  float final_target_a = base_target_a;
  float final_target_b = base_target_b;
  const bool force_safe_active = settings.force_safe_cmd;
  const bool weather_safe_active =
      weather_stale &&
      settings.weather_stale_policy == MODBUS_WINDOWS_WEATHER_STALE_CLOSE_SAFE;
  bool rain_limit_a_active = false;
  bool rain_limit_b_active = false;

  if (force_safe_active) {
    final_target_a = settings.safe_min_percent;
    final_target_b = settings.safe_min_percent;
  } else if (s_window_runtime.cold_close_active) {
    final_target_a = 0.0f;
    final_target_b = 0.0f;
  } else if (weather_safe_active || s_window_runtime.storm_active) {
    final_target_a = settings.safe_min_percent;
    final_target_b = settings.safe_min_percent;
  } else {
    if (!weather_stale &&
        windward_side != MODBUS_WINDOWS_WINDWARD_SIDE_NONE) {
      const window_wind_cap_t windward_cap =
          calculate_wind_cap(weather.wind_speed_ms, &settings.windward,
                             &s_window_runtime.windward_speed_hold);
      const window_wind_cap_t leeward_cap =
          calculate_wind_cap(weather.wind_speed_ms, &settings.leeward,
                             &s_window_runtime.leeward_speed_hold);

      if (windward_side == MODBUS_WINDOWS_WINDWARD_SIDE_A) {
        if (leeward_cap.active) {
          final_target_b =
              apply_leeward_wind_cap(final_target_b, &settings.leeward,
                                      &leeward_cap);
          s_window_runtime.wind_limit_b_active = true;
        }
        if (windward_cap.active) {
          final_target_a = apply_windward_wind_cap(
              final_target_a, &settings.windward, &windward_cap, final_target_b,
              settings.windward_lag_percent);
          s_window_runtime.wind_limit_a_active = true;
        }
      } else if (windward_side == MODBUS_WINDOWS_WINDWARD_SIDE_B) {
        if (leeward_cap.active) {
          final_target_a =
              apply_leeward_wind_cap(final_target_a, &settings.leeward,
                                      &leeward_cap);
          s_window_runtime.wind_limit_a_active = true;
        }
        if (windward_cap.active) {
          final_target_b = apply_windward_wind_cap(
              final_target_b, &settings.windward, &windward_cap, final_target_a,
              settings.windward_lag_percent);
          s_window_runtime.wind_limit_b_active = true;
        }
      }
    }

    if (!weather_stale && weather.rain_active &&
        settings.rain_mode == MODBUS_WINDOWS_RAIN_MODE_WINDWARD) {
      if (windward_side == MODBUS_WINDOWS_WINDWARD_SIDE_A) {
        final_target_a =
            apply_rain_cap(final_target_a, settings.rain_windward_percent);
        rain_limit_a_active = true;
      } else if (windward_side == MODBUS_WINDOWS_WINDWARD_SIDE_B) {
        final_target_b =
            apply_rain_cap(final_target_b, settings.rain_windward_percent);
        rain_limit_b_active = true;
      } else {
        windward_side = MODBUS_WINDOWS_WINDWARD_SIDE_BOTH_UNKNOWN;
        final_target_a =
            apply_rain_cap(final_target_a, settings.rain_windward_percent);
        final_target_b =
            apply_rain_cap(final_target_b, settings.rain_windward_percent);
        rain_limit_a_active = true;
        rain_limit_b_active = true;
      }
    }
  }

  uint16_t active_protection_bits = 0U;
  if (force_safe_active) {
    active_protection_bits |= WINDOWS_PROTECTION_FORCE_SAFE;
  }
  if (weather_safe_active) {
    active_protection_bits |= WINDOWS_PROTECTION_WEATHER_STALE;
  }
  if (s_window_runtime.storm_active) {
    active_protection_bits |= WINDOWS_PROTECTION_STORM;
  }
  if (s_window_runtime.cold_close_active) {
    active_protection_bits |= WINDOWS_PROTECTION_COLD_CLOSE;
  }
  if (rain_limit_a_active) {
    active_protection_bits |= WINDOWS_PROTECTION_RAIN_A;
  }
  if (rain_limit_b_active) {
    active_protection_bits |= WINDOWS_PROTECTION_RAIN_B;
  }
  if (s_window_runtime.wind_limit_a_active) {
    active_protection_bits |= WINDOWS_PROTECTION_WIND_A;
  }
  if (s_window_runtime.wind_limit_b_active) {
    active_protection_bits |= WINDOWS_PROTECTION_WIND_B;
  }
  if (temp_sensor_fault) {
    active_protection_bits |= WINDOWS_PROTECTION_TEMP_SENSOR_FAULT;
  }
  if (hum_sensor_fault) {
    active_protection_bits |= WINDOWS_PROTECTION_HUM_SENSOR_FAULT;
  }

  const bool hard_safety_active =
      force_safe_active || s_window_runtime.cold_close_active ||
      weather_safe_active || s_window_runtime.storm_active;
  if (!s_window_runtime.effective_target_initialized) {
    s_window_runtime.last_effective_target_a_percent = final_target_a;
    s_window_runtime.last_effective_target_b_percent = final_target_b;
    s_window_runtime.effective_target_initialized = true;
  } else {
    final_target_a = stabilize_effective_target(
        final_target_a, s_window_runtime.last_effective_target_a_percent,
        settings.target_hyst_percent, hard_safety_active || rain_limit_a_active);
    final_target_b = stabilize_effective_target(
        final_target_b, s_window_runtime.last_effective_target_b_percent,
        settings.target_hyst_percent, hard_safety_active || rain_limit_b_active);
    s_window_runtime.last_effective_target_a_percent = final_target_a;
    s_window_runtime.last_effective_target_b_percent = final_target_b;
  }

  (void)rll400_set_target(s_window_a_handle, final_target_a);
  (void)rll400_set_target(s_window_b_handle, final_target_b);
  (void)rll400_process(s_window_a_handle);
  (void)rll400_process(s_window_b_handle);
  (void)rll400_get_runtime_status(s_window_a_handle, &status_a);
  (void)rll400_get_runtime_status(s_window_b_handle, &status_b);

  uint16_t windows_status_bits = 0U;
  if (force_safe_active) {
    windows_status_bits |= WINDOWS_STATUS_FORCE_SAFE_ACTIVE;
  }
  if (weather_stale) {
    windows_status_bits |= WINDOWS_STATUS_WEATHER_STALE;
  }
  if (s_window_runtime.storm_active) {
    windows_status_bits |= WINDOWS_STATUS_STORM_ACTIVE;
  }
  if (s_window_runtime.wind_limit_a_active) {
    windows_status_bits |= WINDOWS_STATUS_WIND_LIMIT_A;
  }
  if (s_window_runtime.wind_limit_b_active) {
    windows_status_bits |= WINDOWS_STATUS_WIND_LIMIT_B;
  }
  if (temp_sensor_fault) {
    windows_status_bits |= WINDOWS_STATUS_TEMP_SENSOR_FAULT;
  }
  if (hum_sensor_fault) {
    windows_status_bits |= WINDOWS_STATUS_HUM_SENSOR_FAULT;
  }
  if (system_manual_mode) {
    windows_status_bits |= WINDOWS_STATUS_MANUAL_SYSTEM_MODE;
  }
  if (modbus_autonomous) {
    windows_status_bits |= WINDOWS_STATUS_MODBUS_AUTONOMOUS;
  }
  if (s_window_runtime.cold_close_active) {
    windows_status_bits |= WINDOWS_STATUS_COLD_CLOSE_ACTIVE;
  }
  if (rain_limit_a_active) {
    windows_status_bits |= WINDOWS_STATUS_RAIN_LIMIT_A;
  }
  if (rain_limit_b_active) {
    windows_status_bits |= WINDOWS_STATUS_RAIN_LIMIT_B;
  }
  if (humidity_algorithm) {
    windows_status_bits |= WINDOWS_STATUS_ALGO_HUMIDITY;
  }

  modbus_set_windows_runtime(
      windows_status_bits, build_window_status_bits(&status_a),
      build_window_status_bits(&status_b), (uint16_t)status_a.fault_code,
      (uint16_t)status_b.fault_code,
      temp_sensor_fault ? AIR_TEMP_SENSOR_STATUS_FAULT : AIR_TEMP_SENSOR_STATUS_OK,
      status_a.local_manual_active ? 1U : 0U,
      status_b.local_manual_active ? 1U : 0U);
  modbus_set_windows_target_diagnostics(
      base_target_a, base_target_b, final_target_a, final_target_b,
      active_protection_bits, windward_side);

  if (out_pos_a_percent != NULL) {
    *out_pos_a_percent = status_a.position_percent;
  }
  if (out_pos_b_percent != NULL) {
    *out_pos_b_percent = status_b.position_percent;
  }
}

static void process_curtain(const app_sensor_snapshot_t *sensor_snapshot,
                            float *out_pos_percent) {
  if (s_curtain_handle == NULL) {
    return;
  }

  modbus_weather_runtime_t weather = {0};
  modbus_get_weather_runtime(&weather);

  ds3231_time_t now = {0};
  bool time_valid = false;
  if (s_rtc_available && rtc_handle != NULL &&
      ds3231_get_time(rtc_handle, &now) == ESP_OK) {
    time_valid = true;
  } else {
    const uint32_t sec_of_day =
        (uint32_t)((esp_timer_get_time() / 1000000ULL) % 86400ULL);
    now.hour = (uint8_t)((sec_of_day / 3600U) % 24U);
    now.minute = (uint8_t)((sec_of_day % 3600U) / 60U);
    now.second = (uint8_t)(sec_of_day % 60U);
    time_valid = true;
  }

  const modbus_curtain_ctrl_mode_t raw_mode = modbus_get_curtain_ctrl_mode();
  curtain_controller_mode_t mode = CURTAIN_CONTROLLER_MODE_MANUAL;
  if (raw_mode == MODBUS_CURTAIN_CTRL_MODE_AUTO) {
    mode = CURTAIN_CONTROLLER_MODE_AUTO;
  } else if (raw_mode == MODBUS_CURTAIN_CTRL_MODE_OFF) {
    mode = CURTAIN_CONTROLLER_MODE_OFF;
  }

  curtain_controller_settings_t settings = {
      .mode = mode,
      .schedule_start_hhmm = modbus_get_curtain_schedule_start_hhmm(),
      .schedule_end_hhmm = modbus_get_curtain_schedule_end_hhmm(),
      .manual_target_percent = modbus_get_curtain_manual_target_percent(),
      .outside_target_percent = modbus_get_curtain_outside_target_percent(),
      .position_hysteresis_percent =
          modbus_get_curtain_position_hysteresis_percent(),
      .min_position_percent = modbus_get_curtain_min_position_percent(),
      .max_position_percent = modbus_get_curtain_max_position_percent(),
      .radiation_threshold_wm2 =
          modbus_get_curtain_radiation_threshold_wm2(),
      .radiation_step_wm2 = modbus_get_curtain_radiation_step_wm2(),
      .radiation_step_percent =
          modbus_get_curtain_radiation_step_percent(),
      .radiation_hysteresis_wm2 =
          modbus_get_curtain_radiation_hysteresis_wm2(),
      .cold_delta_c = modbus_get_curtain_cold_delta_c(),
      .cold_hysteresis_c = modbus_get_curtain_cold_hysteresis_c(),
      .cold_target_percent = modbus_get_curtain_cold_target_percent(),
      .heat_delta_c = modbus_get_curtain_heat_delta_c(),
      .heat_hysteresis_c = modbus_get_curtain_heat_hysteresis_c(),
      .heat_target_percent = modbus_get_curtain_heat_target_percent(),
      .humidity_low_threshold_percent =
          modbus_get_curtain_humidity_low_threshold_percent(),
      .humidity_low_hysteresis_percent =
          modbus_get_curtain_humidity_low_hysteresis_percent(),
      .humidity_low_target_percent =
          modbus_get_curtain_humidity_low_target_percent(),
      .humidity_high_threshold_percent =
          modbus_get_curtain_humidity_high_threshold_percent(),
      .humidity_high_hysteresis_percent =
          modbus_get_curtain_humidity_high_hysteresis_percent(),
      .humidity_high_target_percent =
          modbus_get_curtain_humidity_high_target_percent(),
      .humidity_setpoint_percent = modbus_get_air_hum_target_percent(),
      .humidity_setpoint_valid = true,
  };

  const float motion_delta_percent = clampf_local(
      sanitize_positive_or_default(modbus_get_rll400_motion_delta_percent(),
                                   WINDOWS_MOTION_DELTA_DEFAULT_PERCENT),
      0.1f, 20.0f);
  const uint32_t no_motion_timeout_ms = modbus_get_rll400_no_motion_timeout_ms();
  (void)curtain_controller_set_motion_fault_config(
      s_curtain_handle, motion_delta_percent, no_motion_timeout_ms);

  curtain_controller_inputs_t inputs = {
      .hour = now.hour,
      .minute = now.minute,
      .second = now.second,
      .time_valid = time_valid,
      .air_temp_c = sensor_snapshot != NULL ? sensor_snapshot->temp_air : 0.0f,
      .air_temp_valid =
          sensor_snapshot != NULL && sensor_snapshot->temp_air_valid,
      .humidity_percent = sensor_snapshot != NULL ? sensor_snapshot->rh : 0.0f,
      .humidity_valid = sensor_snapshot != NULL && sensor_snapshot->rh_valid,
      .radiation_wm2 = weather.solar_radiation_wm2,
      .radiation_valid = weather.valid && !weather.stale,
      .temp_setpoint_c = modbus_get_air_temp_target_c(),
      .temp_setpoint_valid = true,
  };

  static uint16_t last_fault_reset_token = 0U;
  const uint16_t reset_token = modbus_get_curtain_fault_reset_token();
  if (reset_token != 0U && reset_token != last_fault_reset_token) {
    if (curtain_controller_reset_fault(s_curtain_handle) == ESP_OK) {
      ESP_LOGI(TAG, "Curtain fault reset token accepted: %u",
               (unsigned)reset_token);
    }
    last_fault_reset_token = reset_token;
  }

  const esp_err_t err =
      curtain_controller_process(s_curtain_handle, &settings, &inputs);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Curtain processing failed: %s", esp_err_to_name(err));
  }

  curtain_controller_status_t status = {0};
  if (curtain_controller_get_status(s_curtain_handle, &status) == ESP_OK) {
    modbus_set_curtain_runtime(status.target_percent,
                               status.base_target_percent, status.current_ma,
                               status.status_bits, status.reason_bits,
                               status.position_status_bits,
                               (uint16_t)status.fault_code);
    if (out_pos_percent != NULL) {
      *out_pos_percent = status.position_percent;
    }
  }
}

static void process_co2(const app_sensor_snapshot_t *sensor_snapshot,
                        float window_a_pos_percent,
                        float window_b_pos_percent,
                        co2_controller_status_t *out_status) {
  if (s_co2_handle == NULL) {
    return;
  }

  modbus_weather_runtime_t weather = {0};
  modbus_get_weather_runtime(&weather);

  ds3231_time_t now = {0};
  bool time_valid = false;
  if (s_rtc_available && rtc_handle != NULL &&
      ds3231_get_time(rtc_handle, &now) == ESP_OK) {
    time_valid = true;
  } else {
    const uint32_t sec_of_day =
        (uint32_t)((esp_timer_get_time() / 1000000ULL) % 86400ULL);
    now.hour = (uint8_t)((sec_of_day / 3600U) % 24U);
    now.minute = (uint8_t)((sec_of_day % 3600U) / 60U);
    now.second = (uint8_t)(sec_of_day % 60U);
    time_valid = true;
  }

  const modbus_co2_ctrl_mode_t raw_mode = modbus_get_co2_ctrl_mode();
  co2_controller_mode_t mode = CO2_CONTROLLER_MODE_OFF;
  if (raw_mode == MODBUS_CO2_CTRL_MODE_AUTO) {
    mode = CO2_CONTROLLER_MODE_AUTO;
  } else if (raw_mode == MODBUS_CO2_CTRL_MODE_MANUAL) {
    mode = CO2_CONTROLLER_MODE_MANUAL;
  }

  const uint16_t manual_outputs = modbus_get_co2_manual_outputs();
  co2_controller_settings_t settings = {
      .mode = mode,
      .manual_valve_open = (manual_outputs & 0x01U) != 0U,
      .manual_fan_on = (manual_outputs & 0x02U) != 0U,
      .schedule_start_hhmm = modbus_get_co2_schedule_start_hhmm(),
      .schedule_end_hhmm = modbus_get_co2_schedule_end_hhmm(),
      .low_light_threshold_wm2 = modbus_get_co2_low_light_threshold_wm2(),
      .mid_light_threshold_wm2 = modbus_get_co2_mid_light_threshold_wm2(),
      .high_light_threshold_wm2 = modbus_get_co2_high_light_threshold_wm2(),
      .low_light_target_ppm = modbus_get_co2_low_light_target_ppm(),
      .mid_light_target_ppm = modbus_get_co2_mid_light_target_ppm(),
      .high_light_target_ppm = modbus_get_co2_high_light_target_ppm(),
      .ventilation_limit_low_percent =
          modbus_get_co2_vent_limit_low_percent(),
      .ventilation_limit_high_percent =
          modbus_get_co2_vent_limit_high_percent(),
      .ventilation_cutoff_percent = modbus_get_co2_vent_cutoff_percent(),
      .dosing_hysteresis_ppm = modbus_get_co2_dosing_hysteresis_ppm(),
      .max_safe_ppm = modbus_get_co2_max_safe_ppm(),
      .max_dosing_time_s = modbus_get_co2_max_dosing_time_s(),
      .min_pause_time_s = modbus_get_co2_min_pause_time_s(),
      .no_rise_check_time_s = modbus_get_co2_no_rise_check_time_s(),
      .no_rise_min_delta_ppm = modbus_get_co2_no_rise_min_delta_ppm(),
      .temp_high_delta_c = modbus_get_co2_temp_high_delta_c(),
      .temp_critical_delta_c = modbus_get_co2_temp_critical_delta_c(),
      .humidity_high_delta_percent =
          modbus_get_co2_humidity_high_delta_percent(),
  };

  if (settings.ventilation_limit_high_percent <
      settings.ventilation_limit_low_percent) {
    settings.ventilation_limit_high_percent = settings.ventilation_limit_low_percent;
  }
  if (settings.ventilation_cutoff_percent <
      settings.ventilation_limit_high_percent) {
    settings.ventilation_cutoff_percent = settings.ventilation_limit_high_percent;
  }
  if (settings.temp_critical_delta_c < settings.temp_high_delta_c) {
    settings.temp_critical_delta_c = settings.temp_high_delta_c;
  }

  static uint16_t last_fault_reset_token = 0U;
  const uint16_t reset_token = modbus_get_co2_fault_reset_token();
  if (reset_token != 0U && reset_token != last_fault_reset_token) {
    if (co2_controller_reset_fault(s_co2_handle) == ESP_OK) {
      ESP_LOGI(TAG, "CO2 fault reset token accepted: %u",
               (unsigned)reset_token);
    }
    last_fault_reset_token = reset_token;
  }

  co2_controller_inputs_t inputs = {
      .hour = now.hour,
      .minute = now.minute,
      .second = now.second,
      .time_valid = time_valid,
      .co2_ppm = modbus_get_co2_measured_ppm(),
      .co2_valid = modbus_get_co2_sensor_valid(),
      .radiation_wm2 = weather.solar_radiation_wm2,
      .radiation_valid = weather.valid && !weather.stale,
      .air_temp_c = sensor_snapshot != NULL ? sensor_snapshot->temp_air : 0.0f,
      .air_temp_valid =
          sensor_snapshot != NULL && sensor_snapshot->temp_air_valid,
      .air_temp_target_c = modbus_get_air_temp_target_c(),
      .air_temp_target_valid = true,
      .humidity_percent = sensor_snapshot != NULL ? sensor_snapshot->rh : 0.0f,
      .humidity_valid = sensor_snapshot != NULL && sensor_snapshot->rh_valid,
      .humidity_target_percent = modbus_get_air_hum_target_percent(),
      .humidity_target_valid = true,
      .window_a_percent = window_a_pos_percent,
      .window_a_valid = s_window_a_handle != NULL,
      .window_b_percent = window_b_pos_percent,
      .window_b_valid = s_window_b_handle != NULL,
      .ventilation_valid = s_window_a_handle != NULL && s_window_b_handle != NULL,
      .external_safety_alarm = modbus_get_co2_external_alarm(),
  };

  const esp_err_t err = co2_controller_process(s_co2_handle, &settings, &inputs);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "CO2 processing failed: %s", esp_err_to_name(err));
    (void)apply_co2_outputs(false);
    return;
  }

  co2_controller_status_t status = {0};
  if (co2_controller_get_status(s_co2_handle, &status) == ESP_OK) {
    esp_err_t output_err = apply_co2_outputs(status.valve_open);
    if (output_err != ESP_OK) {
      ESP_LOGW(TAG, "CO2 output write failed: %s", esp_err_to_name(output_err));
      (void)apply_co2_outputs(false);
    }
    modbus_set_co2_runtime(status.target_ppm, status.effective_target_ppm,
                           status.status_bits, status.reason_bits,
                           status.active_protection_bits,
                           (uint16_t)status.fault_code,
                           status.dosing_elapsed_s);
    if (out_status != NULL) {
      *out_status = status;
    }
  }
}

static void process_circulation(const app_sensor_snapshot_t *sensor_snapshot,
                                const co2_controller_status_t *co2_status,
                                float window_a_pos_percent,
                                float window_b_pos_percent,
                                float curtain_pos_percent) {
  if (s_circulation_handle == NULL) {
    return;
  }

  ds3231_time_t now = {0};
  bool time_valid = false;
  if (s_rtc_available && rtc_handle != NULL &&
      ds3231_get_time(rtc_handle, &now) == ESP_OK) {
    time_valid = true;
  } else {
    const uint32_t sec_of_day =
        (uint32_t)((esp_timer_get_time() / 1000000ULL) % 86400ULL);
    now.hour = (uint8_t)((sec_of_day / 3600U) % 24U);
    now.minute = (uint8_t)((sec_of_day % 3600U) / 60U);
    now.second = (uint8_t)(sec_of_day % 60U);
    time_valid = true;
  }

  const modbus_circulation_ctrl_mode_t raw_mode =
      modbus_get_circulation_ctrl_mode();
  circulation_controller_mode_t mode = CIRCULATION_CONTROLLER_MODE_OFF;
  if (raw_mode == MODBUS_CIRC_CTRL_MODE_AUTO) {
    mode = CIRCULATION_CONTROLLER_MODE_AUTO;
  } else if (raw_mode == MODBUS_CIRC_CTRL_MODE_MANUAL) {
    mode = CIRCULATION_CONTROLLER_MODE_MANUAL;
  }

  circulation_controller_settings_t settings = {
      .mode = mode,
      .manual_fan_mask = modbus_get_circulation_manual_fan_mask(),
      .available_fan_mask = modbus_get_circulation_available_fan_mask(),
      .schedule_start_hhmm = modbus_get_circulation_schedule_start_hhmm(),
      .schedule_end_hhmm = modbus_get_circulation_schedule_end_hhmm(),
      .co2_fan_mask = modbus_get_circulation_co2_fan_mask(),
      .heating_fan_mask = modbus_get_circulation_heating_fan_mask(),
      .humidity_fan_mask = modbus_get_circulation_humidity_fan_mask(),
      .day_fan_mask = modbus_get_circulation_day_fan_mask(),
      .night_fan_mask = modbus_get_circulation_night_fan_mask(),
      .vent_limited_fan_mask =
          modbus_get_circulation_vent_limited_fan_mask(),
      .ventilation_limit_percent =
          modbus_get_circulation_vent_limit_percent(),
      .ventilation_cutoff_percent =
          modbus_get_circulation_vent_cutoff_percent(),
      .humidity_high_delta_percent =
          modbus_get_circulation_humidity_high_delta_percent(),
      .day_cycle_on_s = modbus_get_circulation_day_cycle_on_s(),
      .day_cycle_off_s = modbus_get_circulation_day_cycle_off_s(),
      .night_cycle_on_s = modbus_get_circulation_night_cycle_on_s(),
      .night_cycle_off_s = modbus_get_circulation_night_cycle_off_s(),
      .humidity_cycle_on_s = modbus_get_circulation_humidity_cycle_on_s(),
      .humidity_cycle_off_s = modbus_get_circulation_humidity_cycle_off_s(),
      .min_on_s = modbus_get_circulation_min_on_s(),
      .min_off_s = modbus_get_circulation_min_off_s(),
  };
  if (settings.ventilation_cutoff_percent <
      settings.ventilation_limit_percent) {
    settings.ventilation_cutoff_percent = settings.ventilation_limit_percent;
  }

  circulation_controller_inputs_t inputs = {
      .hour = now.hour,
      .minute = now.minute,
      .second = now.second,
      .time_valid = time_valid,
      .co2_dosing_active = co2_status != NULL && co2_status->mixing_requested,
      .heating_active = s_heating_runtime.pump_mask != 0U,
      .humidity_percent = sensor_snapshot != NULL ? sensor_snapshot->rh : 0.0f,
      .humidity_valid = sensor_snapshot != NULL && sensor_snapshot->rh_valid,
      .humidity_target_percent = modbus_get_air_hum_target_percent(),
      .humidity_target_valid = true,
      .window_a_percent = window_a_pos_percent,
      .window_a_valid = s_window_a_handle != NULL,
      .window_b_percent = window_b_pos_percent,
      .window_b_valid = s_window_b_handle != NULL,
      .ventilation_valid = s_window_a_handle != NULL && s_window_b_handle != NULL,
      .curtain_percent = curtain_pos_percent,
      .curtain_valid = s_curtain_handle != NULL,
  };

  const esp_err_t err =
      circulation_controller_process(s_circulation_handle, &settings, &inputs);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Circulation processing failed: %s", esp_err_to_name(err));
    (void)apply_circulation_outputs(0U);
    return;
  }

  circulation_controller_status_t status = {0};
  if (circulation_controller_get_status(s_circulation_handle, &status) ==
      ESP_OK) {
    esp_err_t output_err = apply_circulation_outputs(status.output_fan_mask);
    if (output_err != ESP_OK) {
      ESP_LOGW(TAG, "Circulation output write failed: %s",
               esp_err_to_name(output_err));
      (void)apply_circulation_outputs(0U);
    }
    modbus_set_circulation_runtime(status.output_fan_mask, status.status_bits,
                                   status.reason_bits,
                                   status.protection_bits,
                                   status.requested_fan_mask);
  }
}

static void control_task(void *arg) {
  (void)arg;
  ESP_LOGI(TAG, "Control task started (%ums)", (unsigned)CONTROL_LOOP_MS);

  while (1) {
    bool relay1_on = false;
    bool relay2_on = false;
    modbus_get_light_relay_state(&relay1_on, &relay2_on);
    (void)apply_light_outputs(relay1_on, relay2_on);

    const app_sensor_snapshot_t sensors = get_sensor_snapshot();
    float window_a_pos_percent = 0.0f;
    float window_b_pos_percent = 0.0f;
    float curtain_pos_percent = 0.0f;
    process_windows(&sensors, &window_a_pos_percent, &window_b_pos_percent);
    process_curtain(&sensors, &curtain_pos_percent);
    co2_controller_status_t co2_status = {0};
    process_co2(&sensors, window_a_pos_percent, window_b_pos_percent,
                &co2_status);
    process_circulation(&sensors, &co2_status, window_a_pos_percent,
                        window_b_pos_percent, curtain_pos_percent);

    modbus_set_telemetry(sensors.temp_air, sensors.rh,
                         sensors.water_temp[HEATING_CONTOUR_RAIL],
                         sensors.water_temp[HEATING_CONTOUR_GROW],
                         sensors.water_temp[HEATING_CONTOUR_UNDERTRAY],
                         sensors.water_temp[HEATING_CONTOUR_UPPER],
                         window_a_pos_percent, window_b_pos_percent,
                         curtain_pos_percent);

    vTaskDelay(pdMS_TO_TICKS(CONTROL_LOOP_MS));
  }
}

static void configure_runtime_log_levels(void) {
  if (APP_LOG_WINDOWS_ONLY) {
    esp_log_level_set("*", ESP_LOG_NONE);
  }
  esp_log_level_set("rll400", ESP_LOG_INFO);
  esp_log_level_set("curtain_controller", ESP_LOG_INFO);
  esp_log_level_set("co2_controller", ESP_LOG_INFO);
  esp_log_level_set("circulation_controller", ESP_LOG_INFO);
}

static void worker_task(void *arg) {
  (void)arg;
  ESP_LOGI(TAG, "Worker task started (%us)", (unsigned)(SENSOR_LOOP_MS / 1000U));
  TickType_t last_wake = xTaskGetTickCount();

  app_sensor_snapshot_t sensors = {0};

  while (1) {
    ds3231_time_t rtc_time = {0};
    bool time_ok = false;
    if (s_rtc_available && rtc_handle != NULL) {
      if (ds3231_get_time(rtc_handle, &rtc_time) == ESP_OK) {
        time_ok = true;
      } else {
        ESP_LOGW(TAG, "Failed to read RTC time, using uptime fallback");
      }
    }
    if (!time_ok) {
      const uint32_t sec_of_day =
          (uint32_t)((esp_timer_get_time() / 1000000ULL) % 86400ULL);
      rtc_time.hour = (uint8_t)((sec_of_day / 3600U) % 24U);
      rtc_time.minute = (uint8_t)((sec_of_day % 3600U) / 60U);
      rtc_time.second = (uint8_t)(sec_of_day % 60U);
    }
    ESP_LOGI(TAG,
             "Time: %02u:%02u:%02u source=%s greenhouse_targets[temp=%.1fC hum=%.1f%%]",
             rtc_time.hour, rtc_time.minute, rtc_time.second,
             time_ok ? "rtc" : "uptime", modbus_get_air_temp_target_c(),
             modbus_get_air_hum_target_percent());
    modbus_set_light_current_time(rtc_time.hour, rtc_time.minute, rtc_time.second);

    float pin_mv = 0.0f;
    float u_sensor = 0.0f;
    if (rh_sensor_read(rh_handle, &pin_mv, &u_sensor, &sensors.rh) == ESP_OK) {
      sensors.rh_valid = true;
      ESP_LOGI(TAG, "RH sensor -> V_ads: %.0f mV | RH: %.1f %%", pin_mv, sensors.rh);
    } else {
      sensors.rh_valid = false;
      ESP_LOGE(TAG, "Failed to read RH sensor");
    }

    if (max31865_read_temp(s_air_temp_handle, &sensors.temp_air) == ESP_OK) {
      sensors.temp_air_valid = true;
      ESP_LOGI(TAG, "PT500 (1) air temp: %.2f C", sensors.temp_air);
    } else {
      sensors.temp_air_valid = false;
      ESP_LOGE(TAG, "Failed to read PT500 (1)");
    }

    for (uint8_t i = 0; i < HEATING_CONTOUR_COUNT; ++i) {
      if (max31865_read_temp(s_water_temp_handles[i],
                             &sensors.water_temp[i]) == ESP_OK) {
        sensors.water_temp_valid[i] = true;
        ESP_LOGI(TAG, "PT500 water %s: %.2f C", s_heating_contours[i].name,
                 sensors.water_temp[i]);
      } else {
        sensors.water_temp_valid[i] = false;
        ESP_LOGE(TAG, "Failed to read PT500 water %s",
                 s_heating_contours[i].name);
      }
    }

    apply_sensor_test_overrides(&sensors);
    set_sensor_snapshot(&sensors);

    const esp_err_t heating_err = process_heating_controller(&sensors);
    if (heating_err != ESP_OK) {
      ESP_LOGW(TAG, "Heating processing failed: %s",
               esp_err_to_name(heating_err));
    }

    ESP_LOGI(TAG, "Heap: %lu (min: %lu) | Stack HW: %lu",
             (unsigned long)esp_get_free_heap_size(),
             (unsigned long)esp_get_minimum_free_heap_size(),
             (unsigned long)uxTaskGetStackHighWaterMark(NULL));

    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SENSOR_LOOP_MS));
  }
}

void app_main(void) {
  configure_runtime_log_levels();

  esp_err_t nvs_err = nvs_flash_init();
  if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES ||
      nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    nvs_err = nvs_flash_init();
  }
  ESP_ERROR_CHECK(nvs_err);

  modbus_init();

  ESP_ERROR_CHECK(i2c_master_init());
  ESP_LOGI(TAG, "I2C initialized");

  ds3231_config_t rtc_cfg = {
      .i2c_port = I2C_MASTER_NUM,
      .i2c_addr = DS3231_I2C_ADDR,
  };
  esp_err_t rtc_err = ds3231_init(&rtc_cfg, &rtc_handle);
  if (rtc_err == ESP_OK) {
    s_rtc_available = true;
    ds3231_time_t current_time = {0};
    esp_err_t read_err = ds3231_get_time(rtc_handle, &current_time);
    if (read_err == ESP_OK) {
      ESP_LOGI(TAG, "DS3231 RTC initialized, current time %02u:%02u:%02u",
               current_time.hour, current_time.minute, current_time.second);
    } else {
      ds3231_time_t init_time = {
          .hour = RTC_INIT_HOUR,
          .minute = RTC_INIT_MINUTE,
          .second = RTC_INIT_SECOND,
      };
      esp_err_t set_err = ds3231_set_time(rtc_handle, &init_time);
      if (set_err == ESP_OK) {
        ESP_LOGW(TAG, "DS3231 unreadable, set fallback time to %02u:%02u:%02u",
                 RTC_INIT_HOUR, RTC_INIT_MINUTE, RTC_INIT_SECOND);
      } else {
        ESP_LOGW(TAG, "DS3231 initialized, but failed to set fallback time: %s",
                 esp_err_to_name(set_err));
      }
    }
  } else {
    s_rtc_available = false;
    rtc_handle = NULL;
    ESP_LOGW(TAG, "DS3231 not available (%s), using uptime fallback time",
             esp_err_to_name(rtc_err));
  }

  modbus_bind_rtc_callbacks(modbus_rtc_get_time_cb, modbus_rtc_set_time_cb,
                            NULL);
  ESP_ERROR_CHECK(bt_ascii_control_init());
  ESP_LOGI(TAG, "Bluetooth ASCII control initialized");
  ESP_ERROR_CHECK(init_hc595_outputs());
  ESP_LOGI(TAG, "74HC595 outputs component initialized");

  rh_sensor_config_t rh_cfg = {
      .i2c_port = I2C_MASTER_NUM,
      .i2c_addr = ADS1115_ADDR_GND,
      .r1_ohm = R1,
      .r2_ohm = R2,
      .offset_mv = ADC_OFFSET_MV,
  };
  ESP_ERROR_CHECK(rh_sensor_init(&rh_cfg, &rh_handle));
  ESP_LOGI(TAG, "RH sensor initialized");

  max31865_config_t max_cfg = {
      .host = MAX31865_HOST,
      .miso_io_num = MAX31865_MISO,
      .mosi_io_num = MAX31865_MOSI,
      .sclk_io_num = MAX31865_CLK,
      .cs_io_num = MAX31865_AIR_CS,
      .r_ref = MAX31865_RREF,
      .r0 = MAX31865_R0,
      .three_wire = true,
  };
  ESP_ERROR_CHECK(max31865_init(&max_cfg, &s_air_temp_handle));
  ESP_LOGI(TAG, "MAX31865 air initialized");

  const int water_cs[HEATING_CONTOUR_COUNT] = {
      [HEATING_CONTOUR_RAIL] = MAX31865_WATER_RAIL_CS,
      [HEATING_CONTOUR_UPPER] = MAX31865_WATER_UPPER_CS,
      [HEATING_CONTOUR_UNDERTRAY] = MAX31865_WATER_UNDERTRAY_CS,
      [HEATING_CONTOUR_GROW] = MAX31865_WATER_GROW_CS,
  };
  for (uint8_t i = 0; i < HEATING_CONTOUR_COUNT; ++i) {
    max31865_config_t water_cfg = {
        .host = MAX31865_HOST,
        .miso_io_num = MAX31865_MISO,
        .mosi_io_num = MAX31865_MOSI,
        .sclk_io_num = MAX31865_CLK,
        .cs_io_num = water_cs[i],
        .r_ref = MAX31865_RREF,
        .r0 = MAX31865_R0,
        .three_wire = true,
    };
    ESP_ERROR_CHECK(max31865_init(&water_cfg, &s_water_temp_handles[i]));
    ESP_LOGI(TAG, "MAX31865 water %s initialized (CS=%d)",
             s_heating_contours[i].name, water_cs[i]);
  }

  ESP_ERROR_CHECK(init_heating_controllers());
  ESP_ERROR_CHECK(apply_light_outputs(false, false));
  ESP_ERROR_CHECK(init_co2_controller());
  ESP_ERROR_CHECK(apply_co2_outputs(false));
  ESP_ERROR_CHECK(init_circulation_controller());
  ESP_ERROR_CHECK(apply_circulation_outputs(0U));
  const esp_err_t window_err = init_window_controllers();
  if (window_err != ESP_OK) {
    ESP_LOGW(TAG, "Window controllers disabled: %s", esp_err_to_name(window_err));
  }
  ESP_ERROR_CHECK(init_curtain_controller());

  xTaskCreate(control_task, "ctrl", 6144, NULL, 6, NULL);
  xTaskCreate(worker_task, "worker", 8192, NULL, 5, NULL);
}
