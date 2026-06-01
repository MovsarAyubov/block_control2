#include "modbus_slave.h"

#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_modbus_common.h"
#include "esp_modbus_slave.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include <ctype.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

static const char *TAG = "MB_SLAVE";

// Modbus serial settings
#define MB_PORT_NUM UART_NUM_2
#define MB_DEV_SPEED 19200
#define MB_UART_PARITY UART_PARITY_DISABLE
#define MB_STOP_BITS UART_STOP_BITS_1
#define MB_DATA_BITS UART_DATA_8_BITS

// RS485 pins
#define MB_UART_RXD 25
#define MB_UART_TXD 26
#define MB_UART_RTS 27

#define MODBUS_HEARTBEAT_TIMEOUT_MS 30000U
#define MODBUS_CYCLE_TASK_PERIOD_MS 100U
#define MODBUS_RTC_SYNC_THRESHOLD_MIN 3U
#define MODBUS_WEATHER_STALE_TIMEOUT_MS 30000U
#define MODBUS_WEATHER_MAX_SOURCE_AGE_S 30U
#define MODBUS_LIGHT_STATUS_R1_SCHEDULE_ACTIVE (1U << 0)
#define MODBUS_LIGHT_STATUS_R2_SCHEDULE_ACTIVE (1U << 1)
#define MODBUS_LIGHT_STATUS_R1_ON_DELAY_ACTIVE (1U << 2)
#define MODBUS_LIGHT_STATUS_R2_ON_DELAY_ACTIVE (1U << 3)
#define MODBUS_LIGHT_STATUS_R1_DLI_LIMIT (1U << 4)
#define MODBUS_LIGHT_STATUS_R2_DLI_LIMIT (1U << 5)
#define MODBUS_LIGHT_STATUS_R1_HYST_HOLD (1U << 6)
#define MODBUS_LIGHT_STATUS_R2_HYST_HOLD (1U << 7)
#define MODBUS_LIGHT_STATUS_WEATHER_STALE (1U << 8)
#define MODBUS_LIGHT_STATUS_R1_OUTPUT_ON (1U << 9)
#define MODBUS_LIGHT_STATUS_R2_OUTPUT_ON (1U << 10)
#define MODBUS_LIGHT_RELAY_COUNT 2
#define MODBUS_LIGHT_ZONE_DELAY_STEP_SEC 10U
#define MODBUS_LIGHT_AUTO_APPLY_SETTLE_MS 250U
#define MODBUS_WINDOWS_DEFAULT_CTRL_MODE MODBUS_WINDOWS_CTRL_MODE_AUTO
#define MODBUS_WINDOWS_DEFAULT_FORCE_SAFE_CMD 0U
#define MODBUS_WINDOWS_DEFAULT_TEMP_SETPOINT 250U
#define MODBUS_WINDOWS_DEFAULT_SAFE_MIN_PERCENT 0U
#define MODBUS_WINDOWS_DEFAULT_WIND_LIMIT 80U
#define MODBUS_WINDOWS_DEFAULT_WIND_STORM 150U
#define MODBUS_WINDOWS_DEFAULT_WIND_RECOVER 60U
#define MODBUS_WINDOWS_DEFAULT_AZIMUTH_DEG 0U
#define MODBUS_WINDOWS_DEFAULT_WIND_SECTOR_HALF_WIDTH_DEG 45U
#define MODBUS_WINDOWS_DEFAULT_TEMP_STEP_C 30U
#define MODBUS_WINDOWS_DEFAULT_TEMP_STEP_HYST_C 5U
#define MODBUS_WINDOWS_DEFAULT_TEMP_STEP_TARGET_PERCENT 200U
#define MODBUS_WINDOWS_DEFAULT_TEMP_STEP_MAX_INDEX 5U
#define MODBUS_WINDOWS_DEFAULT_AUTO_ALGO_MODE MODBUS_WINDOWS_AUTO_ALGO_TEMP
#define MODBUS_WINDOWS_DEFAULT_HUM_SETPOINT 800U
#define MODBUS_WINDOWS_DEFAULT_HUM_STEP 50U
#define MODBUS_WINDOWS_DEFAULT_HUM_STEP_HYST 10U
#define MODBUS_WINDOWS_DEFAULT_HUM_STEP_TARGET_PERCENT 200U
#define MODBUS_WINDOWS_DEFAULT_HUM_STEP_MAX_INDEX 5U
#define MODBUS_WINDOWS_DEFAULT_COLD_CLOSE_DELTA 20U
#define MODBUS_WINDOWS_DEFAULT_COLD_CLOSE_HYST 5U
#define MODBUS_WINDOWS_DEFAULT_WINDWARD_MIN_PERCENT 0U
#define MODBUS_WINDOWS_DEFAULT_WINDWARD_MAX_PERCENT 0U
#define MODBUS_WINDOWS_DEFAULT_WINDWARD_REDUCTION_PERCENT_PER_MS 0U
#define MODBUS_WINDOWS_DEFAULT_LEEWARD_MIN_PERCENT 0U
#define MODBUS_WINDOWS_DEFAULT_LEEWARD_MAX_PERCENT 1000U
#define MODBUS_WINDOWS_DEFAULT_LEEWARD_REDUCTION_PERCENT_PER_MS 0U
#define MODBUS_WINDOWS_DEFAULT_WINDWARD_LAG_PERCENT 0U
#define MODBUS_WINDOWS_DEFAULT_RAIN_MODE MODBUS_WINDOWS_RAIN_MODE_WINDWARD
#define MODBUS_WINDOWS_DEFAULT_RAIN_WINDWARD_PERCENT 0U
#define MODBUS_WINDOWS_DEFAULT_WEATHER_STALE_POLICY \
  MODBUS_WINDOWS_WEATHER_STALE_CLOSE_SAFE
#define MODBUS_WINDOWS_DEFAULT_WEATHER_STALE_TIMEOUT_MS 20000U
#define MODBUS_WINDOWS_DEFAULT_WEATHER_SOURCE_AGE_S 20U
#define MODBUS_WINDOWS_DEFAULT_TARGET_HYST_PERCENT 30U
#define MODBUS_WINDOWS_DEFAULT_MOTION_DELTA_PERCENT 5U
#define MODBUS_WINDOWS_DEFAULT_NO_MOTION_TIMEOUT_MS 3000U
#define MODBUS_HEATING_DEFAULT_CTRL_MODE MODBUS_HEATING_CTRL_MODE_AUTO
#define MODBUS_HEATING_DEFAULT_AIR_SETPOINT 200U
#define MODBUS_HEATING_DEFAULT_AIR_HYST 5U
#define MODBUS_HEATING_DEFAULT_STAGE_DELTA_1 3U
#define MODBUS_HEATING_DEFAULT_STAGE_DELTA_2 10U
#define MODBUS_HEATING_DEFAULT_STAGE_DELTA_3 20U
#define MODBUS_HEATING_DEFAULT_STAGE_DELTA_4 30U
#define MODBUS_HEATING_DEFAULT_MIN_ON_S 60U
#define MODBUS_HEATING_DEFAULT_MIN_OFF_S 30U
#define MODBUS_CURTAIN_DEFAULT_CTRL_MODE MODBUS_CURTAIN_CTRL_MODE_MANUAL
#define MODBUS_CURTAIN_DEFAULT_MANUAL_TARGET 1000U
#define MODBUS_CURTAIN_DEFAULT_SCHEDULE_START_HHMM 600U
#define MODBUS_CURTAIN_DEFAULT_SCHEDULE_END_HHMM 2200U
#define MODBUS_CURTAIN_DEFAULT_OUTSIDE_TARGET 1000U
#define MODBUS_CURTAIN_DEFAULT_MIN_POSITION 0U
#define MODBUS_CURTAIN_DEFAULT_MAX_POSITION 1000U
#define MODBUS_CURTAIN_DEFAULT_POSITION_HYST 30U
#define MODBUS_CURTAIN_DEFAULT_RADIATION_THRESHOLD 500U
#define MODBUS_CURTAIN_DEFAULT_RADIATION_STEP_WM2 100U
#define MODBUS_CURTAIN_DEFAULT_RADIATION_STEP_PERCENT 100U
#define MODBUS_CURTAIN_DEFAULT_RADIATION_HYST 50U
#define MODBUS_CURTAIN_DEFAULT_COLD_DELTA 20U
#define MODBUS_CURTAIN_DEFAULT_COLD_HYST 5U
#define MODBUS_CURTAIN_DEFAULT_COLD_TARGET 0U
#define MODBUS_CURTAIN_DEFAULT_HEAT_DELTA 20U
#define MODBUS_CURTAIN_DEFAULT_HEAT_HYST 5U
#define MODBUS_CURTAIN_DEFAULT_HEAT_TARGET 1000U
#define MODBUS_CURTAIN_DEFAULT_HUM_LOW_THRESHOLD 380U
#define MODBUS_CURTAIN_DEFAULT_HUM_LOW_HYST 20U
#define MODBUS_CURTAIN_DEFAULT_HUM_LOW_TARGET 1000U
#define MODBUS_CURTAIN_DEFAULT_HUM_HIGH_THRESHOLD 80U
#define MODBUS_CURTAIN_DEFAULT_HUM_HIGH_HYST 20U
#define MODBUS_CURTAIN_DEFAULT_HUM_HIGH_TARGET 0U
#define MODBUS_CO2_DEFAULT_CTRL_MODE MODBUS_CO2_CTRL_MODE_OFF
#define MODBUS_CO2_DEFAULT_SCHEDULE_START_HHMM 600U
#define MODBUS_CO2_DEFAULT_SCHEDULE_END_HHMM 2000U
#define MODBUS_CO2_DEFAULT_LOW_LIGHT_THRESHOLD_WM2 150U
#define MODBUS_CO2_DEFAULT_MID_LIGHT_THRESHOLD_WM2 350U
#define MODBUS_CO2_DEFAULT_HIGH_LIGHT_THRESHOLD_WM2 600U
#define MODBUS_CO2_DEFAULT_LOW_LIGHT_TARGET_PPM 500U
#define MODBUS_CO2_DEFAULT_MID_LIGHT_TARGET_PPM 700U
#define MODBUS_CO2_DEFAULT_HIGH_LIGHT_TARGET_PPM 900U
#define MODBUS_CO2_DEFAULT_VENT_LIMIT_LOW_PERCENT 100U
#define MODBUS_CO2_DEFAULT_VENT_LIMIT_HIGH_PERCENT 300U
#define MODBUS_CO2_DEFAULT_VENT_CUTOFF_PERCENT 400U
#define MODBUS_CO2_DEFAULT_DOSING_HYST_PPM 50U
#define MODBUS_CO2_DEFAULT_MAX_SAFE_PPM 1200U
#define MODBUS_CO2_DEFAULT_MAX_DOSING_TIME_S 300U
#define MODBUS_CO2_DEFAULT_MIN_PAUSE_TIME_S 30U
#define MODBUS_CO2_DEFAULT_NO_RISE_CHECK_TIME_S 90U
#define MODBUS_CO2_DEFAULT_NO_RISE_MIN_DELTA_PPM 30U
#define MODBUS_CO2_DEFAULT_TEMP_HIGH_DELTA 20U
#define MODBUS_CO2_DEFAULT_TEMP_CRITICAL_DELTA 40U
#define MODBUS_CO2_DEFAULT_HUM_HIGH_DELTA 50U

#define MODBUS_NVS_NAMESPACE "modbus"
#define MODBUS_NVS_KEY_SLAVE_ID "slave_id"
#define MODBUS_NVS_KEY_REMOTE_CFG "remote_cfg"
#define MODBUS_NVS_KEY_AUTONOMOUS_CFG "auto_cfg"
#define MODBUS_NVS_KEY_LIGHT_STATE "light_state"
#define MODBUS_AUTONOMOUS_CFG_MAGIC 0x4D424143U // MBAC
#define MODBUS_LIGHT_STATE_MAGIC 0x4D424C53U // MBLS

#define MODBUS_DEFAULT_SLAVE_ID 1U
#define MODBUS_MIN_SLAVE_ID 1U
#define MODBUS_MAX_SLAVE_ID 247U

#define MODBUS_FC_COUNT 10

typedef struct {
  uint16_t enable;
  uint16_t on_hhmm;
  uint16_t off_hhmm;
} light_period_cfg_t;

typedef struct {
  light_period_cfg_t schedule;
  uint16_t threshold_wm2;
  uint16_t dli_off_limit_jcm2;
} light_relay_cfg_t;

typedef struct {
  light_relay_cfg_t relay[MODBUS_LIGHT_RELAY_COUNT];
  uint16_t hyst_sec;
} light_control_cfg_t;

typedef struct {
  uint32_t ctrl_version;
  uint16_t windows_pos_a_target;
  uint16_t windows_pos_b_target;
  uint16_t curtain_pos_target;
  uint16_t sp_water_rail;
  uint16_t sp_water_grow;
  uint16_t sp_water_upper;
  uint16_t sp_water_undertray;
  light_period_cfg_t periods[MODBUS_LIGHT_MAX_PERIODS];
} remote_ctrl_cfg_t;

typedef struct {
  uint16_t windows_pos_a_target;
  uint16_t windows_pos_b_target;
  uint16_t curtain_pos_target;
  uint16_t sp_water_rail;
  uint16_t sp_water_grow;
  uint16_t sp_water_upper;
  uint16_t sp_water_undertray;
  light_control_cfg_t light;
} autonomous_ctrl_cfg_t;

typedef struct {
  uint32_t magic;
  remote_ctrl_cfg_t cfg;
} persisted_remote_cfg_t;

typedef struct {
  uint32_t magic;
  autonomous_ctrl_cfg_t cfg;
  uint32_t crc32;
} persisted_autonomous_cfg_t;

typedef struct {
  uint32_t magic;
  uint32_t active_ctrl_version;
  uint16_t last_applied_token;
  uint16_t reserved;
  light_control_cfg_t active_cfg;
  uint32_t crc32;
} persisted_light_state_t;

typedef struct {
  uint8_t fc;
  mb_fn_handler_fp original;
  mb_fn_handler_fp wrapper;
} modbus_handler_wrap_t;

typedef struct {
  int16_t out_temp;
  uint16_t out_hum;
  uint16_t wind_speed;
  uint16_t wind_dir;
  uint16_t rain_flag;
  uint16_t solar_rad;
  uint16_t baro_press;
  int16_t dew_point;
  uint16_t status_bits;
  uint16_t source_age_s;
} weather_snapshot_t;

static void *s_mbc_slave_handler = NULL;
static mb_register_area_descriptor_t s_holding_area;
static uint16_t s_holding_regs[MODBUS_HREG_TOTAL_COUNT] = {0};

static volatile uint8_t s_light_hour = 0;
static volatile uint8_t s_light_minute = 0;
static volatile uint8_t s_light_second = 0;
static volatile uint32_t s_light_set_ms = 0;

static volatile modbus_mode_state_t s_mode_state = MODBUS_MODE_REMOTE;
static volatile modbus_mode_reason_t s_mode_reason = MODBUS_REASON_NONE;
static volatile uint32_t s_last_master_seen_ms = 0;
static volatile uint16_t s_good_cycle_streak = 0;
static volatile modbus_apply_status_t s_last_apply_status = MODBUS_APPLY_OK;

static light_control_cfg_t s_staging_light_cfg = {0};
static light_control_cfg_t s_active_light_cfg = {0};
static uint32_t s_active_ctrl_version = 0;
static volatile bool s_apply_pending = false;
static light_control_cfg_t s_apply_pending_light_cfg = {0};
static volatile uint32_t s_apply_ok_count = 0;
static volatile uint32_t s_apply_fail_invalid_count = 0;
static volatile uint32_t s_apply_fail_busy_count = 0;
static volatile uint32_t s_apply_fail_internal_count = 0;
static volatile modbus_apply_status_t s_last_apply_error_code = MODBUS_APPLY_OK;
static volatile uint32_t s_last_apply_ts_ms = 0;
static volatile uint8_t s_last_logged_schedule_mask = UINT8_MAX;
static volatile uint8_t s_last_logged_schedule_mode = UINT8_MAX;
static volatile uint8_t s_light_stable_on_mask = 0;
static volatile uint8_t s_light_pending_valid_mask = 0;
static volatile uint8_t s_light_pending_target_on_mask = 0;
static volatile uint32_t s_light_pending_since_ms[MODBUS_LIGHT_RELAY_COUNT] = {0};
static volatile uint32_t s_light_on_delay_since_ms[MODBUS_LIGHT_RELAY_COUNT] = {0};
static volatile bool s_light_cfg_dirty = false;
static volatile uint32_t s_light_cfg_last_change_ms = 0;
static volatile uint16_t s_last_logged_light_output_percent = UINT16_MAX;
static volatile uint16_t s_last_logged_light_status_bits = UINT16_MAX;

static volatile uint16_t s_rtc_last_token = 0;
static volatile uint16_t s_rtc_pending_token = 0;
static volatile uint16_t s_rtc_pending_hour = 0;
static volatile uint16_t s_rtc_pending_minute = 0;
static volatile bool s_rtc_sync_pending = false;

static volatile uint16_t s_weather_last_token = 0;
static volatile uint16_t s_weather_pending_token = 0;
static volatile bool s_weather_sync_pending = false;
static weather_snapshot_t s_weather_pending_snapshot = {0};
static weather_snapshot_t s_weather_active_snapshot = {0};
static volatile uint32_t s_weather_last_rx_ms = 0;
static volatile bool s_weather_valid = false;
static volatile bool s_weather_stale = true;
static volatile bool s_air_temp_override_active = false;
static volatile bool s_rh_override_active = false;
static volatile int16_t s_air_temp_override_tenths = 0;
static volatile uint16_t s_rh_override_tenths = 0;

static volatile uint32_t s_rtc_sync_applied_count = 0;
static volatile uint32_t s_rtc_sync_noop_count = 0;
static volatile uint32_t s_rtc_sync_fail_count = 0;
static volatile uint32_t s_rtc_sync_reject_count = 0;
static volatile modbus_rtc_set_result_t s_rtc_sync_last_result =
    MODBUS_RTC_SET_RESULT_NONE;

static modbus_rtc_get_time_cb_t s_rtc_get_time_cb = NULL;
static modbus_rtc_set_time_cb_t s_rtc_set_time_cb = NULL;
static void *s_rtc_cb_ctx = NULL;

static uint8_t s_slave_id = MODBUS_DEFAULT_SLAVE_ID;
static remote_ctrl_cfg_t s_remote_active_cfg = {0};

static const remote_ctrl_cfg_t s_remote_cfg_default = {
    .ctrl_version = 1,
    .windows_pos_a_target = 0,
    .windows_pos_b_target = 0,
    .curtain_pos_target = 1000,
    .sp_water_rail = 350,
    .sp_water_grow = 320,
    .sp_water_upper = 360,
    .sp_water_undertray = 300,
    .periods = {{1, 600, 2200}},
};

static const autonomous_ctrl_cfg_t s_autonomous_cfg_default = {
    .windows_pos_a_target = 0,
    .windows_pos_b_target = 0,
    .curtain_pos_target = 1000,
    .sp_water_rail = 350,
    .sp_water_grow = 320,
    .sp_water_upper = 360,
    .sp_water_undertray = 300,
    .light =
        {
            .relay =
                {
                    {
                        .schedule = {1, 600, 2200},
                        .threshold_wm2 = 0,
                        .dli_off_limit_jcm2 = 0,
                    },
                    {
                        .schedule = {1, 600, 2200},
                        .threshold_wm2 = 0,
                        .dli_off_limit_jcm2 = 0,
                    },
                },
            .hyst_sec = 0,
        },
};

static autonomous_ctrl_cfg_t s_autonomous_cfg = {0};

static portMUX_TYPE s_state_lock = portMUX_INITIALIZER_UNLOCKED;

static mb_exception_t modbus_fc_01_wrapper(void *ctx, uint8_t *frame,
                                           uint16_t *len_buf);
static mb_exception_t modbus_fc_02_wrapper(void *ctx, uint8_t *frame,
                                           uint16_t *len_buf);
static mb_exception_t modbus_fc_03_wrapper(void *ctx, uint8_t *frame,
                                           uint16_t *len_buf);
static mb_exception_t modbus_fc_04_wrapper(void *ctx, uint8_t *frame,
                                           uint16_t *len_buf);
static mb_exception_t modbus_fc_05_wrapper(void *ctx, uint8_t *frame,
                                           uint16_t *len_buf);
static mb_exception_t modbus_fc_06_wrapper(void *ctx, uint8_t *frame,
                                           uint16_t *len_buf);
static mb_exception_t modbus_fc_0F_wrapper(void *ctx, uint8_t *frame,
                                           uint16_t *len_buf);
static mb_exception_t modbus_fc_10_wrapper(void *ctx, uint8_t *frame,
                                           uint16_t *len_buf);
static mb_exception_t modbus_fc_11_wrapper(void *ctx, uint8_t *frame,
                                           uint16_t *len_buf);
static mb_exception_t modbus_fc_17_wrapper(void *ctx, uint8_t *frame,
                                           uint16_t *len_buf);

static void queue_rtc_sync_request_from_regs(void);
static bool get_local_time_snapshot(uint8_t *hour, uint8_t *minute,
                                    uint8_t *second);
static void finalize_rtc_sync(uint16_t token, modbus_rtc_set_result_t result);
static void process_pending_rtc_sync(void);
static bool load_weather_snapshot_from_regs(weather_snapshot_t *snapshot,
                                            uint16_t *token);
static bool weather_payload_equal(const weather_snapshot_t *lhs,
                                  const weather_snapshot_t *rhs);
static void queue_weather_sync_request_from_regs(void);
static void finalize_weather_sync(uint16_t token,
                                  modbus_weather_set_result_t result);
static void process_pending_weather_sync(void);
static void queue_apply_request_from_regs(void);
static void sync_staging_schedule_from_current_regs(void);
static void queue_apply_request_from_current_regs(void);
static bool decode_write_holding_span(uint8_t fc, const uint8_t *frame, uint16_t len,
                                      uint16_t *start_reg, uint16_t *reg_count);
static bool reg_span_contains(uint16_t start_reg, uint16_t reg_count,
                              uint16_t target_reg);
static bool reg_span_intersects(uint16_t start_reg, uint16_t reg_count,
                                uint16_t first_reg, uint16_t last_reg);
static bool reg_span_intersects_window_settings(uint16_t start_reg,
                                                uint16_t reg_count);
static bool write_span_intersects_window_settings(bool has_span,
                                                  uint16_t start_reg,
                                                  uint16_t alt_start_reg,
                                                  uint16_t reg_count);
static void log_window_settings_received(uint16_t start_reg,
                                         uint16_t reg_count);
static bool write_span_intersects_curtain_settings(bool has_span,
                                                   uint16_t start_reg,
                                                   uint16_t alt_start_reg,
                                                   uint16_t reg_count);
static void log_curtain_settings_received(uint16_t start_reg,
                                          uint16_t reg_count);
static uint8_t get_active_light_schedule_mask(const light_control_cfg_t *cfg,
                                               uint16_t minute_of_day);
static void log_active_light_schedules_if_changed(
    const light_control_cfg_t *cfg, modbus_mode_state_t mode,
    uint16_t minute_of_day, uint8_t active_mask);
static const char *windows_ctrl_mode_to_string(modbus_windows_ctrl_mode_t mode);
static const char *
windows_auto_algo_to_string(modbus_windows_auto_algo_mode_t mode);
static modbus_apply_status_t apply_control_block(
    const light_control_cfg_t *candidate_cfg);
static esp_err_t apply_ascii_autonomous_update(const autonomous_ctrl_cfg_t *cfg,
                                               char *response,
                                               size_t response_len,
                                               const char *success_text);
static uint16_t modbus_read_holding_reg(uint16_t reg_index);
static void modbus_write_holding_reg(uint16_t reg_index, uint16_t value);

static modbus_handler_wrap_t s_handler_wraps[MODBUS_FC_COUNT] = {
    {.fc = 0x01, .original = NULL, .wrapper = modbus_fc_01_wrapper},
    {.fc = 0x02, .original = NULL, .wrapper = modbus_fc_02_wrapper},
    {.fc = 0x03, .original = NULL, .wrapper = modbus_fc_03_wrapper},
    {.fc = 0x04, .original = NULL, .wrapper = modbus_fc_04_wrapper},
    {.fc = 0x05, .original = NULL, .wrapper = modbus_fc_05_wrapper},
    {.fc = 0x06, .original = NULL, .wrapper = modbus_fc_06_wrapper},
    {.fc = 0x0F, .original = NULL, .wrapper = modbus_fc_0F_wrapper},
    {.fc = 0x10, .original = NULL, .wrapper = modbus_fc_10_wrapper},
    {.fc = 0x11, .original = NULL, .wrapper = modbus_fc_11_wrapper},
    {.fc = 0x17, .original = NULL, .wrapper = modbus_fc_17_wrapper},
};

static uint32_t now_ms(void) {
  return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

static void u32_to_regs(uint32_t value, uint16_t *hi, uint16_t *lo) {
  if (hi) {
    *hi = (uint16_t)((value >> 16U) & 0xFFFFU);
  }
  if (lo) {
    *lo = (uint16_t)(value & 0xFFFFU);
  }
}

static uint16_t float_to_u16_tenths(float value, float min_v, float max_v) {
  if (value < min_v) {
    value = min_v;
  } else if (value > max_v) {
    value = max_v;
  }
  float scaled = value * 10.0f;
  if (scaled < 0.0f) {
    scaled = 0.0f;
  }
  if (scaled > 65535.0f) {
    scaled = 65535.0f;
  }
  return (uint16_t)(scaled + 0.5f);
}

static uint16_t float_to_i16_tenths_raw(float value) {
  if (value < -3276.8f) {
    value = -3276.8f;
  } else if (value > 3276.7f) {
    value = 3276.7f;
  }
  int16_t v = (int16_t)(value * 10.0f);
  return (uint16_t)v;
}

static bool hhmm_valid(uint16_t hhmm) {
  uint16_t hh = (uint16_t)(hhmm / 100U);
  uint16_t mm = (uint16_t)(hhmm % 100U);
  return (hh <= 23U) && (mm <= 59U);
}

static uint16_t hhmm_to_minutes(uint16_t hhmm) {
  uint16_t hh = (uint16_t)(hhmm / 100U);
  uint16_t mm = (uint16_t)(hhmm % 100U);
  return (uint16_t)(hh * 60U + mm);
}

static uint32_t crc32_update(uint32_t crc, const uint8_t *data, size_t len) {
  crc = ~crc;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; ++bit) {
      uint32_t mask = (uint32_t)-(int32_t)(crc & 1U);
      crc = (crc >> 1U) ^ (0xEDB88320U & mask);
    }
  }
  return ~crc;
}

static void light_ctrl_to_regs(const light_control_cfg_t *cfg, uint16_t *regs) {
  if (!cfg || !regs) {
    return;
  }

  regs[MODBUS_HREG_LIGHT_R1_ENABLE] = cfg->relay[0].schedule.enable;
  regs[MODBUS_HREG_LIGHT_R1_ON_HHMM] = cfg->relay[0].schedule.on_hhmm;
  regs[MODBUS_HREG_LIGHT_R1_OFF_HHMM] = cfg->relay[0].schedule.off_hhmm;
  regs[MODBUS_HREG_LIGHT_R1_THRESHOLD_WM2] = cfg->relay[0].threshold_wm2;
  regs[MODBUS_HREG_LIGHT_R1_RESERVED] = 0U;
  regs[MODBUS_HREG_LIGHT_R1_DLI_OFF_LIMIT_JCM2] =
      cfg->relay[0].dli_off_limit_jcm2;
  regs[MODBUS_HREG_LIGHT_R2_ENABLE] = cfg->relay[1].schedule.enable;
  regs[MODBUS_HREG_LIGHT_R2_ON_HHMM] = cfg->relay[1].schedule.on_hhmm;
  regs[MODBUS_HREG_LIGHT_R2_OFF_HHMM] = cfg->relay[1].schedule.off_hhmm;
  regs[MODBUS_HREG_LIGHT_R2_THRESHOLD_WM2] = cfg->relay[1].threshold_wm2;
  regs[MODBUS_HREG_LIGHT_R2_RESERVED] = 0U;
  regs[MODBUS_HREG_LIGHT_R2_DLI_OFF_LIMIT_JCM2] =
      cfg->relay[1].dli_off_limit_jcm2;
  regs[MODBUS_HREG_LIGHT_HYST_SEC] = cfg->hyst_sec;
}

static void regs_to_light_ctrl(const uint16_t *regs, light_control_cfg_t *cfg) {
  if (!cfg || !regs) {
    return;
  }

  cfg->relay[0].schedule.enable = regs[MODBUS_HREG_LIGHT_R1_ENABLE];
  cfg->relay[0].schedule.on_hhmm = regs[MODBUS_HREG_LIGHT_R1_ON_HHMM];
  cfg->relay[0].schedule.off_hhmm = regs[MODBUS_HREG_LIGHT_R1_OFF_HHMM];
  cfg->relay[0].threshold_wm2 = regs[MODBUS_HREG_LIGHT_R1_THRESHOLD_WM2];
  cfg->relay[0].dli_off_limit_jcm2 =
      regs[MODBUS_HREG_LIGHT_R1_DLI_OFF_LIMIT_JCM2];
  cfg->relay[1].schedule.enable = regs[MODBUS_HREG_LIGHT_R2_ENABLE];
  cfg->relay[1].schedule.on_hhmm = regs[MODBUS_HREG_LIGHT_R2_ON_HHMM];
  cfg->relay[1].schedule.off_hhmm = regs[MODBUS_HREG_LIGHT_R2_OFF_HHMM];
  cfg->relay[1].threshold_wm2 = regs[MODBUS_HREG_LIGHT_R2_THRESHOLD_WM2];
  cfg->relay[1].dli_off_limit_jcm2 =
      regs[MODBUS_HREG_LIGHT_R2_DLI_OFF_LIMIT_JCM2];
  cfg->hyst_sec = regs[MODBUS_HREG_LIGHT_HYST_SEC];
}

static bool validate_light_ctrl(const light_control_cfg_t *cfg) {
  if (!cfg) {
    return false;
  }

  for (int i = 0; i < MODBUS_LIGHT_RELAY_COUNT; ++i) {
    const light_relay_cfg_t *relay_cfg = &cfg->relay[i];
    if (relay_cfg->schedule.enable > 1U) {
      return false;
    }
    if (!hhmm_valid(relay_cfg->schedule.on_hhmm) ||
        !hhmm_valid(relay_cfg->schedule.off_hhmm)) {
      return false;
    }
    if (relay_cfg->schedule.enable == 1U &&
        relay_cfg->schedule.on_hhmm == relay_cfg->schedule.off_hhmm) {
      return false;
    }
  }
  if (cfg->hyst_sec > 86400U) {
    return false;
  }

  return true;
}

static bool light_schedule_equal(const light_period_cfg_t *lhs,
                                 const light_period_cfg_t *rhs) {
  if (lhs == NULL || rhs == NULL) {
    return false;
  }

  return lhs->enable == rhs->enable && lhs->on_hhmm == rhs->on_hhmm &&
         lhs->off_hhmm == rhs->off_hhmm;
}

static bool light_ctrl_equal(const light_control_cfg_t *lhs,
                             const light_control_cfg_t *rhs) {
  if (lhs == NULL || rhs == NULL) {
    return false;
  }

  return light_schedule_equal(&lhs->relay[0].schedule, &rhs->relay[0].schedule) &&
         light_schedule_equal(&lhs->relay[1].schedule, &rhs->relay[1].schedule) &&
         lhs->relay[0].threshold_wm2 == rhs->relay[0].threshold_wm2 &&
         lhs->relay[0].dli_off_limit_jcm2 ==
             rhs->relay[0].dli_off_limit_jcm2 &&
         lhs->relay[1].threshold_wm2 == rhs->relay[1].threshold_wm2 &&
         lhs->relay[1].dli_off_limit_jcm2 ==
             rhs->relay[1].dli_off_limit_jcm2 && lhs->hyst_sec == rhs->hyst_sec;
}

static void log_light_schedule_cfg(const char *prefix,
                                   const light_period_cfg_t *schedule) {
  if (prefix == NULL || schedule == NULL) {
    return;
  }

  ESP_LOGI(TAG, "%s EN=%u ON=%04u OFF=%04u", prefix,
           (unsigned)schedule->enable, (unsigned)schedule->on_hhmm,
           (unsigned)schedule->off_hhmm);
}

static void log_light_ctrl_cfg(const char *prefix, const light_control_cfg_t *cfg) {
  if (prefix == NULL || cfg == NULL) {
    return;
  }

  ESP_LOGI(TAG,
           "%s R1[EN=%u ON=%04u OFF=%04u thr=%u dli=%u] "
           "R2[EN=%u ON=%04u OFF=%04u thr=%u dli=%u] hyst_s=%u",
           prefix, (unsigned)cfg->relay[0].schedule.enable,
           (unsigned)cfg->relay[0].schedule.on_hhmm,
           (unsigned)cfg->relay[0].schedule.off_hhmm,
           (unsigned)cfg->relay[0].threshold_wm2,
           (unsigned)cfg->relay[0].dli_off_limit_jcm2,
           (unsigned)cfg->relay[1].schedule.enable,
           (unsigned)cfg->relay[1].schedule.on_hhmm,
           (unsigned)cfg->relay[1].schedule.off_hhmm,
           (unsigned)cfg->relay[1].threshold_wm2,
           (unsigned)cfg->relay[1].dli_off_limit_jcm2, (unsigned)cfg->hyst_sec);
}

static void log_light_runtime_state_if_changed(uint32_t sec_of_day,
                                               uint16_t radiation_wm2,
                                               uint16_t dli_current,
                                               uint16_t output_percent,
                                               uint16_t status_bits,
                                               bool relay1_on,
                                               bool relay2_on) {
  bool should_log = false;

  taskENTER_CRITICAL(&s_state_lock);
  if (s_last_logged_light_output_percent != output_percent ||
      s_last_logged_light_status_bits != status_bits) {
    s_last_logged_light_output_percent = output_percent;
    s_last_logged_light_status_bits = status_bits;
    should_log = true;
  }
  taskEXIT_CRITICAL(&s_state_lock);

  if (!should_log) {
    return;
  }

  uint32_t hh = (sec_of_day / 3600U) % 24U;
  uint32_t mm = (sec_of_day % 3600U) / 60U;
  uint32_t ss = sec_of_day % 60U;

  ESP_LOGI(
      TAG,
      "Light runtime: now=%02u:%02u:%02u out=%u%% relays=%u/%u rad=%u dli=%u "
      "r1_schedule=%u r2_schedule=%u r1_delay=%u r2_delay=%u r1_dli=%u r2_dli=%u "
      "r1_hyst=%u r2_hyst=%u weather_stale=%u "
      "status=0x%04X",
      (unsigned)hh, (unsigned)mm, (unsigned)ss, (unsigned)output_percent,
      relay1_on ? 1U : 0U, relay2_on ? 1U : 0U, (unsigned)radiation_wm2,
      (unsigned)dli_current,
      (status_bits & MODBUS_LIGHT_STATUS_R1_SCHEDULE_ACTIVE) ? 1U : 0U,
      (status_bits & MODBUS_LIGHT_STATUS_R2_SCHEDULE_ACTIVE) ? 1U : 0U,
      (status_bits & MODBUS_LIGHT_STATUS_R1_ON_DELAY_ACTIVE) ? 1U : 0U,
      (status_bits & MODBUS_LIGHT_STATUS_R2_ON_DELAY_ACTIVE) ? 1U : 0U,
      (status_bits & MODBUS_LIGHT_STATUS_R1_DLI_LIMIT) ? 1U : 0U,
      (status_bits & MODBUS_LIGHT_STATUS_R2_DLI_LIMIT) ? 1U : 0U,
      (status_bits & MODBUS_LIGHT_STATUS_R1_HYST_HOLD) ? 1U : 0U,
      (status_bits & MODBUS_LIGHT_STATUS_R2_HYST_HOLD) ? 1U : 0U,
      (status_bits & MODBUS_LIGHT_STATUS_WEATHER_STALE) ? 1U : 0U,
      (unsigned)status_bits);
}

static uint32_t compute_light_state_crc32(
    uint32_t active_ctrl_version, uint16_t last_applied_token,
    const light_control_cfg_t *active_cfg) {
  uint32_t crc = 0;
  crc = crc32_update(crc, (const uint8_t *)&active_ctrl_version,
                     sizeof(active_ctrl_version));
  crc = crc32_update(crc, (const uint8_t *)&last_applied_token,
                     sizeof(last_applied_token));
  crc = crc32_update(crc, (const uint8_t *)&active_cfg->relay[0].schedule.enable,
                     sizeof(active_cfg->relay[0].schedule.enable));
  crc = crc32_update(crc, (const uint8_t *)&active_cfg->relay[0].schedule.on_hhmm,
                     sizeof(active_cfg->relay[0].schedule.on_hhmm));
  crc = crc32_update(crc, (const uint8_t *)&active_cfg->relay[0].schedule.off_hhmm,
                     sizeof(active_cfg->relay[0].schedule.off_hhmm));
  crc = crc32_update(crc,
                     (const uint8_t *)&active_cfg->relay[0].threshold_wm2,
                     sizeof(active_cfg->relay[0].threshold_wm2));
  crc = crc32_update(crc,
                     (const uint8_t *)&active_cfg->relay[0].dli_off_limit_jcm2,
                     sizeof(active_cfg->relay[0].dli_off_limit_jcm2));
  crc = crc32_update(crc, (const uint8_t *)&active_cfg->relay[1].schedule.enable,
                     sizeof(active_cfg->relay[1].schedule.enable));
  crc = crc32_update(crc, (const uint8_t *)&active_cfg->relay[1].schedule.on_hhmm,
                     sizeof(active_cfg->relay[1].schedule.on_hhmm));
  crc = crc32_update(crc, (const uint8_t *)&active_cfg->relay[1].schedule.off_hhmm,
                     sizeof(active_cfg->relay[1].schedule.off_hhmm));
  crc = crc32_update(crc,
                     (const uint8_t *)&active_cfg->relay[1].threshold_wm2,
                     sizeof(active_cfg->relay[1].threshold_wm2));
  crc = crc32_update(crc,
                     (const uint8_t *)&active_cfg->relay[1].dli_off_limit_jcm2,
                     sizeof(active_cfg->relay[1].dli_off_limit_jcm2));
  crc = crc32_update(crc, (const uint8_t *)&active_cfg->hyst_sec,
                     sizeof(active_cfg->hyst_sec));
  return crc;
}

static void set_default_light_state(void) {
  memset(&s_staging_light_cfg, 0, sizeof(s_staging_light_cfg));
  memset(&s_active_light_cfg, 0, sizeof(s_active_light_cfg));
  memset(&s_apply_pending_light_cfg, 0, sizeof(s_apply_pending_light_cfg));
  s_apply_pending = false;
  s_active_ctrl_version = 0;
  s_light_stable_on_mask = 0U;
  s_light_pending_valid_mask = 0U;
  s_light_pending_target_on_mask = 0U;
  memset((void *)s_light_pending_since_ms, 0, sizeof(s_light_pending_since_ms));
  memset((void *)s_light_on_delay_since_ms, 0, sizeof(s_light_on_delay_since_ms));
  s_light_cfg_dirty = false;
  s_light_cfg_last_change_ms = 0U;
  s_last_logged_light_output_percent = UINT16_MAX;
  s_last_logged_light_status_bits = UINT16_MAX;
}

static bool persist_light_state(const light_control_cfg_t *active_cfg,
                                uint32_t active_ctrl_version) {
  if (active_cfg == NULL) {
    return false;
  }

  nvs_handle_t nvs = 0;
  esp_err_t err = nvs_open(MODBUS_NVS_NAMESPACE, NVS_READWRITE, &nvs);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "NVS open failed while saving light state: %s",
             esp_err_to_name(err));
    return false;
  }

  persisted_light_state_t blob = {0};
  blob.magic = MODBUS_LIGHT_STATE_MAGIC;
  blob.active_ctrl_version = active_ctrl_version;
  blob.last_applied_token = 0U; // legacy field, not used by v2 apply semantics.
  blob.active_cfg = *active_cfg;
  blob.crc32 = compute_light_state_crc32(blob.active_ctrl_version,
                                         blob.last_applied_token,
                                         &blob.active_cfg);

  err = nvs_set_blob(nvs, MODBUS_NVS_KEY_LIGHT_STATE, &blob, sizeof(blob));
  if (err == ESP_OK) {
    err = nvs_commit(nvs);
  }
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "NVS save light state failed: %s", esp_err_to_name(err));
  }
  nvs_close(nvs);
  return (err == ESP_OK);
}

static void remote_cfg_to_regs(const remote_ctrl_cfg_t *cfg, uint16_t *regs) {
  if (!cfg || !regs) {
    return;
  }

  uint16_t hi = 0;
  uint16_t lo = 0;
  u32_to_regs(cfg->ctrl_version, &hi, &lo);
  regs[MODBUS_HREG_CTRL_VERSION_HI] = hi;
  regs[MODBUS_HREG_CTRL_VERSION_LO] = lo;

  regs[MODBUS_HREG_WINDOWS_POS_A_TARGET] = cfg->windows_pos_a_target;
  regs[MODBUS_HREG_WINDOWS_POS_B_TARGET] = cfg->windows_pos_b_target;
  regs[MODBUS_HREG_CURTAIN_POS_TARGET] = cfg->curtain_pos_target;
  regs[MODBUS_HREG_SP_WATER_RAIL] = cfg->sp_water_rail;
  regs[MODBUS_HREG_SP_WATER_GROW] = cfg->sp_water_grow;
  regs[MODBUS_HREG_SP_WATER_UPPER] = cfg->sp_water_upper;
  regs[MODBUS_HREG_SP_WATER_UNDERTRAY] = cfg->sp_water_undertray;
}

static bool validate_remote_cfg(const remote_ctrl_cfg_t *cfg) {
  if (!cfg) {
    return false;
  }

  if (cfg->windows_pos_a_target > 1000U || cfg->windows_pos_b_target > 1000U ||
      cfg->curtain_pos_target > 1000U) {
    return false;
  }

  // Temperature setpoints are x10 C, clamp to practical band [0..1200] for v2.1.
  if (cfg->sp_water_rail > 1200U || cfg->sp_water_grow > 1200U ||
      cfg->sp_water_upper > 1200U || cfg->sp_water_undertray > 1200U) {
    return false;
  }

  return true;
}

static void persist_remote_cfg(const remote_ctrl_cfg_t *cfg) {
  if (!cfg) {
    return;
  }

  nvs_handle_t nvs = 0;
  esp_err_t err = nvs_open(MODBUS_NVS_NAMESPACE, NVS_READWRITE, &nvs);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "NVS open failed while saving cfg: %s", esp_err_to_name(err));
    return;
  }

  persisted_remote_cfg_t blob = {
      .magic = 0x4D424346U, // MBCF
      .cfg = *cfg,
  };

  err = nvs_set_blob(nvs, MODBUS_NVS_KEY_REMOTE_CFG, &blob, sizeof(blob));
  if (err == ESP_OK) {
    err = nvs_commit(nvs);
  }
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "NVS save remote cfg failed: %s", esp_err_to_name(err));
  }
  nvs_close(nvs);
}

static void autonomous_targets_to_remote_cfg(const autonomous_ctrl_cfg_t *src,
                                             remote_ctrl_cfg_t *dst) {
  if (src == NULL || dst == NULL) {
    return;
  }

  memset(dst, 0, sizeof(*dst));
  dst->windows_pos_a_target = src->windows_pos_a_target;
  dst->windows_pos_b_target = src->windows_pos_b_target;
  dst->curtain_pos_target = src->curtain_pos_target;
  dst->sp_water_rail = src->sp_water_rail;
  dst->sp_water_grow = src->sp_water_grow;
  dst->sp_water_upper = src->sp_water_upper;
  dst->sp_water_undertray = src->sp_water_undertray;
}

static void normalize_autonomous_cfg(autonomous_ctrl_cfg_t *cfg) {
  if (cfg == NULL) {
    return;
  }

  for (int i = 0; i < MODBUS_LIGHT_RELAY_COUNT; ++i) {
    cfg->light.relay[i].threshold_wm2 = 0U;
    cfg->light.relay[i].dli_off_limit_jcm2 = 0U;
  }
}

static bool validate_autonomous_cfg(const autonomous_ctrl_cfg_t *cfg) {
  if (cfg == NULL) {
    return false;
  }

  remote_ctrl_cfg_t remote_view = {0};
  autonomous_targets_to_remote_cfg(cfg, &remote_view);
  return validate_remote_cfg(&remote_view) && validate_light_ctrl(&cfg->light);
}

static uint32_t compute_autonomous_cfg_crc32(const autonomous_ctrl_cfg_t *cfg) {
  uint32_t crc = 0U;
  crc = crc32_update(crc, (const uint8_t *)&cfg->windows_pos_a_target,
                     sizeof(cfg->windows_pos_a_target));
  crc = crc32_update(crc, (const uint8_t *)&cfg->windows_pos_b_target,
                     sizeof(cfg->windows_pos_b_target));
  crc = crc32_update(crc, (const uint8_t *)&cfg->curtain_pos_target,
                     sizeof(cfg->curtain_pos_target));
  crc = crc32_update(crc, (const uint8_t *)&cfg->sp_water_rail,
                     sizeof(cfg->sp_water_rail));
  crc = crc32_update(crc, (const uint8_t *)&cfg->sp_water_grow,
                     sizeof(cfg->sp_water_grow));
  crc = crc32_update(crc, (const uint8_t *)&cfg->sp_water_upper,
                     sizeof(cfg->sp_water_upper));
  crc = crc32_update(crc, (const uint8_t *)&cfg->sp_water_undertray,
                     sizeof(cfg->sp_water_undertray));
  crc = crc32_update(crc,
                     (const uint8_t *)&cfg->light.relay[0].schedule.enable,
                     sizeof(cfg->light.relay[0].schedule.enable));
  crc = crc32_update(crc,
                     (const uint8_t *)&cfg->light.relay[0].schedule.on_hhmm,
                     sizeof(cfg->light.relay[0].schedule.on_hhmm));
  crc = crc32_update(crc,
                     (const uint8_t *)&cfg->light.relay[0].schedule.off_hhmm,
                     sizeof(cfg->light.relay[0].schedule.off_hhmm));
  crc = crc32_update(crc, (const uint8_t *)&cfg->light.relay[0].threshold_wm2,
                     sizeof(cfg->light.relay[0].threshold_wm2));
  crc = crc32_update(crc,
                     (const uint8_t *)&cfg->light.relay[0].dli_off_limit_jcm2,
                     sizeof(cfg->light.relay[0].dli_off_limit_jcm2));
  crc = crc32_update(crc,
                     (const uint8_t *)&cfg->light.relay[1].schedule.enable,
                     sizeof(cfg->light.relay[1].schedule.enable));
  crc = crc32_update(crc,
                     (const uint8_t *)&cfg->light.relay[1].schedule.on_hhmm,
                     sizeof(cfg->light.relay[1].schedule.on_hhmm));
  crc = crc32_update(crc,
                     (const uint8_t *)&cfg->light.relay[1].schedule.off_hhmm,
                     sizeof(cfg->light.relay[1].schedule.off_hhmm));
  crc = crc32_update(crc, (const uint8_t *)&cfg->light.relay[1].threshold_wm2,
                     sizeof(cfg->light.relay[1].threshold_wm2));
  crc = crc32_update(crc,
                     (const uint8_t *)&cfg->light.relay[1].dli_off_limit_jcm2,
                     sizeof(cfg->light.relay[1].dli_off_limit_jcm2));
  crc = crc32_update(crc, (const uint8_t *)&cfg->light.hyst_sec,
                     sizeof(cfg->light.hyst_sec));
  return crc;
}

static void set_default_autonomous_cfg(void) {
  s_autonomous_cfg = s_autonomous_cfg_default;
  normalize_autonomous_cfg(&s_autonomous_cfg);
}

static bool persist_autonomous_cfg(const autonomous_ctrl_cfg_t *cfg) {
  if (cfg == NULL) {
    return false;
  }

  nvs_handle_t nvs = 0;
  esp_err_t err = nvs_open(MODBUS_NVS_NAMESPACE, NVS_READWRITE, &nvs);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "NVS open failed while saving autonomous cfg: %s",
             esp_err_to_name(err));
    return false;
  }

  persisted_autonomous_cfg_t blob = {
      .magic = MODBUS_AUTONOMOUS_CFG_MAGIC,
      .cfg = *cfg,
      .crc32 = compute_autonomous_cfg_crc32(cfg),
  };

  err = nvs_set_blob(nvs, MODBUS_NVS_KEY_AUTONOMOUS_CFG, &blob, sizeof(blob));
  if (err == ESP_OK) {
    err = nvs_commit(nvs);
  }
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "NVS save autonomous cfg failed: %s", esp_err_to_name(err));
  }
  nvs_close(nvs);
  return (err == ESP_OK);
}

static void log_autonomous_cfg(const char *prefix,
                               const autonomous_ctrl_cfg_t *cfg) {
  if (prefix == NULL || cfg == NULL) {
    return;
  }

  ESP_LOGI(TAG,
           "%s windows[a=%u b=%u] curtain=%u sp[rail=%u grow=%u upper=%u "
           "undertray=%u]",
           prefix, (unsigned)cfg->windows_pos_a_target,
           (unsigned)cfg->windows_pos_b_target,
           (unsigned)cfg->curtain_pos_target, (unsigned)cfg->sp_water_rail,
           (unsigned)cfg->sp_water_grow, (unsigned)cfg->sp_water_upper,
           (unsigned)cfg->sp_water_undertray);
  ESP_LOGI(TAG,
           "Autonomous light cfg R1[EN=%u ON=%04u OFF=%04u] "
           "R2[EN=%u ON=%04u OFF=%04u] hyst_s=%u",
           (unsigned)cfg->light.relay[0].schedule.enable,
           (unsigned)cfg->light.relay[0].schedule.on_hhmm,
           (unsigned)cfg->light.relay[0].schedule.off_hhmm,
           (unsigned)cfg->light.relay[1].schedule.enable,
           (unsigned)cfg->light.relay[1].schedule.on_hhmm,
           (unsigned)cfg->light.relay[1].schedule.off_hhmm,
           (unsigned)cfg->light.hyst_sec);
}

static esp_err_t update_autonomous_cfg(const autonomous_ctrl_cfg_t *cfg) {
  if (cfg == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  autonomous_ctrl_cfg_t normalized_cfg = *cfg;
  normalize_autonomous_cfg(&normalized_cfg);
  if (!validate_autonomous_cfg(&normalized_cfg)) {
    return ESP_ERR_INVALID_ARG;
  }
  if (!persist_autonomous_cfg(&normalized_cfg)) {
    return ESP_FAIL;
  }

  taskENTER_CRITICAL(&s_state_lock);
  s_autonomous_cfg = normalized_cfg;
  s_last_logged_schedule_mask = UINT8_MAX;
  s_last_logged_schedule_mode = UINT8_MAX;
  s_last_logged_light_output_percent = UINT16_MAX;
  s_last_logged_light_status_bits = UINT16_MAX;
  taskEXIT_CRITICAL(&s_state_lock);

  log_autonomous_cfg("Autonomous cfg updated", &normalized_cfg);
  return ESP_OK;
}

static void load_persisted_settings(void) {
  nvs_handle_t nvs = 0;
  esp_err_t err = nvs_open(MODBUS_NVS_NAMESPACE, NVS_READWRITE, &nvs);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "NVS open failed, using defaults: %s", esp_err_to_name(err));
    s_slave_id = MODBUS_DEFAULT_SLAVE_ID;
    s_remote_active_cfg = s_remote_cfg_default;
    set_default_light_state();
    set_default_autonomous_cfg();
    return;
  }

  uint8_t slave_id = MODBUS_DEFAULT_SLAVE_ID;
  err = nvs_get_u8(nvs, MODBUS_NVS_KEY_SLAVE_ID, &slave_id);
  if (err == ESP_ERR_NVS_NOT_FOUND) {
    slave_id = MODBUS_DEFAULT_SLAVE_ID;
    (void)nvs_set_u8(nvs, MODBUS_NVS_KEY_SLAVE_ID, slave_id);
    (void)nvs_commit(nvs);
  }
  if (slave_id < MODBUS_MIN_SLAVE_ID || slave_id > MODBUS_MAX_SLAVE_ID) {
    slave_id = MODBUS_DEFAULT_SLAVE_ID;
  }
  s_slave_id = slave_id;

  bool need_persist_default_remote = false;
  persisted_remote_cfg_t blob = {0};
  size_t blob_size = sizeof(blob);
  err = nvs_get_blob(nvs, MODBUS_NVS_KEY_REMOTE_CFG, &blob, &blob_size);
  if (err == ESP_OK && blob_size == sizeof(blob) && blob.magic == 0x4D424346U &&
      validate_remote_cfg(&blob.cfg)) {
    s_remote_active_cfg = blob.cfg;
  } else {
    s_remote_active_cfg = s_remote_cfg_default;
    need_persist_default_remote = true;
  }

  bool need_persist_default_light = false;
  persisted_light_state_t light_blob = {0};
  size_t light_blob_size = sizeof(light_blob);
  err = nvs_get_blob(nvs, MODBUS_NVS_KEY_LIGHT_STATE, &light_blob, &light_blob_size);
  if (err == ESP_OK && light_blob_size == sizeof(light_blob) &&
      light_blob.magic == MODBUS_LIGHT_STATE_MAGIC &&
      validate_light_ctrl(&light_blob.active_cfg)) {
    uint32_t expected_crc =
        compute_light_state_crc32(light_blob.active_ctrl_version,
                                  light_blob.last_applied_token,
                                  &light_blob.active_cfg);
    if (expected_crc == light_blob.crc32) {
      s_active_light_cfg = light_blob.active_cfg;
      s_staging_light_cfg = light_blob.active_cfg;
      s_active_ctrl_version = light_blob.active_ctrl_version;
    } else {
      set_default_light_state();
      need_persist_default_light = true;
    }
  } else {
    set_default_light_state();
    need_persist_default_light = true;
  }

  bool need_persist_default_autonomous = false;
  persisted_autonomous_cfg_t autonomous_blob = {0};
  size_t autonomous_blob_size = sizeof(autonomous_blob);
  err = nvs_get_blob(nvs, MODBUS_NVS_KEY_AUTONOMOUS_CFG, &autonomous_blob,
                     &autonomous_blob_size);
  if (err == ESP_OK && autonomous_blob_size == sizeof(autonomous_blob) &&
      autonomous_blob.magic == MODBUS_AUTONOMOUS_CFG_MAGIC &&
      validate_autonomous_cfg(&autonomous_blob.cfg)) {
    uint32_t expected_crc = compute_autonomous_cfg_crc32(&autonomous_blob.cfg);
    if (expected_crc == autonomous_blob.crc32) {
      s_autonomous_cfg = autonomous_blob.cfg;
      normalize_autonomous_cfg(&s_autonomous_cfg);
    } else {
      set_default_autonomous_cfg();
      need_persist_default_autonomous = true;
    }
  } else {
    set_default_autonomous_cfg();
    need_persist_default_autonomous = true;
  }

  nvs_close(nvs);

  if (need_persist_default_remote) {
    persist_remote_cfg(&s_remote_active_cfg);
  }
  if (need_persist_default_light) {
    (void)persist_light_state(&s_active_light_cfg, s_active_ctrl_version);
  }
  if (need_persist_default_autonomous) {
    (void)persist_autonomous_cfg(&s_autonomous_cfg);
  }

  log_autonomous_cfg("Autonomous cfg loaded", &s_autonomous_cfg);
}

static bool is_period_active(const light_period_cfg_t *slot, uint16_t now_minute_of_day) {
  if (!slot || slot->enable == 0U) {
    return false;
  }

  if (!hhmm_valid(slot->on_hhmm) || !hhmm_valid(slot->off_hhmm)) {
    return false;
  }

  uint16_t start = hhmm_to_minutes(slot->on_hhmm);
  uint16_t end = hhmm_to_minutes(slot->off_hhmm);
  if (start == end) {
    return false;
  }

  if (start < end) {
    return (now_minute_of_day >= start) && (now_minute_of_day < end);
  }

  // Through midnight.
  return (now_minute_of_day >= start) || (now_minute_of_day < end);
}

static uint8_t get_active_light_schedule_mask(const light_control_cfg_t *cfg,
                                              uint16_t minute_of_day) {
  if (!cfg) {
    return 0U;
  }

  uint8_t active_mask = 0U;
  for (int i = 0; i < MODBUS_LIGHT_RELAY_COUNT; ++i) {
    if (is_period_active(&cfg->relay[i].schedule, minute_of_day)) {
      active_mask |= (uint8_t)(1U << i);
    }
  }
  return active_mask;
}

static void log_active_light_schedules_if_changed(
    const light_control_cfg_t *cfg, modbus_mode_state_t mode,
    uint16_t minute_of_day, uint8_t active_mask) {
  if (cfg == NULL || minute_of_day >= 1440U) {
    return;
  }

  bool should_log = false;
  taskENTER_CRITICAL(&s_state_lock);
  if (s_last_logged_schedule_mask != active_mask ||
      s_last_logged_schedule_mode != (uint8_t)mode) {
    s_last_logged_schedule_mask = active_mask;
    s_last_logged_schedule_mode = (uint8_t)mode;
    should_log = true;
  }
  taskEXIT_CRITICAL(&s_state_lock);

  if (!should_log) {
    return;
  }

  uint16_t hh = (uint16_t)(minute_of_day / 60U);
  uint16_t mm = (uint16_t)(minute_of_day % 60U);
  ESP_LOGI(TAG, "Light schedules active: mode=%s now=%02u:%02u r1=%u r2=%u",
           (mode == MODBUS_MODE_AUTONOMOUS) ? "AUTONOMOUS" : "REMOTE",
           (unsigned)hh, (unsigned)mm, (unsigned)((active_mask & 0x01U) != 0U),
           (unsigned)((active_mask & 0x02U) != 0U));
  if (active_mask == 0U) {
    return;
  }

  for (int i = 0; i < MODBUS_LIGHT_RELAY_COUNT; ++i) {
    if ((active_mask & (uint8_t)(1U << i)) != 0U) {
      ESP_LOGI(TAG, "Relay %d schedule active: %04u..%04u", i + 1,
               (unsigned)cfg->relay[i].schedule.on_hhmm,
               (unsigned)cfg->relay[i].schedule.off_hhmm);
    }
  }
}

static void update_diag_regs_locked(void) {
  modbus_mode_state_t mode = MODBUS_MODE_REMOTE;
  modbus_mode_reason_t reason = MODBUS_REASON_NONE;
  uint32_t last_master_seen_ms = 0;
  uint16_t good_cycle_streak = 0;
  modbus_apply_status_t last_apply_status = MODBUS_APPLY_OK;
  uint32_t apply_ok_count = 0;
  uint32_t apply_fail_invalid_count = 0;
  uint32_t apply_fail_busy_count = 0;
  uint32_t apply_fail_internal_count = 0;
  modbus_apply_status_t last_apply_error_code = MODBUS_APPLY_OK;
  uint32_t last_apply_ts_ms = 0;

  mode = s_mode_state;
  reason = s_mode_reason;
  last_master_seen_ms = s_last_master_seen_ms;
  good_cycle_streak = s_good_cycle_streak;
  last_apply_status = s_last_apply_status;
  apply_ok_count = s_apply_ok_count;
  apply_fail_invalid_count = s_apply_fail_invalid_count;
  apply_fail_busy_count = s_apply_fail_busy_count;
  apply_fail_internal_count = s_apply_fail_internal_count;
  last_apply_error_code = s_last_apply_error_code;
  last_apply_ts_ms = s_last_apply_ts_ms;

  s_holding_regs[MODBUS_HREG_MODE_STATE] = (uint16_t)mode;
  s_holding_regs[MODBUS_HREG_MODE_REASON] = (uint16_t)reason;
  s_holding_regs[MODBUS_HREG_LAST_MASTER_SEEN_MS_LO] =
      (uint16_t)(last_master_seen_ms & 0xFFFFU);
  s_holding_regs[MODBUS_HREG_LAST_MASTER_SEEN_MS_HI] =
      (uint16_t)((last_master_seen_ms >> 16U) & 0xFFFFU);
  s_holding_regs[MODBUS_HREG_GOOD_CYCLE_STREAK] = good_cycle_streak;
  s_holding_regs[MODBUS_HREG_LAST_APPLY_STATUS] = (uint16_t)last_apply_status;
  s_holding_regs[MODBUS_HREG_APPLY_STATUS] = (uint16_t)last_apply_status;

  s_holding_regs[MODBUS_HREG_APPLY_OK_COUNT_HI] =
      (uint16_t)((apply_ok_count >> 16U) & 0xFFFFU);
  s_holding_regs[MODBUS_HREG_APPLY_OK_COUNT_LO] =
      (uint16_t)(apply_ok_count & 0xFFFFU);
  s_holding_regs[MODBUS_HREG_APPLY_FAIL_INVALID_COUNT_HI] =
      (uint16_t)((apply_fail_invalid_count >> 16U) & 0xFFFFU);
  s_holding_regs[MODBUS_HREG_APPLY_FAIL_INVALID_COUNT_LO] =
      (uint16_t)(apply_fail_invalid_count & 0xFFFFU);
  s_holding_regs[MODBUS_HREG_APPLY_FAIL_BUSY_COUNT_HI] =
      (uint16_t)((apply_fail_busy_count >> 16U) & 0xFFFFU);
  s_holding_regs[MODBUS_HREG_APPLY_FAIL_BUSY_COUNT_LO] =
      (uint16_t)(apply_fail_busy_count & 0xFFFFU);
  s_holding_regs[MODBUS_HREG_APPLY_FAIL_INTERNAL_COUNT_HI] =
      (uint16_t)((apply_fail_internal_count >> 16U) & 0xFFFFU);
  s_holding_regs[MODBUS_HREG_APPLY_FAIL_INTERNAL_COUNT_LO] =
      (uint16_t)(apply_fail_internal_count & 0xFFFFU);
  s_holding_regs[MODBUS_HREG_LAST_APPLY_ERROR_CODE] =
      (uint16_t)last_apply_error_code;
  s_holding_regs[MODBUS_HREG_LAST_APPLY_TS_MS_HI] =
      (uint16_t)((last_apply_ts_ms >> 16U) & 0xFFFFU);
  s_holding_regs[MODBUS_HREG_LAST_APPLY_TS_MS_LO] =
      (uint16_t)(last_apply_ts_ms & 0xFFFFU);
}

static void update_master_seen_and_streak(bool success_cycle) {
  uint32_t t = now_ms();
  bool mode_switched = false;
  uint16_t streak_snapshot = 0;

  taskENTER_CRITICAL(&s_state_lock);
  s_last_master_seen_ms = t;

  if (success_cycle) {
    if (s_good_cycle_streak < UINT16_MAX) {
      s_good_cycle_streak++;
    }
  } else {
    s_good_cycle_streak = 0;
  }

  if (s_mode_state == MODBUS_MODE_AUTONOMOUS && s_good_cycle_streak >= 3U) {
    s_mode_state = MODBUS_MODE_REMOTE;
    s_mode_reason = MODBUS_REASON_NONE;
    mode_switched = true;
    streak_snapshot = s_good_cycle_streak;
  }
  taskEXIT_CRITICAL(&s_state_lock);

  if (mode_switched) {
    ESP_LOGI(TAG, "Mode changed: AUTONOMOUS -> REMOTE (good_cycle_streak=%u)",
             (unsigned)streak_snapshot);
    modbus_get_light_relay_state(NULL, NULL);
  }
}

static void queue_rtc_sync_request_from_regs(void) {
  if (s_mbc_slave_handler == NULL) {
    return;
  }

  uint16_t token = 0;
  uint16_t hour = 0;
  uint16_t minute = 0;

  token = s_holding_regs[MODBUS_HREG_RTC_SET_TOKEN];
  hour = s_holding_regs[MODBUS_HREG_RTC_SET_HOUR];
  minute = s_holding_regs[MODBUS_HREG_RTC_SET_MINUTE];

  if (token == 0U) {
    return;
  }

  taskENTER_CRITICAL(&s_state_lock);
  bool same_as_last = (token == s_rtc_last_token);
  bool same_as_pending = (s_rtc_sync_pending && token == s_rtc_pending_token);
  if (!same_as_last && !same_as_pending) {
    s_rtc_pending_token = token;
    s_rtc_pending_hour = hour;
    s_rtc_pending_minute = minute;
    s_rtc_sync_pending = true;
  }
  taskEXIT_CRITICAL(&s_state_lock);
}

static bool get_local_time_snapshot(uint8_t *hour, uint8_t *minute,
                                    uint8_t *second) {
  if (hour == NULL || minute == NULL || second == NULL) {
    return false;
  }

  modbus_rtc_get_time_cb_t rtc_get_cb = NULL;
  void *rtc_ctx = NULL;
  taskENTER_CRITICAL(&s_state_lock);
  rtc_get_cb = s_rtc_get_time_cb;
  rtc_ctx = s_rtc_cb_ctx;
  taskEXIT_CRITICAL(&s_state_lock);

  uint8_t h = 0;
  uint8_t m = 0;
  uint8_t s = 0;

  if (rtc_get_cb != NULL && rtc_get_cb(&h, &m, &s, rtc_ctx) && h <= 23U &&
      m <= 59U && s <= 59U) {
    *hour = h;
    *minute = m;
    *second = s;
    return true;
  }

  return false;
}

static void finalize_rtc_sync(uint16_t token, modbus_rtc_set_result_t result) {
  if (s_mbc_slave_handler != NULL) {
    ESP_ERROR_CHECK(mbc_slave_lock(s_mbc_slave_handler));
    s_holding_regs[MODBUS_HREG_RTC_SET_APPLIED_TOKEN] = token;
    s_holding_regs[MODBUS_HREG_RTC_SET_RESULT] = (uint16_t)result;
    ESP_ERROR_CHECK(mbc_slave_unlock(s_mbc_slave_handler));
  }

  uint32_t applied_count = 0;
  uint32_t noop_count = 0;
  uint32_t fail_count = 0;
  uint32_t reject_count = 0;

  taskENTER_CRITICAL(&s_state_lock);
  s_rtc_last_token = token;
  s_rtc_sync_pending = false;
  s_rtc_pending_token = 0;
  s_rtc_pending_hour = 0;
  s_rtc_pending_minute = 0;
  s_rtc_sync_last_result = result;

  if (result == MODBUS_RTC_SET_RESULT_APPLIED &&
      s_rtc_sync_applied_count < UINT32_MAX) {
    s_rtc_sync_applied_count++;
  } else if (result == MODBUS_RTC_SET_RESULT_NOOP &&
             s_rtc_sync_noop_count < UINT32_MAX) {
    s_rtc_sync_noop_count++;
  } else if (result == MODBUS_RTC_SET_RESULT_FAILED &&
             s_rtc_sync_fail_count < UINT32_MAX) {
    s_rtc_sync_fail_count++;
  } else if (result == MODBUS_RTC_SET_RESULT_REJECT_RANGE &&
             s_rtc_sync_reject_count < UINT32_MAX) {
    s_rtc_sync_reject_count++;
  }

  applied_count = s_rtc_sync_applied_count;
  noop_count = s_rtc_sync_noop_count;
  fail_count = s_rtc_sync_fail_count;
  reject_count = s_rtc_sync_reject_count;
  taskEXIT_CRITICAL(&s_state_lock);

  ESP_LOGI(TAG,
           "RTC sync token=%u result=%u (applied=%lu noop=%lu fail=%lu "
           "reject=%lu)",
           (unsigned)token, (unsigned)result, (unsigned long)applied_count,
           (unsigned long)noop_count, (unsigned long)fail_count,
           (unsigned long)reject_count);

  // If a newer token was written while current token was in progress, queue it.
  queue_rtc_sync_request_from_regs();
}

static void process_pending_rtc_sync(void) {
  bool pending = false;
  uint16_t token = 0;
  uint16_t server_hour = 0;
  uint16_t server_minute = 0;

  taskENTER_CRITICAL(&s_state_lock);
  pending = s_rtc_sync_pending;
  if (pending) {
    token = s_rtc_pending_token;
    server_hour = s_rtc_pending_hour;
    server_minute = s_rtc_pending_minute;
  }
  taskEXIT_CRITICAL(&s_state_lock);

  if (!pending) {
    return;
  }

  modbus_rtc_set_time_cb_t rtc_set_cb = NULL;
  void *rtc_ctx = NULL;
  taskENTER_CRITICAL(&s_state_lock);
  rtc_set_cb = s_rtc_set_time_cb;
  rtc_ctx = s_rtc_cb_ctx;
  taskEXIT_CRITICAL(&s_state_lock);
  if (rtc_set_cb == NULL) {
    ESP_LOGW(TAG, "RTC sync token=%u failed: set callback is not bound",
             (unsigned)token);
    finalize_rtc_sync(token, MODBUS_RTC_SET_RESULT_FAILED);
    return;
  }

  if (server_hour > 23U || server_minute > 59U) {
    finalize_rtc_sync(token, MODBUS_RTC_SET_RESULT_REJECT_RANGE);
    return;
  }

  uint8_t local_hour = 0;
  uint8_t local_minute = 0;
  uint8_t local_second = 0;
  bool have_local_time =
      get_local_time_snapshot(&local_hour, &local_minute, &local_second);
  if (!have_local_time) {
    ESP_LOGW(TAG,
             "RTC sync token=%u: local time unavailable, forcing time update",
             (unsigned)token);
  }
  if (have_local_time) {
    uint16_t server_total_min =
        (uint16_t)(server_hour * 60U + server_minute);
    uint16_t local_total_min =
        (uint16_t)(local_hour * 60U + local_minute);
    uint16_t direct_diff = (server_total_min >= local_total_min)
                               ? (uint16_t)(server_total_min - local_total_min)
                               : (uint16_t)(local_total_min - server_total_min);
    uint16_t drift_min = (direct_diff <= (uint16_t)(1440U - direct_diff))
                             ? direct_diff
                             : (uint16_t)(1440U - direct_diff);

    if (drift_min < MODBUS_RTC_SYNC_THRESHOLD_MIN) {
      finalize_rtc_sync(token, MODBUS_RTC_SET_RESULT_NOOP);
      return;
    }
  }

  if (!rtc_set_cb((uint8_t)server_hour, (uint8_t)server_minute, 0U, rtc_ctx)) {
    finalize_rtc_sync(token, MODBUS_RTC_SET_RESULT_FAILED);
    return;
  }

  modbus_set_light_current_time((uint8_t)server_hour, (uint8_t)server_minute, 0U);
  finalize_rtc_sync(token, MODBUS_RTC_SET_RESULT_APPLIED);
}

static bool load_weather_snapshot_from_regs(weather_snapshot_t *snapshot,
                                            uint16_t *token) {
  if (snapshot == NULL || token == NULL || s_mbc_slave_handler == NULL) {
    return false;
  }

  weather_snapshot_t tmp = {0};
  uint16_t local_token = 0;

  ESP_ERROR_CHECK(mbc_slave_lock(s_mbc_slave_handler));
  tmp.out_temp = (int16_t)s_holding_regs[MODBUS_HREG_WEATHER_OUT_TEMP];
  tmp.out_hum = s_holding_regs[MODBUS_HREG_WEATHER_OUT_HUM];
  tmp.wind_speed = s_holding_regs[MODBUS_HREG_WEATHER_WIND_SPEED];
  tmp.wind_dir = s_holding_regs[MODBUS_HREG_WEATHER_WIND_DIR];
  tmp.rain_flag = s_holding_regs[MODBUS_HREG_WEATHER_RAIN_FLAG];
  tmp.solar_rad = s_holding_regs[MODBUS_HREG_WEATHER_SOLAR_RAD];
  tmp.baro_press = s_holding_regs[MODBUS_HREG_WEATHER_BARO_PRESS];
  tmp.dew_point = (int16_t)s_holding_regs[MODBUS_HREG_WEATHER_DEW_POINT];
  tmp.status_bits = s_holding_regs[MODBUS_HREG_WEATHER_STATUS_BITS];
  tmp.source_age_s = s_holding_regs[MODBUS_HREG_WEATHER_AGE_S];
  local_token = s_holding_regs[MODBUS_HREG_WEATHER_SET_TOKEN];
  ESP_ERROR_CHECK(mbc_slave_unlock(s_mbc_slave_handler));

  *snapshot = tmp;
  *token = local_token;
  return true;
}

static bool weather_payload_equal(const weather_snapshot_t *lhs,
                                  const weather_snapshot_t *rhs) {
  if (lhs == NULL || rhs == NULL) {
    return false;
  }

  return lhs->out_temp == rhs->out_temp && lhs->out_hum == rhs->out_hum &&
         lhs->wind_speed == rhs->wind_speed && lhs->wind_dir == rhs->wind_dir &&
         lhs->rain_flag == rhs->rain_flag && lhs->solar_rad == rhs->solar_rad &&
         lhs->baro_press == rhs->baro_press && lhs->dew_point == rhs->dew_point &&
         lhs->status_bits == rhs->status_bits;
}

static void log_weather_snapshot(const char *prefix, uint16_t token,
                                 const weather_snapshot_t *snapshot) {
  if (prefix == NULL || snapshot == NULL) {
    return;
  }

  ESP_LOGI(TAG,
           "%s token=%u out_temp=%.1fC out_hum=%.1f%% wind=%.1fm/s dir=%u "
           "rain=%u solar=%uW/m2 baro=%.1fhPa dew=%.1fC status=0x%04X age=%us",
           prefix, (unsigned)token, ((float)snapshot->out_temp) / 10.0f,
           ((float)snapshot->out_hum) / 10.0f,
           ((float)snapshot->wind_speed) / 10.0f,
           (unsigned)snapshot->wind_dir, (unsigned)snapshot->rain_flag,
           (unsigned)snapshot->solar_rad, ((float)snapshot->baro_press) / 10.0f,
           ((float)snapshot->dew_point) / 10.0f,
           (unsigned)snapshot->status_bits, (unsigned)snapshot->source_age_s);
}

static void queue_weather_sync_request_from_regs(void) {
  weather_snapshot_t snapshot = {0};
  uint16_t token = 0;
  bool should_log_queued = false;
  if (!load_weather_snapshot_from_regs(&snapshot, &token) || token == 0U) {
    return;
  }

  taskENTER_CRITICAL(&s_state_lock);
  bool same_as_last = (token == s_weather_last_token);
  bool same_as_pending = (s_weather_sync_pending && token == s_weather_pending_token);
  if (!same_as_last && !same_as_pending) {
    s_weather_pending_token = token;
    s_weather_pending_snapshot = snapshot;
    s_weather_sync_pending = true;
    should_log_queued = true;
  }
  taskEXIT_CRITICAL(&s_state_lock);

  if (should_log_queued) {
    log_weather_snapshot("Weather sync QUEUED", token, &snapshot);
  }
}

static void finalize_weather_sync(uint16_t token,
                                  modbus_weather_set_result_t result) {
  if (s_mbc_slave_handler != NULL) {
    ESP_ERROR_CHECK(mbc_slave_lock(s_mbc_slave_handler));
    s_holding_regs[MODBUS_HREG_WEATHER_SET_APPLIED_TOKEN] = token;
    s_holding_regs[MODBUS_HREG_WEATHER_SET_RESULT] = (uint16_t)result;
    ESP_ERROR_CHECK(mbc_slave_unlock(s_mbc_slave_handler));
  }

  taskENTER_CRITICAL(&s_state_lock);
  s_weather_last_token = token;
  s_weather_pending_token = 0;
  s_weather_sync_pending = false;
  taskEXIT_CRITICAL(&s_state_lock);

  queue_weather_sync_request_from_regs();
}

static void process_pending_weather_sync(void) {
  bool pending = false;
  weather_snapshot_t pending_snapshot = {0};
  weather_snapshot_t active_snapshot = {0};
  bool weather_valid = false;
  uint16_t token = 0;
  uint32_t now = now_ms();

  taskENTER_CRITICAL(&s_state_lock);
  pending = s_weather_sync_pending;
  if (pending) {
    pending_snapshot = s_weather_pending_snapshot;
    token = s_weather_pending_token;
    active_snapshot = s_weather_active_snapshot;
    weather_valid = s_weather_valid;
  }
  taskEXIT_CRITICAL(&s_state_lock);

  if (!pending || token == 0U) {
    return;
  }

  if (weather_valid && weather_payload_equal(&pending_snapshot, &active_snapshot)) {
    log_weather_snapshot("Weather sync NOOP", token, &pending_snapshot);
    taskENTER_CRITICAL(&s_state_lock);
    s_weather_last_rx_ms = now;
    s_weather_stale = false;
    taskEXIT_CRITICAL(&s_state_lock);
    finalize_weather_sync(token, MODBUS_WEATHER_SET_RESULT_NOOP);
    return;
  }

  log_weather_snapshot("Weather sync APPLIED", token, &pending_snapshot);
  taskENTER_CRITICAL(&s_state_lock);
  s_weather_active_snapshot = pending_snapshot;
  s_weather_valid = true;
  s_weather_last_rx_ms = now;
  s_weather_stale = false;
  taskEXIT_CRITICAL(&s_state_lock);

  finalize_weather_sync(token, MODBUS_WEATHER_SET_RESULT_APPLIED);
}

static bool decode_write_holding_span(uint8_t fc, const uint8_t *frame, uint16_t len,
                                      uint16_t *start_reg, uint16_t *reg_count) {
  if (frame == NULL || start_reg == NULL || reg_count == NULL) {
    return false;
  }

  uint16_t raw_start = 0;
  uint16_t raw_count = 0;
  switch (fc) {
  case 0x06:
    if (len < 5U) {
      return false;
    }
    raw_start = (uint16_t)(((uint16_t)frame[1] << 8U) | (uint16_t)frame[2]);
    raw_count = 1U;
    break;
  case 0x10:
    if (len < 6U) {
      return false;
    }
    raw_start = (uint16_t)(((uint16_t)frame[1] << 8U) | (uint16_t)frame[2]);
    raw_count = (uint16_t)(((uint16_t)frame[3] << 8U) | (uint16_t)frame[4]);
    break;
  case 0x17:
    if (len < 10U) {
      return false;
    }
    raw_start = (uint16_t)(((uint16_t)frame[5] << 8U) | (uint16_t)frame[6]);
    raw_count = (uint16_t)(((uint16_t)frame[7] << 8U) | (uint16_t)frame[8]);
    break;
  default:
    return false;
  }

  if (raw_count == 0U) {
    return false;
  }

  // Internal register callbacks use one-based addressing, so normalize once.
  *start_reg = (uint16_t)(raw_start + 1U);
  *reg_count = raw_count;
  return true;
}

static bool reg_span_contains(uint16_t start_reg, uint16_t reg_count,
                              uint16_t target_reg) {
  if (reg_count == 0U) {
    return false;
  }
  uint32_t end_reg = (uint32_t)start_reg + (uint32_t)reg_count - 1U;
  return ((uint32_t)target_reg >= (uint32_t)start_reg) &&
         ((uint32_t)target_reg <= end_reg);
}

static bool reg_span_intersects(uint16_t start_reg, uint16_t reg_count,
                                uint16_t first_reg, uint16_t last_reg) {
  if (reg_count == 0U || first_reg > last_reg) {
    return false;
  }
  uint32_t end_reg = (uint32_t)start_reg + (uint32_t)reg_count - 1U;
  if (end_reg < (uint32_t)first_reg) {
    return false;
  }
  if ((uint32_t)start_reg > (uint32_t)last_reg) {
    return false;
  }
  return true;
}

static bool reg_span_intersects_window_settings(uint16_t start_reg,
                                                uint16_t reg_count) {
  return reg_span_intersects(start_reg, reg_count,
                             MODBUS_HREG_WINDOWS_POS_A_TARGET,
                             MODBUS_HREG_WINDOWS_POS_B_TARGET) ||
         reg_span_intersects(start_reg, reg_count,
                             MODBUS_HREG_WINDOWS_CTRL_MODE,
                             MODBUS_HREG_RLL400_NO_MOTION_TIMEOUT_MS) ||
         reg_span_intersects(start_reg, reg_count,
                             MODBUS_HREG_WINDOWS_AUTO_ALGO_MODE,
                             MODBUS_HREG_WINDOWS_WEATHER_STALE_POLICY) ||
         reg_span_intersects(start_reg, reg_count,
                             MODBUS_HREG_WINDOWS_TEMP_STEP_TARGET_PERCENT,
                             MODBUS_HREG_WINDOWS_WEATHER_SOURCE_AGE_S);
}

static bool write_span_intersects_window_settings(bool has_span,
                                                  uint16_t start_reg,
                                                  uint16_t alt_start_reg,
                                                  uint16_t reg_count) {
  return has_span &&
         (reg_span_intersects_window_settings(start_reg, reg_count) ||
          reg_span_intersects_window_settings(alt_start_reg, reg_count));
}

static bool write_span_intersects_greenhouse_targets(bool has_span,
                                                     uint16_t start_reg,
                                                     uint16_t alt_start_reg,
                                                     uint16_t reg_count) {
  return has_span &&
         (reg_span_intersects(start_reg, reg_count,
                              MODBUS_HREG_AIR_TEMP_TARGET,
                              MODBUS_HREG_AIR_HUM_TARGET) ||
          reg_span_intersects(alt_start_reg, reg_count,
                              MODBUS_HREG_AIR_TEMP_TARGET,
                              MODBUS_HREG_AIR_HUM_TARGET));
}

static void log_greenhouse_targets_received(uint16_t start_reg,
                                            uint16_t reg_count) {
  if (s_mbc_slave_handler == NULL) {
    return;
  }

  ESP_ERROR_CHECK(mbc_slave_lock(s_mbc_slave_handler));
  const uint16_t air_temp_target = s_holding_regs[MODBUS_HREG_AIR_TEMP_TARGET];
  const uint16_t air_hum_target = s_holding_regs[MODBUS_HREG_AIR_HUM_TARGET];
  ESP_ERROR_CHECK(mbc_slave_unlock(s_mbc_slave_handler));

  ESP_LOGI(TAG,
           "Greenhouse targets received from master: span=%u..%u "
           "air_temp_target=%.1fC air_hum_target=%.1f%%",
           (unsigned)start_reg, (unsigned)(start_reg + reg_count - 1U),
           ((float)air_temp_target) / 10.0f,
           ((float)air_hum_target) / 10.0f);
}

static void log_window_settings_received(uint16_t start_reg,
                                         uint16_t reg_count) {
  if (s_mbc_slave_handler == NULL) {
    return;
  }

  ESP_ERROR_CHECK(mbc_slave_lock(s_mbc_slave_handler));
  const uint16_t requested_a =
      s_holding_regs[MODBUS_HREG_WINDOWS_POS_A_TARGET];
  const uint16_t requested_b =
      s_holding_regs[MODBUS_HREG_WINDOWS_POS_B_TARGET];
  const modbus_windows_ctrl_mode_t ctrl_mode =
      (s_holding_regs[MODBUS_HREG_WINDOWS_CTRL_MODE] ==
       (uint16_t)MODBUS_WINDOWS_CTRL_MODE_MANUAL)
          ? MODBUS_WINDOWS_CTRL_MODE_MANUAL
          : MODBUS_WINDOWS_CTRL_MODE_AUTO;
  const modbus_windows_auto_algo_mode_t algo_mode =
      (s_holding_regs[MODBUS_HREG_WINDOWS_AUTO_ALGO_MODE] ==
       (uint16_t)MODBUS_WINDOWS_AUTO_ALGO_HUMIDITY)
          ? MODBUS_WINDOWS_AUTO_ALGO_HUMIDITY
          : MODBUS_WINDOWS_AUTO_ALGO_TEMP;
  const uint16_t temp_sp = s_holding_regs[MODBUS_HREG_WINDOWS_TEMP_SETPOINT];
  const uint16_t temp_step = s_holding_regs[MODBUS_HREG_WINDOWS_TEMP_STEP_C];
  const uint16_t temp_hyst =
      s_holding_regs[MODBUS_HREG_WINDOWS_TEMP_STEP_HYST_C];
  const uint16_t temp_open =
      s_holding_regs[MODBUS_HREG_WINDOWS_TEMP_STEP_TARGET_PERCENT];
  const uint16_t hum_sp = s_holding_regs[MODBUS_HREG_WINDOWS_HUM_SETPOINT];
  const uint16_t hum_step = s_holding_regs[MODBUS_HREG_WINDOWS_HUM_STEP];
  const uint16_t hum_hyst = s_holding_regs[MODBUS_HREG_WINDOWS_HUM_STEP_HYST];
  const uint16_t hum_open =
      s_holding_regs[MODBUS_HREG_WINDOWS_HUM_STEP_TARGET_PERCENT];
  const uint16_t cold_delta =
      s_holding_regs[MODBUS_HREG_WINDOWS_COLD_CLOSE_DELTA];
  const uint16_t cold_hyst =
      s_holding_regs[MODBUS_HREG_WINDOWS_COLD_CLOSE_HYST];
  const uint16_t safe_min =
      s_holding_regs[MODBUS_HREG_WINDOWS_SAFE_MIN_PERCENT];
  const uint16_t storm = s_holding_regs[MODBUS_HREG_WINDOWS_WIND_STORM];
  const uint16_t recover = s_holding_regs[MODBUS_HREG_WINDOWS_WIND_RECOVER];
  const uint16_t azimuth = s_holding_regs[MODBUS_HREG_WINDOW_A_AZIMUTH_DEG];
  const uint16_t sector =
      s_holding_regs[MODBUS_HREG_WINDOWS_WIND_SECTOR_HALF_WIDTH_DEG];
  const uint16_t windward_min =
      s_holding_regs[MODBUS_HREG_WINDOWS_WINDWARD_MIN_PERCENT];
  const uint16_t windward_max =
      s_holding_regs[MODBUS_HREG_WINDOWS_WINDWARD_MAX_PERCENT];
  const uint16_t windward_thr =
      s_holding_regs[MODBUS_HREG_WINDOWS_WINDWARD_SPEED_THRESHOLD];
  const uint16_t windward_reduce =
      s_holding_regs[MODBUS_HREG_WINDOWS_WINDWARD_REDUCTION_PERCENT_PER_MS];
  const uint16_t leeward_min =
      s_holding_regs[MODBUS_HREG_WINDOWS_LEEWARD_MIN_PERCENT];
  const uint16_t leeward_max =
      s_holding_regs[MODBUS_HREG_WINDOWS_LEEWARD_MAX_PERCENT];
  const uint16_t leeward_thr =
      s_holding_regs[MODBUS_HREG_WINDOWS_LEEWARD_SPEED_THRESHOLD];
  const uint16_t leeward_reduce =
      s_holding_regs[MODBUS_HREG_WINDOWS_LEEWARD_REDUCTION_PERCENT_PER_MS];
  const uint16_t wind_lag =
      s_holding_regs[MODBUS_HREG_WINDOWS_WINDWARD_LAG_PERCENT];
  const uint16_t rain_mode = s_holding_regs[MODBUS_HREG_WINDOWS_RAIN_MODE];
  const uint16_t rain_pos =
      s_holding_regs[MODBUS_HREG_WINDOWS_RAIN_WINDWARD_PERCENT];
  const uint16_t stale_policy =
      s_holding_regs[MODBUS_HREG_WINDOWS_WEATHER_STALE_POLICY];
  const uint16_t stale_timeout =
      s_holding_regs[MODBUS_HREG_WINDOWS_WEATHER_STALE_TIMEOUT_MS];
  const uint16_t source_age =
      s_holding_regs[MODBUS_HREG_WINDOWS_WEATHER_SOURCE_AGE_S];
  const uint16_t target_hyst =
      s_holding_regs[MODBUS_HREG_RLL400_TARGET_HYST_PERCENT];
  const uint16_t motion_delta =
      s_holding_regs[MODBUS_HREG_RLL400_MOTION_DELTA_PERCENT];
  const uint16_t no_motion =
      s_holding_regs[MODBUS_HREG_RLL400_NO_MOTION_TIMEOUT_MS];
  ESP_ERROR_CHECK(mbc_slave_unlock(s_mbc_slave_handler));

  ESP_LOGI(TAG,
           "Windows settings received from master: span=%u..%u mode=%s "
           "alg=%s requested[A=%.1f%% B=%.1f%%] temp[sp=%.1fC step=%.1fC "
           "hyst=%.1fC open=%.1f%%] hum[sp=%.1f%% step=%.1f%% hyst=%.1f%% "
           "open=%.1f%%] cold[delta=%.1fC hyst=%.1fC]",
           (unsigned)start_reg,
           (unsigned)(start_reg + reg_count - 1U),
           windows_ctrl_mode_to_string(ctrl_mode),
           windows_auto_algo_to_string(algo_mode), ((float)requested_a) / 10.0f,
           ((float)requested_b) / 10.0f, ((float)temp_sp) / 10.0f,
           ((float)temp_step) / 10.0f, ((float)temp_hyst) / 10.0f,
           ((float)temp_open) / 10.0f, ((float)hum_sp) / 10.0f,
           ((float)hum_step) / 10.0f, ((float)hum_hyst) / 10.0f,
           ((float)hum_open) / 10.0f, ((float)cold_delta) / 10.0f,
           ((float)cold_hyst) / 10.0f);
  ESP_LOGI(TAG,
           "Windows protections received from master: safe=%.1f%% storm=%.1fm/s "
           "recover=%.1fm/s azimuth=%u sector=%u windward[min=%.1f%% "
           "max=%.1f%% thr=%.1fm/s reduce=%.1f%%/m/s] leeward[min=%.1f%% "
           "max=%.1f%% thr=%.1fm/s reduce=%.1f%%/m/s] lag=%.1f%% "
           "rain[mode=%u pos=%.1f%%] stale[policy=%u timeout=%ums age=%us] "
           "rll400[hyst=%.1f%% motion=%.1f%% timeout=%ums]",
           ((float)safe_min) / 10.0f, ((float)storm) / 10.0f,
           ((float)recover) / 10.0f, (unsigned)azimuth, (unsigned)sector,
           ((float)windward_min) / 10.0f, ((float)windward_max) / 10.0f,
           ((float)windward_thr) / 10.0f, ((float)windward_reduce) / 10.0f,
           ((float)leeward_min) / 10.0f, ((float)leeward_max) / 10.0f,
           ((float)leeward_thr) / 10.0f, ((float)leeward_reduce) / 10.0f,
           ((float)wind_lag) / 10.0f, (unsigned)rain_mode,
           ((float)rain_pos) / 10.0f, (unsigned)stale_policy,
           (unsigned)stale_timeout, (unsigned)source_age,
           ((float)target_hyst) / 10.0f, ((float)motion_delta) / 10.0f,
           (unsigned)no_motion);
}

static bool write_span_intersects_curtain_settings(bool has_span,
                                                   uint16_t start_reg,
                                                   uint16_t alt_start_reg,
                                                   uint16_t reg_count) {
  return has_span &&
         (reg_span_intersects(start_reg, reg_count,
                              MODBUS_HREG_CURTAIN_POS_TARGET,
                              MODBUS_HREG_CURTAIN_POS_TARGET) ||
          reg_span_intersects(alt_start_reg, reg_count,
                              MODBUS_HREG_CURTAIN_POS_TARGET,
                              MODBUS_HREG_CURTAIN_POS_TARGET) ||
          reg_span_intersects(start_reg, reg_count,
                              MODBUS_HREG_CURTAIN_CTRL_MODE,
                              MODBUS_HREG_CURTAIN_FAULT_RESET_TOKEN) ||
          reg_span_intersects(alt_start_reg, reg_count,
                              MODBUS_HREG_CURTAIN_CTRL_MODE,
                              MODBUS_HREG_CURTAIN_FAULT_RESET_TOKEN));
}

static void log_curtain_settings_received(uint16_t start_reg,
                                          uint16_t reg_count) {
  const modbus_curtain_ctrl_mode_t mode = modbus_get_curtain_ctrl_mode();
  const char *mode_text = "MANUAL";
  if (mode == MODBUS_CURTAIN_CTRL_MODE_AUTO) {
    mode_text = "AUTO";
  } else if (mode == MODBUS_CURTAIN_CTRL_MODE_OFF) {
    mode_text = "OFF";
  }

  ESP_LOGI(TAG,
           "Curtain settings received from master: span=%u..%u mode=%s "
           "requested=%.1f%% manual=%.1f%% schedule=%04u..%04u "
           "limits[min=%.1f%% max=%.1f%% hyst=%.1f%% outside=%.1f%%]",
           (unsigned)start_reg,
           (unsigned)(start_reg + reg_count - 1U), mode_text,
           modbus_get_curtain_target_percent(),
           modbus_get_curtain_manual_target_percent(),
           (unsigned)modbus_get_curtain_schedule_start_hhmm(),
           (unsigned)modbus_get_curtain_schedule_end_hhmm(),
           modbus_get_curtain_min_position_percent(),
           modbus_get_curtain_max_position_percent(),
           modbus_get_curtain_position_hysteresis_percent(),
           modbus_get_curtain_outside_target_percent());
  ESP_LOGI(TAG,
           "Curtain auto rules received from master: radiation[thr=%uW/m2 "
           "step=%uW/m2 open=%.1f%% hyst=%uW/m2] cold[delta=%.1fC "
           "hyst=%.1fC target=%.1f%%] heat[delta=%.1fC hyst=%.1fC "
           "target=%.1f%%] hum_low[delta=%.1f%% hyst=%.1f%% target=%.1f%%] "
           "hum_high[delta=%.1f%% hyst=%.1f%% target=%.1f%%] reset_token=%u",
           (unsigned)modbus_get_curtain_radiation_threshold_wm2(),
           (unsigned)modbus_get_curtain_radiation_step_wm2(),
           modbus_get_curtain_radiation_step_percent(),
           (unsigned)modbus_get_curtain_radiation_hysteresis_wm2(),
           modbus_get_curtain_cold_delta_c(),
           modbus_get_curtain_cold_hysteresis_c(),
           modbus_get_curtain_cold_target_percent(),
           modbus_get_curtain_heat_delta_c(),
           modbus_get_curtain_heat_hysteresis_c(),
           modbus_get_curtain_heat_target_percent(),
           modbus_get_curtain_humidity_low_threshold_percent(),
           modbus_get_curtain_humidity_low_hysteresis_percent(),
           modbus_get_curtain_humidity_low_target_percent(),
           modbus_get_curtain_humidity_high_threshold_percent(),
           modbus_get_curtain_humidity_high_hysteresis_percent(),
           modbus_get_curtain_humidity_high_target_percent(),
           (unsigned)modbus_get_curtain_fault_reset_token());
}

static void sync_staging_schedule_from_current_regs(void) {
  light_control_cfg_t staging = {0};
  light_control_cfg_t prev_staging = {0};
  uint32_t now = now_ms();
  regs_to_light_ctrl(s_holding_regs, &staging);

  taskENTER_CRITICAL(&s_state_lock);
  prev_staging = s_staging_light_cfg;
  s_staging_light_cfg = staging;
  s_light_cfg_dirty = true;
  s_light_cfg_last_change_ms = now;
  taskEXIT_CRITICAL(&s_state_lock);

  if (!light_schedule_equal(&prev_staging.relay[0].schedule,
                            &staging.relay[0].schedule)) {
    log_light_schedule_cfg("Relay 1 schedule received",
                           &staging.relay[0].schedule);
  }
  if (!light_schedule_equal(&prev_staging.relay[1].schedule,
                            &staging.relay[1].schedule)) {
    log_light_schedule_cfg("Relay 2 schedule received",
                           &staging.relay[1].schedule);
  }
  log_light_ctrl_cfg("Light cfg received", &staging);
}

static void queue_apply_request_from_current_regs(void) {
  light_control_cfg_t staging = {0};
  light_control_cfg_t prev_staging = {0};
  regs_to_light_ctrl(s_holding_regs, &staging);

  taskENTER_CRITICAL(&s_state_lock);
  bool was_pending = s_apply_pending;
  prev_staging = s_staging_light_cfg;
  s_staging_light_cfg = staging;
  s_apply_pending_light_cfg = staging;
  s_apply_pending = true;
  s_light_cfg_dirty = false;
  s_light_cfg_last_change_ms = 0U;
  if (was_pending && s_apply_fail_busy_count < UINT32_MAX) {
    s_apply_fail_busy_count++;
  }
  taskEXIT_CRITICAL(&s_state_lock);

  if (!light_schedule_equal(&prev_staging.relay[0].schedule,
                            &staging.relay[0].schedule)) {
    log_light_schedule_cfg("Relay 1 schedule received",
                           &staging.relay[0].schedule);
  }
  if (!light_schedule_equal(&prev_staging.relay[1].schedule,
                            &staging.relay[1].schedule)) {
    log_light_schedule_cfg("Relay 2 schedule received",
                           &staging.relay[1].schedule);
  }
  ESP_LOGI(TAG, "Light cfg auto-queued from registers 110..122");
  log_light_ctrl_cfg("Light cfg queued for apply", &staging);
}

static mb_exception_t invoke_wrapped_handler(uint8_t fc, void *ctx, uint8_t *frame,
                                             uint16_t *len_buf) {
  mb_fn_handler_fp original = NULL;
  for (int i = 0; i < MODBUS_FC_COUNT; ++i) {
    if (s_handler_wraps[i].fc == fc) {
      original = s_handler_wraps[i].original;
      break;
    }
  }

  if (original == NULL) {
    return 0x01; // Illegal function
  }

  mb_exception_t ex = original(ctx, frame, len_buf);
  update_master_seen_and_streak(ex == 0);
  return ex;
}

static mb_exception_t modbus_fc_01_wrapper(void *ctx, uint8_t *frame,
                                           uint16_t *len_buf) {
  return invoke_wrapped_handler(0x01, ctx, frame, len_buf);
}

static mb_exception_t modbus_fc_02_wrapper(void *ctx, uint8_t *frame,
                                           uint16_t *len_buf) {
  return invoke_wrapped_handler(0x02, ctx, frame, len_buf);
}

static mb_exception_t modbus_fc_03_wrapper(void *ctx, uint8_t *frame,
                                           uint16_t *len_buf) {
  return invoke_wrapped_handler(0x03, ctx, frame, len_buf);
}

static mb_exception_t modbus_fc_04_wrapper(void *ctx, uint8_t *frame,
                                           uint16_t *len_buf) {
  return invoke_wrapped_handler(0x04, ctx, frame, len_buf);
}

static mb_exception_t modbus_fc_05_wrapper(void *ctx, uint8_t *frame,
                                           uint16_t *len_buf) {
  return invoke_wrapped_handler(0x05, ctx, frame, len_buf);
}

static mb_exception_t modbus_fc_06_wrapper(void *ctx, uint8_t *frame,
                                           uint16_t *len_buf) {
  uint16_t start_reg = 0;
  uint16_t reg_count = 0;
  uint16_t req_len = (len_buf != NULL) ? *len_buf : 0U;
  bool has_span = decode_write_holding_span(0x06, frame, req_len, &start_reg, &reg_count);
  uint16_t alt_start_reg = (start_reg > 0U) ? (uint16_t)(start_reg - 1U) : start_reg;
  bool writes_schedule =
      has_span &&
      (reg_span_intersects(start_reg, reg_count, MODBUS_HREG_LIGHT_R1_ENABLE,
                           MODBUS_HREG_LIGHT_HYST_SEC) ||
       reg_span_intersects(alt_start_reg, reg_count, MODBUS_HREG_LIGHT_R1_ENABLE,
                           MODBUS_HREG_LIGHT_HYST_SEC));
  bool writes_full_light_cfg =
      has_span &&
      ((reg_span_contains(start_reg, reg_count, MODBUS_HREG_LIGHT_R1_ENABLE) &&
        reg_span_contains(start_reg, reg_count, MODBUS_HREG_LIGHT_HYST_SEC)) ||
       (reg_span_contains(alt_start_reg, reg_count, MODBUS_HREG_LIGHT_R1_ENABLE) &&
        reg_span_contains(alt_start_reg, reg_count, MODBUS_HREG_LIGHT_HYST_SEC)));
  bool writes_weather =
      has_span &&
      (reg_span_intersects(start_reg, reg_count, MODBUS_HREG_WEATHER_OUT_TEMP,
                           MODBUS_HREG_WEATHER_SET_TOKEN) ||
       reg_span_intersects(alt_start_reg, reg_count, MODBUS_HREG_WEATHER_OUT_TEMP,
                           MODBUS_HREG_WEATHER_SET_TOKEN));
  bool writes_window_settings = write_span_intersects_window_settings(
      has_span, start_reg, alt_start_reg, reg_count);
  bool writes_curtain_settings = write_span_intersects_curtain_settings(
      has_span, start_reg, alt_start_reg, reg_count);
  bool writes_greenhouse_targets = write_span_intersects_greenhouse_targets(
      has_span, start_reg, alt_start_reg, reg_count);

  mb_exception_t ex = invoke_wrapped_handler(0x06, ctx, frame, len_buf);
  if (ex == 0) {
    if (writes_greenhouse_targets) {
      log_greenhouse_targets_received(start_reg, reg_count);
    }
    if (writes_window_settings) {
      log_window_settings_received(start_reg, reg_count);
    }
    if (writes_curtain_settings) {
      log_curtain_settings_received(start_reg, reg_count);
    }
    queue_rtc_sync_request_from_regs();
    if (writes_weather) {
      queue_weather_sync_request_from_regs();
    }
    if (writes_full_light_cfg) {
      queue_apply_request_from_current_regs();
    } else if (writes_schedule) {
      sync_staging_schedule_from_current_regs();
    }
  }
  return ex;
}

static mb_exception_t modbus_fc_0F_wrapper(void *ctx, uint8_t *frame,
                                           uint16_t *len_buf) {
  return invoke_wrapped_handler(0x0F, ctx, frame, len_buf);
}

static mb_exception_t modbus_fc_10_wrapper(void *ctx, uint8_t *frame,
                                           uint16_t *len_buf) {
  uint16_t start_reg = 0;
  uint16_t reg_count = 0;
  uint16_t req_len = (len_buf != NULL) ? *len_buf : 0U;
  bool has_span = decode_write_holding_span(0x10, frame, req_len, &start_reg, &reg_count);
  uint16_t alt_start_reg = (start_reg > 0U) ? (uint16_t)(start_reg - 1U) : start_reg;
  bool writes_schedule =
      has_span &&
      (reg_span_intersects(start_reg, reg_count, MODBUS_HREG_LIGHT_R1_ENABLE,
                           MODBUS_HREG_LIGHT_HYST_SEC) ||
       reg_span_intersects(alt_start_reg, reg_count, MODBUS_HREG_LIGHT_R1_ENABLE,
                           MODBUS_HREG_LIGHT_HYST_SEC));
  bool writes_full_light_cfg =
      has_span &&
      ((reg_span_contains(start_reg, reg_count, MODBUS_HREG_LIGHT_R1_ENABLE) &&
        reg_span_contains(start_reg, reg_count, MODBUS_HREG_LIGHT_HYST_SEC)) ||
       (reg_span_contains(alt_start_reg, reg_count, MODBUS_HREG_LIGHT_R1_ENABLE) &&
        reg_span_contains(alt_start_reg, reg_count, MODBUS_HREG_LIGHT_HYST_SEC)));
  bool writes_weather =
      has_span &&
      (reg_span_intersects(start_reg, reg_count, MODBUS_HREG_WEATHER_OUT_TEMP,
                           MODBUS_HREG_WEATHER_SET_TOKEN) ||
       reg_span_intersects(alt_start_reg, reg_count, MODBUS_HREG_WEATHER_OUT_TEMP,
                           MODBUS_HREG_WEATHER_SET_TOKEN));
  bool writes_window_settings = write_span_intersects_window_settings(
      has_span, start_reg, alt_start_reg, reg_count);
  bool writes_curtain_settings = write_span_intersects_curtain_settings(
      has_span, start_reg, alt_start_reg, reg_count);
  bool writes_greenhouse_targets = write_span_intersects_greenhouse_targets(
      has_span, start_reg, alt_start_reg, reg_count);

  mb_exception_t ex = invoke_wrapped_handler(0x10, ctx, frame, len_buf);
  if (ex == 0) {
    if (writes_greenhouse_targets) {
      log_greenhouse_targets_received(start_reg, reg_count);
    }
    if (writes_window_settings) {
      log_window_settings_received(start_reg, reg_count);
    }
    if (writes_curtain_settings) {
      log_curtain_settings_received(start_reg, reg_count);
    }
    queue_rtc_sync_request_from_regs();
    if (writes_weather) {
      queue_weather_sync_request_from_regs();
    }
    if (writes_full_light_cfg) {
      queue_apply_request_from_current_regs();
    } else if (writes_schedule) {
      sync_staging_schedule_from_current_regs();
    }
  }
  return ex;
}

static mb_exception_t modbus_fc_11_wrapper(void *ctx, uint8_t *frame,
                                           uint16_t *len_buf) {
  return invoke_wrapped_handler(0x11, ctx, frame, len_buf);
}

static mb_exception_t modbus_fc_17_wrapper(void *ctx, uint8_t *frame,
                                           uint16_t *len_buf) {
  uint16_t start_reg = 0;
  uint16_t reg_count = 0;
  uint16_t req_len = (len_buf != NULL) ? *len_buf : 0U;
  bool has_span = decode_write_holding_span(0x17, frame, req_len, &start_reg, &reg_count);
  uint16_t alt_start_reg = (start_reg > 0U) ? (uint16_t)(start_reg - 1U) : start_reg;
  bool writes_schedule =
      has_span &&
      (reg_span_intersects(start_reg, reg_count, MODBUS_HREG_LIGHT_R1_ENABLE,
                           MODBUS_HREG_LIGHT_HYST_SEC) ||
       reg_span_intersects(alt_start_reg, reg_count, MODBUS_HREG_LIGHT_R1_ENABLE,
                           MODBUS_HREG_LIGHT_HYST_SEC));
  bool writes_full_light_cfg =
      has_span &&
      ((reg_span_contains(start_reg, reg_count, MODBUS_HREG_LIGHT_R1_ENABLE) &&
        reg_span_contains(start_reg, reg_count, MODBUS_HREG_LIGHT_HYST_SEC)) ||
       (reg_span_contains(alt_start_reg, reg_count, MODBUS_HREG_LIGHT_R1_ENABLE) &&
        reg_span_contains(alt_start_reg, reg_count, MODBUS_HREG_LIGHT_HYST_SEC)));
  bool writes_weather =
      has_span &&
      (reg_span_intersects(start_reg, reg_count, MODBUS_HREG_WEATHER_OUT_TEMP,
                           MODBUS_HREG_WEATHER_SET_TOKEN) ||
       reg_span_intersects(alt_start_reg, reg_count, MODBUS_HREG_WEATHER_OUT_TEMP,
                           MODBUS_HREG_WEATHER_SET_TOKEN));
  bool writes_window_settings = write_span_intersects_window_settings(
      has_span, start_reg, alt_start_reg, reg_count);
  bool writes_curtain_settings = write_span_intersects_curtain_settings(
      has_span, start_reg, alt_start_reg, reg_count);
  bool writes_greenhouse_targets = write_span_intersects_greenhouse_targets(
      has_span, start_reg, alt_start_reg, reg_count);

  mb_exception_t ex = invoke_wrapped_handler(0x17, ctx, frame, len_buf);
  if (ex == 0) {
    if (writes_greenhouse_targets) {
      log_greenhouse_targets_received(start_reg, reg_count);
    }
    if (writes_window_settings) {
      log_window_settings_received(start_reg, reg_count);
    }
    if (writes_curtain_settings) {
      log_curtain_settings_received(start_reg, reg_count);
    }
    queue_rtc_sync_request_from_regs();
    if (writes_weather) {
      queue_weather_sync_request_from_regs();
    }
    if (writes_full_light_cfg) {
      queue_apply_request_from_current_regs();
    } else if (writes_schedule) {
      sync_staging_schedule_from_current_regs();
    }
  }
  return ex;
}

static void install_handler_wrappers(void) {
  for (int i = 0; i < MODBUS_FC_COUNT; ++i) {
    mb_fn_handler_fp original = NULL;
    esp_err_t err = mbc_get_handler(s_mbc_slave_handler, s_handler_wraps[i].fc, &original);
    if (err != ESP_OK) {
      ESP_LOGW(TAG, "FC 0x%02X lookup failed: %s", s_handler_wraps[i].fc,
               esp_err_to_name(err));
      continue;
    }
    if (original == NULL) {
      ESP_LOGW(TAG, "FC 0x%02X lookup returned NULL handler", s_handler_wraps[i].fc);
      continue;
    }

    s_handler_wraps[i].original = original;
    err = mbc_set_handler(s_mbc_slave_handler, s_handler_wraps[i].fc,
                          s_handler_wraps[i].wrapper);
    if (err != ESP_OK) {
      s_handler_wraps[i].original = NULL;
      ESP_LOGW(TAG, "Failed to wrap FC 0x%02X: %s", s_handler_wraps[i].fc,
               esp_err_to_name(err));
    } else {
      ESP_LOGI(TAG, "Wrapped FC 0x%02X", s_handler_wraps[i].fc);
    }
  }
}

static void queue_apply_request_from_regs(void) {
  if (s_mbc_slave_handler == NULL) {
    return;
  }

  light_control_cfg_t staging = {0};
  light_control_cfg_t prev_staging = {0};
  bool staging_changed = false;
  bool auto_apply = false;
  uint32_t now = now_ms();
  ESP_ERROR_CHECK(mbc_slave_lock(s_mbc_slave_handler));
  regs_to_light_ctrl(s_holding_regs, &staging);
  ESP_ERROR_CHECK(mbc_slave_unlock(s_mbc_slave_handler));

  taskENTER_CRITICAL(&s_state_lock);
  prev_staging = s_staging_light_cfg;
  if (!light_ctrl_equal(&prev_staging, &staging)) {
    s_staging_light_cfg = staging;
    s_light_cfg_dirty = true;
    s_light_cfg_last_change_ms = now;
    staging_changed = true;
  } else if (s_light_cfg_dirty &&
             (uint32_t)(now - s_light_cfg_last_change_ms) >=
                 MODBUS_LIGHT_AUTO_APPLY_SETTLE_MS) {
    bool was_pending = s_apply_pending;
    s_apply_pending_light_cfg = staging;
    s_apply_pending = true;
    s_light_cfg_dirty = false;
    s_light_cfg_last_change_ms = 0U;
    auto_apply = true;
    if (was_pending && s_apply_fail_busy_count < UINT32_MAX) {
      s_apply_fail_busy_count++;
    }
  }
  taskEXIT_CRITICAL(&s_state_lock);

  if (staging_changed) {
    if (!light_schedule_equal(&prev_staging.relay[0].schedule,
                              &staging.relay[0].schedule)) {
      log_light_schedule_cfg("Relay 1 schedule received",
                             &staging.relay[0].schedule);
    }
    if (!light_schedule_equal(&prev_staging.relay[1].schedule,
                              &staging.relay[1].schedule)) {
      log_light_schedule_cfg("Relay 2 schedule received",
                             &staging.relay[1].schedule);
    }
    log_light_ctrl_cfg("Light cfg received", &staging);
  }

  if (auto_apply) {
    ESP_LOGI(TAG, "Light cfg auto-queued after %u ms settle",
             (unsigned)MODBUS_LIGHT_AUTO_APPLY_SETTLE_MS);
    log_light_ctrl_cfg("Light cfg queued for apply", &staging);
  }
}

static void finalize_apply_result(modbus_apply_status_t status) {
  uint32_t ts = now_ms();

  taskENTER_CRITICAL(&s_state_lock);
  s_last_apply_status = status;
  s_last_apply_ts_ms = ts;
  if (status == MODBUS_APPLY_OK) {
    s_last_apply_error_code = MODBUS_APPLY_OK;
    if (s_apply_ok_count < UINT32_MAX) {
      s_apply_ok_count++;
    }
  } else if (status == MODBUS_APPLY_ERR_RANGE || status == MODBUS_APPLY_ERR_CRC ||
             status == MODBUS_APPLY_ERR_CMD) {
    s_last_apply_error_code = status;
    if (s_apply_fail_invalid_count < UINT32_MAX) {
      s_apply_fail_invalid_count++;
    }
  } else if (status == MODBUS_APPLY_ERR_BUSY) {
    s_last_apply_error_code = status;
    if (s_apply_fail_busy_count < UINT32_MAX) {
      s_apply_fail_busy_count++;
    }
  } else {
    s_last_apply_error_code = status;
    if (s_apply_fail_internal_count < UINT32_MAX) {
      s_apply_fail_internal_count++;
    }
  }
  taskEXIT_CRITICAL(&s_state_lock);
}

static modbus_apply_status_t apply_control_block(
    const light_control_cfg_t *candidate_cfg) {
  if (s_mbc_slave_handler == NULL) {
    return MODBUS_APPLY_ERR_INTERNAL;
  }
  if (candidate_cfg == NULL) {
    return MODBUS_APPLY_ERR_INTERNAL;
  }

  if (!validate_light_ctrl(candidate_cfg)) {
    return MODBUS_APPLY_ERR_RANGE;
  }

  uint32_t next_version = 0;
  taskENTER_CRITICAL(&s_state_lock);
  next_version = s_active_ctrl_version + 1U;
  taskEXIT_CRITICAL(&s_state_lock);

  if (!persist_light_state(candidate_cfg, next_version)) {
    return MODBUS_APPLY_ERR_INTERNAL;
  }

  taskENTER_CRITICAL(&s_state_lock);
  s_active_light_cfg = *candidate_cfg;
  s_active_ctrl_version = next_version;
  taskEXIT_CRITICAL(&s_state_lock);

  uint16_t ver_hi = 0;
  uint16_t ver_lo = 0;
  u32_to_regs(next_version, &ver_hi, &ver_lo);
  ESP_ERROR_CHECK(mbc_slave_lock(s_mbc_slave_handler));
  s_holding_regs[MODBUS_HREG_ACTIVE_CTRL_VERSION_HI] = ver_hi;
  s_holding_regs[MODBUS_HREG_ACTIVE_CTRL_VERSION_LO] = ver_lo;
  ESP_ERROR_CHECK(mbc_slave_unlock(s_mbc_slave_handler));

  ESP_LOGI(TAG, "Light config applied: active_ctrl_version=%lu",
           (unsigned long)next_version);
  log_light_ctrl_cfg("Light cfg applied", candidate_cfg);

  return MODBUS_APPLY_OK;
}

static void modbus_runtime_task(void *arg) {
  (void)arg;
  while (1) {
    if (s_mbc_slave_handler == NULL) {
      vTaskDelay(pdMS_TO_TICKS(MODBUS_CYCLE_TASK_PERIOD_MS));
      continue;
    }

    // Fallback polling path: detect RTC token writes even if FC wrappers were not hit.
    queue_rtc_sync_request_from_regs();
    process_pending_rtc_sync();
    queue_weather_sync_request_from_regs();
    process_pending_weather_sync();
    queue_apply_request_from_regs();

    light_control_cfg_t pending_light_cfg = {0};
    bool has_pending_apply = false;
    taskENTER_CRITICAL(&s_state_lock);
    has_pending_apply = s_apply_pending;
    if (has_pending_apply) {
      pending_light_cfg = s_apply_pending_light_cfg;
      s_apply_pending = false;
    }
    taskEXIT_CRITICAL(&s_state_lock);

    if (has_pending_apply) {
      modbus_apply_status_t status = apply_control_block(&pending_light_cfg);
      finalize_apply_result(status);
    }

    uint32_t now = now_ms();
    taskENTER_CRITICAL(&s_state_lock);
    bool entered_autonomous = false;
    if (s_weather_valid) {
      uint32_t weather_elapsed = now - s_weather_last_rx_ms;
      s_weather_stale =
          (weather_elapsed > MODBUS_WEATHER_STALE_TIMEOUT_MS) ||
          (s_weather_active_snapshot.source_age_s > MODBUS_WEATHER_MAX_SOURCE_AGE_S);
    } else {
      s_weather_stale = true;
    }
    if (s_mode_state == MODBUS_MODE_REMOTE) {
      uint32_t elapsed = now - s_last_master_seen_ms;
      if (elapsed > MODBUS_HEARTBEAT_TIMEOUT_MS) {
        s_mode_state = MODBUS_MODE_AUTONOMOUS;
        s_mode_reason = MODBUS_REASON_MASTER_TIMEOUT;
        s_good_cycle_streak = 0;
        entered_autonomous = true;
      }
    }
    taskEXIT_CRITICAL(&s_state_lock);

	    if (entered_autonomous) {
	      ESP_LOGW(TAG,
	               "Mode changed: REMOTE -> AUTONOMOUS (master timeout > %u ms)",
	               (unsigned)MODBUS_HEARTBEAT_TIMEOUT_MS);
	      modbus_get_light_relay_state(NULL, NULL);
	    }

    ESP_ERROR_CHECK(mbc_slave_lock(s_mbc_slave_handler));
    update_diag_regs_locked();
    ESP_ERROR_CHECK(mbc_slave_unlock(s_mbc_slave_handler));

    vTaskDelay(pdMS_TO_TICKS(MODBUS_CYCLE_TASK_PERIOD_MS));
  }
}

void modbus_init(void) {
  load_persisted_settings();

  mb_communication_info_t comm_info = {0};
  comm_info.ser_opts.port = MB_PORT_NUM;
  comm_info.ser_opts.mode = MB_RTU;
  comm_info.ser_opts.uid = s_slave_id;
  comm_info.ser_opts.baudrate = MB_DEV_SPEED;
  comm_info.ser_opts.parity = MB_UART_PARITY;
  comm_info.ser_opts.stop_bits = MB_STOP_BITS;
  comm_info.ser_opts.data_bits = MB_DATA_BITS;

  ESP_ERROR_CHECK(mbc_slave_create_serial(&comm_info, &s_mbc_slave_handler));

  ESP_ERROR_CHECK(uart_set_pin(MB_PORT_NUM, MB_UART_TXD, MB_UART_RXD, MB_UART_RTS,
                               UART_PIN_NO_CHANGE));
  ESP_ERROR_CHECK(uart_set_mode(MB_PORT_NUM, UART_MODE_RS485_HALF_DUPLEX));

  memset(s_holding_regs, 0, sizeof(s_holding_regs));
  remote_cfg_to_regs(&s_remote_active_cfg, s_holding_regs);
  light_ctrl_to_regs(&s_staging_light_cfg, s_holding_regs);
  uint16_t ver_hi = 0;
  uint16_t ver_lo = 0;
  u32_to_regs(s_active_ctrl_version, &ver_hi, &ver_lo);
  s_holding_regs[MODBUS_HREG_ACTIVE_CTRL_VERSION_HI] = ver_hi;
  s_holding_regs[MODBUS_HREG_ACTIVE_CTRL_VERSION_LO] = ver_lo;
  s_holding_regs[MODBUS_HREG_LIGHT_CURRENT_DLI_JCM2] = 0;
  s_holding_regs[MODBUS_HREG_LIGHT_OUTPUT_PERCENT] = 0;
  s_holding_regs[MODBUS_HREG_LIGHT_STATUS_BITS] = 0;
  s_holding_regs[MODBUS_HREG_RTC_SET_HOUR] = 0;
  s_holding_regs[MODBUS_HREG_RTC_SET_MINUTE] = 0;
  s_holding_regs[MODBUS_HREG_RTC_SET_TOKEN] = 0;
  s_holding_regs[MODBUS_HREG_RTC_SET_APPLIED_TOKEN] = 0;
  s_holding_regs[MODBUS_HREG_RTC_SET_RESULT] = MODBUS_RTC_SET_RESULT_NONE;
  s_holding_regs[MODBUS_HREG_WEATHER_SET_TOKEN] = 0;
  s_holding_regs[MODBUS_HREG_WEATHER_SET_APPLIED_TOKEN] = 0;
  s_holding_regs[MODBUS_HREG_WEATHER_SET_RESULT] = MODBUS_WEATHER_SET_RESULT_NONE;
  s_holding_regs[MODBUS_HREG_WINDOWS_CTRL_MODE] =
      MODBUS_WINDOWS_DEFAULT_CTRL_MODE;
  s_holding_regs[MODBUS_HREG_WINDOWS_FORCE_SAFE_CMD] =
      MODBUS_WINDOWS_DEFAULT_FORCE_SAFE_CMD;
  s_holding_regs[MODBUS_HREG_WINDOWS_TEMP_SETPOINT] =
      MODBUS_WINDOWS_DEFAULT_TEMP_SETPOINT;
  s_holding_regs[MODBUS_HREG_WINDOWS_SAFE_MIN_PERCENT] =
      MODBUS_WINDOWS_DEFAULT_SAFE_MIN_PERCENT;
  s_holding_regs[MODBUS_HREG_WINDOWS_WIND_LIMIT] =
      MODBUS_WINDOWS_DEFAULT_WIND_LIMIT;
  s_holding_regs[MODBUS_HREG_WINDOWS_WIND_STORM] =
      MODBUS_WINDOWS_DEFAULT_WIND_STORM;
  s_holding_regs[MODBUS_HREG_WINDOWS_WIND_RECOVER] =
      MODBUS_WINDOWS_DEFAULT_WIND_RECOVER;
  s_holding_regs[MODBUS_HREG_WINDOW_A_AZIMUTH_DEG] =
      MODBUS_WINDOWS_DEFAULT_AZIMUTH_DEG;
  s_holding_regs[MODBUS_HREG_WINDOWS_WIND_SECTOR_HALF_WIDTH_DEG] =
      MODBUS_WINDOWS_DEFAULT_WIND_SECTOR_HALF_WIDTH_DEG;
  s_holding_regs[MODBUS_HREG_WINDOWS_TEMP_STEP_C] =
      MODBUS_WINDOWS_DEFAULT_TEMP_STEP_C;
  s_holding_regs[MODBUS_HREG_WINDOWS_TEMP_STEP_HYST_C] =
      MODBUS_WINDOWS_DEFAULT_TEMP_STEP_HYST_C;
  s_holding_regs[MODBUS_HREG_RLL400_TARGET_HYST_PERCENT] =
      MODBUS_WINDOWS_DEFAULT_TARGET_HYST_PERCENT;
  s_holding_regs[MODBUS_HREG_RLL400_MOTION_DELTA_PERCENT] =
      MODBUS_WINDOWS_DEFAULT_MOTION_DELTA_PERCENT;
  s_holding_regs[MODBUS_HREG_RLL400_NO_MOTION_TIMEOUT_MS] =
      MODBUS_WINDOWS_DEFAULT_NO_MOTION_TIMEOUT_MS;
  s_holding_regs[MODBUS_HREG_WINDOW_A_FAULT_RESET_TOKEN] = 0U;
  s_holding_regs[MODBUS_HREG_WINDOW_B_FAULT_RESET_TOKEN] = 0U;
  s_holding_regs[MODBUS_HREG_WINDOWS_STATUS_BITS] = 0U;
  s_holding_regs[MODBUS_HREG_WINDOW_A_STATUS_BITS] = 0U;
  s_holding_regs[MODBUS_HREG_WINDOW_B_STATUS_BITS] = 0U;
  s_holding_regs[MODBUS_HREG_WINDOW_A_FAULT_CODE] = 0U;
  s_holding_regs[MODBUS_HREG_WINDOW_B_FAULT_CODE] = 0U;
  s_holding_regs[MODBUS_HREG_AIR_TEMP_SENSOR_STATUS] = 0U;
  s_holding_regs[MODBUS_HREG_WINDOW_A_LOCAL_MANUAL_ACTIVE] = 0U;
  s_holding_regs[MODBUS_HREG_WINDOW_B_LOCAL_MANUAL_ACTIVE] = 0U;
  s_holding_regs[MODBUS_HREG_WINDOWS_AUTO_ALGO_MODE] =
      MODBUS_WINDOWS_DEFAULT_AUTO_ALGO_MODE;
  s_holding_regs[MODBUS_HREG_WINDOWS_TEMP_STEP_TARGET_PERCENT] =
      MODBUS_WINDOWS_DEFAULT_TEMP_STEP_TARGET_PERCENT;
  s_holding_regs[MODBUS_HREG_WINDOWS_TEMP_STEP_MAX_INDEX] =
      MODBUS_WINDOWS_DEFAULT_TEMP_STEP_MAX_INDEX;
  s_holding_regs[MODBUS_HREG_WINDOWS_HUM_SETPOINT] =
      MODBUS_WINDOWS_DEFAULT_HUM_SETPOINT;
  s_holding_regs[MODBUS_HREG_WINDOWS_HUM_STEP] =
      MODBUS_WINDOWS_DEFAULT_HUM_STEP;
  s_holding_regs[MODBUS_HREG_WINDOWS_HUM_STEP_HYST] =
      MODBUS_WINDOWS_DEFAULT_HUM_STEP_HYST;
  s_holding_regs[MODBUS_HREG_WINDOWS_HUM_STEP_TARGET_PERCENT] =
      MODBUS_WINDOWS_DEFAULT_HUM_STEP_TARGET_PERCENT;
  s_holding_regs[MODBUS_HREG_WINDOWS_HUM_STEP_MAX_INDEX] =
      MODBUS_WINDOWS_DEFAULT_HUM_STEP_MAX_INDEX;
  s_holding_regs[MODBUS_HREG_WINDOWS_COLD_CLOSE_DELTA] =
      MODBUS_WINDOWS_DEFAULT_COLD_CLOSE_DELTA;
  s_holding_regs[MODBUS_HREG_WINDOWS_COLD_CLOSE_HYST] =
      MODBUS_WINDOWS_DEFAULT_COLD_CLOSE_HYST;
  s_holding_regs[MODBUS_HREG_WINDOWS_WINDWARD_MIN_PERCENT] =
      MODBUS_WINDOWS_DEFAULT_WINDWARD_MIN_PERCENT;
  s_holding_regs[MODBUS_HREG_WINDOWS_WINDWARD_MAX_PERCENT] =
      MODBUS_WINDOWS_DEFAULT_WINDWARD_MAX_PERCENT;
  s_holding_regs[MODBUS_HREG_WINDOWS_WINDWARD_SPEED_THRESHOLD] =
      MODBUS_WINDOWS_DEFAULT_WIND_LIMIT;
  s_holding_regs[MODBUS_HREG_WINDOWS_WINDWARD_REDUCTION_PERCENT_PER_MS] =
      MODBUS_WINDOWS_DEFAULT_WINDWARD_REDUCTION_PERCENT_PER_MS;
  s_holding_regs[MODBUS_HREG_WINDOWS_LEEWARD_MIN_PERCENT] =
      MODBUS_WINDOWS_DEFAULT_LEEWARD_MIN_PERCENT;
  s_holding_regs[MODBUS_HREG_WINDOWS_LEEWARD_MAX_PERCENT] =
      MODBUS_WINDOWS_DEFAULT_LEEWARD_MAX_PERCENT;
  s_holding_regs[MODBUS_HREG_WINDOWS_LEEWARD_SPEED_THRESHOLD] =
      MODBUS_WINDOWS_DEFAULT_WIND_LIMIT;
  s_holding_regs[MODBUS_HREG_WINDOWS_LEEWARD_REDUCTION_PERCENT_PER_MS] =
      MODBUS_WINDOWS_DEFAULT_LEEWARD_REDUCTION_PERCENT_PER_MS;
  s_holding_regs[MODBUS_HREG_WINDOWS_WINDWARD_LAG_PERCENT] =
      MODBUS_WINDOWS_DEFAULT_WINDWARD_LAG_PERCENT;
  s_holding_regs[MODBUS_HREG_WINDOWS_RAIN_MODE] =
      MODBUS_WINDOWS_DEFAULT_RAIN_MODE;
  s_holding_regs[MODBUS_HREG_WINDOWS_RAIN_WINDWARD_PERCENT] =
      MODBUS_WINDOWS_DEFAULT_RAIN_WINDWARD_PERCENT;
  s_holding_regs[MODBUS_HREG_WINDOWS_WEATHER_STALE_POLICY] =
      MODBUS_WINDOWS_DEFAULT_WEATHER_STALE_POLICY;
  s_holding_regs[MODBUS_HREG_WINDOWS_WEATHER_STALE_TIMEOUT_MS] =
      MODBUS_WINDOWS_DEFAULT_WEATHER_STALE_TIMEOUT_MS;
  s_holding_regs[MODBUS_HREG_WINDOWS_WEATHER_SOURCE_AGE_S] =
      MODBUS_WINDOWS_DEFAULT_WEATHER_SOURCE_AGE_S;
  s_holding_regs[MODBUS_HREG_WINDOWS_BASE_TARGET_A] = 0U;
  s_holding_regs[MODBUS_HREG_WINDOWS_BASE_TARGET_B] = 0U;
  s_holding_regs[MODBUS_HREG_WINDOWS_EFFECTIVE_TARGET_A] = 0U;
  s_holding_regs[MODBUS_HREG_WINDOWS_EFFECTIVE_TARGET_B] = 0U;
  s_holding_regs[MODBUS_HREG_WINDOWS_ACTIVE_PROTECTION_BITS] = 0U;
  s_holding_regs[MODBUS_HREG_WINDOWS_WINDWARD_SIDE] =
      MODBUS_WINDOWS_WINDWARD_SIDE_NONE;
  s_holding_regs[MODBUS_HREG_HEATING_CTRL_MODE] =
      MODBUS_HEATING_DEFAULT_CTRL_MODE;
  s_holding_regs[MODBUS_HREG_HEATING_AIR_SETPOINT] =
      MODBUS_HEATING_DEFAULT_AIR_SETPOINT;
  s_holding_regs[MODBUS_HREG_HEATING_AIR_HYST] =
      MODBUS_HEATING_DEFAULT_AIR_HYST;
  s_holding_regs[MODBUS_HREG_HEATING_STAGE_DELTA_1] =
      MODBUS_HEATING_DEFAULT_STAGE_DELTA_1;
  s_holding_regs[MODBUS_HREG_HEATING_STAGE_DELTA_2] =
      MODBUS_HEATING_DEFAULT_STAGE_DELTA_2;
  s_holding_regs[MODBUS_HREG_HEATING_STAGE_DELTA_3] =
      MODBUS_HEATING_DEFAULT_STAGE_DELTA_3;
  s_holding_regs[MODBUS_HREG_HEATING_STAGE_DELTA_4] =
      MODBUS_HEATING_DEFAULT_STAGE_DELTA_4;
  s_holding_regs[MODBUS_HREG_HEATING_MIN_ON_S] =
      MODBUS_HEATING_DEFAULT_MIN_ON_S;
  s_holding_regs[MODBUS_HREG_HEATING_MIN_OFF_S] =
      MODBUS_HEATING_DEFAULT_MIN_OFF_S;
  s_holding_regs[MODBUS_HREG_HEATING_MANUAL_PUMP_MASK] = 0U;
  s_holding_regs[MODBUS_HREG_HEATING_MANUAL_VALVE_OPEN_MASK] = 0U;
  s_holding_regs[MODBUS_HREG_HEATING_MANUAL_VALVE_CLOSE_MASK] = 0U;
  s_holding_regs[MODBUS_HREG_HEATING_STATUS_BITS] = 0U;
  s_holding_regs[MODBUS_HREG_HEATING_ACTIVE_STAGE] = 0U;
  s_holding_regs[MODBUS_HREG_HEATING_PUMP_MASK] = 0U;
  s_holding_regs[MODBUS_HREG_HEATING_VALVE_OPEN_MASK] = 0U;
  s_holding_regs[MODBUS_HREG_HEATING_VALVE_CLOSE_MASK] = 0U;
  s_holding_regs[MODBUS_HREG_HEATING_SENSOR_STATUS_BITS] = 0U;
  s_holding_regs[MODBUS_HREG_CURTAIN_CTRL_MODE] =
      MODBUS_CURTAIN_DEFAULT_CTRL_MODE;
  s_holding_regs[MODBUS_HREG_CURTAIN_MANUAL_TARGET] =
      MODBUS_CURTAIN_DEFAULT_MANUAL_TARGET;
  s_holding_regs[MODBUS_HREG_CURTAIN_SCHEDULE_START_HHMM] =
      MODBUS_CURTAIN_DEFAULT_SCHEDULE_START_HHMM;
  s_holding_regs[MODBUS_HREG_CURTAIN_SCHEDULE_END_HHMM] =
      MODBUS_CURTAIN_DEFAULT_SCHEDULE_END_HHMM;
  s_holding_regs[MODBUS_HREG_CURTAIN_OUTSIDE_TARGET] =
      MODBUS_CURTAIN_DEFAULT_OUTSIDE_TARGET;
  s_holding_regs[MODBUS_HREG_CURTAIN_MIN_POSITION] =
      MODBUS_CURTAIN_DEFAULT_MIN_POSITION;
  s_holding_regs[MODBUS_HREG_CURTAIN_MAX_POSITION] =
      MODBUS_CURTAIN_DEFAULT_MAX_POSITION;
  s_holding_regs[MODBUS_HREG_CURTAIN_POSITION_HYST] =
      MODBUS_CURTAIN_DEFAULT_POSITION_HYST;
  s_holding_regs[MODBUS_HREG_CURTAIN_RADIATION_THRESHOLD] =
      MODBUS_CURTAIN_DEFAULT_RADIATION_THRESHOLD;
  s_holding_regs[MODBUS_HREG_CURTAIN_RADIATION_STEP_WM2] =
      MODBUS_CURTAIN_DEFAULT_RADIATION_STEP_WM2;
  s_holding_regs[MODBUS_HREG_CURTAIN_RADIATION_STEP_PERCENT] =
      MODBUS_CURTAIN_DEFAULT_RADIATION_STEP_PERCENT;
  s_holding_regs[MODBUS_HREG_CURTAIN_RADIATION_HYST] =
      MODBUS_CURTAIN_DEFAULT_RADIATION_HYST;
  s_holding_regs[MODBUS_HREG_CURTAIN_COLD_DELTA] =
      MODBUS_CURTAIN_DEFAULT_COLD_DELTA;
  s_holding_regs[MODBUS_HREG_CURTAIN_COLD_HYST] =
      MODBUS_CURTAIN_DEFAULT_COLD_HYST;
  s_holding_regs[MODBUS_HREG_CURTAIN_COLD_TARGET] =
      MODBUS_CURTAIN_DEFAULT_COLD_TARGET;
  s_holding_regs[MODBUS_HREG_CURTAIN_HEAT_DELTA] =
      MODBUS_CURTAIN_DEFAULT_HEAT_DELTA;
  s_holding_regs[MODBUS_HREG_CURTAIN_HEAT_HYST] =
      MODBUS_CURTAIN_DEFAULT_HEAT_HYST;
  s_holding_regs[MODBUS_HREG_CURTAIN_HEAT_TARGET] =
      MODBUS_CURTAIN_DEFAULT_HEAT_TARGET;
  s_holding_regs[MODBUS_HREG_CURTAIN_HUM_LOW_THRESHOLD] =
      MODBUS_CURTAIN_DEFAULT_HUM_LOW_THRESHOLD;
  s_holding_regs[MODBUS_HREG_CURTAIN_HUM_LOW_HYST] =
      MODBUS_CURTAIN_DEFAULT_HUM_LOW_HYST;
  s_holding_regs[MODBUS_HREG_CURTAIN_HUM_LOW_TARGET] =
      MODBUS_CURTAIN_DEFAULT_HUM_LOW_TARGET;
  s_holding_regs[MODBUS_HREG_CURTAIN_HUM_HIGH_THRESHOLD] =
      MODBUS_CURTAIN_DEFAULT_HUM_HIGH_THRESHOLD;
  s_holding_regs[MODBUS_HREG_CURTAIN_HUM_HIGH_HYST] =
      MODBUS_CURTAIN_DEFAULT_HUM_HIGH_HYST;
  s_holding_regs[MODBUS_HREG_CURTAIN_HUM_HIGH_TARGET] =
      MODBUS_CURTAIN_DEFAULT_HUM_HIGH_TARGET;
  s_holding_regs[MODBUS_HREG_CURTAIN_TARGET] = 0U;
  s_holding_regs[MODBUS_HREG_CURTAIN_BASE_TARGET] = 0U;
  s_holding_regs[MODBUS_HREG_CURTAIN_CURRENT_MA] = 0U;
  s_holding_regs[MODBUS_HREG_CURTAIN_STATUS_BITS] = 0U;
  s_holding_regs[MODBUS_HREG_CURTAIN_REASON_BITS] = 0U;
  s_holding_regs[MODBUS_HREG_CURTAIN_POSITION_STATUS_BITS] = 0U;
  s_holding_regs[MODBUS_HREG_CURTAIN_FAULT_CODE] = 0U;
  s_holding_regs[MODBUS_HREG_CURTAIN_FAULT_RESET_TOKEN] = 0U;
  s_holding_regs[MODBUS_HREG_AIR_TEMP_TARGET] =
      MODBUS_WINDOWS_DEFAULT_TEMP_SETPOINT;
  s_holding_regs[MODBUS_HREG_AIR_HUM_TARGET] =
      MODBUS_WINDOWS_DEFAULT_HUM_SETPOINT;
  s_holding_regs[MODBUS_HREG_CO2_MEASURED_PPM] = 420U;
  s_holding_regs[MODBUS_HREG_CO2_SENSOR_VALID] = 0U;
  s_holding_regs[MODBUS_HREG_CO2_CTRL_MODE] = MODBUS_CO2_DEFAULT_CTRL_MODE;
  s_holding_regs[MODBUS_HREG_CO2_MANUAL_OUTPUTS] = 0U;
  s_holding_regs[MODBUS_HREG_CO2_SCHEDULE_START_HHMM] =
      MODBUS_CO2_DEFAULT_SCHEDULE_START_HHMM;
  s_holding_regs[MODBUS_HREG_CO2_SCHEDULE_END_HHMM] =
      MODBUS_CO2_DEFAULT_SCHEDULE_END_HHMM;
  s_holding_regs[MODBUS_HREG_CO2_LOW_LIGHT_THRESHOLD_WM2] =
      MODBUS_CO2_DEFAULT_LOW_LIGHT_THRESHOLD_WM2;
  s_holding_regs[MODBUS_HREG_CO2_MID_LIGHT_THRESHOLD_WM2] =
      MODBUS_CO2_DEFAULT_MID_LIGHT_THRESHOLD_WM2;
  s_holding_regs[MODBUS_HREG_CO2_HIGH_LIGHT_THRESHOLD_WM2] =
      MODBUS_CO2_DEFAULT_HIGH_LIGHT_THRESHOLD_WM2;
  s_holding_regs[MODBUS_HREG_CO2_LOW_LIGHT_TARGET_PPM] =
      MODBUS_CO2_DEFAULT_LOW_LIGHT_TARGET_PPM;
  s_holding_regs[MODBUS_HREG_CO2_MID_LIGHT_TARGET_PPM] =
      MODBUS_CO2_DEFAULT_MID_LIGHT_TARGET_PPM;
  s_holding_regs[MODBUS_HREG_CO2_HIGH_LIGHT_TARGET_PPM] =
      MODBUS_CO2_DEFAULT_HIGH_LIGHT_TARGET_PPM;
  s_holding_regs[MODBUS_HREG_CO2_VENT_LIMIT_LOW_PERCENT] =
      MODBUS_CO2_DEFAULT_VENT_LIMIT_LOW_PERCENT;
  s_holding_regs[MODBUS_HREG_CO2_VENT_LIMIT_HIGH_PERCENT] =
      MODBUS_CO2_DEFAULT_VENT_LIMIT_HIGH_PERCENT;
  s_holding_regs[MODBUS_HREG_CO2_VENT_CUTOFF_PERCENT] =
      MODBUS_CO2_DEFAULT_VENT_CUTOFF_PERCENT;
  s_holding_regs[MODBUS_HREG_CO2_DOSING_HYST_PPM] =
      MODBUS_CO2_DEFAULT_DOSING_HYST_PPM;
  s_holding_regs[MODBUS_HREG_CO2_MAX_SAFE_PPM] =
      MODBUS_CO2_DEFAULT_MAX_SAFE_PPM;
  s_holding_regs[MODBUS_HREG_CO2_MAX_DOSING_TIME_S] =
      MODBUS_CO2_DEFAULT_MAX_DOSING_TIME_S;
  s_holding_regs[MODBUS_HREG_CO2_MIN_PAUSE_TIME_S] =
      MODBUS_CO2_DEFAULT_MIN_PAUSE_TIME_S;
  s_holding_regs[MODBUS_HREG_CO2_NO_RISE_CHECK_TIME_S] =
      MODBUS_CO2_DEFAULT_NO_RISE_CHECK_TIME_S;
  s_holding_regs[MODBUS_HREG_CO2_NO_RISE_MIN_DELTA_PPM] =
      MODBUS_CO2_DEFAULT_NO_RISE_MIN_DELTA_PPM;
  s_holding_regs[MODBUS_HREG_CO2_TEMP_HIGH_DELTA] =
      MODBUS_CO2_DEFAULT_TEMP_HIGH_DELTA;
  s_holding_regs[MODBUS_HREG_CO2_TEMP_CRITICAL_DELTA] =
      MODBUS_CO2_DEFAULT_TEMP_CRITICAL_DELTA;
  s_holding_regs[MODBUS_HREG_CO2_HUM_HIGH_DELTA] =
      MODBUS_CO2_DEFAULT_HUM_HIGH_DELTA;
  s_holding_regs[MODBUS_HREG_CO2_EXTERNAL_ALARM] = 0U;
  s_holding_regs[MODBUS_HREG_CO2_TARGET_PPM] = 0U;
  s_holding_regs[MODBUS_HREG_CO2_EFFECTIVE_TARGET_PPM] = 0U;
  s_holding_regs[MODBUS_HREG_CO2_STATUS_BITS] = 0U;
  s_holding_regs[MODBUS_HREG_CO2_REASON_BITS] = 0U;
  s_holding_regs[MODBUS_HREG_CO2_PROTECTION_BITS] = 0U;
  s_holding_regs[MODBUS_HREG_CO2_FAULT_CODE] = 0U;
  s_holding_regs[MODBUS_HREG_CO2_DOSING_ELAPSED_S] = 0U;
  s_holding_regs[MODBUS_HREG_CO2_FAULT_RESET_TOKEN] = 0U;

  s_last_apply_status = MODBUS_APPLY_OK;
  s_apply_pending = false;
  memset(&s_apply_pending_light_cfg, 0, sizeof(s_apply_pending_light_cfg));
  s_apply_ok_count = 0;
  s_apply_fail_invalid_count = 0;
  s_apply_fail_busy_count = 0;
  s_apply_fail_internal_count = 0;
  s_last_apply_error_code = MODBUS_APPLY_OK;
  s_last_apply_ts_ms = 0;
  s_last_logged_schedule_mask = UINT8_MAX;
  s_last_logged_schedule_mode = UINT8_MAX;
  s_last_logged_light_output_percent = UINT16_MAX;
  s_last_logged_light_status_bits = UINT16_MAX;
  s_light_stable_on_mask = 0U;
  s_light_pending_valid_mask = 0U;
  s_light_pending_target_on_mask = 0U;
  memset((void *)s_light_pending_since_ms, 0, sizeof(s_light_pending_since_ms));
  memset((void *)s_light_on_delay_since_ms, 0, sizeof(s_light_on_delay_since_ms));
  s_light_cfg_dirty = false;
  s_light_cfg_last_change_ms = 0U;
  s_last_master_seen_ms = now_ms();
  s_light_set_ms = s_last_master_seen_ms;
  s_good_cycle_streak = 0;
  s_mode_state = MODBUS_MODE_REMOTE;
  s_mode_reason = MODBUS_REASON_NONE;
  s_rtc_last_token = 0;
  s_rtc_pending_token = 0;
  s_rtc_pending_hour = 0;
  s_rtc_pending_minute = 0;
  s_rtc_sync_pending = false;
  s_rtc_sync_applied_count = 0;
  s_rtc_sync_noop_count = 0;
  s_rtc_sync_fail_count = 0;
  s_rtc_sync_reject_count = 0;
  s_rtc_sync_last_result = MODBUS_RTC_SET_RESULT_NONE;
  s_weather_last_token = 0;
  s_weather_pending_token = 0;
  s_weather_sync_pending = false;
  memset(&s_weather_pending_snapshot, 0, sizeof(s_weather_pending_snapshot));
  memset(&s_weather_active_snapshot, 0, sizeof(s_weather_active_snapshot));
  s_weather_last_rx_ms = 0;
  s_weather_valid = false;
  s_weather_stale = true;

  update_diag_regs_locked();

  s_holding_area.type = MB_PARAM_HOLDING;
  s_holding_area.start_offset = 0;
  s_holding_area.address = (void *)s_holding_regs;
  s_holding_area.size = sizeof(s_holding_regs);
  s_holding_area.access = MB_ACCESS_RW;

  ESP_ERROR_CHECK(mbc_slave_set_descriptor(s_mbc_slave_handler, s_holding_area));
  ESP_ERROR_CHECK(mbc_slave_start(s_mbc_slave_handler));

  install_handler_wrappers();

  xTaskCreate(modbus_runtime_task, "mb_runtime", 4096, NULL, 6, NULL);

  ESP_LOGI(TAG, "Modbus slave initialized (id=%u, UART2, 19200 8N1)",
           (unsigned)s_slave_id);
  ESP_LOGI(TAG, "Light relay turn-on delay=%u s (zone_id=slave_id=%u)",
           (unsigned)((uint32_t)s_slave_id * MODBUS_LIGHT_ZONE_DELAY_STEP_SEC),
           (unsigned)s_slave_id);
}

void modbus_bind_rtc_callbacks(modbus_rtc_get_time_cb_t get_cb,
                               modbus_rtc_set_time_cb_t set_cb, void *ctx) {
  taskENTER_CRITICAL(&s_state_lock);
  s_rtc_get_time_cb = get_cb;
  s_rtc_set_time_cb = set_cb;
  s_rtc_cb_ctx = ctx;
  taskEXIT_CRITICAL(&s_state_lock);
}

void modbus_set_telemetry(float air_temp, float air_hum, float water_rail,
                          float water_grow, float water_undertray,
                          float water_upper_heat, float windows_pos_a,
                          float windows_pos_b, float curtain_pos) {
  if (s_mbc_slave_handler == NULL) {
    return;
  }

  ESP_ERROR_CHECK(mbc_slave_lock(s_mbc_slave_handler));
  s_holding_regs[MODBUS_HREG_AIR_TEMP] = float_to_i16_tenths_raw(air_temp);
  s_holding_regs[MODBUS_HREG_AIR_HUM] = float_to_u16_tenths(air_hum, 0.0f, 100.0f);
  s_holding_regs[MODBUS_HREG_WATER_RAIL] = float_to_i16_tenths_raw(water_rail);
  s_holding_regs[MODBUS_HREG_WATER_GROW] = float_to_i16_tenths_raw(water_grow);
  s_holding_regs[MODBUS_HREG_WATER_UNDERTRAY] =
      float_to_i16_tenths_raw(water_undertray);
  s_holding_regs[MODBUS_HREG_WATER_UPPER_HEAT] =
      float_to_i16_tenths_raw(water_upper_heat);
  s_holding_regs[MODBUS_HREG_WINDOWS_POS_A] =
      float_to_u16_tenths(windows_pos_a, 0.0f, 100.0f);
  s_holding_regs[MODBUS_HREG_WINDOWS_POS_B] =
      float_to_u16_tenths(windows_pos_b, 0.0f, 100.0f);
  s_holding_regs[MODBUS_HREG_CURTAIN_POS] =
      float_to_u16_tenths(curtain_pos, 0.0f, 100.0f);
  ESP_ERROR_CHECK(mbc_slave_unlock(s_mbc_slave_handler));
}

void modbus_set_light_current_time(uint8_t hour, uint8_t minute, uint8_t second) {
  if (hour > 23U || minute > 59U || second > 59U) {
    return;
  }
  uint32_t set_ms = now_ms();
  taskENTER_CRITICAL(&s_state_lock);
  s_light_hour = hour;
  s_light_minute = minute;
  s_light_second = second;
  s_light_set_ms = set_ms;
  taskEXIT_CRITICAL(&s_state_lock);
}

uint8_t modbus_get_light_percent(void) {
  bool relay1 = false;
  bool relay2 = false;
  modbus_get_light_relay_state(&relay1, &relay2);
  if (relay1 && relay2) {
    return 100U;
  }
  if (relay1 || relay2) {
    return 50U;
  }
  return 0U;
}

void modbus_get_light_relay_state(bool *relay1_on, bool *relay2_on) {
  bool relay_out[MODBUS_LIGHT_RELAY_COUNT] = {false, false};
  light_control_cfg_t active_light_cfg = {0};
  light_control_cfg_t effective_light_cfg = {0};
  modbus_mode_state_t mode = MODBUS_MODE_REMOTE;
  uint8_t light_hour = 0;
  uint8_t light_minute = 0;
  uint8_t light_second = 0;
  uint32_t light_set_ms = 0;
  uint8_t prev_stable_on_mask = 0U;
  uint8_t prev_pending_valid_mask = 0U;
  uint8_t prev_pending_target_on_mask = 0U;
  uint32_t prev_pending_since_ms[MODBUS_LIGHT_RELAY_COUNT] = {0};
  uint32_t prev_on_delay_since_ms[MODBUS_LIGHT_RELAY_COUNT] = {0};
  uint16_t radiation_wm2 = 0;
  bool weather_stale = true;
  uint16_t status_bits = 0;
  uint8_t next_stable_on_mask = 0U;
  uint8_t next_pending_valid_mask = 0U;
  uint8_t next_pending_target_on_mask = 0U;
  uint32_t next_pending_since_ms[MODBUS_LIGHT_RELAY_COUNT] = {0};
  uint32_t next_on_delay_since_ms[MODBUS_LIGHT_RELAY_COUNT] = {0};
  uint32_t cycle_now_ms = now_ms();
  uint32_t sec_of_day = 0U;
  uint16_t dli_current = 0U;
  uint32_t zone_turn_on_delay_sec = 0U;

  taskENTER_CRITICAL(&s_state_lock);
  mode = s_mode_state;
  active_light_cfg = s_active_light_cfg;
  effective_light_cfg = active_light_cfg;
  if (mode == MODBUS_MODE_AUTONOMOUS) {
    effective_light_cfg = s_autonomous_cfg.light;
    for (int i = 0; i < MODBUS_LIGHT_RELAY_COUNT; ++i) {
      effective_light_cfg.relay[i].threshold_wm2 = 0U;
      effective_light_cfg.relay[i].dli_off_limit_jcm2 = 0U;
    }
  }
  light_hour = s_light_hour;
  light_minute = s_light_minute;
  light_second = s_light_second;
  light_set_ms = s_light_set_ms;
  prev_stable_on_mask = s_light_stable_on_mask;
  prev_pending_valid_mask = s_light_pending_valid_mask;
  prev_pending_target_on_mask = s_light_pending_target_on_mask;
  memcpy(prev_pending_since_ms, (const void *)s_light_pending_since_ms,
         sizeof(prev_pending_since_ms));
  memcpy(prev_on_delay_since_ms, (const void *)s_light_on_delay_since_ms,
         sizeof(prev_on_delay_since_ms));
  radiation_wm2 = s_weather_active_snapshot.solar_rad;
  weather_stale = (!s_weather_valid || s_weather_stale);
  zone_turn_on_delay_sec =
      (uint32_t)s_slave_id * MODBUS_LIGHT_ZONE_DELAY_STEP_SEC;
  taskEXIT_CRITICAL(&s_state_lock);

  if (light_hour <= 23U && light_minute <= 59U && light_second <= 59U) {
    uint32_t base_sec =
        (uint32_t)light_hour * 3600U + (uint32_t)light_minute * 60U + (uint32_t)light_second;
    uint32_t elapsed_sec = (cycle_now_ms - light_set_ms) / 1000U;
    sec_of_day = (base_sec + elapsed_sec) % 86400U;
    uint16_t minute_of_day = (uint16_t)(sec_of_day / 60U);
    uint8_t active_mask =
        get_active_light_schedule_mask(&effective_light_cfg, minute_of_day);

    log_active_light_schedules_if_changed(&effective_light_cfg, mode,
                                          minute_of_day, active_mask);

    if (weather_stale) {
      status_bits |= MODBUS_LIGHT_STATUS_WEATHER_STALE;
    }

    if (s_mbc_slave_handler != NULL) {
      ESP_ERROR_CHECK(mbc_slave_lock(s_mbc_slave_handler));
      dli_current = s_holding_regs[MODBUS_HREG_LIGHT_CURRENT_DLI_JCM2];
      ESP_ERROR_CHECK(mbc_slave_unlock(s_mbc_slave_handler));
    }

    for (int i = 0; i < MODBUS_LIGHT_RELAY_COUNT; ++i) {
      const light_relay_cfg_t *relay_cfg = &effective_light_cfg.relay[i];
      uint8_t relay_bit = (uint8_t)(1U << i);
      bool schedule_active = (active_mask & relay_bit) != 0U;
      bool prev_stable_on = (prev_stable_on_mask & relay_bit) != 0U;
      bool prev_pending_valid = (prev_pending_valid_mask & relay_bit) != 0U;
      bool prev_pending_target_on =
          (prev_pending_target_on_mask & relay_bit) != 0U;
      bool stable_on = prev_stable_on;
      bool dli_limit_active = false;
      bool hyst_hold = false;
      bool raw_target_valid = false;
      bool raw_target_on = prev_stable_on;

      if (schedule_active) {
        status_bits |= (i == 0) ? MODBUS_LIGHT_STATUS_R1_SCHEDULE_ACTIVE
                                : MODBUS_LIGHT_STATUS_R2_SCHEDULE_ACTIVE;
      }

      if (!schedule_active) {
        stable_on = false;
      } else if (relay_cfg->dli_off_limit_jcm2 > 0U &&
                 dli_current >= relay_cfg->dli_off_limit_jcm2) {
        dli_limit_active = true;
        stable_on = false;
      } else if (relay_cfg->threshold_wm2 == 0U ||
                 radiation_wm2 < relay_cfg->threshold_wm2) {
        raw_target_valid = true;
        raw_target_on = true;
      } else {
        raw_target_valid = true;
        raw_target_on = false;
      }

      if (dli_limit_active) {
        next_pending_since_ms[i] = 0U;
      } else if (raw_target_valid) {
        if (effective_light_cfg.hyst_sec == 0U) {
          stable_on = raw_target_on;
          next_pending_since_ms[i] = 0U;
        } else if (raw_target_on == prev_stable_on) {
          stable_on = raw_target_on;
          next_pending_since_ms[i] = 0U;
        } else if (!prev_pending_valid || prev_pending_target_on != raw_target_on) {
          stable_on = prev_stable_on;
          next_pending_valid_mask |= relay_bit;
          if (raw_target_on) {
            next_pending_target_on_mask |= relay_bit;
          }
          next_pending_since_ms[i] = cycle_now_ms;
          hyst_hold = true;
        } else if ((cycle_now_ms - prev_pending_since_ms[i]) <
                   ((uint32_t)effective_light_cfg.hyst_sec * 1000U)) {
          stable_on = prev_stable_on;
          next_pending_valid_mask |= relay_bit;
          if (prev_pending_target_on) {
            next_pending_target_on_mask |= relay_bit;
          }
          next_pending_since_ms[i] = prev_pending_since_ms[i];
          hyst_hold = true;
        } else {
          stable_on = raw_target_on;
          next_pending_since_ms[i] = 0U;
        }
      } else {
        next_pending_since_ms[i] = 0U;
      }

      if (stable_on) {
        next_stable_on_mask |= relay_bit;
        if (!prev_stable_on || prev_on_delay_since_ms[i] == 0U) {
          next_on_delay_since_ms[i] = cycle_now_ms;
        } else {
          next_on_delay_since_ms[i] = prev_on_delay_since_ms[i];
        }

        if (zone_turn_on_delay_sec > 0U &&
            (cycle_now_ms - next_on_delay_since_ms[i]) <
                (zone_turn_on_delay_sec * 1000U)) {
          status_bits |= (i == 0) ? MODBUS_LIGHT_STATUS_R1_ON_DELAY_ACTIVE
                                  : MODBUS_LIGHT_STATUS_R2_ON_DELAY_ACTIVE;
        } else {
          relay_out[i] = true;
        }
      } else {
        next_on_delay_since_ms[i] = 0U;
      }

      if (dli_limit_active) {
        status_bits |= (i == 0) ? MODBUS_LIGHT_STATUS_R1_DLI_LIMIT
                                : MODBUS_LIGHT_STATUS_R2_DLI_LIMIT;
      }
      if (hyst_hold) {
        status_bits |= (i == 0) ? MODBUS_LIGHT_STATUS_R1_HYST_HOLD
                                : MODBUS_LIGHT_STATUS_R2_HYST_HOLD;
      }
      if (relay_out[i]) {
        status_bits |= (i == 0) ? MODBUS_LIGHT_STATUS_R1_OUTPUT_ON
                                : MODBUS_LIGHT_STATUS_R2_OUTPUT_ON;
      }
    }
  }

  taskENTER_CRITICAL(&s_state_lock);
  s_light_stable_on_mask = next_stable_on_mask;
  s_light_pending_valid_mask = next_pending_valid_mask;
  s_light_pending_target_on_mask = next_pending_target_on_mask;
  memcpy((void *)s_light_pending_since_ms, next_pending_since_ms,
         sizeof(s_light_pending_since_ms));
  memcpy((void *)s_light_on_delay_since_ms, next_on_delay_since_ms,
         sizeof(s_light_on_delay_since_ms));
  taskEXIT_CRITICAL(&s_state_lock);

  if (s_mbc_slave_handler != NULL) {
    uint16_t output_percent =
        (uint16_t)(((relay_out[0] ? 1U : 0U) + (relay_out[1] ? 1U : 0U)) * 50U);
    ESP_ERROR_CHECK(mbc_slave_lock(s_mbc_slave_handler));
    s_holding_regs[MODBUS_HREG_LIGHT_OUTPUT_PERCENT] = output_percent;
    s_holding_regs[MODBUS_HREG_LIGHT_STATUS_BITS] = status_bits;
    ESP_ERROR_CHECK(mbc_slave_unlock(s_mbc_slave_handler));
    log_light_runtime_state_if_changed(sec_of_day, radiation_wm2, dli_current,
                                       output_percent, status_bits,
                                       relay_out[0], relay_out[1]);
  }

  if (relay1_on != NULL) {
    *relay1_on = relay_out[0];
  }
  if (relay2_on != NULL) {
    *relay2_on = relay_out[1];
  }
}

void modbus_set_solar_radiation(float radiation) {
  if (s_mbc_slave_handler == NULL) {
    return;
  }
  if (radiation < 0.0f) {
    radiation = 0.0f;
  } else if (radiation > 65535.0f) {
    radiation = 65535.0f;
  }
  uint16_t radiation_wm2 = (uint16_t)(radiation + 0.5f);
  uint32_t now = now_ms();
  taskENTER_CRITICAL(&s_state_lock);
  s_weather_active_snapshot.solar_rad = radiation_wm2;
  s_weather_last_rx_ms = now;
  s_weather_valid = true;
  s_weather_stale = false;
  taskEXIT_CRITICAL(&s_state_lock);
  ESP_ERROR_CHECK(mbc_slave_lock(s_mbc_slave_handler));
  s_holding_regs[MODBUS_HREG_WEATHER_SOLAR_RAD] = radiation_wm2;
  ESP_ERROR_CHECK(mbc_slave_unlock(s_mbc_slave_handler));
}

#define MODBUS_ASCII_MAX_TOKENS 16
#define MODBUS_ASCII_LINE_MAX 160

static void trim_ascii_whitespace(char *text) {
  if (text == NULL) {
    return;
  }

  size_t start = 0U;
  size_t len = strlen(text);
  while (text[start] != '\0' && isspace((unsigned char)text[start])) {
    start++;
  }
  while (len > start && isspace((unsigned char)text[len - 1U])) {
    len--;
  }
  if (start > 0U) {
    memmove(text, &text[start], len - start);
  }
  text[len - start] = '\0';
}

static size_t tokenize_ascii_command(char *text, char *tokens[],
                                     size_t max_tokens) {
  size_t count = 0U;
  char *saveptr = NULL;
  char *token = strtok_r(text, " \t", &saveptr);
  while (token != NULL) {
    if (count >= max_tokens) {
      return max_tokens + 1U;
    }
    for (char *p = token; *p != '\0'; ++p) {
      *p = (char)tolower((unsigned char)*p);
    }
    tokens[count++] = token;
    token = strtok_r(NULL, " \t", &saveptr);
  }
  return count;
}

static bool parse_u16_ascii(const char *text, uint16_t max_value,
                            uint16_t *out_value) {
  if (text == NULL || out_value == NULL) {
    return false;
  }

  char *end = NULL;
  unsigned long value = strtoul(text, &end, 10);
  if (text == end || end == NULL || *end != '\0' || value > max_value) {
    return false;
  }

  *out_value = (uint16_t)value;
  return true;
}

static bool parse_tenths_ascii(const char *text, float min_value,
                               float max_value, uint16_t *out_tenths) {
  if (text == NULL || out_tenths == NULL) {
    return false;
  }

  char *end = NULL;
  double value = strtod(text, &end);
  if (text == end || end == NULL || *end != '\0' || value < (double)min_value ||
      value > (double)max_value) {
    return false;
  }

  *out_tenths = (uint16_t)(value * 10.0 + 0.5);
  return true;
}

static bool parse_signed_tenths_ascii(const char *text, float min_value,
                                      float max_value, int16_t *out_tenths) {
  if (text == NULL || out_tenths == NULL) {
    return false;
  }

  char *end = NULL;
  double value = strtod(text, &end);
  if (text == end || end == NULL || *end != '\0' || value < (double)min_value ||
      value > (double)max_value) {
    return false;
  }

  double scaled = value * 10.0;
  int32_t tenths =
      (scaled >= 0.0) ? (int32_t)(scaled + 0.5) : (int32_t)(scaled - 0.5);
  if (tenths < (int32_t)INT16_MIN || tenths > (int32_t)INT16_MAX) {
    return false;
  }

  *out_tenths = (int16_t)tenths;
  return true;
}

static bool parse_switch_ascii(const char *text, uint16_t *out_value) {
  if (text == NULL || out_value == NULL) {
    return false;
  }

  if (strcmp(text, "1") == 0 || strcmp(text, "on") == 0 ||
      strcmp(text, "true") == 0 || strcmp(text, "enable") == 0) {
    *out_value = 1U;
    return true;
  }
  if (strcmp(text, "0") == 0 || strcmp(text, "off") == 0 ||
      strcmp(text, "false") == 0 || strcmp(text, "disable") == 0) {
    *out_value = 0U;
    return true;
  }
  return false;
}

static bool parse_hhmm_ascii(const char *text, uint16_t *out_hhmm) {
  if (text == NULL || out_hhmm == NULL) {
    return false;
  }

  char digits[5] = {0};
  size_t len = strlen(text);
  if (len == 5U && text[2] == ':') {
    if (!isdigit((unsigned char)text[0]) || !isdigit((unsigned char)text[1]) ||
        !isdigit((unsigned char)text[3]) || !isdigit((unsigned char)text[4])) {
      return false;
    }
    digits[0] = text[0];
    digits[1] = text[1];
    digits[2] = text[3];
    digits[3] = text[4];
  } else if (len == 4U) {
    for (size_t i = 0; i < len; ++i) {
      if (!isdigit((unsigned char)text[i])) {
        return false;
      }
      digits[i] = text[i];
    }
  } else {
    return false;
  }

  uint16_t hh = (uint16_t)(((uint16_t)(digits[0] - '0') * 10U) +
                           (uint16_t)(digits[1] - '0'));
  uint16_t mm = (uint16_t)(((uint16_t)(digits[2] - '0') * 10U) +
                           (uint16_t)(digits[3] - '0'));
  if (hh > 23U || mm > 59U) {
    return false;
  }

  *out_hhmm = (uint16_t)(hh * 100U + mm);
  return true;
}

static bool parse_light_relay_ascii(const char *text, size_t *relay_index) {
  if (text == NULL || relay_index == NULL) {
    return false;
  }

  if (strcmp(text, "r1") == 0 || strcmp(text, "relay1") == 0 ||
      strcmp(text, "1") == 0) {
    *relay_index = 0U;
    return true;
  }
  if (strcmp(text, "r2") == 0 || strcmp(text, "relay2") == 0 ||
      strcmp(text, "2") == 0) {
    *relay_index = 1U;
    return true;
  }
  return false;
}

static void format_hhmm_ascii(uint16_t hhmm, char *buffer, size_t buffer_len) {
  if (buffer == NULL || buffer_len == 0U) {
    return;
  }

  uint16_t hh = (uint16_t)(hhmm / 100U);
  uint16_t mm = (uint16_t)(hhmm % 100U);
  snprintf(buffer, buffer_len, "%02u:%02u", (unsigned)hh, (unsigned)mm);
}

static const char *mode_to_string(modbus_mode_state_t mode) {
  return (mode == MODBUS_MODE_AUTONOMOUS) ? "AUTONOMOUS" : "REMOTE";
}

static const char *windows_ctrl_mode_to_string(modbus_windows_ctrl_mode_t mode) {
  return (mode == MODBUS_WINDOWS_CTRL_MODE_MANUAL) ? "MANUAL" : "AUTO";
}

static const char *
windows_auto_algo_to_string(modbus_windows_auto_algo_mode_t mode) {
  return (mode == MODBUS_WINDOWS_AUTO_ALGO_HUMIDITY) ? "HUMIDITY" : "TEMP";
}

static const char *
windows_windward_side_to_string(modbus_windows_windward_side_t side) {
  switch (side) {
  case MODBUS_WINDOWS_WINDWARD_SIDE_A:
    return "A";
  case MODBUS_WINDOWS_WINDWARD_SIDE_B:
    return "B";
  case MODBUS_WINDOWS_WINDWARD_SIDE_BOTH_UNKNOWN:
    return "BOTH_UNKNOWN";
  case MODBUS_WINDOWS_WINDWARD_SIDE_NONE:
  default:
    return "NONE";
  }
}

static const char *reason_to_string(modbus_mode_reason_t reason) {
  switch (reason) {
  case MODBUS_REASON_MASTER_TIMEOUT:
    return "MASTER_TIMEOUT";
  case MODBUS_REASON_NONE:
  default:
    return "NONE";
  }
}

static const char *weather_result_to_string(modbus_weather_set_result_t result) {
  switch (result) {
  case MODBUS_WEATHER_SET_RESULT_APPLIED:
    return "APPLIED";
  case MODBUS_WEATHER_SET_RESULT_NOOP:
    return "NOOP";
  case MODBUS_WEATHER_SET_RESULT_FAILED:
    return "FAILED";
  case MODBUS_WEATHER_SET_RESULT_NONE:
  default:
    return "NONE";
  }
}

static void snapshot_autonomous_state(autonomous_ctrl_cfg_t *cfg,
                                      modbus_mode_state_t *mode,
                                      modbus_mode_reason_t *reason) {
  if (cfg == NULL || mode == NULL || reason == NULL) {
    return;
  }

  taskENTER_CRITICAL(&s_state_lock);
  *cfg = s_autonomous_cfg;
  *mode = s_mode_state;
  *reason = s_mode_reason;
  taskEXIT_CRITICAL(&s_state_lock);
}

static void snapshot_weather_ascii_state(weather_snapshot_t *snapshot,
                                         uint16_t *set_token,
                                         uint16_t *applied_token,
                                         modbus_weather_set_result_t *result,
                                         bool *valid, bool *stale) {
  taskENTER_CRITICAL(&s_state_lock);
  if (snapshot != NULL) {
    *snapshot = s_weather_active_snapshot;
  }
  if (valid != NULL) {
    *valid = s_weather_valid;
  }
  if (stale != NULL) {
    *stale = s_weather_stale;
  }
  taskEXIT_CRITICAL(&s_state_lock);

  uint16_t local_set_token = 0U;
  uint16_t local_applied_token = 0U;
  modbus_weather_set_result_t local_result = MODBUS_WEATHER_SET_RESULT_NONE;
  if (s_mbc_slave_handler != NULL) {
    ESP_ERROR_CHECK(mbc_slave_lock(s_mbc_slave_handler));
    local_set_token = s_holding_regs[MODBUS_HREG_WEATHER_SET_TOKEN];
    local_applied_token = s_holding_regs[MODBUS_HREG_WEATHER_SET_APPLIED_TOKEN];
    local_result =
        (modbus_weather_set_result_t)s_holding_regs[MODBUS_HREG_WEATHER_SET_RESULT];
    ESP_ERROR_CHECK(mbc_slave_unlock(s_mbc_slave_handler));
  } else {
    taskENTER_CRITICAL(&s_state_lock);
    local_set_token =
        (s_weather_pending_token != 0U) ? s_weather_pending_token : s_weather_last_token;
    local_applied_token = s_weather_last_token;
    local_result =
        s_weather_valid ? MODBUS_WEATHER_SET_RESULT_APPLIED
                        : MODBUS_WEATHER_SET_RESULT_NONE;
    taskEXIT_CRITICAL(&s_state_lock);
  }

  if (set_token != NULL) {
    *set_token = local_set_token;
  }
  if (applied_token != NULL) {
    *applied_token = local_applied_token;
  }
  if (result != NULL) {
    *result = local_result;
  }
}

static uint16_t allocate_weather_ascii_token(void) {
  uint16_t token = 0U;

  taskENTER_CRITICAL(&s_state_lock);
  token = s_weather_last_token;
  if (s_weather_pending_token > token) {
    token = s_weather_pending_token;
  }
  taskEXIT_CRITICAL(&s_state_lock);

  if (s_mbc_slave_handler != NULL) {
    ESP_ERROR_CHECK(mbc_slave_lock(s_mbc_slave_handler));
    if (s_holding_regs[MODBUS_HREG_WEATHER_SET_TOKEN] > token) {
      token = s_holding_regs[MODBUS_HREG_WEATHER_SET_TOKEN];
    }
    if (s_holding_regs[MODBUS_HREG_WEATHER_SET_APPLIED_TOKEN] > token) {
      token = s_holding_regs[MODBUS_HREG_WEATHER_SET_APPLIED_TOKEN];
    }
    ESP_ERROR_CHECK(mbc_slave_unlock(s_mbc_slave_handler));
  }

  token = (uint16_t)(token + 1U);
  if (token == 0U) {
    token = 1U;
  }
  return token;
}

static esp_err_t write_weather_snapshot_to_regs(const weather_snapshot_t *snapshot,
                                                uint16_t token) {
  if (snapshot == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  if (s_mbc_slave_handler == NULL) {
    return ESP_ERR_INVALID_STATE;
  }

  ESP_ERROR_CHECK(mbc_slave_lock(s_mbc_slave_handler));
  s_holding_regs[MODBUS_HREG_WEATHER_OUT_TEMP] = (uint16_t)snapshot->out_temp;
  s_holding_regs[MODBUS_HREG_WEATHER_OUT_HUM] = snapshot->out_hum;
  s_holding_regs[MODBUS_HREG_WEATHER_WIND_SPEED] = snapshot->wind_speed;
  s_holding_regs[MODBUS_HREG_WEATHER_WIND_DIR] = snapshot->wind_dir;
  s_holding_regs[MODBUS_HREG_WEATHER_RAIN_FLAG] = snapshot->rain_flag;
  s_holding_regs[MODBUS_HREG_WEATHER_SOLAR_RAD] = snapshot->solar_rad;
  s_holding_regs[MODBUS_HREG_WEATHER_BARO_PRESS] = snapshot->baro_press;
  s_holding_regs[MODBUS_HREG_WEATHER_DEW_POINT] = (uint16_t)snapshot->dew_point;
  s_holding_regs[MODBUS_HREG_WEATHER_STATUS_BITS] = snapshot->status_bits;
  s_holding_regs[MODBUS_HREG_WEATHER_AGE_S] = snapshot->source_age_s;
  s_holding_regs[MODBUS_HREG_WEATHER_SET_TOKEN] = token;
  ESP_ERROR_CHECK(mbc_slave_unlock(s_mbc_slave_handler));
  return ESP_OK;
}

static void format_autonomous_summary(char *response, size_t response_len) {
  if (response == NULL || response_len == 0U) {
    return;
  }

  autonomous_ctrl_cfg_t cfg = {0};
  modbus_mode_state_t mode = MODBUS_MODE_REMOTE;
  modbus_mode_reason_t reason = MODBUS_REASON_NONE;
  snapshot_autonomous_state(&cfg, &mode, &reason);

  char r1_on[6] = {0};
  char r1_off[6] = {0};
  char r2_on[6] = {0};
  char r2_off[6] = {0};
  format_hhmm_ascii(cfg.light.relay[0].schedule.on_hhmm, r1_on, sizeof(r1_on));
  format_hhmm_ascii(cfg.light.relay[0].schedule.off_hhmm, r1_off,
                    sizeof(r1_off));
  format_hhmm_ascii(cfg.light.relay[1].schedule.on_hhmm, r2_on, sizeof(r2_on));
  format_hhmm_ascii(cfg.light.relay[1].schedule.off_hhmm, r2_off,
                    sizeof(r2_off));

  snprintf(
      response, response_len,
      "OK mode=%s reason=%s windows[a=%.1f%% b=%.1f%%] curtain=%.1f%% "
      "sp[rail=%.1fC grow=%.1fC upper=%.1fC undertray=%.1fC] "
      "light[r1 en=%u on=%s off=%s, r2 en=%u on=%s off=%s, hyst=%u]",
      mode_to_string(mode), reason_to_string(reason),
      ((float)cfg.windows_pos_a_target) / 10.0f,
      ((float)cfg.windows_pos_b_target) / 10.0f,
      ((float)cfg.curtain_pos_target) / 10.0f,
      ((float)cfg.sp_water_rail) / 10.0f, ((float)cfg.sp_water_grow) / 10.0f,
      ((float)cfg.sp_water_upper) / 10.0f,
      ((float)cfg.sp_water_undertray) / 10.0f,
      (unsigned)cfg.light.relay[0].schedule.enable, r1_on, r1_off,
      (unsigned)cfg.light.relay[1].schedule.enable, r2_on, r2_off,
      (unsigned)cfg.light.hyst_sec);
}

static void format_weather_summary(char *response, size_t response_len) {
  if (response == NULL || response_len == 0U) {
    return;
  }

  weather_snapshot_t snapshot = {0};
  uint16_t set_token = 0U;
  uint16_t applied_token = 0U;
  modbus_weather_set_result_t result = MODBUS_WEATHER_SET_RESULT_NONE;
  bool valid = false;
  bool stale = true;
  snapshot_weather_ascii_state(&snapshot, &set_token, &applied_token, &result,
                               &valid, &stale);

  snprintf(response, response_len,
           "OK weather valid=%u stale=%u token=%u applied=%u result=%s "
           "out=%.1fC hum=%.1f%% wind=%.1fm/s dir=%u rain=%u solar=%uW/m2 "
           "baro=%.1fhPa dew=%.1fC status=0x%04X age=%us",
           valid ? 1U : 0U, stale ? 1U : 0U, (unsigned)set_token,
           (unsigned)applied_token, weather_result_to_string(result),
           ((float)snapshot.out_temp) / 10.0f, ((float)snapshot.out_hum) / 10.0f,
           ((float)snapshot.wind_speed) / 10.0f, (unsigned)snapshot.wind_dir,
           (unsigned)snapshot.rain_flag, (unsigned)snapshot.solar_rad,
           ((float)snapshot.baro_press) / 10.0f,
           ((float)snapshot.dew_point) / 10.0f, (unsigned)snapshot.status_bits,
           (unsigned)snapshot.source_age_s);
}

static void format_windows_summary(char *response, size_t response_len) {
  if (response == NULL || response_len == 0U) {
    return;
  }

  const modbus_windows_ctrl_mode_t ctrl_mode =
      (modbus_read_holding_reg(MODBUS_HREG_WINDOWS_CTRL_MODE) ==
       (uint16_t)MODBUS_WINDOWS_CTRL_MODE_MANUAL)
          ? MODBUS_WINDOWS_CTRL_MODE_MANUAL
          : MODBUS_WINDOWS_CTRL_MODE_AUTO;
  const modbus_windows_auto_algo_mode_t algo_mode =
      (modbus_read_holding_reg(MODBUS_HREG_WINDOWS_AUTO_ALGO_MODE) ==
       (uint16_t)MODBUS_WINDOWS_AUTO_ALGO_HUMIDITY)
          ? MODBUS_WINDOWS_AUTO_ALGO_HUMIDITY
          : MODBUS_WINDOWS_AUTO_ALGO_TEMP;
  const modbus_windows_windward_side_t windward_side =
      (modbus_windows_windward_side_t)
          modbus_read_holding_reg(MODBUS_HREG_WINDOWS_WINDWARD_SIDE);

  snprintf(response, response_len,
           "OK windows win_mode=%s alg=%s windward=%s "
           "requested[a=%.1f%% b=%.1f%%] base[a=%.1f%% b=%.1f%%] "
           "effective[a=%.1f%% b=%.1f%%] pos[a=%.1f%% b=%.1f%%] "
           "prot=0x%04X status[sys=0x%04X a=0x%04X b=0x%04X] "
           "fault[a=%u b=%u]",
           windows_ctrl_mode_to_string(ctrl_mode),
           windows_auto_algo_to_string(algo_mode),
           windows_windward_side_to_string(windward_side),
           ((float)modbus_read_holding_reg(MODBUS_HREG_WINDOWS_POS_A_TARGET)) /
               10.0f,
           ((float)modbus_read_holding_reg(MODBUS_HREG_WINDOWS_POS_B_TARGET)) /
               10.0f,
           ((float)modbus_read_holding_reg(MODBUS_HREG_WINDOWS_BASE_TARGET_A)) /
               10.0f,
           ((float)modbus_read_holding_reg(MODBUS_HREG_WINDOWS_BASE_TARGET_B)) /
               10.0f,
           ((float)modbus_read_holding_reg(
                MODBUS_HREG_WINDOWS_EFFECTIVE_TARGET_A)) /
               10.0f,
           ((float)modbus_read_holding_reg(
                MODBUS_HREG_WINDOWS_EFFECTIVE_TARGET_B)) /
               10.0f,
           ((float)modbus_read_holding_reg(MODBUS_HREG_WINDOWS_POS_A)) / 10.0f,
           ((float)modbus_read_holding_reg(MODBUS_HREG_WINDOWS_POS_B)) / 10.0f,
           (unsigned)modbus_read_holding_reg(
               MODBUS_HREG_WINDOWS_ACTIVE_PROTECTION_BITS),
           (unsigned)modbus_read_holding_reg(MODBUS_HREG_WINDOWS_STATUS_BITS),
           (unsigned)modbus_read_holding_reg(MODBUS_HREG_WINDOW_A_STATUS_BITS),
           (unsigned)modbus_read_holding_reg(MODBUS_HREG_WINDOW_B_STATUS_BITS),
           (unsigned)modbus_read_holding_reg(MODBUS_HREG_WINDOW_A_FAULT_CODE),
           (unsigned)modbus_read_holding_reg(MODBUS_HREG_WINDOW_B_FAULT_CODE));
}

static void format_curtain_summary(char *response, size_t response_len) {
  if (response == NULL || response_len == 0U) {
    return;
  }

  const modbus_curtain_ctrl_mode_t mode = modbus_get_curtain_ctrl_mode();
  const char *mode_text = "MANUAL";
  if (mode == MODBUS_CURTAIN_CTRL_MODE_AUTO) {
    mode_text = "AUTO";
  } else if (mode == MODBUS_CURTAIN_CTRL_MODE_OFF) {
    mode_text = "OFF";
  }

  snprintf(response, response_len,
           "OK curtain mode=%s requested=%.1f%% manual=%.1f%% target=%.1f%% "
           "base=%.1f%% pos=%.1f%% current=%.1fmA status=0x%04X "
           "reason=0x%04X pos_status=0x%04X fault=%u",
           mode_text, modbus_get_curtain_target_percent(),
           modbus_get_curtain_manual_target_percent(),
           ((float)modbus_read_holding_reg(MODBUS_HREG_CURTAIN_TARGET)) / 10.0f,
           ((float)modbus_read_holding_reg(MODBUS_HREG_CURTAIN_BASE_TARGET)) /
               10.0f,
           ((float)modbus_read_holding_reg(MODBUS_HREG_CURTAIN_POS)) / 10.0f,
           ((float)modbus_read_holding_reg(MODBUS_HREG_CURTAIN_CURRENT_MA)) /
               10.0f,
           (unsigned)modbus_read_holding_reg(MODBUS_HREG_CURTAIN_STATUS_BITS),
           (unsigned)modbus_read_holding_reg(MODBUS_HREG_CURTAIN_REASON_BITS),
           (unsigned)modbus_read_holding_reg(
               MODBUS_HREG_CURTAIN_POSITION_STATUS_BITS),
           (unsigned)modbus_read_holding_reg(MODBUS_HREG_CURTAIN_FAULT_CODE));
}

static esp_err_t apply_ascii_weather_update(const weather_snapshot_t *snapshot,
                                            char *response, size_t response_len,
                                            const char *success_text) {
  if (snapshot == NULL || response == NULL || response_len == 0U ||
      success_text == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  const uint16_t token = allocate_weather_ascii_token();
  modbus_weather_set_result_t result = MODBUS_WEATHER_SET_RESULT_NONE;

  if (s_mbc_slave_handler != NULL) {
    esp_err_t err = write_weather_snapshot_to_regs(snapshot, token);
    if (err != ESP_OK) {
      snprintf(response, response_len, "ERR failed to stage weather snapshot");
      return err;
    }

    queue_weather_sync_request_from_regs();
    process_pending_weather_sync();

    uint16_t applied_token = 0U;
    snapshot_weather_ascii_state(NULL, NULL, &applied_token, &result, NULL, NULL);
    if (applied_token != token ||
        (result != MODBUS_WEATHER_SET_RESULT_APPLIED &&
         result != MODBUS_WEATHER_SET_RESULT_NOOP)) {
      snprintf(response, response_len, "ERR failed to apply weather snapshot");
      return ESP_FAIL;
    }
  } else {
    const uint32_t now = now_ms();
    taskENTER_CRITICAL(&s_state_lock);
    s_weather_active_snapshot = *snapshot;
    s_weather_last_token = token;
    s_weather_pending_token = 0U;
    s_weather_sync_pending = false;
    s_weather_valid = true;
    s_weather_stale = false;
    s_weather_last_rx_ms = now;
    taskEXIT_CRITICAL(&s_state_lock);
    result = MODBUS_WEATHER_SET_RESULT_APPLIED;
  }

  snprintf(response, response_len, "OK %s token=%u result=%s", success_text,
           (unsigned)token, weather_result_to_string(result));
  return ESP_OK;
}

static esp_err_t handle_ascii_weather_short_alias_command(size_t token_count,
                                                          char *tokens[],
                                                          char *response,
                                                          size_t response_len) {
  if (token_count != 2U || tokens == NULL || response == NULL ||
      response_len == 0U) {
    return ESP_ERR_NOT_SUPPORTED;
  }

  const char *name = tokens[0];
  const char *value = tokens[1];
  weather_snapshot_t snapshot = {0};
  snapshot_weather_ascii_state(&snapshot, NULL, NULL, NULL, NULL, NULL);

  if (strcmp(name, "wx_temp") == 0) {
    int16_t temp_tenths = 0;
    if (!parse_signed_tenths_ascii(value, -60.0f, 80.0f, &temp_tenths)) {
      snprintf(response, response_len, "ERR invalid outside temperature");
      return ESP_ERR_INVALID_ARG;
    }
    snapshot.out_temp = temp_tenths;
    char ok_text[48] = {0};
    snprintf(ok_text, sizeof(ok_text), "wx_temp=%.1fC",
             ((float)temp_tenths) / 10.0f);
    return apply_ascii_weather_update(&snapshot, response, response_len, ok_text);
  }

  if (strcmp(name, "wx_hum") == 0) {
    uint16_t hum_tenths = 0U;
    if (!parse_tenths_ascii(value, 0.0f, 100.0f, &hum_tenths)) {
      snprintf(response, response_len, "ERR invalid outside humidity");
      return ESP_ERR_INVALID_ARG;
    }
    snapshot.out_hum = hum_tenths;
    char ok_text[48] = {0};
    snprintf(ok_text, sizeof(ok_text), "wx_hum=%.1f%%",
             ((float)hum_tenths) / 10.0f);
    return apply_ascii_weather_update(&snapshot, response, response_len, ok_text);
  }

  if (strcmp(name, "wx_wind") == 0) {
    uint16_t wind_tenths = 0U;
    if (!parse_tenths_ascii(value, 0.0f, 100.0f, &wind_tenths)) {
      snprintf(response, response_len, "ERR invalid wind speed");
      return ESP_ERR_INVALID_ARG;
    }
    snapshot.wind_speed = wind_tenths;
    char ok_text[48] = {0};
    snprintf(ok_text, sizeof(ok_text), "wx_wind=%.1fm/s",
             ((float)wind_tenths) / 10.0f);
    return apply_ascii_weather_update(&snapshot, response, response_len, ok_text);
  }

  if (strcmp(name, "wx_dir") == 0) {
    uint16_t wind_dir = 0U;
    if (!parse_u16_ascii(value, 359U, &wind_dir)) {
      snprintf(response, response_len, "ERR invalid wind direction");
      return ESP_ERR_INVALID_ARG;
    }
    snapshot.wind_dir = wind_dir;
    char ok_text[48] = {0};
    snprintf(ok_text, sizeof(ok_text), "wx_dir=%u", (unsigned)wind_dir);
    return apply_ascii_weather_update(&snapshot, response, response_len, ok_text);
  }

  if (strcmp(name, "wx_rain") == 0) {
    uint16_t rain_flag = 0U;
    if (!parse_switch_ascii(value, &rain_flag)) {
      snprintf(response, response_len, "ERR invalid rain flag");
      return ESP_ERR_INVALID_ARG;
    }
    snapshot.rain_flag = rain_flag;
    char ok_text[48] = {0};
    snprintf(ok_text, sizeof(ok_text), "wx_rain=%u", (unsigned)rain_flag);
    return apply_ascii_weather_update(&snapshot, response, response_len, ok_text);
  }

  if (strcmp(name, "wx_solar") == 0) {
    uint16_t solar_rad = 0U;
    if (!parse_u16_ascii(value, UINT16_MAX, &solar_rad)) {
      snprintf(response, response_len, "ERR invalid solar radiation");
      return ESP_ERR_INVALID_ARG;
    }
    snapshot.solar_rad = solar_rad;
    char ok_text[48] = {0};
    snprintf(ok_text, sizeof(ok_text), "wx_solar=%uW/m2",
             (unsigned)solar_rad);
    return apply_ascii_weather_update(&snapshot, response, response_len, ok_text);
  }

  if (strcmp(name, "wx_baro") == 0) {
    uint16_t baro_tenths = 0U;
    if (!parse_tenths_ascii(value, 300.0f, 1200.0f, &baro_tenths)) {
      snprintf(response, response_len, "ERR invalid barometric pressure");
      return ESP_ERR_INVALID_ARG;
    }
    snapshot.baro_press = baro_tenths;
    char ok_text[48] = {0};
    snprintf(ok_text, sizeof(ok_text), "wx_baro=%.1fhPa",
             ((float)baro_tenths) / 10.0f);
    return apply_ascii_weather_update(&snapshot, response, response_len, ok_text);
  }

  if (strcmp(name, "wx_dew") == 0) {
    int16_t dew_tenths = 0;
    if (!parse_signed_tenths_ascii(value, -80.0f, 80.0f, &dew_tenths)) {
      snprintf(response, response_len, "ERR invalid dew point");
      return ESP_ERR_INVALID_ARG;
    }
    snapshot.dew_point = dew_tenths;
    char ok_text[48] = {0};
    snprintf(ok_text, sizeof(ok_text), "wx_dew=%.1fC",
             ((float)dew_tenths) / 10.0f);
    return apply_ascii_weather_update(&snapshot, response, response_len, ok_text);
  }

  if (strcmp(name, "wx_age") == 0) {
    uint16_t age_s = 0U;
    if (!parse_u16_ascii(value, UINT16_MAX, &age_s)) {
      snprintf(response, response_len, "ERR invalid weather age");
      return ESP_ERR_INVALID_ARG;
    }
    snapshot.source_age_s = age_s;
    char ok_text[48] = {0};
    snprintf(ok_text, sizeof(ok_text), "wx_age=%us", (unsigned)age_s);
    return apply_ascii_weather_update(&snapshot, response, response_len, ok_text);
  }

  if (strcmp(name, "wx_stat") == 0) {
    uint16_t status_bits = 0U;
    if (!parse_u16_ascii(value, UINT16_MAX, &status_bits)) {
      snprintf(response, response_len, "ERR invalid weather status bits");
      return ESP_ERR_INVALID_ARG;
    }
    snapshot.status_bits = status_bits;
    char ok_text[48] = {0};
    snprintf(ok_text, sizeof(ok_text), "wx_stat=0x%04X",
             (unsigned)status_bits);
    return apply_ascii_weather_update(&snapshot, response, response_len, ok_text);
  }

  return ESP_ERR_NOT_SUPPORTED;
}

static esp_err_t handle_ascii_sensor_test_alias_command(size_t token_count,
                                                        char *tokens[],
                                                        char *response,
                                                        size_t response_len) {
  if (token_count != 2U || tokens == NULL || response == NULL ||
      response_len == 0U) {
    return ESP_ERR_NOT_SUPPORTED;
  }

  const char *name = tokens[0];
  const char *value = tokens[1];
  const bool disable_value =
      strcmp(value, "off") == 0 || strcmp(value, "real") == 0 ||
      strcmp(value, "sensor") == 0 || strcmp(value, "disable") == 0;

  if (strcmp(name, "sensor_override") == 0 ||
      strcmp(name, "sensor_test") == 0) {
    uint16_t enabled = 0U;
    if (parse_switch_ascii(value, &enabled)) {
      taskENTER_CRITICAL(&s_state_lock);
      if (!enabled) {
        s_air_temp_override_active = false;
        s_rh_override_active = false;
      }
      taskEXIT_CRITICAL(&s_state_lock);

      if (enabled) {
        snprintf(response, response_len,
                 "ERR set air_temp/air_rh value to enable override");
        return ESP_ERR_INVALID_ARG;
      }

      snprintf(response, response_len, "OK sensor_override=off");
      return ESP_OK;
    }

    snprintf(response, response_len, "ERR invalid sensor_override value");
    return ESP_ERR_INVALID_ARG;
  }

  if (strcmp(name, "air_temp") == 0 || strcmp(name, "test_air_temp") == 0) {
    if (disable_value) {
      taskENTER_CRITICAL(&s_state_lock);
      s_air_temp_override_active = false;
      taskEXIT_CRITICAL(&s_state_lock);
      snprintf(response, response_len, "OK air_temp=real");
      return ESP_OK;
    }

    int16_t temp_tenths = 0;
    if (!parse_signed_tenths_ascii(value, -60.0f, 120.0f, &temp_tenths)) {
      snprintf(response, response_len, "ERR invalid air temperature");
      return ESP_ERR_INVALID_ARG;
    }

    taskENTER_CRITICAL(&s_state_lock);
    s_air_temp_override_tenths = temp_tenths;
    s_air_temp_override_active = true;
    taskEXIT_CRITICAL(&s_state_lock);

    snprintf(response, response_len, "OK air_temp=%.1fC override=on",
             ((float)temp_tenths) / 10.0f);
    return ESP_OK;
  }

  if (strcmp(name, "air_rh") == 0 || strcmp(name, "test_rh") == 0 ||
      strcmp(name, "rh") == 0) {
    if (disable_value) {
      taskENTER_CRITICAL(&s_state_lock);
      s_rh_override_active = false;
      taskEXIT_CRITICAL(&s_state_lock);
      snprintf(response, response_len, "OK air_rh=real");
      return ESP_OK;
    }

    uint16_t rh_tenths = 0U;
    if (!parse_tenths_ascii(value, 0.0f, 100.0f, &rh_tenths)) {
      snprintf(response, response_len, "ERR invalid air humidity");
      return ESP_ERR_INVALID_ARG;
    }

    taskENTER_CRITICAL(&s_state_lock);
    s_rh_override_tenths = rh_tenths;
    s_rh_override_active = true;
    taskEXIT_CRITICAL(&s_state_lock);

    snprintf(response, response_len, "OK air_rh=%.1f%% override=on",
             ((float)rh_tenths) / 10.0f);
    return ESP_OK;
  }

  return ESP_ERR_NOT_SUPPORTED;
}

typedef struct {
  const char *name;
  uint16_t reg;
  float min_value;
  float max_value;
  const char *unit;
} windows_tenths_alias_t;

static esp_err_t handle_ascii_window_settings_alias_command(
    size_t token_count, char *tokens[], char *response, size_t response_len) {
  if (token_count != 2U || tokens == NULL || response == NULL ||
      response_len == 0U) {
    return ESP_ERR_NOT_SUPPORTED;
  }

  const char *name = tokens[0];
  const char *value = tokens[1];

  if (strcmp(name, "win_alg") == 0) {
    modbus_windows_auto_algo_mode_t mode = MODBUS_WINDOWS_AUTO_ALGO_TEMP;
    if (strcmp(value, "temp") == 0 || strcmp(value, "temperature") == 0 ||
        strcmp(value, "0") == 0) {
      mode = MODBUS_WINDOWS_AUTO_ALGO_TEMP;
    } else if (strcmp(value, "hum") == 0 || strcmp(value, "humidity") == 0 ||
               strcmp(value, "1") == 0) {
      mode = MODBUS_WINDOWS_AUTO_ALGO_HUMIDITY;
    } else {
      snprintf(response, response_len, "ERR invalid windows algorithm");
      return ESP_ERR_INVALID_ARG;
    }

    modbus_write_holding_reg(MODBUS_HREG_WINDOWS_AUTO_ALGO_MODE,
                             (uint16_t)mode);
    snprintf(response, response_len, "OK win_alg=%s",
             windows_auto_algo_to_string(mode));
    return ESP_OK;
  }

  if (strcmp(name, "rain_mode") == 0) {
    uint16_t enabled = 0U;
    modbus_windows_rain_mode_t mode = MODBUS_WINDOWS_RAIN_MODE_DISABLED;
    if (parse_switch_ascii(value, &enabled)) {
      mode = enabled ? MODBUS_WINDOWS_RAIN_MODE_WINDWARD
                     : MODBUS_WINDOWS_RAIN_MODE_DISABLED;
    } else if (strcmp(value, "windward") == 0) {
      mode = MODBUS_WINDOWS_RAIN_MODE_WINDWARD;
    } else {
      snprintf(response, response_len, "ERR invalid rain mode");
      return ESP_ERR_INVALID_ARG;
    }

    modbus_write_holding_reg(MODBUS_HREG_WINDOWS_RAIN_MODE, (uint16_t)mode);
    snprintf(response, response_len, "OK rain_mode=%s",
             (mode == MODBUS_WINDOWS_RAIN_MODE_WINDWARD) ? "WINDWARD" : "OFF");
    return ESP_OK;
  }

  if (strcmp(name, "wx_stale_ms") == 0 || strcmp(name, "wx_age_max") == 0) {
    uint16_t raw_value = 0U;
    uint16_t max_value = UINT16_MAX;
    uint16_t reg = 0U;

    if (strcmp(name, "wx_stale_ms") == 0) {
      max_value = 60000U;
      reg = MODBUS_HREG_WINDOWS_WEATHER_STALE_TIMEOUT_MS;
    } else {
      max_value = 600U;
      reg = MODBUS_HREG_WINDOWS_WEATHER_SOURCE_AGE_S;
    }

    if (!parse_u16_ascii(value, max_value, &raw_value)) {
      snprintf(response, response_len, "ERR invalid %s value", name);
      return ESP_ERR_INVALID_ARG;
    }

    modbus_write_holding_reg(reg, raw_value);
    snprintf(response, response_len, "OK %s=%u", name, (unsigned)raw_value);
    return ESP_OK;
  }

  static const windows_tenths_alias_t aliases[] = {
      {"temp_open", MODBUS_HREG_WINDOWS_TEMP_STEP_TARGET_PERCENT, 0.0f, 100.0f,
       "%"},
      {"hum_sp", MODBUS_HREG_WINDOWS_HUM_SETPOINT, 0.0f, 100.0f, "%"},
      {"hum_step", MODBUS_HREG_WINDOWS_HUM_STEP, 0.1f, 100.0f, "%"},
      {"hum_hyst", MODBUS_HREG_WINDOWS_HUM_STEP_HYST, 0.0f, 100.0f, "%"},
      {"hum_open", MODBUS_HREG_WINDOWS_HUM_STEP_TARGET_PERCENT, 0.0f, 100.0f,
       "%"},
      {"cold_close", MODBUS_HREG_WINDOWS_COLD_CLOSE_DELTA, 0.0f, 50.0f, "C"},
      {"cold_hyst", MODBUS_HREG_WINDOWS_COLD_CLOSE_HYST, 0.0f, 50.0f, "C"},
      {"windward_min", MODBUS_HREG_WINDOWS_WINDWARD_MIN_PERCENT, 0.0f, 100.0f,
       "%"},
      {"windward_max", MODBUS_HREG_WINDOWS_WINDWARD_MAX_PERCENT, 0.0f, 100.0f,
       "%"},
      {"windward_thr", MODBUS_HREG_WINDOWS_WINDWARD_SPEED_THRESHOLD, 0.0f,
       100.0f, "m/s"},
      {"windward_reduce",
       MODBUS_HREG_WINDOWS_WINDWARD_REDUCTION_PERCENT_PER_MS, 0.0f, 100.0f,
       "%/m/s"},
      {"leeward_min", MODBUS_HREG_WINDOWS_LEEWARD_MIN_PERCENT, 0.0f, 100.0f,
       "%"},
      {"leeward_max", MODBUS_HREG_WINDOWS_LEEWARD_MAX_PERCENT, 0.0f, 100.0f,
       "%"},
      {"leeward_thr", MODBUS_HREG_WINDOWS_LEEWARD_SPEED_THRESHOLD, 0.0f,
       100.0f, "m/s"},
      {"leeward_reduce", MODBUS_HREG_WINDOWS_LEEWARD_REDUCTION_PERCENT_PER_MS,
       0.0f, 100.0f, "%/m/s"},
      {"wind_lag", MODBUS_HREG_WINDOWS_WINDWARD_LAG_PERCENT, 0.0f, 100.0f,
       "%"},
      {"rain_pos", MODBUS_HREG_WINDOWS_RAIN_WINDWARD_PERCENT, 0.0f, 100.0f,
       "%"},
  };

  for (size_t i = 0; i < sizeof(aliases) / sizeof(aliases[0]); ++i) {
    if (strcmp(name, aliases[i].name) != 0) {
      continue;
    }

    uint16_t raw_tenths = 0U;
    if (!parse_tenths_ascii(value, aliases[i].min_value, aliases[i].max_value,
                            &raw_tenths)) {
      snprintf(response, response_len, "ERR invalid %s value", aliases[i].name);
      return ESP_ERR_INVALID_ARG;
    }

    modbus_write_holding_reg(aliases[i].reg, raw_tenths);
    snprintf(response, response_len, "OK %s=%.1f%s", aliases[i].name,
             ((float)raw_tenths) / 10.0f, aliases[i].unit);
    return ESP_OK;
  }

  return ESP_ERR_NOT_SUPPORTED;
}

static esp_err_t handle_ascii_short_alias_command(size_t token_count,
                                                  char *tokens[],
                                                  char *response,
                                                  size_t response_len) {
  if (token_count != 2U || tokens == NULL || response == NULL ||
      response_len == 0U) {
    return ESP_ERR_NOT_SUPPORTED;
  }

  esp_err_t weather_result = handle_ascii_weather_short_alias_command(
      token_count, tokens, response, response_len);
  if (weather_result != ESP_ERR_NOT_SUPPORTED) {
    return weather_result;
  }

  esp_err_t sensor_test_result = handle_ascii_sensor_test_alias_command(
      token_count, tokens, response, response_len);
  if (sensor_test_result != ESP_ERR_NOT_SUPPORTED) {
    return sensor_test_result;
  }

  esp_err_t window_settings_result = handle_ascii_window_settings_alias_command(
      token_count, tokens, response, response_len);
  if (window_settings_result != ESP_ERR_NOT_SUPPORTED) {
    return window_settings_result;
  }

  autonomous_ctrl_cfg_t cfg = {0};
  modbus_mode_state_t mode_snapshot = MODBUS_MODE_REMOTE;
  modbus_mode_reason_t reason_snapshot = MODBUS_REASON_NONE;
  snapshot_autonomous_state(&cfg, &mode_snapshot, &reason_snapshot);
  (void)reason_snapshot;

  const char *name = tokens[0];
  const char *value = tokens[1];

  if (strcmp(name, "win_mode") == 0 || strcmp(name, "windows_mode") == 0) {
    modbus_windows_ctrl_mode_t ctrl_mode = MODBUS_WINDOWS_CTRL_MODE_AUTO;
    if (strcmp(value, "auto") == 0 || strcmp(value, "0") == 0) {
      ctrl_mode = MODBUS_WINDOWS_CTRL_MODE_AUTO;
    } else if (strcmp(value, "manual") == 0 || strcmp(value, "1") == 0) {
      ctrl_mode = MODBUS_WINDOWS_CTRL_MODE_MANUAL;
    } else {
      snprintf(response, response_len, "ERR invalid windows mode");
      return ESP_ERR_INVALID_ARG;
    }

    modbus_write_holding_reg(MODBUS_HREG_WINDOWS_CTRL_MODE,
                             (uint16_t)ctrl_mode);
    snprintf(response, response_len, "OK win_mode=%s",
             windows_ctrl_mode_to_string(ctrl_mode));
    return ESP_OK;
  }

  if (strcmp(name, "win_a_pos") == 0 || strcmp(name, "set_a_pos") == 0) {
    uint16_t percent_tenths = 0U;
    if (!parse_tenths_ascii(value, 0.0f, 100.0f, &percent_tenths)) {
      snprintf(response, response_len, "ERR invalid percent value");
      return ESP_ERR_INVALID_ARG;
    }
    if (mode_snapshot != MODBUS_MODE_AUTONOMOUS) {
      snprintf(response, response_len,
               "ERR windows target is available only in AUTONOMOUS mode");
      return ESP_ERR_INVALID_STATE;
    }
    cfg.windows_pos_a_target = percent_tenths;
    modbus_write_holding_reg(MODBUS_HREG_WINDOWS_POS_A_TARGET, percent_tenths);
    const modbus_windows_ctrl_mode_t ctrl_mode =
        (modbus_read_holding_reg(MODBUS_HREG_WINDOWS_CTRL_MODE) ==
         (uint16_t)MODBUS_WINDOWS_CTRL_MODE_MANUAL)
            ? MODBUS_WINDOWS_CTRL_MODE_MANUAL
            : MODBUS_WINDOWS_CTRL_MODE_AUTO;
    char ok_text[80] = {0};
    snprintf(ok_text, sizeof(ok_text),
             "win_a_pos=%.1f%% requested=%.1f%% mode=%s",
             ((float)percent_tenths) / 10.0f,
             ((float)percent_tenths) / 10.0f,
             windows_ctrl_mode_to_string(ctrl_mode));
    return apply_ascii_autonomous_update(&cfg, response, response_len, ok_text);
  }

  if (strcmp(name, "win_b_pos") == 0 || strcmp(name, "set_b_pos") == 0) {
    uint16_t percent_tenths = 0U;
    if (!parse_tenths_ascii(value, 0.0f, 100.0f, &percent_tenths)) {
      snprintf(response, response_len, "ERR invalid percent value");
      return ESP_ERR_INVALID_ARG;
    }
    if (mode_snapshot != MODBUS_MODE_AUTONOMOUS) {
      snprintf(response, response_len,
               "ERR windows target is available only in AUTONOMOUS mode");
      return ESP_ERR_INVALID_STATE;
    }
    cfg.windows_pos_b_target = percent_tenths;
    modbus_write_holding_reg(MODBUS_HREG_WINDOWS_POS_B_TARGET, percent_tenths);
    const modbus_windows_ctrl_mode_t ctrl_mode =
        (modbus_read_holding_reg(MODBUS_HREG_WINDOWS_CTRL_MODE) ==
         (uint16_t)MODBUS_WINDOWS_CTRL_MODE_MANUAL)
            ? MODBUS_WINDOWS_CTRL_MODE_MANUAL
            : MODBUS_WINDOWS_CTRL_MODE_AUTO;
    char ok_text[80] = {0};
    snprintf(ok_text, sizeof(ok_text),
             "win_b_pos=%.1f%% requested=%.1f%% mode=%s",
             ((float)percent_tenths) / 10.0f,
             ((float)percent_tenths) / 10.0f,
             windows_ctrl_mode_to_string(ctrl_mode));
    return apply_ascii_autonomous_update(&cfg, response, response_len, ok_text);
  }

  if (strcmp(name, "curt_pos") == 0) {
    uint16_t percent_tenths = 0U;
    if (!parse_tenths_ascii(value, 0.0f, 100.0f, &percent_tenths)) {
      snprintf(response, response_len, "ERR invalid percent value");
      return ESP_ERR_INVALID_ARG;
    }
    cfg.curtain_pos_target = percent_tenths;
    char ok_text[48] = {0};
    snprintf(ok_text, sizeof(ok_text), "curt_pos=%.1f%%",
             ((float)percent_tenths) / 10.0f);
    return apply_ascii_autonomous_update(&cfg, response, response_len, ok_text);
  }

  if (strcmp(name, "sp_rail") == 0 || strcmp(name, "sp_grow") == 0 ||
      strcmp(name, "sp_upper") == 0 || strcmp(name, "sp_under") == 0) {
    uint16_t temp_tenths = 0U;
    if (!parse_tenths_ascii(value, 0.0f, 120.0f, &temp_tenths)) {
      snprintf(response, response_len, "ERR invalid temperature value");
      return ESP_ERR_INVALID_ARG;
    }

    if (strcmp(name, "sp_rail") == 0) {
      cfg.sp_water_rail = temp_tenths;
    } else if (strcmp(name, "sp_grow") == 0) {
      cfg.sp_water_grow = temp_tenths;
    } else if (strcmp(name, "sp_upper") == 0) {
      cfg.sp_water_upper = temp_tenths;
    } else {
      cfg.sp_water_undertray = temp_tenths;
    }

    char ok_text[48] = {0};
    snprintf(ok_text, sizeof(ok_text), "%s=%.1fC", name,
             ((float)temp_tenths) / 10.0f);
    return apply_ascii_autonomous_update(&cfg, response, response_len, ok_text);
  }

  if (strcmp(name, "l1_on") == 0 || strcmp(name, "l1_off") == 0 ||
      strcmp(name, "l2_on") == 0 || strcmp(name, "l2_off") == 0) {
    uint16_t hhmm = 0U;
    if (!parse_hhmm_ascii(value, &hhmm)) {
      snprintf(response, response_len, "ERR invalid HH:MM value");
      return ESP_ERR_INVALID_ARG;
    }

    const size_t relay_index = (name[1] == '1') ? 0U : 1U;
    if (strstr(name, "_on") != NULL) {
      cfg.light.relay[relay_index].schedule.on_hhmm = hhmm;
    } else {
      cfg.light.relay[relay_index].schedule.off_hhmm = hhmm;
    }

    char hhmm_text[6] = {0};
    char ok_text[48] = {0};
    format_hhmm_ascii(hhmm, hhmm_text, sizeof(hhmm_text));
    snprintf(ok_text, sizeof(ok_text), "%s=%s", name, hhmm_text);
    return apply_ascii_autonomous_update(&cfg, response, response_len, ok_text);
  }

  if (strcmp(name, "l1_thr") == 0 || strcmp(name, "l2_thr") == 0 ||
      strcmp(name, "l1_dli") == 0 || strcmp(name, "l2_dli") == 0) {
    snprintf(response, response_len,
             "ERR setpoint not used in autonomous mode");
    return ESP_ERR_NOT_SUPPORTED;
  }

  if (strcmp(name, "light_hyst") == 0) {
    uint16_t raw_value = 0U;
    if (!parse_u16_ascii(value, UINT16_MAX, &raw_value)) {
      snprintf(response, response_len, "ERR invalid integer value");
      return ESP_ERR_INVALID_ARG;
    }

    cfg.light.hyst_sec = raw_value;

    char ok_text[48] = {0};
    snprintf(ok_text, sizeof(ok_text), "%s=%u", name, (unsigned)raw_value);
    return apply_ascii_autonomous_update(&cfg, response, response_len, ok_text);
  }

  if (strcmp(name, "l1_en") == 0 || strcmp(name, "l2_en") == 0) {
    uint16_t enable_value = 0U;
    if (!parse_switch_ascii(value, &enable_value)) {
      snprintf(response, response_len, "ERR invalid enable value");
      return ESP_ERR_INVALID_ARG;
    }

    const size_t relay_index = (name[1] == '1') ? 0U : 1U;
    cfg.light.relay[relay_index].schedule.enable = enable_value;

    char ok_text[48] = {0};
    snprintf(ok_text, sizeof(ok_text), "%s=%u", name, (unsigned)enable_value);
    return apply_ascii_autonomous_update(&cfg, response, response_len, ok_text);
  }

  return ESP_ERR_NOT_SUPPORTED;
}

static esp_err_t apply_ascii_autonomous_update(const autonomous_ctrl_cfg_t *cfg,
                                               char *response,
                                               size_t response_len,
                                               const char *success_text) {
  esp_err_t err = update_autonomous_cfg(cfg);
  if (err == ESP_OK) {
    snprintf(response, response_len, "OK %s", success_text);
  } else if (err == ESP_ERR_INVALID_ARG) {
    snprintf(response, response_len, "ERR value out of range");
  } else {
    snprintf(response, response_len, "ERR failed to save autonomous config");
  }
  return err;
}

static esp_err_t handle_ascii_show_command(size_t token_count, char *tokens[],
                                           char *response,
                                           size_t response_len) {
  if (response == NULL || response_len == 0U) {
    return ESP_ERR_INVALID_ARG;
  }

  if (token_count == 0U ||
      (token_count == 1U && strcmp(tokens[0], "autonomous") == 0)) {
    format_autonomous_summary(response, response_len);
    return ESP_OK;
  }

  if (token_count == 1U && strcmp(tokens[0], "mode") == 0) {
    modbus_mode_state_t mode = MODBUS_MODE_REMOTE;
    modbus_mode_reason_t reason = MODBUS_REASON_NONE;
    autonomous_ctrl_cfg_t cfg = {0};
    snapshot_autonomous_state(&cfg, &mode, &reason);
    (void)cfg;
    snprintf(response, response_len, "OK mode=%s reason=%s",
             mode_to_string(mode), reason_to_string(reason));
    return ESP_OK;
  }

  if (token_count == 1U && strcmp(tokens[0], "weather") == 0) {
    format_weather_summary(response, response_len);
    return ESP_OK;
  }

  if (token_count == 1U && strcmp(tokens[0], "sensors") == 0) {
    modbus_sensor_test_override_t override = {0};
    modbus_get_sensor_test_override(&override);
    snprintf(response, response_len,
             "OK sensor_override air_temp=%s",
             override.air_temp_override_active ? "on" : "off");
    const size_t used = strlen(response);
    if (used < response_len) {
      snprintf(response + used, response_len - used,
               " value=%.1fC air_rh=%s value=%.1f%%",
               override.air_temp_c,
               override.rh_override_active ? "on" : "off",
               override.rh_percent);
    }
    return ESP_OK;
  }

  if (token_count == 1U && strcmp(tokens[0], "windows") == 0) {
    format_windows_summary(response, response_len);
    return ESP_OK;
  }

  if (token_count == 1U && strcmp(tokens[0], "curtain") == 0) {
    format_curtain_summary(response, response_len);
    return ESP_OK;
  }

  if (token_count == 2U && strcmp(tokens[0], "light") == 0) {
    size_t relay_index = 0U;
    if (!parse_light_relay_ascii(tokens[1], &relay_index)) {
      snprintf(response, response_len, "ERR unknown relay");
      return ESP_ERR_INVALID_ARG;
    }

    autonomous_ctrl_cfg_t cfg = {0};
    modbus_mode_state_t mode = MODBUS_MODE_REMOTE;
    modbus_mode_reason_t reason = MODBUS_REASON_NONE;
    snapshot_autonomous_state(&cfg, &mode, &reason);

    char on_hhmm[6] = {0};
    char off_hhmm[6] = {0};
    format_hhmm_ascii(cfg.light.relay[relay_index].schedule.on_hhmm, on_hhmm,
                      sizeof(on_hhmm));
    format_hhmm_ascii(cfg.light.relay[relay_index].schedule.off_hhmm,
                      off_hhmm, sizeof(off_hhmm));
    snprintf(response, response_len,
             "OK light %s en=%u on=%s off=%s hyst=%u mode=%s",
             (relay_index == 0U) ? "r1" : "r2",
             (unsigned)cfg.light.relay[relay_index].schedule.enable, on_hhmm,
             off_hhmm, (unsigned)cfg.light.hyst_sec, mode_to_string(mode));
    return ESP_OK;
  }

  snprintf(response, response_len, "ERR unsupported show command");
  return ESP_ERR_NOT_SUPPORTED;
}

static esp_err_t handle_ascii_set_command(size_t token_count, char *tokens[],
                                          char *response,
                                          size_t response_len) {
  if (token_count == 0U || response == NULL || response_len == 0U) {
    return ESP_ERR_INVALID_ARG;
  }

  esp_err_t short_alias_result =
      handle_ascii_short_alias_command(token_count, tokens, response,
                                       response_len);
  if (short_alias_result != ESP_ERR_NOT_SUPPORTED) {
    return short_alias_result;
  }

  if (token_count == 3U && strcmp(tokens[0], "weather") == 0) {
    const char *alias_name = NULL;
    if (strcmp(tokens[1], "temp") == 0 || strcmp(tokens[1], "out_temp") == 0) {
      alias_name = "wx_temp";
    } else if (strcmp(tokens[1], "hum") == 0 ||
               strcmp(tokens[1], "out_hum") == 0) {
      alias_name = "wx_hum";
    } else if (strcmp(tokens[1], "wind") == 0 ||
               strcmp(tokens[1], "wind_speed") == 0) {
      alias_name = "wx_wind";
    } else if (strcmp(tokens[1], "dir") == 0 ||
               strcmp(tokens[1], "wind_dir") == 0) {
      alias_name = "wx_dir";
    } else if (strcmp(tokens[1], "rain") == 0) {
      alias_name = "wx_rain";
    } else if (strcmp(tokens[1], "solar") == 0) {
      alias_name = "wx_solar";
    } else if (strcmp(tokens[1], "baro") == 0 ||
               strcmp(tokens[1], "baro_press") == 0) {
      alias_name = "wx_baro";
    } else if (strcmp(tokens[1], "dew") == 0 ||
               strcmp(tokens[1], "dew_point") == 0) {
      alias_name = "wx_dew";
    } else if (strcmp(tokens[1], "age") == 0) {
      alias_name = "wx_age";
    } else if (strcmp(tokens[1], "status") == 0 ||
               strcmp(tokens[1], "status_bits") == 0) {
      alias_name = "wx_stat";
    }

    if (alias_name != NULL) {
      char *alias_tokens[2] = {(char *)alias_name, tokens[2]};
      return handle_ascii_weather_short_alias_command(2U, alias_tokens, response,
                                                      response_len);
    }

    snprintf(response, response_len, "ERR unknown weather field");
    return ESP_ERR_INVALID_ARG;
  }

  autonomous_ctrl_cfg_t cfg = {0};
  modbus_mode_state_t mode_snapshot = MODBUS_MODE_REMOTE;
  modbus_mode_reason_t reason_snapshot = MODBUS_REASON_NONE;
  snapshot_autonomous_state(&cfg, &mode_snapshot, &reason_snapshot);
  (void)reason_snapshot;

  if (token_count == 5U && strcmp(tokens[0], "windows") == 0 &&
      strcmp(tokens[2], "pos") == 0 && strcmp(tokens[3], "target") == 0) {
    uint16_t percent_tenths = 0U;
    if (!parse_tenths_ascii(tokens[4], 0.0f, 100.0f, &percent_tenths)) {
      snprintf(response, response_len, "ERR invalid percent value");
      return ESP_ERR_INVALID_ARG;
    }
    if (mode_snapshot != MODBUS_MODE_AUTONOMOUS) {
      snprintf(response, response_len,
               "ERR windows target is available only in AUTONOMOUS mode");
      return ESP_ERR_INVALID_STATE;
    }

    if (strcmp(tokens[1], "a") == 0) {
      cfg.windows_pos_a_target = percent_tenths;
      modbus_write_holding_reg(MODBUS_HREG_WINDOWS_POS_A_TARGET, percent_tenths);
      char ok_text[64] = {0};
      snprintf(ok_text, sizeof(ok_text), "windows a pos target=%.1f%%",
               ((float)percent_tenths) / 10.0f);
      return apply_ascii_autonomous_update(&cfg, response, response_len,
                                           ok_text);
    }
    if (strcmp(tokens[1], "b") == 0) {
      cfg.windows_pos_b_target = percent_tenths;
      modbus_write_holding_reg(MODBUS_HREG_WINDOWS_POS_B_TARGET, percent_tenths);
      char ok_text[64] = {0};
      snprintf(ok_text, sizeof(ok_text), "windows b pos target=%.1f%%",
               ((float)percent_tenths) / 10.0f);
      return apply_ascii_autonomous_update(&cfg, response, response_len,
                                           ok_text);
    }

    snprintf(response, response_len, "ERR unknown window channel");
    return ESP_ERR_INVALID_ARG;
  }

  if (token_count == 4U && strcmp(tokens[0], "curtain") == 0 &&
      strcmp(tokens[1], "pos") == 0 && strcmp(tokens[2], "target") == 0) {
    uint16_t percent_tenths = 0U;
    if (!parse_tenths_ascii(tokens[3], 0.0f, 100.0f, &percent_tenths)) {
      snprintf(response, response_len, "ERR invalid percent value");
      return ESP_ERR_INVALID_ARG;
    }
    cfg.curtain_pos_target = percent_tenths;
    char ok_text[64] = {0};
    snprintf(ok_text, sizeof(ok_text), "curtain pos target=%.1f%%",
             ((float)percent_tenths) / 10.0f);
    return apply_ascii_autonomous_update(&cfg, response, response_len, ok_text);
  }

  if (token_count == 4U && strcmp(tokens[0], "sp") == 0 &&
      strcmp(tokens[1], "water") == 0) {
    uint16_t temp_tenths = 0U;
    if (!parse_tenths_ascii(tokens[3], 0.0f, 120.0f, &temp_tenths)) {
      snprintf(response, response_len, "ERR invalid temperature value");
      return ESP_ERR_INVALID_ARG;
    }

    if (strcmp(tokens[2], "rail") == 0) {
      cfg.sp_water_rail = temp_tenths;
    } else if (strcmp(tokens[2], "grow") == 0) {
      cfg.sp_water_grow = temp_tenths;
    } else if (strcmp(tokens[2], "upper") == 0) {
      cfg.sp_water_upper = temp_tenths;
    } else if (strcmp(tokens[2], "undertray") == 0) {
      cfg.sp_water_undertray = temp_tenths;
    } else {
      snprintf(response, response_len, "ERR unknown water setpoint");
      return ESP_ERR_INVALID_ARG;
    }

    char ok_text[64] = {0};
    snprintf(ok_text, sizeof(ok_text), "sp water %s=%.1fC", tokens[2],
             ((float)temp_tenths) / 10.0f);
    return apply_ascii_autonomous_update(&cfg, response, response_len, ok_text);
  }

  if (token_count >= 3U && strcmp(tokens[0], "light") == 0 &&
      strcmp(tokens[1], "hyst") == 0) {
    const char *value_token = NULL;
    if (token_count == 4U && strcmp(tokens[2], "sec") == 0) {
      value_token = tokens[3];
    } else if (token_count == 3U) {
      value_token = tokens[2];
    }

    if (value_token == NULL) {
      snprintf(response, response_len, "ERR expected light hyst sec <value>");
      return ESP_ERR_INVALID_ARG;
    }

    uint16_t hyst_sec = 0U;
    if (!parse_u16_ascii(value_token, UINT16_MAX, &hyst_sec)) {
      snprintf(response, response_len, "ERR invalid hysteresis value");
      return ESP_ERR_INVALID_ARG;
    }

    cfg.light.hyst_sec = hyst_sec;
    char ok_text[64] = {0};
    snprintf(ok_text, sizeof(ok_text), "light hyst sec=%u", (unsigned)hyst_sec);
    return apply_ascii_autonomous_update(&cfg, response, response_len, ok_text);
  }

  if (token_count >= 4U && strcmp(tokens[0], "light") == 0) {
    size_t relay_index = 0U;
    if (!parse_light_relay_ascii(tokens[1], &relay_index)) {
      snprintf(response, response_len, "ERR unknown relay");
      return ESP_ERR_INVALID_ARG;
    }

    light_relay_cfg_t *relay_cfg = &cfg.light.relay[relay_index];

    if (strcmp(tokens[2], "enable") == 0 && token_count == 4U) {
      uint16_t enable_value = 0U;
      if (!parse_switch_ascii(tokens[3], &enable_value)) {
        snprintf(response, response_len, "ERR invalid enable value");
        return ESP_ERR_INVALID_ARG;
      }
      relay_cfg->schedule.enable = enable_value;
      char ok_text[64] = {0};
      snprintf(ok_text, sizeof(ok_text), "light %s enable=%u",
               (relay_index == 0U) ? "r1" : "r2", (unsigned)enable_value);
      return apply_ascii_autonomous_update(&cfg, response, response_len,
                                           ok_text);
    }

    if (strcmp(tokens[2], "on") == 0 && token_count == 5U &&
        strcmp(tokens[3], "hhmm") == 0) {
      uint16_t hhmm = 0U;
      if (!parse_hhmm_ascii(tokens[4], &hhmm)) {
        snprintf(response, response_len, "ERR invalid HH:MM value");
        return ESP_ERR_INVALID_ARG;
      }
      relay_cfg->schedule.on_hhmm = hhmm;
      char hhmm_text[6] = {0};
      char ok_text[64] = {0};
      format_hhmm_ascii(hhmm, hhmm_text, sizeof(hhmm_text));
      snprintf(ok_text, sizeof(ok_text), "light %s on hhmm=%s",
               (relay_index == 0U) ? "r1" : "r2", hhmm_text);
      return apply_ascii_autonomous_update(&cfg, response, response_len,
                                           ok_text);
    }

    if (strcmp(tokens[2], "off") == 0 && token_count == 5U &&
        strcmp(tokens[3], "hhmm") == 0) {
      uint16_t hhmm = 0U;
      if (!parse_hhmm_ascii(tokens[4], &hhmm)) {
        snprintf(response, response_len, "ERR invalid HH:MM value");
        return ESP_ERR_INVALID_ARG;
      }
      relay_cfg->schedule.off_hhmm = hhmm;
      char hhmm_text[6] = {0};
      char ok_text[64] = {0};
      format_hhmm_ascii(hhmm, hhmm_text, sizeof(hhmm_text));
      snprintf(ok_text, sizeof(ok_text), "light %s off hhmm=%s",
               (relay_index == 0U) ? "r1" : "r2", hhmm_text);
      return apply_ascii_autonomous_update(&cfg, response, response_len,
                                           ok_text);
    }

    if (strcmp(tokens[2], "threshold") == 0 ||
        strcmp(tokens[2], "dli") == 0) {
      snprintf(response, response_len,
               "ERR setpoint not used in autonomous mode");
      return ESP_ERR_NOT_SUPPORTED;
    }
  }

  snprintf(response, response_len, "ERR unsupported set command");
  return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t modbus_handle_ascii_command(const char *line, char *response,
                                      size_t response_len) {
  if (line == NULL || response == NULL || response_len == 0U) {
    return ESP_ERR_INVALID_ARG;
  }

  response[0] = '\0';
  size_t raw_len = strlen(line);
  if (raw_len >= MODBUS_ASCII_LINE_MAX) {
    snprintf(response, response_len, "ERR command too long");
    return ESP_ERR_INVALID_SIZE;
  }

  char local_line[MODBUS_ASCII_LINE_MAX] = {0};
  memcpy(local_line, line, raw_len + 1U);
  trim_ascii_whitespace(local_line);
  if (local_line[0] == '\0') {
    snprintf(response, response_len, "ERR empty command");
    return ESP_ERR_INVALID_ARG;
  }

  char *tokens[MODBUS_ASCII_MAX_TOKENS] = {0};
  size_t token_count =
      tokenize_ascii_command(local_line, tokens, MODBUS_ASCII_MAX_TOKENS);
  if (token_count == 0U) {
    snprintf(response, response_len, "ERR empty command");
    return ESP_ERR_INVALID_ARG;
  }
  if (token_count > MODBUS_ASCII_MAX_TOKENS) {
    snprintf(response, response_len, "ERR too many tokens");
    return ESP_ERR_INVALID_SIZE;
  }

  if (strcmp(tokens[0], "help") == 0 || strcmp(tokens[0], "?") == 0) {
    snprintf(response, response_len,
             "OK show: autonomous mode light weather windows sensors; set: win_a_pos win_b_pos "
             "curt_pos sp_rail sp_grow sp_upper sp_under l1_en l1_on l1_off "
             "l2_en l2_on l2_off light_hyst win_mode win_alg temp_open "
             "hum_sp hum_step hum_hyst hum_open cold_close cold_hyst "
             "windward_min windward_max "
             "windward_thr windward_reduce leeward_min leeward_max leeward_thr "
             "leeward_reduce wind_lag rain_mode rain_pos wx_stale_ms wx_age_max "
             "wx_temp wx_hum wx_wind wx_dir wx_rain wx_solar wx_baro wx_dew "
             "wx_age wx_stat air_temp air_rh sensor_override");
    return ESP_OK;
  }

  if (strcmp(tokens[0], "show") == 0 || strcmp(tokens[0], "get") == 0) {
    return handle_ascii_show_command(token_count - 1U, &tokens[1], response,
                                     response_len);
  }

  if (strcmp(tokens[0], "set") == 0) {
    return handle_ascii_set_command(token_count - 1U, &tokens[1], response,
                                    response_len);
  }

  return handle_ascii_set_command(token_count, tokens, response, response_len);
}

static uint16_t modbus_read_holding_reg(uint16_t reg_index) {
  uint16_t value = 0U;
  if (reg_index >= MODBUS_HREG_TOTAL_COUNT) {
    return value;
  }

  if (s_mbc_slave_handler != NULL) {
    ESP_ERROR_CHECK(mbc_slave_lock(s_mbc_slave_handler));
    value = s_holding_regs[reg_index];
    ESP_ERROR_CHECK(mbc_slave_unlock(s_mbc_slave_handler));
  } else {
    value = s_holding_regs[reg_index];
  }
  return value;
}

static void modbus_write_holding_reg(uint16_t reg_index, uint16_t value) {
  if (reg_index >= MODBUS_HREG_TOTAL_COUNT) {
    return;
  }

  if (s_mbc_slave_handler != NULL) {
    ESP_ERROR_CHECK(mbc_slave_lock(s_mbc_slave_handler));
    s_holding_regs[reg_index] = value;
    ESP_ERROR_CHECK(mbc_slave_unlock(s_mbc_slave_handler));
  } else {
    s_holding_regs[reg_index] = value;
  }
}

static float modbus_raw_percent_to_float(uint16_t raw_percent_tenths) {
  if (raw_percent_tenths > 1000U) {
    raw_percent_tenths = 1000U;
  }
  return ((float)raw_percent_tenths) / 10.0f;
}

float modbus_get_window_a_target_percent(void) {
  if (modbus_get_mode_state() == MODBUS_MODE_AUTONOMOUS) {
    uint16_t raw_target = 0U;
    taskENTER_CRITICAL(&s_state_lock);
    raw_target = s_autonomous_cfg.windows_pos_a_target;
    taskEXIT_CRITICAL(&s_state_lock);
    return modbus_raw_percent_to_float(raw_target);
  }

  return modbus_raw_percent_to_float(
      modbus_read_holding_reg(MODBUS_HREG_WINDOWS_POS_A_TARGET));
}

float modbus_get_window_b_target_percent(void) {
  if (modbus_get_mode_state() == MODBUS_MODE_AUTONOMOUS) {
    uint16_t raw_target = 0U;
    taskENTER_CRITICAL(&s_state_lock);
    raw_target = s_autonomous_cfg.windows_pos_b_target;
    taskEXIT_CRITICAL(&s_state_lock);
    return modbus_raw_percent_to_float(raw_target);
  }

  return modbus_raw_percent_to_float(
      modbus_read_holding_reg(MODBUS_HREG_WINDOWS_POS_B_TARGET));
}

float modbus_get_curtain_target_percent(void) {
  if (modbus_get_mode_state() == MODBUS_MODE_AUTONOMOUS) {
    uint16_t raw_target = 0U;
    taskENTER_CRITICAL(&s_state_lock);
    raw_target = s_autonomous_cfg.curtain_pos_target;
    taskEXIT_CRITICAL(&s_state_lock);
    return modbus_raw_percent_to_float(raw_target);
  }

  return modbus_raw_percent_to_float(
      modbus_read_holding_reg(MODBUS_HREG_CURTAIN_POS_TARGET));
}

modbus_windows_ctrl_mode_t modbus_get_windows_ctrl_mode(void) {
  const uint16_t raw_mode = modbus_read_holding_reg(MODBUS_HREG_WINDOWS_CTRL_MODE);
  return (raw_mode == (uint16_t)MODBUS_WINDOWS_CTRL_MODE_MANUAL)
             ? MODBUS_WINDOWS_CTRL_MODE_MANUAL
             : MODBUS_WINDOWS_CTRL_MODE_AUTO;
}

bool modbus_get_windows_force_safe_cmd(void) {
  return modbus_read_holding_reg(MODBUS_HREG_WINDOWS_FORCE_SAFE_CMD) != 0U;
}

float modbus_get_air_temp_target_c(void) {
  return ((float)modbus_read_holding_reg(MODBUS_HREG_AIR_TEMP_TARGET)) / 10.0f;
}

float modbus_get_windows_temp_setpoint_c(void) {
  return ((float)modbus_read_holding_reg(MODBUS_HREG_WINDOWS_TEMP_SETPOINT)) /
         10.0f;
}

float modbus_get_windows_safe_min_percent(void) {
  return modbus_raw_percent_to_float(
      modbus_read_holding_reg(MODBUS_HREG_WINDOWS_SAFE_MIN_PERCENT));
}

float modbus_get_windows_wind_limit_ms(void) {
  return ((float)modbus_read_holding_reg(MODBUS_HREG_WINDOWS_WIND_LIMIT)) / 10.0f;
}

float modbus_get_windows_wind_storm_ms(void) {
  return ((float)modbus_read_holding_reg(MODBUS_HREG_WINDOWS_WIND_STORM)) / 10.0f;
}

float modbus_get_windows_wind_recover_ms(void) {
  return ((float)modbus_read_holding_reg(MODBUS_HREG_WINDOWS_WIND_RECOVER)) / 10.0f;
}

uint16_t modbus_get_window_a_azimuth_deg(void) {
  const uint16_t raw = modbus_read_holding_reg(MODBUS_HREG_WINDOW_A_AZIMUTH_DEG);
  return (uint16_t)(raw % 360U);
}

uint16_t modbus_get_windows_wind_sector_half_width_deg(void) {
  uint16_t raw =
      modbus_read_holding_reg(MODBUS_HREG_WINDOWS_WIND_SECTOR_HALF_WIDTH_DEG);
  if (raw > 180U) {
    raw = 180U;
  }
  return raw;
}

float modbus_get_windows_temp_step_c(void) {
  return ((float)modbus_read_holding_reg(MODBUS_HREG_WINDOWS_TEMP_STEP_C)) / 10.0f;
}

float modbus_get_windows_temp_step_hysteresis_c(void) {
  return ((float)modbus_read_holding_reg(MODBUS_HREG_WINDOWS_TEMP_STEP_HYST_C)) /
         10.0f;
}

float modbus_get_windows_temp_step_target_increment_percent(void) {
  return modbus_raw_percent_to_float(
      modbus_read_holding_reg(MODBUS_HREG_WINDOWS_TEMP_STEP_TARGET_PERCENT));
}

uint16_t modbus_get_windows_temp_step_max_index(void) {
  return modbus_read_holding_reg(MODBUS_HREG_WINDOWS_TEMP_STEP_MAX_INDEX);
}

modbus_windows_auto_algo_mode_t modbus_get_windows_auto_algo_mode(void) {
  const uint16_t raw =
      modbus_read_holding_reg(MODBUS_HREG_WINDOWS_AUTO_ALGO_MODE);
  return (raw == (uint16_t)MODBUS_WINDOWS_AUTO_ALGO_HUMIDITY)
             ? MODBUS_WINDOWS_AUTO_ALGO_HUMIDITY
             : MODBUS_WINDOWS_AUTO_ALGO_TEMP;
}

float modbus_get_air_hum_target_percent(void) {
  return modbus_raw_percent_to_float(
      modbus_read_holding_reg(MODBUS_HREG_AIR_HUM_TARGET));
}

float modbus_get_windows_humidity_setpoint_percent(void) {
  return modbus_raw_percent_to_float(
      modbus_read_holding_reg(MODBUS_HREG_WINDOWS_HUM_SETPOINT));
}

float modbus_get_windows_humidity_step_percent(void) {
  return ((float)modbus_read_holding_reg(MODBUS_HREG_WINDOWS_HUM_STEP)) / 10.0f;
}

float modbus_get_windows_humidity_step_hysteresis_percent(void) {
  return ((float)modbus_read_holding_reg(MODBUS_HREG_WINDOWS_HUM_STEP_HYST)) /
         10.0f;
}

float modbus_get_windows_humidity_step_target_increment_percent(void) {
  return modbus_raw_percent_to_float(
      modbus_read_holding_reg(MODBUS_HREG_WINDOWS_HUM_STEP_TARGET_PERCENT));
}

uint16_t modbus_get_windows_humidity_step_max_index(void) {
  return modbus_read_holding_reg(MODBUS_HREG_WINDOWS_HUM_STEP_MAX_INDEX);
}

float modbus_get_windows_cold_close_delta_c(void) {
  return ((float)modbus_read_holding_reg(MODBUS_HREG_WINDOWS_COLD_CLOSE_DELTA)) /
         10.0f;
}

float modbus_get_windows_cold_close_hysteresis_c(void) {
  return ((float)modbus_read_holding_reg(MODBUS_HREG_WINDOWS_COLD_CLOSE_HYST)) /
         10.0f;
}

float modbus_get_windows_windward_min_percent(void) {
  return modbus_raw_percent_to_float(
      modbus_read_holding_reg(MODBUS_HREG_WINDOWS_WINDWARD_MIN_PERCENT));
}

float modbus_get_windows_windward_max_percent(void) {
  return modbus_raw_percent_to_float(
      modbus_read_holding_reg(MODBUS_HREG_WINDOWS_WINDWARD_MAX_PERCENT));
}

float modbus_get_windows_windward_speed_threshold_ms(void) {
  return ((float)modbus_read_holding_reg(
              MODBUS_HREG_WINDOWS_WINDWARD_SPEED_THRESHOLD)) /
         10.0f;
}

float modbus_get_windows_windward_reduction_percent_per_ms(void) {
  return ((float)modbus_read_holding_reg(
              MODBUS_HREG_WINDOWS_WINDWARD_REDUCTION_PERCENT_PER_MS)) /
         10.0f;
}

float modbus_get_windows_leeward_min_percent(void) {
  return modbus_raw_percent_to_float(
      modbus_read_holding_reg(MODBUS_HREG_WINDOWS_LEEWARD_MIN_PERCENT));
}

float modbus_get_windows_leeward_max_percent(void) {
  return modbus_raw_percent_to_float(
      modbus_read_holding_reg(MODBUS_HREG_WINDOWS_LEEWARD_MAX_PERCENT));
}

float modbus_get_windows_leeward_speed_threshold_ms(void) {
  return ((float)modbus_read_holding_reg(
              MODBUS_HREG_WINDOWS_LEEWARD_SPEED_THRESHOLD)) /
         10.0f;
}

float modbus_get_windows_leeward_reduction_percent_per_ms(void) {
  return ((float)modbus_read_holding_reg(
              MODBUS_HREG_WINDOWS_LEEWARD_REDUCTION_PERCENT_PER_MS)) /
         10.0f;
}

float modbus_get_windows_windward_lag_percent(void) {
  return modbus_raw_percent_to_float(
      modbus_read_holding_reg(MODBUS_HREG_WINDOWS_WINDWARD_LAG_PERCENT));
}

modbus_windows_rain_mode_t modbus_get_windows_rain_mode(void) {
  const uint16_t raw = modbus_read_holding_reg(MODBUS_HREG_WINDOWS_RAIN_MODE);
  return (raw == (uint16_t)MODBUS_WINDOWS_RAIN_MODE_WINDWARD)
             ? MODBUS_WINDOWS_RAIN_MODE_WINDWARD
             : MODBUS_WINDOWS_RAIN_MODE_DISABLED;
}

float modbus_get_windows_rain_windward_percent(void) {
  return modbus_raw_percent_to_float(
      modbus_read_holding_reg(MODBUS_HREG_WINDOWS_RAIN_WINDWARD_PERCENT));
}

modbus_windows_weather_stale_policy_t
modbus_get_windows_weather_stale_policy(void) {
  const uint16_t raw =
      modbus_read_holding_reg(MODBUS_HREG_WINDOWS_WEATHER_STALE_POLICY);
  return (raw == (uint16_t)MODBUS_WINDOWS_WEATHER_STALE_IGNORE)
             ? MODBUS_WINDOWS_WEATHER_STALE_IGNORE
             : MODBUS_WINDOWS_WEATHER_STALE_CLOSE_SAFE;
}

uint32_t modbus_get_windows_weather_stale_timeout_ms(void) {
  return (uint32_t)modbus_read_holding_reg(
      MODBUS_HREG_WINDOWS_WEATHER_STALE_TIMEOUT_MS);
}

uint16_t modbus_get_windows_weather_source_age_limit_s(void) {
  return modbus_read_holding_reg(MODBUS_HREG_WINDOWS_WEATHER_SOURCE_AGE_S);
}

float modbus_get_rll400_target_hysteresis_percent(void) {
  return ((float)modbus_read_holding_reg(MODBUS_HREG_RLL400_TARGET_HYST_PERCENT)) /
         10.0f;
}

float modbus_get_rll400_motion_delta_percent(void) {
  return ((float)modbus_read_holding_reg(MODBUS_HREG_RLL400_MOTION_DELTA_PERCENT)) /
         10.0f;
}

uint32_t modbus_get_rll400_no_motion_timeout_ms(void) {
  return (uint32_t)modbus_read_holding_reg(MODBUS_HREG_RLL400_NO_MOTION_TIMEOUT_MS);
}

uint16_t modbus_get_window_fault_reset_token(modbus_window_channel_t channel) {
  return modbus_read_holding_reg((channel == MODBUS_WINDOW_CHANNEL_B)
                                     ? MODBUS_HREG_WINDOW_B_FAULT_RESET_TOKEN
                                     : MODBUS_HREG_WINDOW_A_FAULT_RESET_TOKEN);
}

void modbus_get_weather_runtime(modbus_weather_runtime_t *out_runtime) {
  if (out_runtime == NULL) {
    return;
  }

  weather_snapshot_t snapshot = {0};
  uint32_t last_rx_ms = 0U;
  bool valid = false;
  bool stale = true;

  taskENTER_CRITICAL(&s_state_lock);
  snapshot = s_weather_active_snapshot;
  last_rx_ms = s_weather_last_rx_ms;
  valid = s_weather_valid;
  stale = s_weather_stale;
  taskEXIT_CRITICAL(&s_state_lock);

  memset(out_runtime, 0, sizeof(*out_runtime));
  out_runtime->wind_speed_ms = ((float)snapshot.wind_speed) / 10.0f;
  out_runtime->wind_dir_deg = (uint16_t)(snapshot.wind_dir % 360U);
  out_runtime->source_age_s = snapshot.source_age_s;
  out_runtime->status_bits = snapshot.status_bits;
  out_runtime->solar_radiation_wm2 = (float)snapshot.solar_rad;
  out_runtime->valid = valid;
  out_runtime->stale = stale;
  out_runtime->rain_active = (snapshot.rain_flag != 0U);
  if (valid && last_rx_ms != 0U) {
    const uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
    out_runtime->rx_age_ms = now_ms - last_rx_ms;
  }
}

void modbus_get_sensor_test_override(
    modbus_sensor_test_override_t *out_override) {
  if (out_override == NULL) {
    return;
  }

  modbus_sensor_test_override_t local = {0};
  taskENTER_CRITICAL(&s_state_lock);
  local.air_temp_override_active = s_air_temp_override_active;
  local.rh_override_active = s_rh_override_active;
  local.air_temp_c = ((float)s_air_temp_override_tenths) / 10.0f;
  local.rh_percent = ((float)s_rh_override_tenths) / 10.0f;
  taskEXIT_CRITICAL(&s_state_lock);

  *out_override = local;
}

void modbus_set_windows_runtime(uint16_t windows_status_bits,
                                uint16_t window_a_status_bits,
                                uint16_t window_b_status_bits,
                                uint16_t window_a_fault_code,
                                uint16_t window_b_fault_code,
                                uint16_t air_temp_sensor_status,
                                uint16_t window_a_local_manual_active,
                                uint16_t window_b_local_manual_active) {
  modbus_write_holding_reg(MODBUS_HREG_WINDOWS_STATUS_BITS, windows_status_bits);
  modbus_write_holding_reg(MODBUS_HREG_WINDOW_A_STATUS_BITS,
                           window_a_status_bits);
  modbus_write_holding_reg(MODBUS_HREG_WINDOW_B_STATUS_BITS,
                           window_b_status_bits);
  modbus_write_holding_reg(MODBUS_HREG_WINDOW_A_FAULT_CODE,
                           window_a_fault_code);
  modbus_write_holding_reg(MODBUS_HREG_WINDOW_B_FAULT_CODE,
                           window_b_fault_code);
  modbus_write_holding_reg(MODBUS_HREG_AIR_TEMP_SENSOR_STATUS,
                           air_temp_sensor_status);
  modbus_write_holding_reg(MODBUS_HREG_WINDOW_A_LOCAL_MANUAL_ACTIVE,
                           window_a_local_manual_active);
  modbus_write_holding_reg(MODBUS_HREG_WINDOW_B_LOCAL_MANUAL_ACTIVE,
                           window_b_local_manual_active);
}

void modbus_set_windows_target_diagnostics(
    float base_target_a_percent, float base_target_b_percent,
    float effective_target_a_percent, float effective_target_b_percent,
    uint16_t active_protection_bits,
    modbus_windows_windward_side_t windward_side) {
  modbus_write_holding_reg(
      MODBUS_HREG_WINDOWS_BASE_TARGET_A,
      float_to_u16_tenths(base_target_a_percent, 0.0f, 100.0f));
  modbus_write_holding_reg(
      MODBUS_HREG_WINDOWS_BASE_TARGET_B,
      float_to_u16_tenths(base_target_b_percent, 0.0f, 100.0f));
  modbus_write_holding_reg(
      MODBUS_HREG_WINDOWS_EFFECTIVE_TARGET_A,
      float_to_u16_tenths(effective_target_a_percent, 0.0f, 100.0f));
  modbus_write_holding_reg(
      MODBUS_HREG_WINDOWS_EFFECTIVE_TARGET_B,
      float_to_u16_tenths(effective_target_b_percent, 0.0f, 100.0f));
  modbus_write_holding_reg(MODBUS_HREG_WINDOWS_ACTIVE_PROTECTION_BITS,
                           active_protection_bits);
  modbus_write_holding_reg(MODBUS_HREG_WINDOWS_WINDWARD_SIDE,
                           (uint16_t)windward_side);
}

modbus_curtain_ctrl_mode_t modbus_get_curtain_ctrl_mode(void) {
  const uint16_t raw = modbus_read_holding_reg(MODBUS_HREG_CURTAIN_CTRL_MODE);
  if (raw == (uint16_t)MODBUS_CURTAIN_CTRL_MODE_AUTO) {
    return MODBUS_CURTAIN_CTRL_MODE_AUTO;
  }
  if (raw == (uint16_t)MODBUS_CURTAIN_CTRL_MODE_OFF) {
    return MODBUS_CURTAIN_CTRL_MODE_OFF;
  }
  return MODBUS_CURTAIN_CTRL_MODE_MANUAL;
}

float modbus_get_curtain_manual_target_percent(void) {
  const uint16_t raw = modbus_read_holding_reg(MODBUS_HREG_CURTAIN_MANUAL_TARGET);
  if (raw != MODBUS_CURTAIN_DEFAULT_MANUAL_TARGET ||
      modbus_read_holding_reg(MODBUS_HREG_CURTAIN_POS_TARGET) ==
          MODBUS_CURTAIN_DEFAULT_MANUAL_TARGET) {
    return modbus_raw_percent_to_float(raw);
  }
  return modbus_get_curtain_target_percent();
}

uint16_t modbus_get_curtain_schedule_start_hhmm(void) {
  return modbus_read_holding_reg(MODBUS_HREG_CURTAIN_SCHEDULE_START_HHMM);
}

uint16_t modbus_get_curtain_schedule_end_hhmm(void) {
  return modbus_read_holding_reg(MODBUS_HREG_CURTAIN_SCHEDULE_END_HHMM);
}

float modbus_get_curtain_outside_target_percent(void) {
  return modbus_raw_percent_to_float(
      modbus_read_holding_reg(MODBUS_HREG_CURTAIN_OUTSIDE_TARGET));
}

float modbus_get_curtain_min_position_percent(void) {
  return modbus_raw_percent_to_float(
      modbus_read_holding_reg(MODBUS_HREG_CURTAIN_MIN_POSITION));
}

float modbus_get_curtain_max_position_percent(void) {
  return modbus_raw_percent_to_float(
      modbus_read_holding_reg(MODBUS_HREG_CURTAIN_MAX_POSITION));
}

float modbus_get_curtain_position_hysteresis_percent(void) {
  return modbus_raw_percent_to_float(
      modbus_read_holding_reg(MODBUS_HREG_CURTAIN_POSITION_HYST));
}

uint16_t modbus_get_curtain_radiation_threshold_wm2(void) {
  return modbus_read_holding_reg(MODBUS_HREG_CURTAIN_RADIATION_THRESHOLD);
}

uint16_t modbus_get_curtain_radiation_step_wm2(void) {
  return modbus_read_holding_reg(MODBUS_HREG_CURTAIN_RADIATION_STEP_WM2);
}

float modbus_get_curtain_radiation_step_percent(void) {
  return modbus_raw_percent_to_float(
      modbus_read_holding_reg(MODBUS_HREG_CURTAIN_RADIATION_STEP_PERCENT));
}

uint16_t modbus_get_curtain_radiation_hysteresis_wm2(void) {
  return modbus_read_holding_reg(MODBUS_HREG_CURTAIN_RADIATION_HYST);
}

float modbus_get_curtain_cold_delta_c(void) {
  return ((float)modbus_read_holding_reg(MODBUS_HREG_CURTAIN_COLD_DELTA)) /
         10.0f;
}

float modbus_get_curtain_cold_hysteresis_c(void) {
  return ((float)modbus_read_holding_reg(MODBUS_HREG_CURTAIN_COLD_HYST)) /
         10.0f;
}

float modbus_get_curtain_cold_target_percent(void) {
  return modbus_raw_percent_to_float(
      modbus_read_holding_reg(MODBUS_HREG_CURTAIN_COLD_TARGET));
}

float modbus_get_curtain_heat_delta_c(void) {
  return ((float)modbus_read_holding_reg(MODBUS_HREG_CURTAIN_HEAT_DELTA)) /
         10.0f;
}

float modbus_get_curtain_heat_hysteresis_c(void) {
  return ((float)modbus_read_holding_reg(MODBUS_HREG_CURTAIN_HEAT_HYST)) /
         10.0f;
}

float modbus_get_curtain_heat_target_percent(void) {
  return modbus_raw_percent_to_float(
      modbus_read_holding_reg(MODBUS_HREG_CURTAIN_HEAT_TARGET));
}

float modbus_get_curtain_humidity_low_threshold_percent(void) {
  return modbus_raw_percent_to_float(
      modbus_read_holding_reg(MODBUS_HREG_CURTAIN_HUM_LOW_THRESHOLD));
}

float modbus_get_curtain_humidity_low_hysteresis_percent(void) {
  return modbus_raw_percent_to_float(
      modbus_read_holding_reg(MODBUS_HREG_CURTAIN_HUM_LOW_HYST));
}

float modbus_get_curtain_humidity_low_target_percent(void) {
  return modbus_raw_percent_to_float(
      modbus_read_holding_reg(MODBUS_HREG_CURTAIN_HUM_LOW_TARGET));
}

float modbus_get_curtain_humidity_high_threshold_percent(void) {
  return modbus_raw_percent_to_float(
      modbus_read_holding_reg(MODBUS_HREG_CURTAIN_HUM_HIGH_THRESHOLD));
}

float modbus_get_curtain_humidity_high_hysteresis_percent(void) {
  return modbus_raw_percent_to_float(
      modbus_read_holding_reg(MODBUS_HREG_CURTAIN_HUM_HIGH_HYST));
}

float modbus_get_curtain_humidity_high_target_percent(void) {
  return modbus_raw_percent_to_float(
      modbus_read_holding_reg(MODBUS_HREG_CURTAIN_HUM_HIGH_TARGET));
}

uint16_t modbus_get_curtain_fault_reset_token(void) {
  return modbus_read_holding_reg(MODBUS_HREG_CURTAIN_FAULT_RESET_TOKEN);
}

void modbus_set_curtain_runtime(float target_percent, float base_target_percent,
                                float current_ma, uint16_t status_bits,
                                uint16_t reason_bits,
                                uint16_t position_status_bits,
                                uint16_t fault_code) {
  modbus_write_holding_reg(
      MODBUS_HREG_CURTAIN_TARGET,
      float_to_u16_tenths(target_percent, 0.0f, 100.0f));
  modbus_write_holding_reg(
      MODBUS_HREG_CURTAIN_BASE_TARGET,
      float_to_u16_tenths(base_target_percent, 0.0f, 100.0f));
  modbus_write_holding_reg(
      MODBUS_HREG_CURTAIN_CURRENT_MA,
      float_to_u16_tenths(current_ma, 0.0f, 30.0f));
  modbus_write_holding_reg(MODBUS_HREG_CURTAIN_STATUS_BITS, status_bits);
  modbus_write_holding_reg(MODBUS_HREG_CURTAIN_REASON_BITS, reason_bits);
  modbus_write_holding_reg(MODBUS_HREG_CURTAIN_POSITION_STATUS_BITS,
                           position_status_bits);
  modbus_write_holding_reg(MODBUS_HREG_CURTAIN_FAULT_CODE, fault_code);
}

static uint16_t modbus_get_autonomous_water_setpoint_raw(
    modbus_water_channel_t channel) {
  uint16_t raw_target = 0U;
  taskENTER_CRITICAL(&s_state_lock);
  switch (channel) {
  case MODBUS_WATER_CHANNEL_RAIL:
    raw_target = s_autonomous_cfg.sp_water_rail;
    break;
  case MODBUS_WATER_CHANNEL_GROW:
    raw_target = s_autonomous_cfg.sp_water_grow;
    break;
  case MODBUS_WATER_CHANNEL_UPPER:
    raw_target = s_autonomous_cfg.sp_water_upper;
    break;
  case MODBUS_WATER_CHANNEL_UNDERTRAY:
    raw_target = s_autonomous_cfg.sp_water_undertray;
    break;
  case MODBUS_WATER_CHANNEL_COUNT:
  default:
    raw_target = 0U;
    break;
  }
  taskEXIT_CRITICAL(&s_state_lock);
  return raw_target;
}

static uint16_t modbus_get_remote_water_setpoint_raw(modbus_water_channel_t channel) {
  uint16_t raw_target = 0U;

  if (s_mbc_slave_handler == NULL) {
    return raw_target;
  }

  ESP_ERROR_CHECK(mbc_slave_lock(s_mbc_slave_handler));
  switch (channel) {
  case MODBUS_WATER_CHANNEL_RAIL:
    raw_target = s_holding_regs[MODBUS_HREG_SP_WATER_RAIL];
    break;
  case MODBUS_WATER_CHANNEL_GROW:
    raw_target = s_holding_regs[MODBUS_HREG_SP_WATER_GROW];
    break;
  case MODBUS_WATER_CHANNEL_UPPER:
    raw_target = s_holding_regs[MODBUS_HREG_SP_WATER_UPPER];
    break;
  case MODBUS_WATER_CHANNEL_UNDERTRAY:
    raw_target = s_holding_regs[MODBUS_HREG_SP_WATER_UNDERTRAY];
    break;
  case MODBUS_WATER_CHANNEL_COUNT:
  default:
    raw_target = 0U;
    break;
  }
  ESP_ERROR_CHECK(mbc_slave_unlock(s_mbc_slave_handler));

  return raw_target;
}

float modbus_get_water_setpoint_c(modbus_water_channel_t channel) {
  uint16_t raw_target = 0U;

  if (modbus_get_mode_state() == MODBUS_MODE_AUTONOMOUS) {
    raw_target = modbus_get_autonomous_water_setpoint_raw(channel);
  } else {
    raw_target = modbus_get_remote_water_setpoint_raw(channel);
  }

  if (raw_target > 1200U) {
    raw_target = 1200U;
  }

  return ((float)raw_target) / 10.0f;
}

modbus_heating_ctrl_mode_t modbus_get_heating_ctrl_mode(void) {
  const uint16_t raw = modbus_read_holding_reg(MODBUS_HREG_HEATING_CTRL_MODE);
  if (raw == (uint16_t)MODBUS_HEATING_CTRL_MODE_MANUAL) {
    return MODBUS_HEATING_CTRL_MODE_MANUAL;
  }
  if (raw == (uint16_t)MODBUS_HEATING_CTRL_MODE_OFF) {
    return MODBUS_HEATING_CTRL_MODE_OFF;
  }
  return MODBUS_HEATING_CTRL_MODE_AUTO;
}

float modbus_get_heating_air_setpoint_c(void) {
  return ((float)modbus_read_holding_reg(MODBUS_HREG_HEATING_AIR_SETPOINT)) /
         10.0f;
}

float modbus_get_heating_air_hysteresis_c(void) {
  return ((float)modbus_read_holding_reg(MODBUS_HREG_HEATING_AIR_HYST)) / 10.0f;
}

float modbus_get_heating_stage_delta_c(uint8_t stage_index) {
  if (stage_index >= MODBUS_WATER_CHANNEL_COUNT) {
    return 0.0f;
  }
  return ((float)modbus_read_holding_reg(
              (uint16_t)(MODBUS_HREG_HEATING_STAGE_DELTA_1 + stage_index))) /
         10.0f;
}

uint16_t modbus_get_heating_min_on_s(void) {
  return modbus_read_holding_reg(MODBUS_HREG_HEATING_MIN_ON_S);
}

uint16_t modbus_get_heating_min_off_s(void) {
  return modbus_read_holding_reg(MODBUS_HREG_HEATING_MIN_OFF_S);
}

uint16_t modbus_get_heating_manual_pump_mask(void) {
  return modbus_read_holding_reg(MODBUS_HREG_HEATING_MANUAL_PUMP_MASK);
}

uint16_t modbus_get_heating_manual_valve_open_mask(void) {
  return modbus_read_holding_reg(MODBUS_HREG_HEATING_MANUAL_VALVE_OPEN_MASK);
}

uint16_t modbus_get_heating_manual_valve_close_mask(void) {
  return modbus_read_holding_reg(MODBUS_HREG_HEATING_MANUAL_VALVE_CLOSE_MASK);
}

void modbus_set_heating_runtime(uint16_t status_bits, uint16_t active_stage,
                                uint16_t pump_mask, uint16_t valve_open_mask,
                                uint16_t valve_close_mask,
                                uint16_t sensor_status_bits) {
  modbus_write_holding_reg(MODBUS_HREG_HEATING_STATUS_BITS, status_bits);
  modbus_write_holding_reg(MODBUS_HREG_HEATING_ACTIVE_STAGE, active_stage);
  modbus_write_holding_reg(MODBUS_HREG_HEATING_PUMP_MASK, pump_mask);
  modbus_write_holding_reg(MODBUS_HREG_HEATING_VALVE_OPEN_MASK,
                           valve_open_mask);
  modbus_write_holding_reg(MODBUS_HREG_HEATING_VALVE_CLOSE_MASK,
                           valve_close_mask);
  modbus_write_holding_reg(MODBUS_HREG_HEATING_SENSOR_STATUS_BITS,
                           sensor_status_bits);
}

uint16_t modbus_get_co2_measured_ppm(void) {
  return modbus_read_holding_reg(MODBUS_HREG_CO2_MEASURED_PPM);
}

bool modbus_get_co2_sensor_valid(void) {
  return modbus_read_holding_reg(MODBUS_HREG_CO2_SENSOR_VALID) != 0U;
}

modbus_co2_ctrl_mode_t modbus_get_co2_ctrl_mode(void) {
  const uint16_t raw = modbus_read_holding_reg(MODBUS_HREG_CO2_CTRL_MODE);
  if (raw == (uint16_t)MODBUS_CO2_CTRL_MODE_MANUAL) {
    return MODBUS_CO2_CTRL_MODE_MANUAL;
  }
  if (raw == (uint16_t)MODBUS_CO2_CTRL_MODE_AUTO) {
    return MODBUS_CO2_CTRL_MODE_AUTO;
  }
  return MODBUS_CO2_CTRL_MODE_OFF;
}

uint16_t modbus_get_co2_manual_outputs(void) {
  return modbus_read_holding_reg(MODBUS_HREG_CO2_MANUAL_OUTPUTS);
}

uint16_t modbus_get_co2_schedule_start_hhmm(void) {
  return modbus_read_holding_reg(MODBUS_HREG_CO2_SCHEDULE_START_HHMM);
}

uint16_t modbus_get_co2_schedule_end_hhmm(void) {
  return modbus_read_holding_reg(MODBUS_HREG_CO2_SCHEDULE_END_HHMM);
}

uint16_t modbus_get_co2_low_light_threshold_wm2(void) {
  return modbus_read_holding_reg(MODBUS_HREG_CO2_LOW_LIGHT_THRESHOLD_WM2);
}

uint16_t modbus_get_co2_mid_light_threshold_wm2(void) {
  return modbus_read_holding_reg(MODBUS_HREG_CO2_MID_LIGHT_THRESHOLD_WM2);
}

uint16_t modbus_get_co2_high_light_threshold_wm2(void) {
  return modbus_read_holding_reg(MODBUS_HREG_CO2_HIGH_LIGHT_THRESHOLD_WM2);
}

uint16_t modbus_get_co2_low_light_target_ppm(void) {
  return modbus_read_holding_reg(MODBUS_HREG_CO2_LOW_LIGHT_TARGET_PPM);
}

uint16_t modbus_get_co2_mid_light_target_ppm(void) {
  return modbus_read_holding_reg(MODBUS_HREG_CO2_MID_LIGHT_TARGET_PPM);
}

uint16_t modbus_get_co2_high_light_target_ppm(void) {
  return modbus_read_holding_reg(MODBUS_HREG_CO2_HIGH_LIGHT_TARGET_PPM);
}

static uint16_t modbus_get_co2_percent_reg(uint16_t reg_index) {
  uint16_t raw = modbus_read_holding_reg(reg_index);
  if (raw > 1000U) {
    raw = 1000U;
  }
  return (uint16_t)(raw / 10U);
}

uint16_t modbus_get_co2_vent_limit_low_percent(void) {
  return modbus_get_co2_percent_reg(MODBUS_HREG_CO2_VENT_LIMIT_LOW_PERCENT);
}

uint16_t modbus_get_co2_vent_limit_high_percent(void) {
  return modbus_get_co2_percent_reg(MODBUS_HREG_CO2_VENT_LIMIT_HIGH_PERCENT);
}

uint16_t modbus_get_co2_vent_cutoff_percent(void) {
  return modbus_get_co2_percent_reg(MODBUS_HREG_CO2_VENT_CUTOFF_PERCENT);
}

uint16_t modbus_get_co2_dosing_hysteresis_ppm(void) {
  return modbus_read_holding_reg(MODBUS_HREG_CO2_DOSING_HYST_PPM);
}

uint16_t modbus_get_co2_max_safe_ppm(void) {
  return modbus_read_holding_reg(MODBUS_HREG_CO2_MAX_SAFE_PPM);
}

uint16_t modbus_get_co2_max_dosing_time_s(void) {
  return modbus_read_holding_reg(MODBUS_HREG_CO2_MAX_DOSING_TIME_S);
}

uint16_t modbus_get_co2_min_pause_time_s(void) {
  return modbus_read_holding_reg(MODBUS_HREG_CO2_MIN_PAUSE_TIME_S);
}

uint16_t modbus_get_co2_no_rise_check_time_s(void) {
  return modbus_read_holding_reg(MODBUS_HREG_CO2_NO_RISE_CHECK_TIME_S);
}

uint16_t modbus_get_co2_no_rise_min_delta_ppm(void) {
  return modbus_read_holding_reg(MODBUS_HREG_CO2_NO_RISE_MIN_DELTA_PPM);
}

float modbus_get_co2_temp_high_delta_c(void) {
  return ((float)modbus_read_holding_reg(MODBUS_HREG_CO2_TEMP_HIGH_DELTA)) /
         10.0f;
}

float modbus_get_co2_temp_critical_delta_c(void) {
  return ((float)modbus_read_holding_reg(MODBUS_HREG_CO2_TEMP_CRITICAL_DELTA)) /
         10.0f;
}

float modbus_get_co2_humidity_high_delta_percent(void) {
  return ((float)modbus_read_holding_reg(MODBUS_HREG_CO2_HUM_HIGH_DELTA)) /
         10.0f;
}

bool modbus_get_co2_external_alarm(void) {
  return modbus_read_holding_reg(MODBUS_HREG_CO2_EXTERNAL_ALARM) != 0U;
}

uint16_t modbus_get_co2_fault_reset_token(void) {
  return modbus_read_holding_reg(MODBUS_HREG_CO2_FAULT_RESET_TOKEN);
}

void modbus_set_co2_runtime(uint16_t target_ppm, uint16_t effective_target_ppm,
                            uint16_t status_bits, uint16_t reason_bits,
                            uint16_t protection_bits, uint16_t fault_code,
                            uint16_t dosing_elapsed_s) {
  modbus_write_holding_reg(MODBUS_HREG_CO2_TARGET_PPM, target_ppm);
  modbus_write_holding_reg(MODBUS_HREG_CO2_EFFECTIVE_TARGET_PPM,
                           effective_target_ppm);
  modbus_write_holding_reg(MODBUS_HREG_CO2_STATUS_BITS, status_bits);
  modbus_write_holding_reg(MODBUS_HREG_CO2_REASON_BITS, reason_bits);
  modbus_write_holding_reg(MODBUS_HREG_CO2_PROTECTION_BITS, protection_bits);
  modbus_write_holding_reg(MODBUS_HREG_CO2_FAULT_CODE, fault_code);
  modbus_write_holding_reg(MODBUS_HREG_CO2_DOSING_ELAPSED_S,
                           dosing_elapsed_s);
}

modbus_mode_state_t modbus_get_mode_state(void) {
  modbus_mode_state_t mode = MODBUS_MODE_REMOTE;
  taskENTER_CRITICAL(&s_state_lock);
  mode = s_mode_state;
  taskEXIT_CRITICAL(&s_state_lock);
  return mode;
}

bool modbus_is_autonomous(void) {
  return modbus_get_mode_state() == MODBUS_MODE_AUTONOMOUS;
}
