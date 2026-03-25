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
#include <stdbool.h>
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

#define MODBUS_NVS_NAMESPACE "modbus"
#define MODBUS_NVS_KEY_SLAVE_ID "slave_id"
#define MODBUS_NVS_KEY_REMOTE_CFG "remote_cfg"
#define MODBUS_NVS_KEY_LIGHT_STATE "light_state"
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
  uint32_t magic;
  remote_ctrl_cfg_t cfg;
} persisted_remote_cfg_t;

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

static const remote_ctrl_cfg_t s_autonomous_cfg = {
    .ctrl_version = 0,
    .windows_pos_a_target = 0,
    .windows_pos_b_target = 0,
    .curtain_pos_target = 1000,
    .sp_water_rail = 350,
    .sp_water_grow = 320,
    .sp_water_upper = 360,
    .sp_water_undertray = 300,
    .periods = {{1, 600, 2200}},
};

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
static uint8_t get_active_light_schedule_mask(const light_control_cfg_t *cfg,
                                              uint16_t minute_of_day);
static void log_active_light_schedules_if_changed(
    const light_control_cfg_t *cfg, modbus_mode_state_t mode,
    uint16_t minute_of_day, uint8_t active_mask);
static modbus_apply_status_t apply_control_block(
    const light_control_cfg_t *candidate_cfg);

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

static void load_persisted_settings(void) {
  nvs_handle_t nvs = 0;
  esp_err_t err = nvs_open(MODBUS_NVS_NAMESPACE, NVS_READWRITE, &nvs);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "NVS open failed, using defaults: %s", esp_err_to_name(err));
    s_slave_id = MODBUS_DEFAULT_SLAVE_ID;
    s_remote_active_cfg = s_autonomous_cfg;
    s_remote_active_cfg.ctrl_version = 1;
    set_default_light_state();
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
    s_remote_active_cfg = s_autonomous_cfg;
    s_remote_active_cfg.ctrl_version = 1;
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

  nvs_close(nvs);

  if (need_persist_default_remote) {
    persist_remote_cfg(&s_remote_active_cfg);
  }
  if (need_persist_default_light) {
    (void)persist_light_state(&s_active_light_cfg, s_active_ctrl_version);
  }
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

  mb_exception_t ex = invoke_wrapped_handler(0x06, ctx, frame, len_buf);
  if (ex == 0) {
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

  mb_exception_t ex = invoke_wrapped_handler(0x10, ctx, frame, len_buf);
  if (ex == 0) {
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

  mb_exception_t ex = invoke_wrapped_handler(0x17, ctx, frame, len_buf);
  if (ex == 0) {
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
      // Safety action on mode switch: force both windows to fully closed (0%).
      ESP_ERROR_CHECK(mbc_slave_lock(s_mbc_slave_handler));
      s_holding_regs[MODBUS_HREG_WINDOWS_POS_A_TARGET] = 0;
      s_holding_regs[MODBUS_HREG_WINDOWS_POS_B_TARGET] = 0;
      ESP_ERROR_CHECK(mbc_slave_unlock(s_mbc_slave_handler));
      ESP_LOGW(TAG,
               "Mode changed: REMOTE -> AUTONOMOUS (master timeout > %u ms), "
               "forced windows targets A/B to 0%%",
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
    effective_light_cfg.relay[0].schedule = s_autonomous_cfg.periods[0];
    effective_light_cfg.relay[1].schedule = s_autonomous_cfg.periods[0];
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
        if (active_light_cfg.hyst_sec == 0U) {
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
                   ((uint32_t)active_light_cfg.hyst_sec * 1000U)) {
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

float modbus_get_window_a_target_percent(void) {
  if (modbus_get_mode_state() == MODBUS_MODE_AUTONOMOUS) {
    return ((float)s_autonomous_cfg.windows_pos_a_target) / 10.0f;
  }

  uint16_t raw_target = 0;
  if (s_mbc_slave_handler != NULL) {
    ESP_ERROR_CHECK(mbc_slave_lock(s_mbc_slave_handler));
    raw_target = s_holding_regs[MODBUS_HREG_WINDOWS_POS_A_TARGET];
    ESP_ERROR_CHECK(mbc_slave_unlock(s_mbc_slave_handler));
  }

  if (raw_target > 1000U) {
    raw_target = 1000U;
  }
  return ((float)raw_target) / 10.0f;
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
