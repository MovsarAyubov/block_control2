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
#define MODBUS_SOLAR_DEFAULT_THRESHOLD_X10 8000U
#define MODBUS_RTC_SYNC_THRESHOLD_MIN 3U

#define MODBUS_NVS_NAMESPACE "modbus"
#define MODBUS_NVS_KEY_SLAVE_ID "slave_id"
#define MODBUS_NVS_KEY_REMOTE_CFG "remote_cfg"

#define MODBUS_DEFAULT_SLAVE_ID 1U
#define MODBUS_MIN_SLAVE_ID 1U
#define MODBUS_MAX_SLAVE_ID 247U

#define MODBUS_CTRL_REG_FIRST MODBUS_HREG_CTRL_VERSION_HI
#define MODBUS_CTRL_REG_LAST MODBUS_HREG_SCH3_OFF_HHMM
#define MODBUS_CTRL_REG_COUNT (MODBUS_CTRL_REG_LAST - MODBUS_CTRL_REG_FIRST + 1)

#define MODBUS_FC_COUNT 10

// APPLY command values
#define MODBUS_APPLY_CMD_NONE 0
#define MODBUS_APPLY_CMD_APPLY 1

typedef struct {
  uint16_t enable;
  uint16_t on_hhmm;
  uint16_t off_hhmm;
} light_period_cfg_t;

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
  uint8_t fc;
  mb_fn_handler_fp original;
  mb_fn_handler_fp wrapper;
} modbus_handler_wrap_t;

static void *s_mbc_slave_handler = NULL;
static mb_register_area_descriptor_t s_holding_area;
static uint16_t s_holding_regs[MODBUS_HREG_TOTAL_COUNT] = {0};

static volatile uint8_t s_light_hour = 0;
static volatile uint8_t s_light_minute = 0;
static volatile uint8_t s_light_second = 0;

static volatile modbus_mode_state_t s_mode_state = MODBUS_MODE_REMOTE;
static volatile modbus_mode_reason_t s_mode_reason = MODBUS_REASON_NONE;
static volatile uint32_t s_last_master_seen_ms = 0;
static volatile uint16_t s_good_cycle_streak = 0;
static volatile modbus_apply_status_t s_last_apply_status = MODBUS_APPLY_OK;

static volatile uint16_t s_rtc_last_token = 0;
static volatile uint16_t s_rtc_pending_token = 0;
static volatile uint16_t s_rtc_pending_hour = 0;
static volatile uint16_t s_rtc_pending_minute = 0;
static volatile bool s_rtc_sync_pending = false;
static uint16_t s_rtc_observed_hour = UINT16_MAX;
static uint16_t s_rtc_observed_minute = UINT16_MAX;
static uint16_t s_rtc_observed_token = UINT16_MAX;
static bool s_hreg_window_inited = false;
static uint16_t s_hreg_window_snapshot[9] = {0}; // [138..146]

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
    .periods = {{1, 600, 2200}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}},
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

static uint32_t regs_to_u32(uint16_t hi, uint16_t lo) {
  return ((uint32_t)hi << 16U) | (uint32_t)lo;
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

static uint32_t compute_ctrl_crc32(const uint16_t *regs, size_t count) {
  // CRC32 over little-endian bytes of 16-bit registers.
  uint32_t crc = 0;
  for (size_t i = 0; i < count; ++i) {
    uint8_t bytes[2] = {(uint8_t)(regs[i] & 0xFFU),
                        (uint8_t)((regs[i] >> 8U) & 0xFFU)};
    crc = crc32_update(crc, bytes, sizeof(bytes));
  }
  return crc;
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

  regs[MODBUS_HREG_SCH0_ENABLE] = cfg->periods[0].enable;
  regs[MODBUS_HREG_SCH0_ON_HHMM] = cfg->periods[0].on_hhmm;
  regs[MODBUS_HREG_SCH0_OFF_HHMM] = cfg->periods[0].off_hhmm;
  regs[MODBUS_HREG_SCH1_ENABLE] = cfg->periods[1].enable;
  regs[MODBUS_HREG_SCH1_ON_HHMM] = cfg->periods[1].on_hhmm;
  regs[MODBUS_HREG_SCH1_OFF_HHMM] = cfg->periods[1].off_hhmm;
  regs[MODBUS_HREG_SCH2_ENABLE] = cfg->periods[2].enable;
  regs[MODBUS_HREG_SCH2_ON_HHMM] = cfg->periods[2].on_hhmm;
  regs[MODBUS_HREG_SCH2_OFF_HHMM] = cfg->periods[2].off_hhmm;
  regs[MODBUS_HREG_SCH3_ENABLE] = cfg->periods[3].enable;
  regs[MODBUS_HREG_SCH3_ON_HHMM] = cfg->periods[3].on_hhmm;
  regs[MODBUS_HREG_SCH3_OFF_HHMM] = cfg->periods[3].off_hhmm;
}

static void regs_to_remote_cfg(const uint16_t *regs, remote_ctrl_cfg_t *cfg) {
  if (!cfg || !regs) {
    return;
  }

  cfg->ctrl_version =
      regs_to_u32(regs[MODBUS_HREG_CTRL_VERSION_HI], regs[MODBUS_HREG_CTRL_VERSION_LO]);
  cfg->windows_pos_a_target = regs[MODBUS_HREG_WINDOWS_POS_A_TARGET];
  cfg->windows_pos_b_target = regs[MODBUS_HREG_WINDOWS_POS_B_TARGET];
  cfg->curtain_pos_target = regs[MODBUS_HREG_CURTAIN_POS_TARGET];
  cfg->sp_water_rail = regs[MODBUS_HREG_SP_WATER_RAIL];
  cfg->sp_water_grow = regs[MODBUS_HREG_SP_WATER_GROW];
  cfg->sp_water_upper = regs[MODBUS_HREG_SP_WATER_UPPER];
  cfg->sp_water_undertray = regs[MODBUS_HREG_SP_WATER_UNDERTRAY];

  cfg->periods[0].enable = regs[MODBUS_HREG_SCH0_ENABLE];
  cfg->periods[0].on_hhmm = regs[MODBUS_HREG_SCH0_ON_HHMM];
  cfg->periods[0].off_hhmm = regs[MODBUS_HREG_SCH0_OFF_HHMM];
  cfg->periods[1].enable = regs[MODBUS_HREG_SCH1_ENABLE];
  cfg->periods[1].on_hhmm = regs[MODBUS_HREG_SCH1_ON_HHMM];
  cfg->periods[1].off_hhmm = regs[MODBUS_HREG_SCH1_OFF_HHMM];
  cfg->periods[2].enable = regs[MODBUS_HREG_SCH2_ENABLE];
  cfg->periods[2].on_hhmm = regs[MODBUS_HREG_SCH2_ON_HHMM];
  cfg->periods[2].off_hhmm = regs[MODBUS_HREG_SCH2_OFF_HHMM];
  cfg->periods[3].enable = regs[MODBUS_HREG_SCH3_ENABLE];
  cfg->periods[3].on_hhmm = regs[MODBUS_HREG_SCH3_ON_HHMM];
  cfg->periods[3].off_hhmm = regs[MODBUS_HREG_SCH3_OFF_HHMM];
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

  for (int i = 0; i < MODBUS_LIGHT_MAX_PERIODS; ++i) {
    if (cfg->periods[i].enable > 1U) {
      return false;
    }
    if (!hhmm_valid(cfg->periods[i].on_hhmm) || !hhmm_valid(cfg->periods[i].off_hhmm)) {
      return false;
    }
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

  persisted_remote_cfg_t blob = {0};
  size_t blob_size = sizeof(blob);
  err = nvs_get_blob(nvs, MODBUS_NVS_KEY_REMOTE_CFG, &blob, &blob_size);
  if (err == ESP_OK && blob_size == sizeof(blob) && blob.magic == 0x4D424346U &&
      validate_remote_cfg(&blob.cfg)) {
    s_remote_active_cfg = blob.cfg;
  } else {
    s_remote_active_cfg = s_autonomous_cfg;
    s_remote_active_cfg.ctrl_version = 1;
    persist_remote_cfg(&s_remote_active_cfg);
  }

  nvs_close(nvs);
}

static bool is_now_in_periods(const remote_ctrl_cfg_t *cfg, uint16_t minute_of_day) {
  if (!cfg) {
    return false;
  }

  for (int i = 0; i < MODBUS_LIGHT_MAX_PERIODS; ++i) {
    if (cfg->periods[i].enable == 0U) {
      continue;
    }

    uint16_t on_hhmm = cfg->periods[i].on_hhmm;
    uint16_t off_hhmm = cfg->periods[i].off_hhmm;
    if (!hhmm_valid(on_hhmm) || !hhmm_valid(off_hhmm)) {
      continue;
    }

    uint16_t start = hhmm_to_minutes(on_hhmm);
    uint16_t end = hhmm_to_minutes(off_hhmm);

    // ON == OFF means 24h ON.
    if (start == end) {
      return true;
    }

    if (start < end) {
      if (minute_of_day >= start && minute_of_day < end) {
        return true;
      }
    } else {
      // Through midnight.
      if (minute_of_day >= start || minute_of_day < end) {
        return true;
      }
    }
  }

  return false;
}

static void update_diag_regs_locked(void) {
  s_holding_regs[MODBUS_HREG_MODE_STATE] = (uint16_t)s_mode_state;
  s_holding_regs[MODBUS_HREG_MODE_REASON] = (uint16_t)s_mode_reason;
  s_holding_regs[MODBUS_HREG_LAST_MASTER_SEEN_MS_LO] =
      (uint16_t)(s_last_master_seen_ms & 0xFFFFU);
  s_holding_regs[MODBUS_HREG_LAST_MASTER_SEEN_MS_HI] =
      (uint16_t)((s_last_master_seen_ms >> 16U) & 0xFFFFU);
  s_holding_regs[MODBUS_HREG_GOOD_CYCLE_STREAK] = s_good_cycle_streak;
  s_holding_regs[MODBUS_HREG_LAST_APPLY_STATUS] = (uint16_t)s_last_apply_status;
  s_holding_regs[MODBUS_HREG_APPLY_STATUS] = (uint16_t)s_last_apply_status;
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
  }
}

static void queue_rtc_sync_request_from_regs(void) {
  if (s_mbc_slave_handler == NULL) {
    return;
  }

  uint16_t token = 0;
  uint16_t hour = 0;
  uint16_t minute = 0;
  uint16_t applied_token = 0;
  uint16_t result = 0;

  if (!s_hreg_window_inited) {
    for (int i = 0; i < 9; ++i) {
      s_hreg_window_snapshot[i] = s_holding_regs[138 + i];
    }
    s_hreg_window_inited = true;
  } else {
    for (int i = 0; i < 9; ++i) {
      uint16_t reg_addr = (uint16_t)(138 + i);
      uint16_t new_val = s_holding_regs[reg_addr];
      uint16_t old_val = s_hreg_window_snapshot[i];
      if (new_val != old_val) {
        ESP_LOGI(TAG, "HREG[%u] changed: %u -> %u", (unsigned)reg_addr,
                 (unsigned)old_val, (unsigned)new_val);
        s_hreg_window_snapshot[i] = new_val;
      }
    }
  }

  token = s_holding_regs[MODBUS_HREG_RTC_SET_TOKEN];
  hour = s_holding_regs[MODBUS_HREG_RTC_SET_HOUR];
  minute = s_holding_regs[MODBUS_HREG_RTC_SET_MINUTE];
  applied_token = s_holding_regs[MODBUS_HREG_RTC_SET_APPLIED_TOKEN];
  result = s_holding_regs[MODBUS_HREG_RTC_SET_RESULT];

  if (hour != s_rtc_observed_hour || minute != s_rtc_observed_minute ||
      token != s_rtc_observed_token) {
    s_rtc_observed_hour = hour;
    s_rtc_observed_minute = minute;
    s_rtc_observed_token = token;
    ESP_LOGI(TAG, "RTC regs changed: 140=%u 141=%u 142=%u 143=%u 144=%u",
             (unsigned)hour, (unsigned)minute, (unsigned)token,
             (unsigned)applied_token, (unsigned)result);
  }

  if (token == 0U) {
    return;
  }

  bool queued = false;
  taskENTER_CRITICAL(&s_state_lock);
  bool same_as_last = (token == s_rtc_last_token);
  bool same_as_pending = (s_rtc_sync_pending && token == s_rtc_pending_token);
  if (!same_as_last && !same_as_pending) {
    s_rtc_pending_token = token;
    s_rtc_pending_hour = hour;
    s_rtc_pending_minute = minute;
    s_rtc_sync_pending = true;
    queued = true;
  }
  taskEXIT_CRITICAL(&s_state_lock);

  if (queued) {
    ESP_LOGI(TAG, "RTC sync request queued: token=%u target=%02u:%02u",
             (unsigned)token, (unsigned)hour, (unsigned)minute);
  }
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

  bool callbacks_ready = false;
  taskENTER_CRITICAL(&s_state_lock);
  callbacks_ready =
      (s_rtc_get_time_cb != NULL) && (s_rtc_set_time_cb != NULL);
  taskEXIT_CRITICAL(&s_state_lock);
  if (!callbacks_ready) {
    // RTC layer is not bound yet; keep pending request and retry next cycle.
    return;
  }

  if (server_hour > 23U || server_minute > 59U) {
    finalize_rtc_sync(token, MODBUS_RTC_SET_RESULT_REJECT_RANGE);
    return;
  }

  uint8_t local_hour = 0;
  uint8_t local_minute = 0;
  uint8_t local_second = 0;
  if (!get_local_time_snapshot(&local_hour, &local_minute, &local_second)) {
    finalize_rtc_sync(token, MODBUS_RTC_SET_RESULT_FAILED);
    return;
  }

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

  modbus_rtc_set_time_cb_t rtc_set_cb = NULL;
  void *rtc_ctx = NULL;
  taskENTER_CRITICAL(&s_state_lock);
  rtc_set_cb = s_rtc_set_time_cb;
  rtc_ctx = s_rtc_cb_ctx;
  taskEXIT_CRITICAL(&s_state_lock);

  if (rtc_set_cb == NULL ||
      !rtc_set_cb((uint8_t)server_hour, (uint8_t)server_minute, 0U, rtc_ctx)) {
    finalize_rtc_sync(token, MODBUS_RTC_SET_RESULT_FAILED);
    return;
  }

  modbus_set_light_current_time((uint8_t)server_hour, (uint8_t)server_minute, 0U);
  finalize_rtc_sync(token, MODBUS_RTC_SET_RESULT_APPLIED);
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
  mb_exception_t ex = invoke_wrapped_handler(0x06, ctx, frame, len_buf);
  if (ex == 0) {
    queue_rtc_sync_request_from_regs();
  }
  return ex;
}

static mb_exception_t modbus_fc_0F_wrapper(void *ctx, uint8_t *frame,
                                           uint16_t *len_buf) {
  return invoke_wrapped_handler(0x0F, ctx, frame, len_buf);
}

static mb_exception_t modbus_fc_10_wrapper(void *ctx, uint8_t *frame,
                                           uint16_t *len_buf) {
  mb_exception_t ex = invoke_wrapped_handler(0x10, ctx, frame, len_buf);
  if (ex == 0) {
    queue_rtc_sync_request_from_regs();
  }
  return ex;
}

static mb_exception_t modbus_fc_11_wrapper(void *ctx, uint8_t *frame,
                                           uint16_t *len_buf) {
  return invoke_wrapped_handler(0x11, ctx, frame, len_buf);
}

static mb_exception_t modbus_fc_17_wrapper(void *ctx, uint8_t *frame,
                                           uint16_t *len_buf) {
  mb_exception_t ex = invoke_wrapped_handler(0x17, ctx, frame, len_buf);
  if (ex == 0) {
    queue_rtc_sync_request_from_regs();
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

static remote_ctrl_cfg_t get_effective_cfg(void) {
  remote_ctrl_cfg_t cfg = {0};
  taskENTER_CRITICAL(&s_state_lock);
  if (s_mode_state == MODBUS_MODE_AUTONOMOUS) {
    cfg = s_autonomous_cfg;
  } else {
    cfg = s_remote_active_cfg;
  }
  taskEXIT_CRITICAL(&s_state_lock);
  return cfg;
}

static modbus_apply_status_t apply_control_block(void) {
  if (s_mbc_slave_handler == NULL) {
    return MODBUS_APPLY_ERR_INTERNAL;
  }

  uint16_t snapshot[MODBUS_HREG_TOTAL_COUNT] = {0};

  ESP_ERROR_CHECK(mbc_slave_lock(s_mbc_slave_handler));
  memcpy(snapshot, s_holding_regs, sizeof(snapshot));
  ESP_ERROR_CHECK(mbc_slave_unlock(s_mbc_slave_handler));

  uint32_t expected_crc =
      regs_to_u32(snapshot[MODBUS_HREG_CTRL_CRC_HI], snapshot[MODBUS_HREG_CTRL_CRC_LO]);
  uint32_t actual_crc =
      compute_ctrl_crc32(&snapshot[MODBUS_CTRL_REG_FIRST], MODBUS_CTRL_REG_COUNT);

  if (expected_crc != actual_crc) {
    return MODBUS_APPLY_ERR_CRC;
  }

  remote_ctrl_cfg_t candidate = {0};
  regs_to_remote_cfg(snapshot, &candidate);

  if (!validate_remote_cfg(&candidate)) {
    return MODBUS_APPLY_ERR_RANGE;
  }

  taskENTER_CRITICAL(&s_state_lock);
  s_remote_active_cfg = candidate;
  taskEXIT_CRITICAL(&s_state_lock);

  persist_remote_cfg(&candidate);

  ESP_ERROR_CHECK(mbc_slave_lock(s_mbc_slave_handler));
  s_holding_regs[MODBUS_HREG_ACTIVE_CTRL_VERSION_HI] = snapshot[MODBUS_HREG_CTRL_VERSION_HI];
  s_holding_regs[MODBUS_HREG_ACTIVE_CTRL_VERSION_LO] = snapshot[MODBUS_HREG_CTRL_VERSION_LO];
  ESP_ERROR_CHECK(mbc_slave_unlock(s_mbc_slave_handler));

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

    uint16_t apply_cmd = 0;
    ESP_ERROR_CHECK(mbc_slave_lock(s_mbc_slave_handler));
    apply_cmd = s_holding_regs[MODBUS_HREG_APPLY_CMD];
    ESP_ERROR_CHECK(mbc_slave_unlock(s_mbc_slave_handler));

    if (apply_cmd != MODBUS_APPLY_CMD_NONE) {
      modbus_apply_status_t status = MODBUS_APPLY_ERR_CMD;
      if (apply_cmd == MODBUS_APPLY_CMD_APPLY) {
        status = apply_control_block();
      }

      taskENTER_CRITICAL(&s_state_lock);
      s_last_apply_status = status;
      taskEXIT_CRITICAL(&s_state_lock);

      ESP_ERROR_CHECK(mbc_slave_lock(s_mbc_slave_handler));
      s_holding_regs[MODBUS_HREG_APPLY_CMD] = MODBUS_APPLY_CMD_NONE;
      update_diag_regs_locked();
      ESP_ERROR_CHECK(mbc_slave_unlock(s_mbc_slave_handler));
    }

    uint32_t now = now_ms();
    taskENTER_CRITICAL(&s_state_lock);
    bool entered_autonomous = false;
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
  s_holding_regs[MODBUS_HREG_ACTIVE_CTRL_VERSION_HI] =
      s_holding_regs[MODBUS_HREG_CTRL_VERSION_HI];
  s_holding_regs[MODBUS_HREG_ACTIVE_CTRL_VERSION_LO] =
      s_holding_regs[MODBUS_HREG_CTRL_VERSION_LO];
  s_holding_regs[MODBUS_HREG_SOLAR_RADIATION_X10] = 0;
  s_holding_regs[MODBUS_HREG_SOLAR_UPPER_THRESHOLD_X10] =
      MODBUS_SOLAR_DEFAULT_THRESHOLD_X10;
  s_holding_regs[MODBUS_HREG_LIGHT_REDUCTION_ACTIVE] = 0;
  s_holding_regs[MODBUS_HREG_RTC_SET_HOUR] = 0;
  s_holding_regs[MODBUS_HREG_RTC_SET_MINUTE] = 0;
  s_holding_regs[MODBUS_HREG_RTC_SET_TOKEN] = 0;
  s_holding_regs[MODBUS_HREG_RTC_SET_APPLIED_TOKEN] = 0;
  s_holding_regs[MODBUS_HREG_RTC_SET_RESULT] = MODBUS_RTC_SET_RESULT_NONE;

  s_last_apply_status = MODBUS_APPLY_OK;
  s_last_master_seen_ms = now_ms();
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
  s_light_hour = hour;
  s_light_minute = minute;
  s_light_second = second;
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
  bool out1 = false;
  bool out2 = false;
  bool reduction = false;

  if (s_light_hour <= 23U && s_light_minute <= 59U && s_light_second <= 59U) {
    remote_ctrl_cfg_t cfg = get_effective_cfg();
    uint16_t minute_of_day = (uint16_t)(s_light_hour * 60U + s_light_minute);
    bool schedule_active = is_now_in_periods(&cfg, minute_of_day);

    if (schedule_active) {
      uint16_t radiation_x10 = 0;
      uint16_t threshold_x10 = MODBUS_SOLAR_DEFAULT_THRESHOLD_X10;

      if (s_mbc_slave_handler != NULL) {
        ESP_ERROR_CHECK(mbc_slave_lock(s_mbc_slave_handler));
        radiation_x10 = s_holding_regs[MODBUS_HREG_SOLAR_RADIATION_X10];
        threshold_x10 = s_holding_regs[MODBUS_HREG_SOLAR_UPPER_THRESHOLD_X10];
        ESP_ERROR_CHECK(mbc_slave_unlock(s_mbc_slave_handler));
      }

      if (threshold_x10 > 0U && radiation_x10 >= threshold_x10) {
        // Upper radiation threshold reached: switch one relay off (50% light).
        out1 = true;
        out2 = false;
        reduction = true;
      } else {
        // Inside schedule and radiation below threshold: full light.
        out1 = true;
        out2 = true;
      }
    }
  }

  if (s_mbc_slave_handler != NULL) {
    ESP_ERROR_CHECK(mbc_slave_lock(s_mbc_slave_handler));
    s_holding_regs[MODBUS_HREG_LIGHT_REDUCTION_ACTIVE] = reduction ? 1U : 0U;
    ESP_ERROR_CHECK(mbc_slave_unlock(s_mbc_slave_handler));
  }

  if (relay1_on != NULL) {
    *relay1_on = out1;
  }
  if (relay2_on != NULL) {
    *relay2_on = out2;
  }
}

void modbus_set_solar_radiation(float radiation) {
  if (s_mbc_slave_handler == NULL) {
    return;
  }
  uint16_t radiation_x10 = float_to_u16_tenths(radiation, 0.0f, 6553.5f);
  ESP_ERROR_CHECK(mbc_slave_lock(s_mbc_slave_handler));
  s_holding_regs[MODBUS_HREG_SOLAR_RADIATION_X10] = radiation_x10;
  ESP_ERROR_CHECK(mbc_slave_unlock(s_mbc_slave_handler));
}

float modbus_get_window_a_target_percent(void) {
  remote_ctrl_cfg_t cfg = get_effective_cfg();
  return ((float)cfg.windows_pos_a_target) / 10.0f;
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
