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
#define MODBUS_NVS_KEY_LIGHT_STATE "light_state"
#define MODBUS_LIGHT_STATE_MAGIC 0x4D424C53U // MBLS

#define MODBUS_DEFAULT_SLAVE_ID 1U
#define MODBUS_MIN_SLAVE_ID 1U
#define MODBUS_MAX_SLAVE_ID 247U

#define MODBUS_FC_COUNT 10

// APPLY command values
#define MODBUS_APPLY_CMD_NONE 0

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
  uint32_t magic;
  uint32_t active_ctrl_version;
  uint16_t last_applied_token;
  uint16_t reserved;
  light_period_cfg_t active_schedule[MODBUS_LIGHT_MAX_PERIODS];
  uint32_t crc32;
} persisted_light_state_t;

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
static volatile uint32_t s_light_set_ms = 0;

static volatile modbus_mode_state_t s_mode_state = MODBUS_MODE_REMOTE;
static volatile modbus_mode_reason_t s_mode_reason = MODBUS_REASON_NONE;
static volatile uint32_t s_last_master_seen_ms = 0;
static volatile uint16_t s_good_cycle_streak = 0;
static volatile modbus_apply_status_t s_last_apply_status = MODBUS_APPLY_OK;

static light_period_cfg_t s_staging_schedule[MODBUS_LIGHT_MAX_PERIODS] = {0};
static light_period_cfg_t s_active_schedule[MODBUS_LIGHT_MAX_PERIODS] = {0};
static uint32_t s_active_ctrl_version = 0;
static volatile bool s_apply_pending = false;
static light_period_cfg_t s_apply_pending_schedule[MODBUS_LIGHT_MAX_PERIODS] = {0};
static volatile uint32_t s_apply_ok_count = 0;
static volatile uint32_t s_apply_fail_invalid_count = 0;
static volatile uint32_t s_apply_fail_busy_count = 0;
static volatile uint32_t s_apply_fail_internal_count = 0;
static volatile modbus_apply_status_t s_last_apply_error_code = MODBUS_APPLY_OK;
static volatile uint32_t s_last_apply_ts_ms = 0;
static volatile uint8_t s_last_logged_schedule_mask = UINT8_MAX;
static volatile uint8_t s_last_logged_schedule_mode = UINT8_MAX;

static volatile uint16_t s_rtc_last_token = 0;
static volatile uint16_t s_rtc_pending_token = 0;
static volatile uint16_t s_rtc_pending_hour = 0;
static volatile uint16_t s_rtc_pending_minute = 0;
static volatile bool s_rtc_sync_pending = false;

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
static void queue_apply_request_from_regs(void);
static void sync_staging_schedule_from_current_regs(void);
static void queue_apply_request_from_current_regs(void);
static bool decode_write_holding_span(uint8_t fc, const uint8_t *frame, uint16_t len,
                                      uint16_t *start_reg, uint16_t *reg_count);
static bool reg_span_contains(uint16_t start_reg, uint16_t reg_count,
                              uint16_t target_reg);
static bool reg_span_intersects(uint16_t start_reg, uint16_t reg_count,
                                uint16_t first_reg, uint16_t last_reg);
static uint8_t get_active_schedule_mask(const light_period_cfg_t *periods,
                                        uint16_t minute_of_day);
static void log_active_light_schedules_if_changed(
    const light_period_cfg_t *periods, modbus_mode_state_t mode,
    uint16_t minute_of_day, uint8_t active_mask);
static modbus_apply_status_t apply_control_block(
    const light_period_cfg_t *candidate_schedule);

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

static void light_schedule_to_regs(const light_period_cfg_t *periods, uint16_t *regs) {
  if (!periods || !regs) {
    return;
  }

  regs[MODBUS_HREG_SCH0_ENABLE] = periods[0].enable;
  regs[MODBUS_HREG_SCH0_ON_HHMM] = periods[0].on_hhmm;
  regs[MODBUS_HREG_SCH0_OFF_HHMM] = periods[0].off_hhmm;
  regs[MODBUS_HREG_SCH1_ENABLE] = periods[1].enable;
  regs[MODBUS_HREG_SCH1_ON_HHMM] = periods[1].on_hhmm;
  regs[MODBUS_HREG_SCH1_OFF_HHMM] = periods[1].off_hhmm;
  regs[MODBUS_HREG_SCH2_ENABLE] = periods[2].enable;
  regs[MODBUS_HREG_SCH2_ON_HHMM] = periods[2].on_hhmm;
  regs[MODBUS_HREG_SCH2_OFF_HHMM] = periods[2].off_hhmm;
  regs[MODBUS_HREG_SCH3_ENABLE] = periods[3].enable;
  regs[MODBUS_HREG_SCH3_ON_HHMM] = periods[3].on_hhmm;
  regs[MODBUS_HREG_SCH3_OFF_HHMM] = periods[3].off_hhmm;
}

static void regs_to_light_schedule(const uint16_t *regs, light_period_cfg_t *periods) {
  if (!periods || !regs) {
    return;
  }

  periods[0].enable = regs[MODBUS_HREG_SCH0_ENABLE];
  periods[0].on_hhmm = regs[MODBUS_HREG_SCH0_ON_HHMM];
  periods[0].off_hhmm = regs[MODBUS_HREG_SCH0_OFF_HHMM];
  periods[1].enable = regs[MODBUS_HREG_SCH1_ENABLE];
  periods[1].on_hhmm = regs[MODBUS_HREG_SCH1_ON_HHMM];
  periods[1].off_hhmm = regs[MODBUS_HREG_SCH1_OFF_HHMM];
  periods[2].enable = regs[MODBUS_HREG_SCH2_ENABLE];
  periods[2].on_hhmm = regs[MODBUS_HREG_SCH2_ON_HHMM];
  periods[2].off_hhmm = regs[MODBUS_HREG_SCH2_OFF_HHMM];
  periods[3].enable = regs[MODBUS_HREG_SCH3_ENABLE];
  periods[3].on_hhmm = regs[MODBUS_HREG_SCH3_ON_HHMM];
  periods[3].off_hhmm = regs[MODBUS_HREG_SCH3_OFF_HHMM];
}

static bool validate_light_schedule(const light_period_cfg_t *periods) {
  if (!periods) {
    return false;
  }

  for (int i = 0; i < MODBUS_LIGHT_MAX_PERIODS; ++i) {
    if (periods[i].enable > 1U) {
      return false;
    }
    if (!hhmm_valid(periods[i].on_hhmm) || !hhmm_valid(periods[i].off_hhmm)) {
      return false;
    }
    if (periods[i].enable == 1U && periods[i].on_hhmm == periods[i].off_hhmm) {
      return false;
    }
  }
  return true;
}

static uint32_t compute_light_state_crc32(
    uint32_t active_ctrl_version, uint16_t last_applied_token,
    const light_period_cfg_t *active_schedule) {
  uint32_t crc = 0;
  crc = crc32_update(crc, (const uint8_t *)&active_ctrl_version,
                     sizeof(active_ctrl_version));
  crc = crc32_update(crc, (const uint8_t *)&last_applied_token,
                     sizeof(last_applied_token));
  for (int i = 0; i < MODBUS_LIGHT_MAX_PERIODS; ++i) {
    uint16_t fields[3] = {active_schedule[i].enable, active_schedule[i].on_hhmm,
                          active_schedule[i].off_hhmm};
    crc = crc32_update(crc, (const uint8_t *)fields, sizeof(fields));
  }
  return crc;
}

static void set_default_light_state(void) {
  memset(s_staging_schedule, 0, sizeof(s_staging_schedule));
  memset(s_active_schedule, 0, sizeof(s_active_schedule));
  memset(s_apply_pending_schedule, 0, sizeof(s_apply_pending_schedule));
  s_apply_pending = false;
  s_active_ctrl_version = 0;
}

static bool persist_light_state(const light_period_cfg_t *active_schedule,
                                uint32_t active_ctrl_version) {
  if (active_schedule == NULL) {
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
  memcpy(blob.active_schedule, active_schedule, sizeof(blob.active_schedule));
  blob.crc32 = compute_light_state_crc32(blob.active_ctrl_version,
                                         blob.last_applied_token,
                                         blob.active_schedule);

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
      validate_light_schedule(light_blob.active_schedule)) {
    uint32_t expected_crc =
        compute_light_state_crc32(light_blob.active_ctrl_version,
                                  light_blob.last_applied_token,
                                  light_blob.active_schedule);
    if (expected_crc == light_blob.crc32) {
      memcpy(s_active_schedule, light_blob.active_schedule, sizeof(s_active_schedule));
      memcpy(s_staging_schedule, light_blob.active_schedule, sizeof(s_staging_schedule));
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
    (void)persist_light_state(s_active_schedule, s_active_ctrl_version);
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

static uint8_t get_active_schedule_mask(const light_period_cfg_t *periods,
                                        uint16_t minute_of_day) {
  if (!periods) {
    return 0U;
  }

  uint8_t active_mask = 0U;
  for (int i = 0; i < MODBUS_LIGHT_MAX_PERIODS; ++i) {
    if (is_period_active(&periods[i], minute_of_day)) {
      active_mask |= (uint8_t)(1U << i);
    }
  }
  return active_mask;
}

static void log_active_light_schedules_if_changed(
    const light_period_cfg_t *periods, modbus_mode_state_t mode,
    uint16_t minute_of_day, uint8_t active_mask) {
  if (periods == NULL || minute_of_day >= 1440U) {
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
  ESP_LOGI(TAG, "Light schedules active: mode=%s now=%02u:%02u mask=0x%02X",
           (mode == MODBUS_MODE_AUTONOMOUS) ? "AUTONOMOUS" : "REMOTE",
           (unsigned)hh, (unsigned)mm, (unsigned)active_mask);
  if (active_mask == 0U) {
    return;
  }

  for (int i = 0; i < MODBUS_LIGHT_MAX_PERIODS; ++i) {
    if ((active_mask & (uint8_t)(1U << i)) != 0U) {
      ESP_LOGI(TAG, "Light schedule slot %d active: %04u..%04u", i,
               (unsigned)periods[i].on_hhmm, (unsigned)periods[i].off_hhmm);
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
  light_period_cfg_t staging[MODBUS_LIGHT_MAX_PERIODS] = {0};
  regs_to_light_schedule(s_holding_regs, staging);

  taskENTER_CRITICAL(&s_state_lock);
  memcpy(s_staging_schedule, staging, sizeof(s_staging_schedule));
  taskEXIT_CRITICAL(&s_state_lock);
}

static void queue_apply_request_from_current_regs(void) {
  light_period_cfg_t staging[MODBUS_LIGHT_MAX_PERIODS] = {0};
  regs_to_light_schedule(s_holding_regs, staging);
  s_holding_regs[MODBUS_HREG_APPLY_CMD] = MODBUS_APPLY_CMD_NONE;

  taskENTER_CRITICAL(&s_state_lock);
  bool was_pending = s_apply_pending;
  memcpy(s_staging_schedule, staging, sizeof(s_staging_schedule));
  memcpy(s_apply_pending_schedule, staging, sizeof(s_apply_pending_schedule));
  s_apply_pending = true;
  if (was_pending && s_apply_fail_busy_count < UINT32_MAX) {
    s_apply_fail_busy_count++;
  }
  taskEXIT_CRITICAL(&s_state_lock);
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
      (reg_span_intersects(start_reg, reg_count, MODBUS_HREG_SCH0_ENABLE,
                           MODBUS_HREG_SCH3_OFF_HHMM) ||
       reg_span_intersects(alt_start_reg, reg_count, MODBUS_HREG_SCH0_ENABLE,
                           MODBUS_HREG_SCH3_OFF_HHMM));
  bool writes_apply =
      has_span &&
      (reg_span_contains(start_reg, reg_count, MODBUS_HREG_APPLY_CMD) ||
       reg_span_contains(alt_start_reg, reg_count, MODBUS_HREG_APPLY_CMD));

  mb_exception_t ex = invoke_wrapped_handler(0x06, ctx, frame, len_buf);
  if (ex == 0) {
    queue_rtc_sync_request_from_regs();
    if (writes_apply) {
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
      (reg_span_intersects(start_reg, reg_count, MODBUS_HREG_SCH0_ENABLE,
                           MODBUS_HREG_SCH3_OFF_HHMM) ||
       reg_span_intersects(alt_start_reg, reg_count, MODBUS_HREG_SCH0_ENABLE,
                           MODBUS_HREG_SCH3_OFF_HHMM));
  bool writes_apply =
      has_span &&
      (reg_span_contains(start_reg, reg_count, MODBUS_HREG_APPLY_CMD) ||
       reg_span_contains(alt_start_reg, reg_count, MODBUS_HREG_APPLY_CMD));

  mb_exception_t ex = invoke_wrapped_handler(0x10, ctx, frame, len_buf);
  if (ex == 0) {
    queue_rtc_sync_request_from_regs();
    if (writes_apply) {
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
      (reg_span_intersects(start_reg, reg_count, MODBUS_HREG_SCH0_ENABLE,
                           MODBUS_HREG_SCH3_OFF_HHMM) ||
       reg_span_intersects(alt_start_reg, reg_count, MODBUS_HREG_SCH0_ENABLE,
                           MODBUS_HREG_SCH3_OFF_HHMM));
  bool writes_apply =
      has_span &&
      (reg_span_contains(start_reg, reg_count, MODBUS_HREG_APPLY_CMD) ||
       reg_span_contains(alt_start_reg, reg_count, MODBUS_HREG_APPLY_CMD));

  mb_exception_t ex = invoke_wrapped_handler(0x17, ctx, frame, len_buf);
  if (ex == 0) {
    queue_rtc_sync_request_from_regs();
    if (writes_apply) {
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

  light_period_cfg_t staging[MODBUS_LIGHT_MAX_PERIODS] = {0};
  uint16_t apply_cmd = MODBUS_APPLY_CMD_NONE;
  ESP_ERROR_CHECK(mbc_slave_lock(s_mbc_slave_handler));
  regs_to_light_schedule(s_holding_regs, staging);
  apply_cmd = s_holding_regs[MODBUS_HREG_APPLY_CMD];
  if (apply_cmd != MODBUS_APPLY_CMD_NONE) {
    s_holding_regs[MODBUS_HREG_APPLY_CMD] = MODBUS_APPLY_CMD_NONE;
  }
  ESP_ERROR_CHECK(mbc_slave_unlock(s_mbc_slave_handler));

  taskENTER_CRITICAL(&s_state_lock);
  memcpy(s_staging_schedule, staging, sizeof(s_staging_schedule));
  if (apply_cmd != MODBUS_APPLY_CMD_NONE) {
    bool was_pending = s_apply_pending;
    memcpy(s_apply_pending_schedule, staging, sizeof(s_apply_pending_schedule));
    s_apply_pending = true;
    if (was_pending && s_apply_fail_busy_count < UINT32_MAX) {
      s_apply_fail_busy_count++;
    }
  }
  taskEXIT_CRITICAL(&s_state_lock);
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
    const light_period_cfg_t *candidate_schedule) {
  if (s_mbc_slave_handler == NULL) {
    return MODBUS_APPLY_ERR_INTERNAL;
  }
  if (candidate_schedule == NULL) {
    return MODBUS_APPLY_ERR_INTERNAL;
  }

  if (!validate_light_schedule(candidate_schedule)) {
    return MODBUS_APPLY_ERR_RANGE;
  }

  uint32_t next_version = 0;
  taskENTER_CRITICAL(&s_state_lock);
  next_version = s_active_ctrl_version + 1U;
  taskEXIT_CRITICAL(&s_state_lock);

  if (!persist_light_state(candidate_schedule, next_version)) {
    return MODBUS_APPLY_ERR_INTERNAL;
  }

  taskENTER_CRITICAL(&s_state_lock);
  memcpy(s_active_schedule, candidate_schedule, sizeof(s_active_schedule));
  for (int i = 0; i < MODBUS_LIGHT_MAX_PERIODS; ++i) {
    s_remote_active_cfg.periods[i] = candidate_schedule[i];
  }
  s_active_ctrl_version = next_version;
  taskEXIT_CRITICAL(&s_state_lock);

  uint16_t ver_hi = 0;
  uint16_t ver_lo = 0;
  u32_to_regs(next_version, &ver_hi, &ver_lo);
  ESP_ERROR_CHECK(mbc_slave_lock(s_mbc_slave_handler));
  s_holding_regs[MODBUS_HREG_ACTIVE_CTRL_VERSION_HI] = ver_hi;
  s_holding_regs[MODBUS_HREG_ACTIVE_CTRL_VERSION_LO] = ver_lo;
  ESP_ERROR_CHECK(mbc_slave_unlock(s_mbc_slave_handler));

  ESP_LOGI(TAG, "Light schedule applied: active_ctrl_version=%lu",
           (unsigned long)next_version);
  for (int i = 0; i < MODBUS_LIGHT_MAX_PERIODS; ++i) {
    ESP_LOGI(TAG, "Light schedule slot %d: EN=%u ON=%04u OFF=%04u", i,
             (unsigned)candidate_schedule[i].enable,
             (unsigned)candidate_schedule[i].on_hhmm,
             (unsigned)candidate_schedule[i].off_hhmm);
  }

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
    queue_apply_request_from_regs();

    light_period_cfg_t pending_schedule[MODBUS_LIGHT_MAX_PERIODS] = {0};
    bool has_pending_apply = false;
    taskENTER_CRITICAL(&s_state_lock);
    has_pending_apply = s_apply_pending;
    if (has_pending_apply) {
      memcpy(pending_schedule, s_apply_pending_schedule, sizeof(pending_schedule));
      s_apply_pending = false;
    }
    taskEXIT_CRITICAL(&s_state_lock);

    if (has_pending_apply) {
      modbus_apply_status_t status = apply_control_block(pending_schedule);
      finalize_apply_result(status);
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
  light_schedule_to_regs(s_staging_schedule, s_holding_regs);
  uint16_t ver_hi = 0;
  uint16_t ver_lo = 0;
  u32_to_regs(s_active_ctrl_version, &ver_hi, &ver_lo);
  s_holding_regs[MODBUS_HREG_ACTIVE_CTRL_VERSION_HI] = ver_hi;
  s_holding_regs[MODBUS_HREG_ACTIVE_CTRL_VERSION_LO] = ver_lo;
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
  s_apply_pending = false;
  memset(s_apply_pending_schedule, 0, sizeof(s_apply_pending_schedule));
  s_apply_ok_count = 0;
  s_apply_fail_invalid_count = 0;
  s_apply_fail_busy_count = 0;
  s_apply_fail_internal_count = 0;
  s_last_apply_error_code = MODBUS_APPLY_OK;
  s_last_apply_ts_ms = 0;
  s_last_logged_schedule_mask = UINT8_MAX;
  s_last_logged_schedule_mode = UINT8_MAX;
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
  bool out1 = false;
  bool out2 = false;
  bool reduction = false;
  light_period_cfg_t effective_schedule[MODBUS_LIGHT_MAX_PERIODS] = {0};
  modbus_mode_state_t mode = MODBUS_MODE_REMOTE;
  uint8_t light_hour = 0;
  uint8_t light_minute = 0;
  uint8_t light_second = 0;
  uint32_t light_set_ms = 0;

  taskENTER_CRITICAL(&s_state_lock);
  mode = s_mode_state;
  if (mode == MODBUS_MODE_AUTONOMOUS) {
    memcpy(effective_schedule, s_autonomous_cfg.periods, sizeof(effective_schedule));
  } else {
    memcpy(effective_schedule, s_active_schedule, sizeof(effective_schedule));
  }
  light_hour = s_light_hour;
  light_minute = s_light_minute;
  light_second = s_light_second;
  light_set_ms = s_light_set_ms;
  taskEXIT_CRITICAL(&s_state_lock);

  if (light_hour <= 23U && light_minute <= 59U && light_second <= 59U) {
    uint32_t base_sec =
        (uint32_t)light_hour * 3600U + (uint32_t)light_minute * 60U + (uint32_t)light_second;
    uint32_t elapsed_sec = (now_ms() - light_set_ms) / 1000U;
    uint32_t sec_of_day = (base_sec + elapsed_sec) % 86400U;
    uint16_t minute_of_day = (uint16_t)(sec_of_day / 60U);
    uint8_t active_mask = get_active_schedule_mask(effective_schedule, minute_of_day);
    bool schedule_active = (active_mask != 0U);

    log_active_light_schedules_if_changed(effective_schedule, mode, minute_of_day,
                                          active_mask);

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
