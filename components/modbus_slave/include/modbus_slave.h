#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MODBUS_HREG_TOTAL_COUNT 160

// Telemetry map (MUST): base + 0..8, int16 x10
#define MODBUS_HREG_AIR_TEMP 0
#define MODBUS_HREG_AIR_HUM 1
#define MODBUS_HREG_WATER_RAIL 2
#define MODBUS_HREG_WATER_GROW 3
#define MODBUS_HREG_WATER_UNDERTRAY 4
#define MODBUS_HREG_WATER_UPPER_HEAT 5
#define MODBUS_HREG_WINDOWS_POS_A 6
#define MODBUS_HREG_WINDOWS_POS_B 7
#define MODBUS_HREG_CURTAIN_POS 8

// Control map (MUST): 100..127
#define MODBUS_CTRL_BASE 100
#define MODBUS_HREG_CTRL_VERSION_HI 100
#define MODBUS_HREG_CTRL_VERSION_LO 101
#define MODBUS_HREG_MODE_CMD 102
#define MODBUS_HREG_WINDOWS_POS_A_TARGET 103
#define MODBUS_HREG_WINDOWS_POS_B_TARGET 104
#define MODBUS_HREG_CURTAIN_POS_TARGET 105
#define MODBUS_HREG_SP_WATER_RAIL 106
#define MODBUS_HREG_SP_WATER_GROW 107
#define MODBUS_HREG_SP_WATER_UPPER 108
#define MODBUS_HREG_SP_WATER_UNDERTRAY 109

#define MODBUS_LIGHT_MAX_PERIODS 4
#define MODBUS_HREG_SCH0_ENABLE 110
#define MODBUS_HREG_SCH0_ON_HHMM 111
#define MODBUS_HREG_SCH0_OFF_HHMM 112
#define MODBUS_HREG_SCH1_ENABLE 113
#define MODBUS_HREG_SCH1_ON_HHMM 114
#define MODBUS_HREG_SCH1_OFF_HHMM 115
#define MODBUS_HREG_SCH2_ENABLE 116
#define MODBUS_HREG_SCH2_ON_HHMM 117
#define MODBUS_HREG_SCH2_OFF_HHMM 118
#define MODBUS_HREG_SCH3_ENABLE 119
#define MODBUS_HREG_SCH3_ON_HHMM 120
#define MODBUS_HREG_SCH3_OFF_HHMM 121

#define MODBUS_HREG_APPLY_CMD 122
#define MODBUS_HREG_CTRL_CRC_LO 123
#define MODBUS_HREG_CTRL_CRC_HI 124
#define MODBUS_HREG_APPLY_STATUS 125
#define MODBUS_HREG_ACTIVE_CTRL_VERSION_HI 126
#define MODBUS_HREG_ACTIVE_CTRL_VERSION_LO 127

// Diagnostics (RO)
#define MODBUS_HREG_MODE_STATE 128
#define MODBUS_HREG_MODE_REASON 129
#define MODBUS_HREG_LAST_MASTER_SEEN_MS_LO 130
#define MODBUS_HREG_LAST_MASTER_SEEN_MS_HI 131
#define MODBUS_HREG_GOOD_CYCLE_STREAK 132
#define MODBUS_HREG_LAST_APPLY_STATUS 133

// Light/radiation runtime registers
#define MODBUS_HREG_SOLAR_RADIATION_X10 134      // current solar radiation (RW, from master)
#define MODBUS_HREG_SOLAR_UPPER_THRESHOLD_X10 135 // upper threshold (RW)
#define MODBUS_HREG_LIGHT_REDUCTION_ACTIVE 136   // 1 when forced 50% by radiation

// RTC sync command/result registers
#define MODBUS_HREG_RTC_SET_HOUR 140          // W, 0..23
#define MODBUS_HREG_RTC_SET_MINUTE 141        // W, 0..59
#define MODBUS_HREG_RTC_SET_TOKEN 142         // W, non-zero token triggers processing
#define MODBUS_HREG_RTC_SET_APPLIED_TOKEN 143 // R, last processed token
#define MODBUS_HREG_RTC_SET_RESULT 144        // R, see modbus_rtc_set_result_t

typedef enum {
  MODBUS_MODE_REMOTE = 0,
  MODBUS_MODE_AUTONOMOUS = 1,
} modbus_mode_state_t;

typedef enum {
  MODBUS_REASON_NONE = 0,
  MODBUS_REASON_MASTER_TIMEOUT = 1,
} modbus_mode_reason_t;

typedef enum {
  MODBUS_APPLY_OK = 0,
  MODBUS_APPLY_ERR_CRC = 1,
  MODBUS_APPLY_ERR_RANGE = 2,
  MODBUS_APPLY_ERR_BUSY = 3,
  MODBUS_APPLY_ERR_CMD = 4,
  MODBUS_APPLY_ERR_INTERNAL = 5,
} modbus_apply_status_t;

typedef enum {
  MODBUS_RTC_SET_RESULT_NONE = 0,
  MODBUS_RTC_SET_RESULT_APPLIED = 2,
  MODBUS_RTC_SET_RESULT_REJECT_RANGE = 3,
  MODBUS_RTC_SET_RESULT_FAILED = 4,
  MODBUS_RTC_SET_RESULT_NOOP = 5,
} modbus_rtc_set_result_t;

typedef bool (*modbus_rtc_get_time_cb_t)(uint8_t *hour, uint8_t *minute,
                                         uint8_t *second, void *ctx);
typedef bool (*modbus_rtc_set_time_cb_t)(uint8_t hour, uint8_t minute,
                                         uint8_t second, void *ctx);

void modbus_init(void);
void modbus_bind_rtc_callbacks(modbus_rtc_get_time_cb_t get_cb,
                               modbus_rtc_set_time_cb_t set_cb, void *ctx);

void modbus_set_telemetry(float air_temp, float air_hum, float water_rail,
                          float water_grow, float water_undertray,
                          float water_upper_heat, float windows_pos_a,
                          float windows_pos_b, float curtain_pos);

void modbus_set_light_current_time(uint8_t hour, uint8_t minute, uint8_t second);
uint8_t modbus_get_light_percent(void);
void modbus_get_light_relay_state(bool *relay1_on, bool *relay2_on);
void modbus_set_solar_radiation(float radiation);

float modbus_get_window_a_target_percent(void);
modbus_mode_state_t modbus_get_mode_state(void);
bool modbus_is_autonomous(void);

#ifdef __cplusplus
}
#endif
