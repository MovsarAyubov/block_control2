#pragma once

#include "esp_err.h"
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MODBUS_HREG_TOTAL_COUNT 225

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

#define MODBUS_LIGHT_MAX_PERIODS 1
#define MODBUS_HREG_LIGHT_R1_ENABLE 110
#define MODBUS_HREG_LIGHT_R1_ON_HHMM 111
#define MODBUS_HREG_LIGHT_R1_OFF_HHMM 112
#define MODBUS_HREG_LIGHT_R1_THRESHOLD_WM2 113
#define MODBUS_HREG_LIGHT_R1_RESERVED 114
#define MODBUS_HREG_LIGHT_R1_DLI_OFF_LIMIT_JCM2 115
#define MODBUS_HREG_LIGHT_R2_ENABLE 116
#define MODBUS_HREG_LIGHT_R2_ON_HHMM 117
#define MODBUS_HREG_LIGHT_R2_OFF_HHMM 118
#define MODBUS_HREG_LIGHT_R2_THRESHOLD_WM2 119
#define MODBUS_HREG_LIGHT_R2_RESERVED 120
#define MODBUS_HREG_LIGHT_R2_DLI_OFF_LIMIT_JCM2 121
#define MODBUS_HREG_LIGHT_HYST_SEC 122

#define MODBUS_HREG_CTRL_CRC_LO 123
#define MODBUS_HREG_CTRL_CRC_HI 124
#define MODBUS_HREG_APPLY_STATUS 125
#define MODBUS_HREG_ACTIVE_CTRL_VERSION_HI 126
#define MODBUS_HREG_ACTIVE_CTRL_VERSION_LO 127

// Diagnostics (RO): 9..14
#define MODBUS_HREG_MODE_STATE 9
#define MODBUS_HREG_MODE_REASON 10
#define MODBUS_HREG_LAST_MASTER_SEEN_MS_LO 11
#define MODBUS_HREG_LAST_MASTER_SEEN_MS_HI 12
#define MODBUS_HREG_GOOD_CYCLE_STREAK 13
#define MODBUS_HREG_LAST_APPLY_STATUS 14

// Light/radiation runtime registers
#define MODBUS_HREG_LIGHT_CURRENT_DLI_JCM2 134     // current accumulated DLI (RW, from master)
#define MODBUS_HREG_LIGHT_OUTPUT_PERCENT 135       // current light output 0/50/100 (RO)
#define MODBUS_HREG_LIGHT_STATUS_BITS 136          // current light state flags (RO)

// RTC sync command/result registers
#define MODBUS_HREG_RTC_SET_HOUR 140          // W, 0..23
#define MODBUS_HREG_RTC_SET_MINUTE 141        // W, 0..59
#define MODBUS_HREG_RTC_SET_TOKEN 142         // W, non-zero token triggers processing
#define MODBUS_HREG_RTC_SET_APPLIED_TOKEN 143 // R, last processed token
#define MODBUS_HREG_RTC_SET_RESULT 144        // R, see modbus_rtc_set_result_t

// Apply diagnostics (RO)
#define MODBUS_HREG_APPLY_OK_COUNT_HI 145
#define MODBUS_HREG_APPLY_OK_COUNT_LO 146
#define MODBUS_HREG_APPLY_FAIL_INVALID_COUNT_HI 147
#define MODBUS_HREG_APPLY_FAIL_INVALID_COUNT_LO 148
#define MODBUS_HREG_APPLY_FAIL_BUSY_COUNT_HI 149
#define MODBUS_HREG_APPLY_FAIL_BUSY_COUNT_LO 150
#define MODBUS_HREG_APPLY_FAIL_INTERNAL_COUNT_HI 151
#define MODBUS_HREG_APPLY_FAIL_INTERNAL_COUNT_LO 152
#define MODBUS_HREG_LAST_APPLY_ERROR_CODE 153
#define MODBUS_HREG_LAST_APPLY_TS_MS_HI 154
#define MODBUS_HREG_LAST_APPLY_TS_MS_LO 155

// Weather sync snapshot/token/result registers
#define MODBUS_HREG_WEATHER_OUT_TEMP 158
#define MODBUS_HREG_WEATHER_OUT_HUM 159
#define MODBUS_HREG_WEATHER_WIND_SPEED 160
#define MODBUS_HREG_WEATHER_WIND_DIR 161
#define MODBUS_HREG_WEATHER_RAIN_FLAG 162
#define MODBUS_HREG_WEATHER_SOLAR_RAD 163
#define MODBUS_HREG_WEATHER_BARO_PRESS 164
#define MODBUS_HREG_WEATHER_DEW_POINT 165
#define MODBUS_HREG_WEATHER_STATUS_BITS 166
#define MODBUS_HREG_WEATHER_AGE_S 167
#define MODBUS_HREG_WEATHER_SET_TOKEN 168
#define MODBUS_HREG_WEATHER_SET_APPLIED_TOKEN 169
#define MODBUS_HREG_WEATHER_SET_RESULT 170

// Window control/runtime registers
#define MODBUS_HREG_WINDOWS_CTRL_MODE 171
#define MODBUS_HREG_WINDOWS_FORCE_SAFE_CMD 172
#define MODBUS_HREG_WINDOWS_TEMP_SETPOINT 173
#define MODBUS_HREG_WINDOWS_SAFE_MIN_PERCENT 174
#define MODBUS_HREG_WINDOWS_WIND_LIMIT 175
#define MODBUS_HREG_WINDOWS_WIND_STORM 176
#define MODBUS_HREG_WINDOWS_WIND_RECOVER 177
#define MODBUS_HREG_WINDOW_A_AZIMUTH_DEG 178
#define MODBUS_HREG_WINDOWS_WIND_SECTOR_HALF_WIDTH_DEG 179
#define MODBUS_HREG_WINDOWS_TEMP_STEP_C 180
#define MODBUS_HREG_WINDOWS_TEMP_STEP_HYST_C 181
#define MODBUS_HREG_RLL400_TARGET_HYST_PERCENT 182
#define MODBUS_HREG_RLL400_MOTION_DELTA_PERCENT 183
#define MODBUS_HREG_RLL400_NO_MOTION_TIMEOUT_MS 184
#define MODBUS_HREG_WINDOW_A_FAULT_RESET_TOKEN 185
#define MODBUS_HREG_WINDOW_B_FAULT_RESET_TOKEN 186
#define MODBUS_HREG_WINDOWS_STATUS_BITS 187
#define MODBUS_HREG_WINDOW_A_STATUS_BITS 188
#define MODBUS_HREG_WINDOW_B_STATUS_BITS 189
#define MODBUS_HREG_WINDOW_A_FAULT_CODE 190
#define MODBUS_HREG_WINDOW_B_FAULT_CODE 191
#define MODBUS_HREG_AIR_TEMP_SENSOR_STATUS 192
#define MODBUS_HREG_WINDOW_A_LOCAL_MANUAL_ACTIVE 193
#define MODBUS_HREG_WINDOW_B_LOCAL_MANUAL_ACTIVE 194
#define MODBUS_HREG_WINDOWS_AUTO_ALGO_MODE 195
#define MODBUS_HREG_WINDOWS_HUM_SETPOINT 196
#define MODBUS_HREG_WINDOWS_HUM_STEP 197
#define MODBUS_HREG_WINDOWS_HUM_STEP_HYST 198
#define MODBUS_HREG_WINDOWS_COLD_CLOSE_DELTA 199
#define MODBUS_HREG_WINDOWS_COLD_CLOSE_HYST 200
#define MODBUS_HREG_WINDOWS_WINDWARD_MIN_PERCENT 201
#define MODBUS_HREG_WINDOWS_WINDWARD_MAX_PERCENT 202
#define MODBUS_HREG_WINDOWS_WINDWARD_SPEED_THRESHOLD 203
#define MODBUS_HREG_WINDOWS_WINDWARD_REDUCTION_PERCENT_PER_MS 204
#define MODBUS_HREG_WINDOWS_LEEWARD_MIN_PERCENT 205
#define MODBUS_HREG_WINDOWS_LEEWARD_MAX_PERCENT 206
#define MODBUS_HREG_WINDOWS_LEEWARD_SPEED_THRESHOLD 207
#define MODBUS_HREG_WINDOWS_LEEWARD_REDUCTION_PERCENT_PER_MS 208
#define MODBUS_HREG_WINDOWS_WINDWARD_LAG_PERCENT 209
#define MODBUS_HREG_WINDOWS_RAIN_MODE 210
#define MODBUS_HREG_WINDOWS_RAIN_WINDWARD_PERCENT 211
#define MODBUS_HREG_WINDOWS_WEATHER_STALE_POLICY 212
#define MODBUS_HREG_WINDOWS_BASE_TARGET_A 213
#define MODBUS_HREG_WINDOWS_BASE_TARGET_B 214
#define MODBUS_HREG_WINDOWS_EFFECTIVE_TARGET_A 215
#define MODBUS_HREG_WINDOWS_EFFECTIVE_TARGET_B 216
#define MODBUS_HREG_WINDOWS_ACTIVE_PROTECTION_BITS 217
#define MODBUS_HREG_WINDOWS_WINDWARD_SIDE 218
#define MODBUS_HREG_WINDOWS_TEMP_STEP_TARGET_PERCENT 219
#define MODBUS_HREG_WINDOWS_TEMP_STEP_MAX_INDEX 220
#define MODBUS_HREG_WINDOWS_HUM_STEP_TARGET_PERCENT 221
#define MODBUS_HREG_WINDOWS_HUM_STEP_MAX_INDEX 222
#define MODBUS_HREG_WINDOWS_WEATHER_STALE_TIMEOUT_MS 223
#define MODBUS_HREG_WINDOWS_WEATHER_SOURCE_AGE_S 224

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
  MODBUS_APPLY_NOOP = 6,
} modbus_apply_status_t;

typedef enum {
  MODBUS_RTC_SET_RESULT_NONE = 0,
  MODBUS_RTC_SET_RESULT_APPLIED = 2,
  MODBUS_RTC_SET_RESULT_REJECT_RANGE = 3,
  MODBUS_RTC_SET_RESULT_FAILED = 4,
  MODBUS_RTC_SET_RESULT_NOOP = 5,
} modbus_rtc_set_result_t;

typedef enum {
  MODBUS_WEATHER_SET_RESULT_NONE = 0,
  MODBUS_WEATHER_SET_RESULT_APPLIED = 2,
  MODBUS_WEATHER_SET_RESULT_FAILED = 4,
  MODBUS_WEATHER_SET_RESULT_NOOP = 5,
} modbus_weather_set_result_t;

typedef enum {
  MODBUS_WATER_CHANNEL_RAIL = 0,
  MODBUS_WATER_CHANNEL_GROW = 1,
  MODBUS_WATER_CHANNEL_UPPER = 2,
  MODBUS_WATER_CHANNEL_UNDERTRAY = 3,
  MODBUS_WATER_CHANNEL_COUNT = 4,
} modbus_water_channel_t;

typedef enum {
  MODBUS_WINDOWS_CTRL_MODE_AUTO = 0,
  MODBUS_WINDOWS_CTRL_MODE_MANUAL = 1,
} modbus_windows_ctrl_mode_t;

typedef enum {
  MODBUS_WINDOWS_AUTO_ALGO_TEMP = 0,
  MODBUS_WINDOWS_AUTO_ALGO_HUMIDITY = 1,
} modbus_windows_auto_algo_mode_t;

typedef enum {
  MODBUS_WINDOWS_RAIN_MODE_DISABLED = 0,
  MODBUS_WINDOWS_RAIN_MODE_WINDWARD = 1,
} modbus_windows_rain_mode_t;

typedef enum {
  MODBUS_WINDOWS_WEATHER_STALE_CLOSE_SAFE = 0,
  MODBUS_WINDOWS_WEATHER_STALE_IGNORE = 1,
} modbus_windows_weather_stale_policy_t;

typedef enum {
  MODBUS_WINDOWS_WINDWARD_SIDE_NONE = 0,
  MODBUS_WINDOWS_WINDWARD_SIDE_A = 1,
  MODBUS_WINDOWS_WINDWARD_SIDE_B = 2,
  MODBUS_WINDOWS_WINDWARD_SIDE_BOTH_UNKNOWN = 3,
} modbus_windows_windward_side_t;

typedef enum {
  MODBUS_WINDOW_CHANNEL_A = 0,
  MODBUS_WINDOW_CHANNEL_B = 1,
} modbus_window_channel_t;

typedef struct {
  float wind_speed_ms;
  uint16_t wind_dir_deg;
  uint16_t source_age_s;
  uint16_t status_bits;
  uint32_t rx_age_ms;
  bool valid;
  bool stale;
  bool rain_active;
} modbus_weather_runtime_t;

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
esp_err_t modbus_handle_ascii_command(const char *line, char *response,
                                      size_t response_len);

float modbus_get_window_a_target_percent(void);
float modbus_get_window_b_target_percent(void);
modbus_windows_ctrl_mode_t modbus_get_windows_ctrl_mode(void);
bool modbus_get_windows_force_safe_cmd(void);
float modbus_get_windows_temp_setpoint_c(void);
float modbus_get_windows_safe_min_percent(void);
float modbus_get_windows_wind_limit_ms(void);
float modbus_get_windows_wind_storm_ms(void);
float modbus_get_windows_wind_recover_ms(void);
uint16_t modbus_get_window_a_azimuth_deg(void);
uint16_t modbus_get_windows_wind_sector_half_width_deg(void);
float modbus_get_windows_temp_step_c(void);
float modbus_get_windows_temp_step_hysteresis_c(void);
float modbus_get_windows_temp_step_target_increment_percent(void);
uint16_t modbus_get_windows_temp_step_max_index(void);
modbus_windows_auto_algo_mode_t modbus_get_windows_auto_algo_mode(void);
float modbus_get_windows_humidity_setpoint_percent(void);
float modbus_get_windows_humidity_step_percent(void);
float modbus_get_windows_humidity_step_hysteresis_percent(void);
float modbus_get_windows_humidity_step_target_increment_percent(void);
uint16_t modbus_get_windows_humidity_step_max_index(void);
float modbus_get_windows_cold_close_delta_c(void);
float modbus_get_windows_cold_close_hysteresis_c(void);
float modbus_get_windows_windward_min_percent(void);
float modbus_get_windows_windward_max_percent(void);
float modbus_get_windows_windward_speed_threshold_ms(void);
float modbus_get_windows_windward_reduction_percent_per_ms(void);
float modbus_get_windows_leeward_min_percent(void);
float modbus_get_windows_leeward_max_percent(void);
float modbus_get_windows_leeward_speed_threshold_ms(void);
float modbus_get_windows_leeward_reduction_percent_per_ms(void);
float modbus_get_windows_windward_lag_percent(void);
modbus_windows_rain_mode_t modbus_get_windows_rain_mode(void);
float modbus_get_windows_rain_windward_percent(void);
modbus_windows_weather_stale_policy_t
modbus_get_windows_weather_stale_policy(void);
uint32_t modbus_get_windows_weather_stale_timeout_ms(void);
uint16_t modbus_get_windows_weather_source_age_limit_s(void);
float modbus_get_rll400_target_hysteresis_percent(void);
float modbus_get_rll400_motion_delta_percent(void);
uint32_t modbus_get_rll400_no_motion_timeout_ms(void);
uint16_t modbus_get_window_fault_reset_token(modbus_window_channel_t channel);
void modbus_get_weather_runtime(modbus_weather_runtime_t *out_runtime);
void modbus_set_windows_runtime(uint16_t windows_status_bits,
                                uint16_t window_a_status_bits,
                                uint16_t window_b_status_bits,
                                uint16_t window_a_fault_code,
                                uint16_t window_b_fault_code,
                                uint16_t air_temp_sensor_status,
                                uint16_t window_a_local_manual_active,
                                uint16_t window_b_local_manual_active);
void modbus_set_windows_target_diagnostics(
    float base_target_a_percent, float base_target_b_percent,
    float effective_target_a_percent, float effective_target_b_percent,
    uint16_t active_protection_bits,
    modbus_windows_windward_side_t windward_side);
float modbus_get_water_setpoint_c(modbus_water_channel_t channel);
modbus_mode_state_t modbus_get_mode_state(void);
bool modbus_is_autonomous(void);

#ifdef __cplusplus
}
#endif
