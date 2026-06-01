#pragma once

#include "esp_err.h"
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MODBUS_HREG_TOTAL_COUNT 310

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

// Heating control/runtime registers
#define MODBUS_HREG_HEATING_CTRL_MODE 225
#define MODBUS_HREG_HEATING_AIR_SETPOINT 226
#define MODBUS_HREG_HEATING_AIR_HYST 227
#define MODBUS_HREG_HEATING_STAGE_DELTA_1 228
#define MODBUS_HREG_HEATING_STAGE_DELTA_2 229
#define MODBUS_HREG_HEATING_STAGE_DELTA_3 230
#define MODBUS_HREG_HEATING_STAGE_DELTA_4 231
#define MODBUS_HREG_HEATING_MIN_ON_S 232
#define MODBUS_HREG_HEATING_MIN_OFF_S 233
#define MODBUS_HREG_HEATING_MANUAL_PUMP_MASK 234
#define MODBUS_HREG_HEATING_MANUAL_VALVE_OPEN_MASK 235
#define MODBUS_HREG_HEATING_MANUAL_VALVE_CLOSE_MASK 236
#define MODBUS_HREG_HEATING_STATUS_BITS 237
#define MODBUS_HREG_HEATING_ACTIVE_STAGE 238
#define MODBUS_HREG_HEATING_PUMP_MASK 239
#define MODBUS_HREG_HEATING_VALVE_OPEN_MASK 240
#define MODBUS_HREG_HEATING_VALVE_CLOSE_MASK 241
#define MODBUS_HREG_HEATING_SENSOR_STATUS_BITS 242

// Curtain control/runtime registers
#define MODBUS_HREG_CURTAIN_CTRL_MODE 243
#define MODBUS_HREG_CURTAIN_MANUAL_TARGET 244
#define MODBUS_HREG_CURTAIN_SCHEDULE_START_HHMM 245
#define MODBUS_HREG_CURTAIN_SCHEDULE_END_HHMM 246
#define MODBUS_HREG_CURTAIN_OUTSIDE_TARGET 247
#define MODBUS_HREG_CURTAIN_MIN_POSITION 248
#define MODBUS_HREG_CURTAIN_MAX_POSITION 249
#define MODBUS_HREG_CURTAIN_POSITION_HYST 250
#define MODBUS_HREG_CURTAIN_RADIATION_THRESHOLD 251
#define MODBUS_HREG_CURTAIN_RADIATION_STEP_WM2 252
#define MODBUS_HREG_CURTAIN_RADIATION_STEP_PERCENT 253
#define MODBUS_HREG_CURTAIN_RADIATION_HYST 254
#define MODBUS_HREG_CURTAIN_COLD_DELTA 255
#define MODBUS_HREG_CURTAIN_COLD_HYST 256
#define MODBUS_HREG_CURTAIN_COLD_TARGET 257
#define MODBUS_HREG_CURTAIN_HEAT_DELTA 258
#define MODBUS_HREG_CURTAIN_HEAT_HYST 259
#define MODBUS_HREG_CURTAIN_HEAT_TARGET 260
#define MODBUS_HREG_CURTAIN_HUM_LOW_THRESHOLD 261
#define MODBUS_HREG_CURTAIN_HUM_LOW_HYST 262
#define MODBUS_HREG_CURTAIN_HUM_LOW_TARGET 263
#define MODBUS_HREG_CURTAIN_HUM_HIGH_THRESHOLD 264
#define MODBUS_HREG_CURTAIN_HUM_HIGH_HYST 265
#define MODBUS_HREG_CURTAIN_HUM_HIGH_TARGET 266
#define MODBUS_HREG_CURTAIN_TARGET 267
#define MODBUS_HREG_CURTAIN_BASE_TARGET 268
#define MODBUS_HREG_CURTAIN_CURRENT_MA 269
#define MODBUS_HREG_CURTAIN_STATUS_BITS 270
#define MODBUS_HREG_CURTAIN_REASON_BITS 271
#define MODBUS_HREG_CURTAIN_POSITION_STATUS_BITS 272
#define MODBUS_HREG_CURTAIN_FAULT_CODE 273
#define MODBUS_HREG_CURTAIN_FAULT_RESET_TOKEN 274

// Global greenhouse targets
#define MODBUS_HREG_AIR_TEMP_TARGET 275
#define MODBUS_HREG_AIR_HUM_TARGET 276

// CO2 control/runtime registers
#define MODBUS_HREG_CO2_MEASURED_PPM 277
#define MODBUS_HREG_CO2_SENSOR_VALID 278
#define MODBUS_HREG_CO2_CTRL_MODE 279
#define MODBUS_HREG_CO2_MANUAL_OUTPUTS 280
#define MODBUS_HREG_CO2_SCHEDULE_START_HHMM 281
#define MODBUS_HREG_CO2_SCHEDULE_END_HHMM 282
#define MODBUS_HREG_CO2_LOW_LIGHT_THRESHOLD_WM2 283
#define MODBUS_HREG_CO2_MID_LIGHT_THRESHOLD_WM2 284
#define MODBUS_HREG_CO2_HIGH_LIGHT_THRESHOLD_WM2 285
#define MODBUS_HREG_CO2_LOW_LIGHT_TARGET_PPM 286
#define MODBUS_HREG_CO2_MID_LIGHT_TARGET_PPM 287
#define MODBUS_HREG_CO2_HIGH_LIGHT_TARGET_PPM 288
#define MODBUS_HREG_CO2_VENT_LIMIT_LOW_PERCENT 289
#define MODBUS_HREG_CO2_VENT_LIMIT_HIGH_PERCENT 290
#define MODBUS_HREG_CO2_VENT_CUTOFF_PERCENT 291
#define MODBUS_HREG_CO2_DOSING_HYST_PPM 292
#define MODBUS_HREG_CO2_MAX_SAFE_PPM 293
#define MODBUS_HREG_CO2_MAX_DOSING_TIME_S 294
#define MODBUS_HREG_CO2_MIN_PAUSE_TIME_S 295
#define MODBUS_HREG_CO2_NO_RISE_CHECK_TIME_S 296
#define MODBUS_HREG_CO2_NO_RISE_MIN_DELTA_PPM 297
#define MODBUS_HREG_CO2_TEMP_HIGH_DELTA 298
#define MODBUS_HREG_CO2_TEMP_CRITICAL_DELTA 299
#define MODBUS_HREG_CO2_HUM_HIGH_DELTA 300
#define MODBUS_HREG_CO2_EXTERNAL_ALARM 301
#define MODBUS_HREG_CO2_TARGET_PPM 302
#define MODBUS_HREG_CO2_EFFECTIVE_TARGET_PPM 303
#define MODBUS_HREG_CO2_STATUS_BITS 304
#define MODBUS_HREG_CO2_REASON_BITS 305
#define MODBUS_HREG_CO2_PROTECTION_BITS 306
#define MODBUS_HREG_CO2_FAULT_CODE 307
#define MODBUS_HREG_CO2_DOSING_ELAPSED_S 308
#define MODBUS_HREG_CO2_FAULT_RESET_TOKEN 309

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
  MODBUS_HEATING_CTRL_MODE_AUTO = 0,
  MODBUS_HEATING_CTRL_MODE_OFF = 1,
  MODBUS_HEATING_CTRL_MODE_MANUAL = 2,
} modbus_heating_ctrl_mode_t;

typedef enum {
  MODBUS_CO2_CTRL_MODE_AUTO = 0,
  MODBUS_CO2_CTRL_MODE_OFF = 1,
  MODBUS_CO2_CTRL_MODE_MANUAL = 2,
} modbus_co2_ctrl_mode_t;

typedef enum {
  MODBUS_CURTAIN_CTRL_MODE_AUTO = 0,
  MODBUS_CURTAIN_CTRL_MODE_MANUAL = 1,
  MODBUS_CURTAIN_CTRL_MODE_OFF = 2,
} modbus_curtain_ctrl_mode_t;

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
  float solar_radiation_wm2;
  uint32_t rx_age_ms;
  bool valid;
  bool stale;
  bool rain_active;
} modbus_weather_runtime_t;

typedef struct {
  bool air_temp_override_active;
  bool rh_override_active;
  float air_temp_c;
  float rh_percent;
} modbus_sensor_test_override_t;

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
void modbus_get_sensor_test_override(
    modbus_sensor_test_override_t *out_override);

float modbus_get_window_a_target_percent(void);
float modbus_get_window_b_target_percent(void);
float modbus_get_curtain_target_percent(void);
modbus_windows_ctrl_mode_t modbus_get_windows_ctrl_mode(void);
bool modbus_get_windows_force_safe_cmd(void);
float modbus_get_air_temp_target_c(void);
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
float modbus_get_air_hum_target_percent(void);
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
modbus_curtain_ctrl_mode_t modbus_get_curtain_ctrl_mode(void);
float modbus_get_curtain_manual_target_percent(void);
uint16_t modbus_get_curtain_schedule_start_hhmm(void);
uint16_t modbus_get_curtain_schedule_end_hhmm(void);
float modbus_get_curtain_outside_target_percent(void);
float modbus_get_curtain_min_position_percent(void);
float modbus_get_curtain_max_position_percent(void);
float modbus_get_curtain_position_hysteresis_percent(void);
uint16_t modbus_get_curtain_radiation_threshold_wm2(void);
uint16_t modbus_get_curtain_radiation_step_wm2(void);
float modbus_get_curtain_radiation_step_percent(void);
uint16_t modbus_get_curtain_radiation_hysteresis_wm2(void);
float modbus_get_curtain_cold_delta_c(void);
float modbus_get_curtain_cold_hysteresis_c(void);
float modbus_get_curtain_cold_target_percent(void);
float modbus_get_curtain_heat_delta_c(void);
float modbus_get_curtain_heat_hysteresis_c(void);
float modbus_get_curtain_heat_target_percent(void);
float modbus_get_curtain_humidity_low_threshold_percent(void);
float modbus_get_curtain_humidity_low_hysteresis_percent(void);
float modbus_get_curtain_humidity_low_target_percent(void);
float modbus_get_curtain_humidity_high_threshold_percent(void);
float modbus_get_curtain_humidity_high_hysteresis_percent(void);
float modbus_get_curtain_humidity_high_target_percent(void);
uint16_t modbus_get_curtain_fault_reset_token(void);
void modbus_set_curtain_runtime(float target_percent, float base_target_percent,
                                float current_ma, uint16_t status_bits,
                                uint16_t reason_bits,
                                uint16_t position_status_bits,
                                uint16_t fault_code);
float modbus_get_water_setpoint_c(modbus_water_channel_t channel);
modbus_heating_ctrl_mode_t modbus_get_heating_ctrl_mode(void);
float modbus_get_heating_air_setpoint_c(void);
float modbus_get_heating_air_hysteresis_c(void);
float modbus_get_heating_stage_delta_c(uint8_t stage_index);
uint16_t modbus_get_heating_min_on_s(void);
uint16_t modbus_get_heating_min_off_s(void);
uint16_t modbus_get_heating_manual_pump_mask(void);
uint16_t modbus_get_heating_manual_valve_open_mask(void);
uint16_t modbus_get_heating_manual_valve_close_mask(void);
void modbus_set_heating_runtime(uint16_t status_bits, uint16_t active_stage,
                                uint16_t pump_mask, uint16_t valve_open_mask,
                                uint16_t valve_close_mask,
                                uint16_t sensor_status_bits);
uint16_t modbus_get_co2_measured_ppm(void);
bool modbus_get_co2_sensor_valid(void);
modbus_co2_ctrl_mode_t modbus_get_co2_ctrl_mode(void);
uint16_t modbus_get_co2_manual_outputs(void);
uint16_t modbus_get_co2_schedule_start_hhmm(void);
uint16_t modbus_get_co2_schedule_end_hhmm(void);
uint16_t modbus_get_co2_low_light_threshold_wm2(void);
uint16_t modbus_get_co2_mid_light_threshold_wm2(void);
uint16_t modbus_get_co2_high_light_threshold_wm2(void);
uint16_t modbus_get_co2_low_light_target_ppm(void);
uint16_t modbus_get_co2_mid_light_target_ppm(void);
uint16_t modbus_get_co2_high_light_target_ppm(void);
uint16_t modbus_get_co2_vent_limit_low_percent(void);
uint16_t modbus_get_co2_vent_limit_high_percent(void);
uint16_t modbus_get_co2_vent_cutoff_percent(void);
uint16_t modbus_get_co2_dosing_hysteresis_ppm(void);
uint16_t modbus_get_co2_max_safe_ppm(void);
uint16_t modbus_get_co2_max_dosing_time_s(void);
uint16_t modbus_get_co2_min_pause_time_s(void);
uint16_t modbus_get_co2_no_rise_check_time_s(void);
uint16_t modbus_get_co2_no_rise_min_delta_ppm(void);
float modbus_get_co2_temp_high_delta_c(void);
float modbus_get_co2_temp_critical_delta_c(void);
float modbus_get_co2_humidity_high_delta_percent(void);
bool modbus_get_co2_external_alarm(void);
uint16_t modbus_get_co2_fault_reset_token(void);
void modbus_set_co2_runtime(uint16_t target_ppm, uint16_t effective_target_ppm,
                            uint16_t status_bits, uint16_t reason_bits,
                            uint16_t protection_bits, uint16_t fault_code,
                            uint16_t dosing_elapsed_s);
modbus_mode_state_t modbus_get_mode_state(void);
bool modbus_is_autonomous(void);

#ifdef __cplusplus
}
#endif
