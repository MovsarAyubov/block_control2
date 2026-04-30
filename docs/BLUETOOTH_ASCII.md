# Bluetooth ASCII Control

This project now supports changing autonomous setpoints over Bluetooth Classic
SPP with plain ASCII commands.

## Transport

- Device name: `RH-AUTONOMOUS`
- SPP service name: `RH_ASCII`
- Pairing PIN: `1234`
- Command format: one command per line, terminated by `\n` or `\r\n`

## Behavior

- Commands update the autonomous configuration stored in NVS.
- Changes take effect immediately while the node is in `AUTONOMOUS` mode.
- The updated autonomous configuration survives reboot.
- `REMOTE` mode logic is unchanged; Modbus still controls remote setpoints.
- Window automation tuning commands write runtime holding registers directly.
- Weather mock commands update the active weather snapshot in RAM immediately.
- Weather mock data does not survive reboot.
- Window safety treats weather as unsafe when `wx_stat` is non-zero.

## Short Names

The primary Bluetooth interface uses short readable names.

### Required Setpoints

| Modbus | Short name | Example |
| --- | --- | --- |
| `103` `WINDOWS_POS_A_TARGET` | `win_a_pos` | `set win_a_pos 50` |
| `104` `WINDOWS_POS_B_TARGET` | `win_b_pos` | `set win_b_pos 25.5` |
| `105` `CURTAIN_POS_TARGET` | `curt_pos` | `set curt_pos 100` |
| `106` `SP_WATER_RAIL` | `sp_rail` | `set sp_rail 35.0` |
| `107` `SP_WATER_GROW` | `sp_grow` | `set sp_grow 32.0` |
| `108` `SP_WATER_UPPER` | `sp_upper` | `set sp_upper 36.0` |
| `109` `SP_WATER_UNDERTRAY` | `sp_under` | `set sp_under 30.0` |
| `111` `LIGHT_R1_ON_HHMM` | `l1_on` | `set l1_on 14:00` |
| `112` `LIGHT_R1_OFF_HHMM` | `l1_off` | `set l1_off 22:00` |
| `117` `LIGHT_R2_ON_HHMM` | `l2_on` | `set l2_on 14:00` |
| `118` `LIGHT_R2_OFF_HHMM` | `l2_off` | `set l2_off 22:00` |
| `122` `LIGHT_HYST_SEC` | `light_hyst` | `set light_hyst 30` |
| `171` `WINDOWS_CTRL_MODE` | `win_mode` | `set win_mode manual` |

### Window Automation Names

| Modbus | Short name | Example |
| --- | --- | --- |
| `195` `WINDOWS_AUTO_ALGO_MODE` | `win_alg` | `set win_alg temp` |
| `219` `WINDOWS_TEMP_STEP_TARGET_PERCENT` | `temp_open` | `set temp_open 20` |
| `196` `WINDOWS_HUM_SETPOINT` | `hum_sp` | `set hum_sp 80` |
| `197` `WINDOWS_HUM_STEP` | `hum_step` | `set hum_step 5` |
| `198` `WINDOWS_HUM_STEP_HYST` | `hum_hyst` | `set hum_hyst 1` |
| `221` `WINDOWS_HUM_STEP_TARGET_PERCENT` | `hum_open` | `set hum_open 20` |
| `199` `WINDOWS_COLD_CLOSE_DELTA` | `cold_close` | `set cold_close 2` |
| `200` `WINDOWS_COLD_CLOSE_HYST` | `cold_hyst` | `set cold_hyst 0.5` |
| `201` `WINDOWS_WINDWARD_MIN_PERCENT` | `windward_min` | `set windward_min 0` |
| `202` `WINDOWS_WINDWARD_MAX_PERCENT` | `windward_max` | `set windward_max 60` |
| `203` `WINDOWS_WINDWARD_SPEED_THRESHOLD` | `windward_thr` | `set windward_thr 1` |
| `204` `WINDOWS_WINDWARD_REDUCTION_PERCENT_PER_MS` | `windward_reduce` | `set windward_reduce 10` |
| `205` `WINDOWS_LEEWARD_MIN_PERCENT` | `leeward_min` | `set leeward_min 0` |
| `206` `WINDOWS_LEEWARD_MAX_PERCENT` | `leeward_max` | `set leeward_max 100` |
| `207` `WINDOWS_LEEWARD_SPEED_THRESHOLD` | `leeward_thr` | `set leeward_thr 8` |
| `208` `WINDOWS_LEEWARD_REDUCTION_PERCENT_PER_MS` | `leeward_reduce` | `set leeward_reduce 0` |
| `209` `WINDOWS_WINDWARD_LAG_PERCENT` | `wind_lag` | `set wind_lag 20` |
| `210` `WINDOWS_RAIN_MODE` | `rain_mode` | `set rain_mode windward` |
| `211` `WINDOWS_RAIN_WINDWARD_PERCENT` | `rain_pos` | `set rain_pos 0` |
| `223` `WINDOWS_WEATHER_STALE_TIMEOUT_MS` | `wx_stale_ms` | `set wx_stale_ms 20000` |
| `224` `WINDOWS_WEATHER_SOURCE_AGE_S` | `wx_age_max` | `set wx_age_max 20` |

### Not Used In Autonomous Bluetooth Control

| Modbus | Status |
| --- | --- |
| `113` `LIGHT_R1_THRESHOLD_WM2` | not used |
| `114` `RESERVED` | not used |
| `115` `LIGHT_R1_DLI_OFF_LIMIT_JCM2` | not used |
| `119` `LIGHT_R2_THRESHOLD_WM2` | not used |
| `120` `RESERVED` | not used |
| `121` `LIGHT_R2_DLI_OFF_LIMIT_JCM2` | not used |

### Optional Names

| Modbus | Short name | Example |
| --- | --- | --- |
| `110` `LIGHT_R1_ENABLE` | `l1_en` | `set l1_en on` |
| `116` `LIGHT_R2_ENABLE` | `l2_en` | `set l2_en off` |

### Weather Mock Names

| Weather field | Short name | Example |
| --- | --- | --- |
| Outside temperature, `C` | `wx_temp` | `set wx_temp 12.3` |
| Outside humidity, `%` | `wx_hum` | `set wx_hum 78.5` |
| Wind speed, `m/s` | `wx_wind` | `set wx_wind 4.2` |
| Wind direction, `deg` | `wx_dir` | `set wx_dir 180` |
| Rain flag, `0/1` or `off/on` | `wx_rain` | `set wx_rain on` |
| Solar radiation, `W/m2` | `wx_solar` | `set wx_solar 650` |
| Barometric pressure, `hPa` | `wx_baro` | `set wx_baro 1008.4` |
| Dew point, `C` | `wx_dew` | `set wx_dew 6.1` |
| Source age, `s` | `wx_age` | `set wx_age 3` |
| Status bits | `wx_stat` | `set wx_stat 1` |

## Supported Commands

### Introspection

- `help`
- `show autonomous`
- `show mode`
- `show weather`
- `show windows`
- `show light r1`
- `show light r2`

### Short Command Form

- `set win_a_pos 50`
- `set win_b_pos 25.5`
- `set_a_pos 50`
- `set_b_pos 25.5`
- `set win_mode manual`
- `set win_alg temp`
- `set win_alg hum`
- `set temp_open 20`
- `set hum_sp 80`
- `set hum_step 5`
- `set hum_hyst 1`
- `set hum_open 20`
- `set cold_close 2`
- `set cold_hyst 0.5`
- `set windward_min 0`
- `set windward_max 60`
- `set windward_thr 1`
- `set windward_reduce 10`
- `set leeward_min 0`
- `set leeward_max 100`
- `set leeward_thr 8`
- `set leeward_reduce 0`
- `set wind_lag 20`
- `set rain_mode windward`
- `set rain_pos 0`
- `set wx_stale_ms 20000`
- `set wx_age_max 20`
- `set curt_pos 100`
- `set sp_rail 35.0`
- `set sp_grow 32.0`
- `set sp_upper 36.0`
- `set sp_under 30.0`
- `set l1_on 14:00`
- `set l1_off 22:00`
- `set l2_on 14:00`
- `set l2_off 22:00`
- `set light_hyst 30`
- `set wx_temp 12.3`
- `set wx_hum 78.5`
- `set wx_wind 4.2`
- `set wx_dir 180`
- `set wx_rain on`
- `set wx_solar 650`
- `set wx_baro 1008.4`
- `set wx_dew 6.1`
- `set wx_age 3`
- `set wx_stat 1`

Values:

- Position targets are in percent, valid range `0..100`
- `win_mode` accepts `auto`, `manual`, `0`, or `1`
- `win_alg` accepts `temp`, `hum`, `temperature`, `humidity`, `0`, or `1`
- Window position targets are accepted only while system `mode=AUTONOMOUS`
- Window automation percent values use `0..100`
- `wx_stale_ms` is weather receive timeout in milliseconds, effective range `1000..60000`; `0` falls back to default
- `wx_age_max` is maximum weather source age in seconds, effective range `1..600`; `0` falls back to default
- Wind thresholds are in `m/s`, valid range `0..100.0`
- Wind reduction is in `% per m/s` above the role threshold
- `rain_mode` accepts `off`, `on`, or `windward`
- Water setpoints are in degrees Celsius, valid range `0..120.0`
- Time accepts either `HH:MM` or `HHMM`
- Threshold, DLI and hysteresis are integer values
- `wx_temp` range: `-60.0..80.0 C`
- `wx_hum` range: `0..100 %`
- `wx_wind` range: `0..100.0 m/s`
- `wx_dir` range: `0..359`
- `wx_rain` accepts `0/1`, `off/on`, `false/true`
- `wx_solar` and `wx_stat` are integer values
- `wx_baro` range: `300.0..1200.0 hPa`
- `wx_dew` range: `-80.0..80.0 C`
- `wx_age` is source age in seconds

### Legacy Long Form

The previous long-form syntax is still accepted for compatibility:

- `set windows a pos target 50`
- `set curtain pos target 100`
- `set sp water rail 35.0`
- `set light r1 on hhmm 14:00`
- `set light r1 off hhmm 22:00`
- `set light r1 enable on`
- `set light hyst sec 30`
- `set weather temp 12.3`
- `set weather hum 78.5`
- `set weather wind 4.2`
- `set weather dir 180`
- `set weather rain on`
- `set weather solar 650`
- `set weather baro 1008.4`
- `set weather dew 6.1`
- `set weather age 3`
- `set weather status 1`

## Responses

- Success replies start with `OK`
- Validation or syntax problems start with `ERR`

Example:

```text
set win_a_pos 50
OK win_a_pos=50.0% requested=50.0% mode=AUTO

show windows
OK windows win_mode=MANUAL alg=TEMP windward=A requested[a=50.0% b=0.0%] base[a=50.0% b=0.0%] effective[a=40.0% b=60.0%] pos[a=... b=...] prot=0x00C0 ...

set wx_solar 650
OK wx_solar=650W/m2 token=1 result=APPLIED

show weather
OK weather valid=1 stale=0 token=1 applied=1 result=APPLIED ...
```
