# Modbus Карта Регистров

Формат значений по умолчанию: `x10`, если рядом не указано иное.

## Telemetry (RO), `0..8`
- `0` AIR_TEMP
- `1` AIR_HUM
- `2` WATER_RAIL
- `3` WATER_GROW
- `4` WATER_UNDERTRAY
- `5` WATER_UPPER_HEAT
- `6` WINDOWS_POS_A
- `7` WINDOWS_POS_B
- `8` CURTAIN_POS

## Diagnostics (RO), `9..14`
- `9` MODE_STATE
- `10` MODE_REASON
- `11` LAST_MASTER_SEEN_MS_LO
- `12` LAST_MASTER_SEEN_MS_HI
- `13` GOOD_CYCLE_STREAK
- `14` LAST_APPLY_STATUS

## Control (RW), `100..127`
- `100` CTRL_VERSION_HI
- `101` CTRL_VERSION_LO
- `102` MODE_CMD
- `103` WINDOWS_POS_A_TARGET
- `104` WINDOWS_POS_B_TARGET
- `105` CURTAIN_POS_TARGET
- `106` SP_WATER_RAIL
- `107` SP_WATER_GROW
- `108` SP_WATER_UPPER
- `109` SP_WATER_UNDERTRAY
- `110` LIGHT_R1_ENABLE
- `111` LIGHT_R1_ON_HHMM
- `112` LIGHT_R1_OFF_HHMM
- `113` LIGHT_R1_THRESHOLD_WM2
- `114` RESERVED
- `115` LIGHT_R1_DLI_OFF_LIMIT_JCM2
- `116` LIGHT_R2_ENABLE
- `117` LIGHT_R2_ON_HHMM
- `118` LIGHT_R2_OFF_HHMM
- `119` LIGHT_R2_THRESHOLD_WM2
- `120` RESERVED
- `121` LIGHT_R2_DLI_OFF_LIMIT_JCM2
- `122` LIGHT_HYST_SEC
- `123` CTRL_CRC_LO (`legacy/reserved`)
- `124` CTRL_CRC_HI (`legacy/reserved`)
- `125` APPLY_STATUS
- `126` ACTIVE_CTRL_VERSION_HI
- `127` ACTIVE_CTRL_VERSION_LO

### Water Valve Channels
- `valve_3way` component controls one physical 3-way valve per handle
- If the device has two valves, create two independent `valve_3way` handles
- `106 SP_WATER_RAIL` -> setpoint for rail valve
- `107 SP_WATER_GROW` -> setpoint for grow valve
- `108 SP_WATER_UPPER` -> setpoint for upper valve
- `109 SP_WATER_UNDERTRAY` -> setpoint for undertray valve
- Actual water temperature is measured locally by the ESP32 via `max31865`
- The server sends only the target setpoint for the selected circuit
- Control rule: if `actual_temp > setpoint + hysteresis`, command `CLOSE`
- Control rule: if `actual_temp < setpoint - hysteresis`, command `OPEN`
- Inside the hysteresis band both GPIO outputs are forced `OFF`
- Each valve channel has an interlock: `OPEN` and `CLOSE` outputs cannot be active at the same time

### 74HC595 Output Mapping
- ESP32 `GPIO2` -> `74HC595 SER/DS`
- ESP32 `GPIO4` -> `74HC595 SHCP/SRCLK`
- ESP32 `GPIO18` -> `74HC595 STCP/RCLK`
- `Q0` -> Light relay 1
- `Q1` -> Light relay 2
- `Q2` -> 3-way valve `OPEN`
- `Q3` -> 3-way valve `CLOSE`
- Current implementation uses `74HC595` for the two light relays and one `3-way valve`

### Bluetooth ASCII aliases for autonomous setpoints
- Required: `103..109`, `111..112`, `117..118`, `122`
- `103` -> `win_a_pos`
- `104` -> `win_b_pos`
- `105` -> `curt_pos`
- `106` -> `sp_rail`
- `107` -> `sp_grow`
- `108` -> `sp_upper`
- `109` -> `sp_under`
- `111` -> `l1_on`
- `112` -> `l1_off`
- `113..115` -> not used in autonomous Bluetooth control
- `117` -> `l2_on`
- `118` -> `l2_off`
- `119..121` -> not used in autonomous Bluetooth control
- `122` -> `light_hyst`
- Optional: `110` -> `l1_en`, `116` -> `l2_en`
- See `docs/BLUETOOTH_ASCII.md` for command examples.

## Light Runtime
- `134` LIGHT_CURRENT_DLI_JCM2 (`RW`, текущее накопленное `Дж/см²`, пишет мастер)
- `135` LIGHT_OUTPUT_PERCENT (`RO`, суммарный выход `0/50/100`)
- `136` LIGHT_STATUS_BITS (`RO`)

### Light Status Bits (`136`)
- `bit0` relay 1: расписание активно
- `bit1` relay 2: расписание активно
- `bit2` relay 1: активна задержка физического включения
- `bit3` relay 2: активна задержка физического включения
- `bit4` relay 1: отключено по DLI
- `bit5` relay 2: отключено по DLI
- `bit6` relay 1: удержание по `LIGHT_HYST_SEC`
- `bit7` relay 2: удержание по `LIGHT_HYST_SEC`
- `bit8` weather stale
- `bit9` relay 1: выход реально включен
- `bit10` relay 2: выход реально включен

## RTC Sync, `140..144`
- `140` RTC_SET_HOUR
- `141` RTC_SET_MINUTE
- `142` RTC_SET_TOKEN
- `143` RTC_SET_APPLIED_TOKEN
- `144` RTC_SET_RESULT

## Apply Diagnostics, `145..155`
- `145` APPLY_OK_COUNT_HI
- `146` APPLY_OK_COUNT_LO
- `147` APPLY_FAIL_INVALID_COUNT_HI
- `148` APPLY_FAIL_INVALID_COUNT_LO
- `149` APPLY_FAIL_BUSY_COUNT_HI
- `150` APPLY_FAIL_BUSY_COUNT_LO
- `151` APPLY_FAIL_INTERNAL_COUNT_HI
- `152` APPLY_FAIL_INTERNAL_COUNT_LO
- `153` LAST_APPLY_ERROR_CODE
- `154` LAST_APPLY_TS_MS_HI
- `155` LAST_APPLY_TS_MS_LO

## Weather Sync, `158..170`
- `158` WEATHER_OUT_TEMP
- `159` WEATHER_OUT_HUM
- `160` WEATHER_WIND_SPEED
- `161` WEATHER_WIND_DIR
- `162` WEATHER_RAIN_FLAG
- `163` WEATHER_SOLAR_RAD
- `164` WEATHER_BARO_PRESS
- `165` WEATHER_DEW_POINT
- `166` WEATHER_STATUS_BITS
- `167` WEATHER_AGE_S
- `168` WEATHER_SET_TOKEN
- `169` WEATHER_SET_APPLIED_TOKEN
- `170` WEATHER_SET_RESULT

## Логика Освещения
- Мгновенная радиация для света берется из `163 WEATHER_SOLAR_RAD`.
- Каждый relay работает независимо и имеет свой блок `ENABLE/ON/OFF/THRESHOLD/DLI`.
- Вне расписания relay выключен.
- Если `LIGHT_CURRENT_DLI_JCM2 >= relay_DLI_limit`, relay выключается с приоритетом над порогами `Вт/м²`.
- Если `THRESHOLD == 0` или `WEATHER_SOLAR_RAD < THRESHOLD`, целевое состояние relay = `ON`.
- Если `WEATHER_SOLAR_RAD >= THRESHOLD`, целевое состояние relay = `OFF`.
- `LIGHT_HYST_SEC` общий для обоих relay и задает временное удержание перед сменой устойчивого состояния.
- После перехода relay в устойчивое `ON` физический выход включается не сразу, а через `zone_id * 10 s`.
- В текущей реализации `zone_id` равен `slave_id`, сохраненному в NVS.
- `LIGHT_OUTPUT_PERCENT` в `135` рассчитывается по реально включенным выходам: `0`, `50`, `100`.

## Apply Семантика Световой Конфигурации
- Предпочтительный способ: один `FC16(110, 13)` для полного блока `110..122`.
- Полная запись `110..122` ставит конфиг в apply сразу.
- Частичные записи в `110..122` сначала попадают в staging, затем автоматически применяются после `250 ms` без новых изменений.
- Успех: `125 = 0` и изменение `126..127`.
- Ошибка: `125 != 0`, версия `126..127` не меняется.

## APPLY Status (`125`)
- `0` APPLIED/OK
- `2` INVALID_RANGE
- `3` BUSY
- `5` INTERNAL_ERROR

## RTC Result (`144`)
- `2` APPLIED
- `5` NOOP
- `3` REJECT_RANGE
- `4` FAILED

## Weather Result (`170`)
- `2` APPLIED
- `5` NOOP
- `4` FAILED

## Persist/Reboot
- Active light config и `ACTIVE_CTRL_VERSION` сохраняются в NVS (`light_state`) с CRC32.
- После reboot staging и active light config восстанавливаются из NVS.
- Weather snapshot хранится только в RAM и после reboot не восстанавливается.
- Weather stale является внутренним состоянием slave и сам по себе не переводит узел в `AUTONOMOUS`.
