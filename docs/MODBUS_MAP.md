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
- The output latch is a cascade of three `74HC595` chips, `24` logical bits.
- `bit0` -> Light relay 1
- `bit1` -> Light relay 2
- `bit2` -> Rail 3-way valve `OPEN`
- `bit3` -> Rail 3-way valve `CLOSE`
- `bit4` -> Upper 3-way valve `OPEN`
- `bit5` -> Upper 3-way valve `CLOSE`
- `bit6` -> Undertray 3-way valve `OPEN`
- `bit7` -> Undertray 3-way valve `CLOSE`
- `bit8` -> Grow pipe 3-way valve `OPEN`
- `bit9` -> Grow pipe 3-way valve `CLOSE`
- `bit10` -> Rail pump contactor
- `bit11` -> Upper pump contactor
- `bit12` -> Undertray pump contactor
- `bit13` -> Grow pipe pump contactor
- `bit14` -> Curtain `OPEN`
- `bit15` -> Curtain `CLOSE`
- `bit16..23` -> reserved
- Valve outputs have an interlock: `OPEN` and `CLOSE` for the same valve are never driven together.

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

## Window Control, `171..224`
- `171` WINDOWS_CTRL_MODE: `0=AUTO`, `1=MANUAL`
- `172` WINDOWS_FORCE_SAFE_CMD
- `173` WINDOWS_TEMP_SETPOINT, `x10 C`
- `174` WINDOWS_SAFE_MIN_PERCENT, `x10 %`
- `175` WINDOWS_WIND_LIMIT, legacy/default threshold, `x10 m/s`
- `176` WINDOWS_WIND_STORM, common storm threshold, `x10 m/s`
- `177` WINDOWS_WIND_RECOVER, storm recovery threshold, `x10 m/s`
- `178` WINDOW_A_AZIMUTH_DEG
- `179` WINDOWS_WIND_SECTOR_HALF_WIDTH_DEG
- `180` WINDOWS_TEMP_STEP_C, `x10 C`
- `181` WINDOWS_TEMP_STEP_HYST_C, `x10 C`
- `182` RLL400_TARGET_HYST_PERCENT, `x10 %`; also stabilizes small
  effective-target changes before they are sent to RLL400
- `183` ACTUATOR_MOTION_DELTA_PERCENT, `x10 %`; shared by windows and curtain
- `184` ACTUATOR_NO_MOTION_TIMEOUT_MS; shared by windows and curtain, `0` disables no-motion fault
- `185` WINDOW_A_FAULT_RESET_TOKEN
- `186` WINDOW_B_FAULT_RESET_TOKEN
- `187` WINDOWS_STATUS_BITS
- `188` WINDOW_A_STATUS_BITS
- `189` WINDOW_B_STATUS_BITS
- `190` WINDOW_A_FAULT_CODE
- `191` WINDOW_B_FAULT_CODE
- `192` AIR_TEMP_SENSOR_STATUS
- `193` WINDOW_A_LOCAL_MANUAL_ACTIVE
- `194` WINDOW_B_LOCAL_MANUAL_ACTIVE
- `195` WINDOWS_AUTO_ALGO_MODE: `0=TEMP`, `1=HUMIDITY`
- `196` WINDOWS_HUM_SETPOINT, `x10 %`
- `197` WINDOWS_HUM_STEP, `x10 %`
- `198` WINDOWS_HUM_STEP_HYST, `x10 %`
- `199` WINDOWS_COLD_CLOSE_DELTA, `x10 C`
- `200` WINDOWS_COLD_CLOSE_HYST, `x10 C`
- `201` WINDOWS_WINDWARD_MIN_PERCENT, `x10 %`
- `202` WINDOWS_WINDWARD_MAX_PERCENT, `x10 %`
- `203` WINDOWS_WINDWARD_SPEED_THRESHOLD, `x10 m/s`
- `204` WINDOWS_WINDWARD_REDUCTION_PERCENT_PER_MS, `x10 %/m/s`
- `205` WINDOWS_LEEWARD_MIN_PERCENT, `x10 %`
- `206` WINDOWS_LEEWARD_MAX_PERCENT, `x10 %`
- `207` WINDOWS_LEEWARD_SPEED_THRESHOLD, `x10 m/s`
- `208` WINDOWS_LEEWARD_REDUCTION_PERCENT_PER_MS, `x10 %/m/s`
- `209` WINDOWS_WINDWARD_LAG_PERCENT, `x10 %`
- `210` WINDOWS_RAIN_MODE: `0=OFF`, `1=WINDWARD`
- `211` WINDOWS_RAIN_WINDWARD_PERCENT, `x10 %`
- `212` WINDOWS_WEATHER_STALE_POLICY: `0=CLOSE_SAFE`, `1=IGNORE`
- `213` WINDOWS_BASE_TARGET_A, `x10 %`, RO
- `214` WINDOWS_BASE_TARGET_B, `x10 %`, RO
- `215` WINDOWS_EFFECTIVE_TARGET_A, `x10 %`, RO
- `216` WINDOWS_EFFECTIVE_TARGET_B, `x10 %`, RO
- `217` WINDOWS_ACTIVE_PROTECTION_BITS, RO
- `218` WINDOWS_WINDWARD_SIDE: `0=NONE`, `1=A`, `2=B`, `3=BOTH_UNKNOWN`, RO
- `219` WINDOWS_TEMP_STEP_TARGET_PERCENT, `x10 %`
- `220` WINDOWS_TEMP_STEP_MAX_INDEX, legacy/reserved
- `221` WINDOWS_HUM_STEP_TARGET_PERCENT, `x10 %`
- `222` WINDOWS_HUM_STEP_MAX_INDEX, legacy/reserved

Wind reduction is applied to the already calculated target:
`wind_target = base_target - (wind_speed - threshold) * reduction`.
Then the wind target is clamped by the role limits:
`wind_target = clamp(wind_target, min_percent, max_percent)`.
For the windward window, `209 WINDOWS_WINDWARD_LAG_PERCENT` also limits the
windward target relative to the leeward target. Storm, rain, and cold-close
protections have priority over the wind min/max limits and may command a target
outside those limits. The wind speed used by the formula is held until the
measured wind changes by at least `1.0 m/s`. This gives one target recalculation
per `1 m/s` instead of reacting to every `0.1 m/s` update from the weather
station.
- `223` WINDOWS_WEATHER_STALE_TIMEOUT_MS
- `224` WINDOWS_WEATHER_SOURCE_AGE_S

## Heating Control, `225..242`
- `225` HEATING_CTRL_MODE: `0=AUTO`, `1=OFF`, `2=MANUAL`
- `226` HEATING_AIR_SETPOINT, `x10 C`, default `20.0 C`
- `227` HEATING_AIR_HYST, `x10 C`, default `0.5 C`
- `228` HEATING_STAGE_DELTA_1, `x10 C`, default `0.3 C`
- `229` HEATING_STAGE_DELTA_2, `x10 C`, default `1.0 C`
- `230` HEATING_STAGE_DELTA_3, `x10 C`, default `2.0 C`
- `231` HEATING_STAGE_DELTA_4, `x10 C`, default `3.0 C`
- `232` HEATING_MIN_ON_S, default `60 s`
- `233` HEATING_MIN_OFF_S, default `30 s`
- `234` HEATING_MANUAL_PUMP_MASK`
- `235` HEATING_MANUAL_VALVE_OPEN_MASK`
- `236` HEATING_MANUAL_VALVE_CLOSE_MASK`
- `237` HEATING_STATUS_BITS, RO
- `238` HEATING_ACTIVE_STAGE, RO
- `239` HEATING_PUMP_MASK, RO
- `240` HEATING_VALVE_OPEN_MASK, RO
- `241` HEATING_VALVE_CLOSE_MASK, RO
- `242` HEATING_SENSOR_STATUS_BITS, RO

### Heating Algorithm
- In `AUTO`, heating stages are selected by air temperature deficit:
  `HEATING_AIR_SETPOINT - AIR_TEMP`.
- Stage order: `rail -> upper -> undertray -> grow`.
- Stage masks:
  - stage `0`: all contours off
  - stage `1`: `rail`
  - stage `2`: `rail + upper`
  - stage `3`: `rail + upper + undertray`
  - stage `4`: `rail + upper + undertray + grow`
- If the air temperature sensor is invalid, all pumps are disabled and valves go to fail-safe close/stop.
- Each active contour enables its pump only when its water sensor is valid.
- Active contour valve control:
  - `actual_water > setpoint + hysteresis` -> `CLOSE`
  - `actual_water < setpoint - hysteresis` -> `OPEN`
  - inside the hysteresis band -> `STOP`
- Inactive or water-fault contours turn the pump off and close the valve for a limited safe-close window, then stop the valve output.
- In `MANUAL`, masks `234..236` directly request pump/open/close outputs; close requests are ignored where the same contour also has an open request.

### Heating Masks
- `bit0` -> rail
- `bit1` -> upper
- `bit2` -> undertray
- `bit3` -> grow

### Heating Status Bits (`237`)
- `bit0` heating enabled, at least one pump active
- `bit1` manual mode active
- `bit2` off mode active
- `bit3` air temperature sensor fault
- `bit4` at least one water temperature sensor fault
- `bit5` min-on hold active
- `bit6` min-off hold active

### Heating Sensor Status Bits (`242`)
- `bit0` air temperature fault
- `bit1` rail water temperature fault
- `bit2` upper water temperature fault
- `bit3` undertray water temperature fault
- `bit4` grow water temperature fault

## Curtain Control, `243..274`
- `243` CURTAIN_CTRL_MODE: `0=AUTO`, `1=MANUAL`, `2=OFF`
- `244` CURTAIN_MANUAL_TARGET, `x10 %`
- `245` CURTAIN_SCHEDULE_START_HHMM
- `246` CURTAIN_SCHEDULE_END_HHMM
- `247` CURTAIN_OUTSIDE_TARGET, `x10 %`, reserved for outside-schedule policy
- `248` CURTAIN_MIN_POSITION, `x10 %`
- `249` CURTAIN_MAX_POSITION, `x10 %`
- `250` CURTAIN_POSITION_HYST, `x10 %`
- `251` CURTAIN_RADIATION_THRESHOLD, `W/m2`
- `252` CURTAIN_RADIATION_STEP_WM2, `W/m2`
- `253` CURTAIN_RADIATION_STEP_PERCENT, `x10 %`; when radiation is active, target is `248 + step_count * 253`
- `254` CURTAIN_RADIATION_HYST, `W/m2`
- `255` CURTAIN_COLD_DELTA, `x10 C`; with `275`, cold target activates below `275 - 255 - 256` and releases at `275 - 255`
- `256` CURTAIN_COLD_HYST, `x10 C`
- `257` CURTAIN_COLD_TARGET, `x10 %`
- `258` CURTAIN_HEAT_DELTA, `x10 C`; with `275`, heat target activates above `275 + 258 + 259` and releases at `275 + 258`
- `259` CURTAIN_HEAT_HYST, `x10 C`
- `260` CURTAIN_HEAT_TARGET, `x10 %`
- `261` CURTAIN_HUM_LOW_DELTA, `x10 %RH`; with `276`, low-humidity target activates below `276 - 261 - 262` and releases at `276 - 261`
- `262` CURTAIN_HUM_LOW_HYST, `x10 %RH`
- `263` CURTAIN_HUM_LOW_TARGET, `x10 %`
- `264` CURTAIN_HUM_HIGH_DELTA, `x10 %RH`; with `276`, high-humidity target activates above `276 + 264 + 265` and releases at `276 + 264`
- `265` CURTAIN_HUM_HIGH_HYST, `x10 %RH`
- `266` CURTAIN_HUM_HIGH_TARGET, `x10 %`
- `267` CURTAIN_TARGET, `x10 %`, RO
- `268` CURTAIN_BASE_TARGET, `x10 %`, RO
- `269` CURTAIN_CURRENT_MA, `x10 mA`, RO
- `270` CURTAIN_STATUS_BITS, RO
- `271` CURTAIN_REASON_BITS, RO
- `272` CURTAIN_POSITION_STATUS_BITS, RO
- `273` CURTAIN_FAULT_CODE, RO
- `274` CURTAIN_FAULT_RESET_TOKEN

In AUTO mode, the curtain base target is register `248` (`CURTAIN_MIN_POSITION`).
Active automatic rules assign their own target (`radiation`, `257`, `260`,
`263`, or `266`). If no automatic rule is active, the target remains `248`.
If several rules are active at the same time, target priority is:
overheating (`260`) -> overcooling (`257`) -> high humidity (`266`) ->
low humidity (`263`) -> radiation.

## Global Greenhouse Targets, `275..276`
- `275` AIR_TEMP_TARGET, `x10 C`; greenhouse air temperature target used by curtain and shared greenhouse control logic
- `276` AIR_HUM_TARGET, `x10 %`; greenhouse air humidity target used by curtain and shared greenhouse control logic

### Curtain Status Bits (`270`)
- `bit0` output enabled
- `bit1` manual mode
- `bit2` auto mode
- `bit3` off mode
- `bit4` position valid
- `bit5` moving open
- `bit6` moving close
- `bit7` at target
- `bit8` fault active
- `bit9` OPEN output active
- `bit10` CLOSE output active

### Curtain Reason Bits (`271`)
- `bit0` schedule active
- `bit1` radiation active
- `bit2` cold close active
- `bit3` heat open active
- `bit4` humidity low open active
- `bit5` humidity high close active
- `bit6` manual mode
- `bit7` outside schedule
- `bit8` temperature sensor fault
- `bit9` humidity sensor fault
- `bit10` radiation fault
- `bit11` time fault

### Curtain Position Status Bits (`272`)
- `bit0` position valid
- `bit1` at target
- `bit2` moving open
- `bit3` moving close
- `bit4` encoder fault
- `bit5` no-motion fault

### Curtain Fault Codes (`273`)
- `0` NONE
- `1` ENCODER
- `2` NO_MOTION
- `3` OUTPUT

### Window Protection Bits
- `bit0` force safe active
- `bit1` weather stale safe policy active
- `bit2` storm active
- `bit3` cold close active
- `bit4` rain limiting A
- `bit5` rain limiting B
- `bit6` wind limiting A
- `bit7` wind limiting B
- `bit8` temperature sensor fault
- `bit9` humidity sensor fault

Weather is treated as unsafe/stale by the window controller when the weather
snapshot is missing, stale by age, or `WEATHER_STATUS_BITS bit15` is set.
Normal station status bits such as `bit0=active` and `bit1=data valid` do not
trigger weather safe closing.

## Recommended HMI Set For Windows

### Daily Operator Set
- `103` WINDOWS_POS_A_TARGET, `x10 %`, manual target for window A
- `104` WINDOWS_POS_B_TARGET, `x10 %`, manual target for window B
- `171` WINDOWS_CTRL_MODE: `0=AUTO`, `1=MANUAL`
- `173` WINDOWS_TEMP_SETPOINT, `x10 C`
- `180` WINDOWS_TEMP_STEP_C, `x10 C`
- `181` WINDOWS_TEMP_STEP_HYST_C, `x10 C`
- `195` WINDOWS_AUTO_ALGO_MODE: `0=TEMP`, `1=HUMIDITY`
- `196` WINDOWS_HUM_SETPOINT, `x10 %`
- `197` WINDOWS_HUM_STEP, `x10 %`
- `198` WINDOWS_HUM_STEP_HYST, `x10 %`
- `199` WINDOWS_COLD_CLOSE_DELTA, `x10 C`
- `200` WINDOWS_COLD_CLOSE_HYST, `x10 C`
- `219` WINDOWS_TEMP_STEP_TARGET_PERCENT, `x10 %`
- `221` WINDOWS_HUM_STEP_TARGET_PERCENT, `x10 %`

### Protection And Climate Tuning
- `172` WINDOWS_FORCE_SAFE_CMD
- `174` WINDOWS_SAFE_MIN_PERCENT, `x10 %`
- `176` WINDOWS_WIND_STORM, `x10 m/s`
- `177` WINDOWS_WIND_RECOVER, `x10 m/s`
- `178` WINDOW_A_AZIMUTH_DEG
- `179` WINDOWS_WIND_SECTOR_HALF_WIDTH_DEG
- `201` WINDOWS_WINDWARD_MIN_PERCENT, `x10 %`
- `202` WINDOWS_WINDWARD_MAX_PERCENT, `x10 %`
- `203` WINDOWS_WINDWARD_SPEED_THRESHOLD, `x10 m/s`
- `204` WINDOWS_WINDWARD_REDUCTION_PERCENT_PER_MS, `x10 %/m/s`
- `205` WINDOWS_LEEWARD_MIN_PERCENT, `x10 %`
- `206` WINDOWS_LEEWARD_MAX_PERCENT, `x10 %`
- `207` WINDOWS_LEEWARD_SPEED_THRESHOLD, `x10 m/s`
- `208` WINDOWS_LEEWARD_REDUCTION_PERCENT_PER_MS, `x10 %/m/s`
- `209` WINDOWS_WINDWARD_LAG_PERCENT, `x10 %`
- `210` WINDOWS_RAIN_MODE: `0=OFF`, `1=WINDWARD`
- `211` WINDOWS_RAIN_WINDWARD_PERCENT, `x10 %`
- `212` WINDOWS_WEATHER_STALE_POLICY: `0=CLOSE_SAFE`, `1=IGNORE`
- `223` WINDOWS_WEATHER_STALE_TIMEOUT_MS
- `224` WINDOWS_WEATHER_SOURCE_AGE_S

### Commissioning And Service
- `182` RLL400_TARGET_HYST_PERCENT, `x10 %`; used as motor deadband and
  effective-target stabilization deadband
- `183` ACTUATOR_MOTION_DELTA_PERCENT, `x10 %`; shared by windows and curtain
- `184` ACTUATOR_NO_MOTION_TIMEOUT_MS; shared by windows and curtain, `0` disables no-motion fault

### Usually Hide From The Operator Panel
- `175` WINDOWS_WIND_LIMIT, legacy/default fallback
- `185` WINDOW_A_FAULT_RESET_TOKEN, service
- `186` WINDOW_B_FAULT_RESET_TOKEN, service
- `187..194` status and fault telemetry, RO
- `213..218` computed diagnostics, RO
- `220` WINDOWS_TEMP_STEP_MAX_INDEX, legacy/reserved
- `222` WINDOWS_HUM_STEP_MAX_INDEX, legacy/reserved

## Persist/Reboot
- Active light config и `ACTIVE_CTRL_VERSION` сохраняются в NVS (`light_state`) с CRC32.
- После reboot staging и active light config восстанавливаются из NVS.
- Weather snapshot хранится только в RAM и после reboot не восстанавливается.
- Weather stale является внутренним состоянием slave и сам по себе не переводит узел в `AUTONOMOUS`.
