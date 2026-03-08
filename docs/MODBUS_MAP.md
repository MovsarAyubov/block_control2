# Modbus карта регистров (кратко)

Формат значений по умолчанию: `x10` (fixed point), если не указано иное.

## Telemetry (RO), 0..8
- 0 AIR_TEMP
- 1 AIR_HUM
- 2 WATER_RAIL
- 3 WATER_GROW
- 4 WATER_UNDERTRAY
- 5 WATER_UPPER_HEAT
- 6 WINDOWS_POS_A
- 7 WINDOWS_POS_B
- 8 CURTAIN_POS

## Control (RW), 100..127
- 100 CTRL_VERSION_HI
- 101 CTRL_VERSION_LO
- 102 MODE_CMD
- 103 WINDOWS_POS_A_TARGET
- 104 WINDOWS_POS_B_TARGET
- 105 CURTAIN_POS_TARGET
- 106 SP_WATER_RAIL
- 107 SP_WATER_GROW
- 108 SP_WATER_UPPER
- 109 SP_WATER_UNDERTRAY
- 110 SCH0_ENABLE
- 111 SCH0_ON_HHMM
- 112 SCH0_OFF_HHMM
- 113 SCH1_ENABLE
- 114 SCH1_ON_HHMM
- 115 SCH1_OFF_HHMM
- 116 SCH2_ENABLE
- 117 SCH2_ON_HHMM
- 118 SCH2_OFF_HHMM
- 119 SCH3_ENABLE
- 120 SCH3_ON_HHMM
- 121 SCH3_OFF_HHMM
- 122 APPLY_CMD (W, триггер apply; каждая успешная запись в этот регистр = попытка apply)
- 123 CTRL_CRC_LO (legacy/reserved)
- 124 CTRL_CRC_HI (legacy/reserved)
- 125 APPLY_STATUS (RO, статус последнего apply)
- 126 ACTIVE_CTRL_VERSION_HI (RO)
- 127 ACTIVE_CTRL_VERSION_LO (RO)

## Diagnostics/runtime
- 128 MODE_STATE (0=REMOTE, 1=AUTONOMOUS)
- 129 MODE_REASON
- 130 LAST_MASTER_SEEN_MS_LO
- 131 LAST_MASTER_SEEN_MS_HI
- 132 GOOD_CYCLE_STREAK
- 133 LAST_APPLY_STATUS
- 134 SOLAR_RADIATION_X10 (RW, пишет мастер)
- 135 SOLAR_UPPER_THRESHOLD_X10 (RW)
- 136 LIGHT_REDUCTION_ACTIVE (RO, 0/1)
- 140 RTC_SET_HOUR (W, 0..23)
- 141 RTC_SET_MINUTE (W, 0..59)
- 142 RTC_SET_TOKEN (W, ненулевой новый токен запускает обработку)
- 143 RTC_SET_APPLIED_TOKEN (RO, токен последнего обработанного запроса)
- 144 RTC_SET_RESULT (RO)
- 145 APPLY_OK_COUNT_HI
- 146 APPLY_OK_COUNT_LO
- 147 APPLY_FAIL_INVALID_COUNT_HI
- 148 APPLY_FAIL_INVALID_COUNT_LO
- 149 APPLY_FAIL_BUSY_COUNT_HI
- 150 APPLY_FAIL_BUSY_COUNT_LO
- 151 APPLY_FAIL_INTERNAL_COUNT_HI
- 152 APPLY_FAIL_INTERNAL_COUNT_LO
- 153 LAST_APPLY_ERROR_CODE
- 154 LAST_APPLY_TS_MS_HI
- 155 LAST_APPLY_TS_MS_LO

## Свет
Логика реле вычисляется внутри slave:
- Вне расписания: 0%
- В расписании и радиация < порога: 100%
- В расписании и радиация >= порога: 50%

Дополнительно:
- `110..121` - staging schedule (теневая конфигурация).
- active schedule меняется только в apply-процедуре после записи в `122`.
- Для `ENABLE=1` интервал `ON == OFF` считается невалидным (reject).
- Интервалы через полночь (`ON > OFF`) валидны.
- Если новый apply приходит во время обработки предыдущего: очередь глубиной 1 (`latest wins`).

## APPLY status (`125`)
- 0 APPLIED/OK
- 2 INVALID_RANGE
- 3 BUSY
- 5 INTERNAL_ERROR

## Persist/Reboot стратегия
- Active schedule + `ACTIVE_CTRL_VERSION` сохраняются в NVS (`light_state`) с CRC32.
- При невалидном/отсутствующем blob используется safe-default: все `SCHx_ENABLE=0`.
- После старта `110..121` отражают staging schedule (из сохраненного active либо safe-default).

## RTC sync (`RTC_SET_RESULT`, регистр 144)
- 2 APPLIED (RTC обновлен)
- 5 NOOP (расхождение ниже порога `RTC_SYNC_THRESHOLD_MIN`)
- 3 REJECT_RANGE (невалидные `hour/minute`)
- 4 FAILED (ошибка чтения/записи RTC)
