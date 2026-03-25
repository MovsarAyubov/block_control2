# MASTER_SEQUENCE: Последовательность Операций Мастера

Документ описывает рекомендуемый алгоритм обмена с ESP32 slave по Modbus RTU.

## 1. Параметры Линии
- Протокол: Modbus RTU
- UART: 19200, 8N1
- `slave_id`: берется из пусконаладки

## 2. Базовые Правила
1. Любой валидный Modbus-запрос считается heartbeat для slave.
2. При отсутствии heartbeat более `30 s` slave уходит в `AUTONOMOUS`.
3. Возврат в `REMOTE`: 3 успешных цикла связи подряд.
4. Для света мастер обязан передавать:
   - текущее накопленное `DLI` в `134`
   - weather snapshot в `158..168`

## 3. Стартовая Инициализация

### Шаг 1: Проверка Связи
- Прочитать:
  - `9` MODE_STATE
  - `10` MODE_REASON
  - `11..12` LAST_MASTER_SEEN_MS
  - `13` GOOD_CYCLE_STREAK
  - `14` LAST_APPLY_STATUS

### Шаг 2: Передать Световую Конфигурацию
Предпочтительный вариант:
1. Записать `FC16(110, 13)` для полного блока `110..122`.
2. Читать:
   - `125` APPLY_STATUS
   - `126..127` ACTIVE_CTRL_VERSION
3. Успех: `125 = 0` и версия `126..127` изменилась.

Допустимый вариант для multi-write:
1. Записать `110..122` несколькими запросами.
2. После последней записи выдержать не менее `250 ms`.
3. Читать `125` и `126..127`.
4. Slave сам поставит staging-конфиг в apply после паузы без новых изменений.

### Состав Блока `110..122`
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

## 4. Периодический Runtime-Цикл Мастера

Рекомендуемый период: `1 s` или быстрее при необходимости.

### Шаг 1: Передача Текущего DLI
- Записать в `134` текущее накопленное значение `Дж/см²`.

### Шаг 2: Передача Weather Snapshot
- Записать `FC16(158, 11)`:
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
- Затем читать `169` и `170`.
- Обработка завершена, когда `169 == token`.
- Успех для мастера: `170 = 2` или `170 = 5`.

### Шаг 3: Чтение Телеметрии
- Прочитать `0..8`.

### Шаг 4: Чтение Статусов
- Прочитать `9..14`.
- Дополнительно прочитать:
  - `135` LIGHT_OUTPUT_PERCENT
  - `136` LIGHT_STATUS_BITS

## 5. RTC-Синхронизация
1. Записать:
   - `140` RTC_SET_HOUR
   - `141` RTC_SET_MINUTE
2. Записать новый ненулевой token в `142`.
3. Читать `143` и `144`, пока `143 == token`.

Интерпретация `144`:
- `2` APPLIED
- `5` NOOP
- `3` REJECT_RANGE
- `4` FAILED

## 6. Интерпретация Световой Логики
- `163 WEATHER_SOLAR_RAD` является мгновенной радиацией для обоих relay.
- Каждый relay имеет собственные `ENABLE/ON/OFF/THRESHOLD/DLI`.
- Если `THRESHOLD == 0` или `WEATHER_SOLAR_RAD < THRESHOLD`, relay стремится включиться.
- Если `WEATHER_SOLAR_RAD >= THRESHOLD`, relay стремится выключиться.
- `LIGHT_HYST_SEC` общий для обоих relay и задает выдержку в секундах перед сменой устойчивого состояния.
- После перехода relay в устойчивое `ON` физическое включение задерживается на `zone_id * 10 s`.
- В текущей реализации `zone_id = slave_id`.

## 7. Интерпретация Ключевых Статусов
- `125 APPLY_STATUS`:
  - `0` APPLIED/OK
  - `2` INVALID_RANGE
  - `3` BUSY
  - `5` INTERNAL_ERROR
- `135 LIGHT_OUTPUT_PERCENT`:
  - `0`
  - `50`
  - `100`
- `136 LIGHT_STATUS_BITS`:
  - `bit0` relay 1: расписание активно
  - `bit1` relay 2: расписание активно
  - `bit2` relay 1: активна задержка включения
  - `bit3` relay 2: активна задержка включения
  - `bit4` relay 1: отключено по DLI
  - `bit5` relay 2: отключено по DLI
  - `bit6` relay 1: удержание по гистерезису
  - `bit7` relay 2: удержание по гистерезису
  - `bit8` weather stale
  - `bit9` relay 1: выход реально включен
  - `bit10` relay 2: выход реально включен
- `170 WEATHER_SET_RESULT`:
  - `2` APPLIED
  - `5` NOOP
  - `4` FAILED

## 8. Минимальный Набор Запросов Для Надежной Работы
Каждый цикл:
1. Write `134`
2. Write `FC16(158, 11)`
3. Read `169..170` до подтверждения token
4. Read `0..8`
5. Read `9..14`
6. Read `135..136`

Этого достаточно для heartbeat, мониторинга, управления светом и доставки weather snapshot.
