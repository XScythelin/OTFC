# OTFC CLI: подробная памятка (команды, параметры, режимы, конфликты)

Документ собран по фактической реализации CLI/Controller/Actuator/MSP в проекте.

Важно:

- Везде ниже значения указаны так, как реально читаются в коде.
- Если параметр не ограничивается явно в setter, в CLI можно записать почти любое число, и это может привести к неадекватному поведению.
- После save рекомендуется reboot, чтобы гарантированно применить изменения во всех подсистемах.

## 1) Вход/выход и базовая механика CLI

- Вход в CLI: отправить символ `#` в serial stream.
- При входе включается флаг `ARMING_DISABLED_CLI`.
- CLI привязывается к одному stream: команды из другого stream игнорируются.
- Выход из CLI:
- `reboot` или `exit`: выход + `Hardware::restart()`.
- `Ctrl-D`: выход без сохранения (`unsaved changes lost`).

## 2) Полный список top-level команд

Команды, которые реально обрабатываются в `execute()`:

- `help`
- `version`
- `wifi` (условно, если WiFi-сборка)
- `tasks` (условно, FreeRTOS)
- `devinfo`
- `get`
- `set`
- `dump`
- `cal`
- `gps`
- `preset`
- `load`
- `save`
- `eeprom` (заглушка)
- `scaler`
- `mixer`
- `status`
- `baro`
- `rangefinder`
- `i2cscan`
- `flow`
- `stats`
- `reboot`
- `exit`
- `defaults`
- `motors`
- `logs`
- `flash` (условно, если USE_FLASHFS)

## 3) Подробный синтаксис каждой команды

### 3.1 `help`

Синтаксис:

- `help`

Действие: печатает список команд.

### 3.2 `version`

Синтаксис:

- `version`

Действие: печатает версию/таргет.

### 3.3 `devinfo`

Синтаксис:

- `devinfo`

Действие: версия, частота CPU, размеры `ModelConfig`/`ModelState`, free heap.

### 3.4 `wifi` (если включено)

Синтаксис:

- `wifi`

Действие: печатает ST/AP IP, MAC, mode, channel.

### 3.5 `tasks` (если включено)

Синтаксис:

- `tasks`

Действие: печатает число задач RTOS.

### 3.6 `get`

Синтаксис:

- `get`
- `get <substring>`

Параметры:

- `<substring>`: подстрока-фильтр по имени параметра.

Действие:

- Без аргумента: печатает все параметры в формате `set name value`.
- С аргументом: только те, где `name` содержит подстроку.

### 3.7 `set`

Синтаксис:

- `set <param> <value...>`
- `set <param> = <value...>`

Действие:

- Обновляет параметр по его типу (bool/int/enum/string/bitmask/структура).
- Сразу печатает обновленное значение.

Ошибки:

- Если нет `<param>`: `param required`.
- Если имя неизвестно: `param not found`.

### 3.8 `dump`

Синтаксис:

- `dump`

Действие:

- Печатает скрипт: `defaults` + все `set ...` + `save`.

### 3.9 `defaults`

Синтаксис:

- `defaults`

Действие:

- `Model::reset()` (сброс к дефолтам).
- Не сохраняет автоматически.

### 3.10 `save`

Синтаксис:

- `save`

Действие:

- Сохраняет конфиг в storage.
- Печатает подсказку `Saved, type reboot to apply changes`.

### 3.11 `load`

Синтаксис:

- `load`

Действие:

- `Model::load()` из storage.

### 3.12 `reboot` / `exit`

Синтаксис:

- `reboot`
- `exit`

Действие:

- Завершает CLI и перезапускает железо.

### 3.13 `status`

Синтаксис:

- `status`

Действие:

- Версия, counters, обнаруженные девайсы, input-rate.
- Печатает `arm flags` (все активные `ArmingDisabledFlags`).
- Печатает `sensor fault` (NO_BARO/NO_RANGEFINDER/NO_FLOW).

### 3.14 `stats`

Синтаксис:

- `stats`

Действие:

- Детальные тайминги/нагрузка по счетчикам задач.

### 3.15 `logs`

Синтаксис:

- `logs`

Действие:

- Печатает ring-buffer логгера + текущее использование.

### 3.16 `motors`

Синтаксис:

- `motors`

Действие:

- Печатает count моторов и текущее `pin/us` для каналов.

### 3.17 `cal`

Синтаксис:

- `cal`
- `cal gyro`
- `cal mag`
- `cal reset_accel`
- `cal reset_gyro`
- `cal reset_mag`
- `cal reset_all`

Действие:

- Без аргумента: печатает offsets/scales gyro/accel/mag.
- `gyro`: калибровка gyro (только если не armed).
- `mag`: калибровка mag (только если не armed).
- `reset_*`: сброс runtime-оффсетов.

### 3.18 `gps`

Синтаксис:

- `gps`
- `gps set_home`
- `gps clear_home`

Действие:

- Без аргумента: GPS status.
- `set_home`: попытка зафиксировать home (нужен fix).
- `clear_home`: сброс флага home.

### 3.19 `preset`

Синтаксис:

- `preset`
- `preset scaler`
- `preset modes`
- `preset micrus`
- `preset brobot`

Действие:

- `preset`: печать списка.
- `scaler`: ставит преднастройки `scaler[0..2]`.
- `modes`: заполняет несколько `conditions[]` шаблоном.
- `micrus`/`brobot`: сейчас только `OK` без массивной логики внутри CLI.

### 3.20 `scaler`

Синтаксис:

- `scaler`

Действие:

- Печать активных scaler-правил и вычисленного scale.

### 3.21 `mixer`

Синтаксис:

- `mixer`

Действие:

- Печать активного mixer как набор `set mix_*`.

### 3.22 `baro`

Синтаксис:

- `baro`

Действие: barometer status.

### 3.23 `rangefinder`

Синтаксис:

- `rangefinder`

Действие: rangefinder status.

### 3.24 `i2cscan`

Синтаксис:

- `i2cscan`

Действие: скан I2C адресов.

### 3.25 `flow`

Синтаксис:

- `flow`
- `flow watch`
- `flow watch <period_ms>`
- `flow watch stop`

Параметры:

- `<period_ms>`: период вывода, минимум 10 ms (меньше будет принудительно 10).

Действие:

- `flow`: вывод статусной информации потока.
- `flow watch`: запуск фонового стрима (`t_ms q v n mx my`).
- Остановка watch: `flow watch stop` или клавиша `q`.

### 3.26 `flash` (если USE_FLASHFS)

Синтаксис:

- `flash`
- `flash partitions`
- `flash journal`
- `flash erase`
- `flash test`
- `flash print [addr] [size]`

Параметры:

- `addr`: стартовый адрес чтения (по умолчанию 0).
- `size`: длина, в коде clamp от 8 до 128*1024.

Действие:

- `flash`: total/used/free.
- `erase`: полный erase flashfs.
- `print`: сырое чтение чанками по 256.

### 3.27 `eeprom`

Синтаксис:

- `eeprom`

Статус: в текущем коде тело закомментировано (фактически no-op).

## 4) Полная логика `set`: типы, парсинг, важные нюансы

### 4.1 Формы записи

- `set name value`
- `set name = value`

`=` отдельным токеном допустим (CLI это поддерживает).

### 4.2 Типы скалярных параметров

- `bool`: принимает только `0` или `1`.
- `byte/short/int`: `String(...).toInt()`.
- `float`: `String(...).toFloat()`.
- `string`: копируется в буфер с maxLen.
- `enum`: можно числом индекса или текстом choice (case-insensitive).
- `bitmask` (`feature_*`, `blackbox_log_*`): `0/1` чистит/ставит конкретный бит.

### 4.3 Составные параметры (важно: многопараметричные)

#### `input_N`

Формат:

- `set input_N <map> <min> <neutral> <max> <fsMode> <fsValue>`

Поля:

- `map`: индекс входа/маппинга.
- `min/neutral/max`: в us.
- `fsMode`: `A` (auto), `H` (hold), `S` (set).
- `fsValue`: целевое значение в failsafe для режима `S`.

Дефолты структуры:

- `min=1000`, `neutral=1500`, `max=2000`, `map=0`, `fsMode=0(A)`, `fsValue=1500`.

#### `output_N`

Формат:

- `set output_N <type> <reverse> <min> <neutral> <max>`

Поля:

- `type`: `S` servo или `M` motor.
- `reverse`: `R` reverse или `N` normal.
- `min/neutral/max`: выходные лимиты канала.

#### `serial_X`

Формат:

- `set serial_X <functionMask> <baud> <blackboxBaud>`

Поля:

- `functionMask`: битовая маска функций порта.
- `baud`: основная скорость порта.
- `blackboxBaud`: скорость для blackbox-стрима.

#### `scaler_X`

Формат:

- `set scaler_X <dimension> <channel> <minScale> <maxScale>`

Поля:

- `dimension`: битовая маска `ScalerDimension`.
- `channel`: канал управления скейлером (обычно AUX).
- `minScale/maxScale`: масштаб в процентах (внутри хранится как int16).

#### `mode_X`

Формат:

- `set mode_X <id> <ch> <min> <max> <logicMode> <linkId>`

Поля:

- `id`: ID FlightMode.
- `ch`: индекс канала (`AXIS_AUX_1` и выше).
- `min/max`: диапазон активации в us (условие строгое: `val > min && val < max`).
- `logicMode`: доп. логика связи (сохраняется/идет в MSP).
- `linkId`: идентификатор связи (сохраняется/идет в MSP).

Критично:

- В текущем runtime (`updateModeMask`) используются только `id/ch/min/max`.
- `logicMode/linkId` сейчас не участвуют в фактическом решении включения режима.

#### `mix_N`

Формат:

- `set mix_N <src> <dst> <rate>`

Поля и ограничения:

- `src`: clamp `0..MIXER_SOURCE_MAX-1`.
- `dst`: clamp `0..OUTPUT_CHANNELS-1`.
- `rate`: clamp `-1000..1000`.

## 5) Режимы и BOX (полная карта)

### 5.1 FlightMode ID

- `0` MODE_ARMED
- `1` MODE_AIRMODE
- `2` MODE_ANGLE
- `3` MODE_ALTHOLD
- `4` MODE_BUZZER
- `5` MODE_FAILSAFE
- `6` MODE_BLACKBOX
- `7` MODE_BLACKBOX_ERASE
- `8` MODE_COUNT (служебно)
- `9` MODE_GPIO_OUTPUT
- `10` MODE_MAG
- `11` MODE_SURFACE
- `12` MODE_POSHOLD

### 5.2 BOX names, которые публикуются по MSP

- ARM
- AIRMODE
- ANGLE
- ALTHOLD
- BEEPER
- FAILSAFE
- BLACKBOX
- BLACKBOXERASE
- GPIO
- MAG
- SURFACE
- POSHOLD

## 6) Условия активации режимов и безопасность

Алгоритм:

- Из `conditions[0..7]` строится `newMask` по диапазонам каналов.
- Пишется `maskSwitch` (что запросили тумблеры).
- Добавляется принудительный FAILSAFE-бит при активной failsafe phase.
- Для каждого режима, который изменился с 0 на 1, вызывается `canActivateMode`.
- Если `canActivateMode` вернул false, бит срезается.

`canActivateMode`:

- `MODE_ARMED`: можно только если `!armingDisabled && throttleLow`.
- `MODE_ANGLE`: нужен активный accel.
- `MODE_AIRMODE`: нужен `airmodeAllowed`.
- `MODE_ALTHOLD`: нужен `baroReadyForAltHold()`.
- `MODE_SURFACE`: нужен `baroReadyForAltHold()`.
- Остальные: true.

Дополнительные флаги арминга:

- `ARMING_DISABLED_BOXFAILSAFE`: если switch FAILSAFE активен.
- `ARMING_DISABLED_ARM_SWITCH`: если ARM switch включен, но арминг запрещен по другим причинам.

## 7) ALTHOLD/SURFACE/POSHOLD: максимально подробно

## 7.1 Общая связка

- `isAltHoldActive()` = `MODE_ALTHOLD || MODE_SURFACE`.
- `isPosHoldActive()` = `MODE_POSHOLD`.
- POSHOLD-контур работает только в ANGLE + ARMED + valid flow + valid rangefinder.

## 7.2 ALTHOLD: параметры и эффект каждого

Параметры:

- `pid_althold_throttle_mode` (enum): `STICK`, `MID_STICK`, `HOVER`.
- `pid_althold_deadband` (%): deadband вокруг нейтрали газа.
- `pid_althold_manual_climb_rate` (cm/s): максимум ручной вертикальной скорости.
- `pid_althold_sensor_fault_disarm_delay` (ms): задержка до forced disarm при персистентной неисправности.
- `pid_althold_baro_fallback` (bool): разрешить fallback при нездоровом баро.
- `pid_althold_iterm_center` (%): центр hover-базы для режима HOVER.
- `pid_althold_iterm_range` (%): диапазон i-term around center.
- `pid_althold_baro_p_weight` (%/loop): вес позиционной баро-коррекции.
- `pid_althold_baro_v_weight` (%/sample): вес baro vario-коррекции.
- `pid_althold_acc_bias_ground` (x1e-4): bias alpha на земле.
- `pid_althold_acc_bias_air` (x1e-4): bias alpha в полете.

Дефолты (из `AltHoldConfig`):

- throttle_mode=`STICK`
- deadband=`15`
- manual_climb_rate=`100`
- sensor_fault_disarm_delay=`250`
- baro_fallback=`true`
- iterm_center=`50`
- iterm_range=`50`
- baro_p_weight=`2`
- baro_v_weight=`20`
- surface_weight=`5`
- acc_bias_ground=`100`
- acc_bias_air=`5`

Как работает throttle mode:

- `STICK`: нейтраль берется с текущего стика, если газ не low; иначе mid-stick.
- `MID_STICK`: нейтраль всегда 0.0.
- `HOVER`: нейтраль = map(`itermCenter`, 0..100 -> -1..1).

Как формируется setpoint по газу:

- Стик проходит через deadband.
- Нормализуется в `[-1..1]`.
- Умножается на `manual_climb_rate` (внутри переводится из cm/s в m/s).

## 7.3 SURFACE: параметры и реальные требования

Параметры:

- `pid_althold_surface_weight` (%): доля rangefinder в высотной смеси.
- `rangefinder_dev`, `rangefinder_bus`, `rangefinder_max_range`.
- `rangefinder_lpf_type`, `rangefinder_lpf_freq`.
- `feature_rangefinder` должен быть включен.

Ключевые условия:

- Для активации `MODE_SURFACE` все равно проверяется `baroReadyForAltHold()`.
- Для фактической работы surface нужен valid rangefinder.
- `surface_weight < 100%`: baro участвует.
- `surface_weight == 100%`: baro-участие в смеси минимально, но gate на активацию через baro остается.

## 7.4 POSHOLD: параметры и формулы

Параметры:

- `pid_poshold_pos_p`, `pid_poshold_pos_i`.
- `pid_poshold_vel_p`, `pid_poshold_vel_i`, `pid_poshold_vel_d`.
- `poshold_flow_gain_x`, `poshold_flow_gain_y`.
- `poshold_deadband` (%).
- `poshold_angle_limit` (deg).
- `poshold_use_gyro_comp` (bool).
- `poshold_vel_lpf_type`, `poshold_vel_lpf_freq`.

Условия включения POSHOLD-контура:

- `MODE_POSHOLD` активен.
- `MODE_ANGLE` активен.
- `MODE_ARMED` активен.
- flow свежий (`<=200ms`) и valid.
- rangefinder active+valid.
- высота rangefinder `>0.05m`.

Вычисления:

- Flow motion -> angular rate -> ground velocity через `flowGain*height`.
- При `useGyroComp=1` вычитается gyro-компонента вращения.
- Position PID строит velocity setpoint.
- Velocity PID строит lean angle setpoint.

## 8) Конфликты/перекрытия (важно в полете)

- `POSHOLD` без `ANGLE`: switch может гореть, но удержание не включится.
- `POSHOLD` без flow/rangefinder: немедленный disengage, сброс внутренних интеграторов poshold.
- `SURFACE` без rangefinder: `NO_RANGEFINDER`, при persist fault возможен disarm.
- `ALTHOLD/SURFACE` при критическом sensor fault: forced disarm после `pid_althold_sensor_fault_disarm_delay`.
- `Airmode`: switch недостаточен до тех пор, пока не выполнено условие `airmodeAllowed` (после arm и throttle > 1400us).
- `MODE_ARMED`: никогда не включится, если любой armingDisabled flag активен.

## 9) Арминг-флаги, которые чаще всего мешают

Список из status (`arm flags`) включает:

- `NO_GYRO`, `FAILSAFE`, `RX_FAILSAFE`, `BOXFAILSAFE`, `THROTTLE`, `ANGLE`, `CALIBRATING`, `CLI`, `GPS`, `REBOOT_REQUIRED`, `MOTOR_PROTOCOL`, `NO_BARO`, `NO_RANGEFINDER`, `NO_FLOW`, `ARM_SWITCH` и др.

Практика:

- Для althold/surface/poshold ключевые fault-флаги: `NO_BARO`, `NO_RANGEFINDER`, `NO_FLOW`.

## 10) Готовые команды настройки ALTHOLD/SURFACE/POSHOLD

Пример ALTHOLD:

- `set mode_3 3 5 1700 2100 0 0`
- `set pid_althold_throttle_mode STICK`
- `set pid_althold_deadband 15`
- `set pid_althold_manual_climb_rate 100`
- `set pid_althold_sensor_fault_disarm_delay 250`
- `save`
- `reboot`

Пример SURFACE:

- `set feature_rangefinder 1`
- `set rangefinder_dev VL53L0X`
- `set mode_4 11 6 1700 2100 0 0`
- `set pid_althold_surface_weight 20`
- `save`
- `reboot`

Пример POSHOLD:

- `set mode_5 12 7 1700 2100 0 0`
- `set mode_2 2 6 1700 2100 0 0`
- `set poshold_deadband 5`
- `set poshold_angle_limit 20`
- `set poshold_use_gyro_comp 1`
- `save`
- `reboot`

## 11) Полный каталог `set`-параметров

Ниже полный список имен параметров, которые объявлены в таблице CLI.

Примечание по точности:

- `telemetry` в исходнике сейчас закомментирован и фактически недоступен.
- `telemetry_interval` доступен.

### 11.1 Feature / Debug

- feature_gps, feature_rangefinder, feature_dyn_notch, feature_motor_stop, feature_rx_ppm, feature_rx_serial, feature_rx_spi, feature_soft_serial, feature_telemetry
- debug_mode, debug_axis

### 11.2 Gyro / Accel / Mag / Baro / Rangefinder

- gyro_bus, gyro_dev, gyro_dlpf, gyro_align, gyro_lpf_type, gyro_lpf_freq, gyro_lpf2_type, gyro_lpf2_freq, gyro_lpf3_type, gyro_lpf3_freq
- gyro_notch1_freq, gyro_notch1_cutoff, gyro_notch2_freq, gyro_notch2_cutoff
- gyro_dyn_lpf_min, gyro_dyn_lpf_max, gyro_dyn_notch_q, gyro_dyn_notch_count, gyro_dyn_notch_min, gyro_dyn_notch_max
- gyro_rpm_harmonics, gyro_rpm_q, gyro_rpm_min_freq, gyro_rpm_fade, gyro_rpm_weight_1, gyro_rpm_weight_2, gyro_rpm_weight_3, gyro_rpm_tlm_lpf_freq
- gyro_offset_x, gyro_offset_y, gyro_offset_z
- accel_bus, accel_dev, accel_lpf_type, accel_lpf_freq, accel_offset_x, accel_offset_y, accel_offset_z, accel_trim_roll, accel_trim_pitch
- mag_bus, mag_dev, mag_align, mag_filter_type, mag_filter_lpf, mag_offset_x, mag_offset_y, mag_offset_z, mag_scale_x, mag_scale_y, mag_scale_z
- baro_bus, baro_dev, baro_lpf_type, baro_lpf_freq
- rangefinder_bus, rangefinder_dev, rangefinder_max_range, rangefinder_lpf_type, rangefinder_lpf_freq

### 11.3 GPS / Board / Battery

- gps_min_sats, gps_set_home_once, gps_gnss_mode
- gps_enable_dual_band, gps_enable_gps, gps_enable_glonass, gps_enable_galileo, gps_enable_beidou, gps_enable_qzss, gps_enable_sbas
- board_align_roll, board_align_pitch, board_align_yaw
- vbat_source, vbat_scale, vbat_mul, vbat_div, vbat_cell_warn
- ibat_source, ibat_scale, ibat_offset

### 11.4 Fusion / Input / Failsafe / Arming / VTX

- fusion_mode, fusion_gain_p, fusion_gain_i
- input_rate_type
- input_roll_rate, input_roll_srate, input_roll_expo, input_roll_limit
- input_pitch_rate, input_pitch_srate, input_pitch_expo, input_pitch_limit
- input_yaw_rate, input_yaw_srate, input_yaw_expo, input_yaw_limit
- input_deadband, input_min, input_mid, input_max
- input_interpolation, input_interpolation_interval
- input_filter_type, input_lpf_type, input_lpf_freq, input_lpf_factor, input_ff_lpf_type, input_ff_lpf_freq
- input_rssi_channel
- input_0, input_1, input_2, input_3, input_4, input_5, input_6, input_7, input_8, input_9, input_10, input_11, input_12, input_13, input_14, input_15
- failsafe_delay, failsafe_kill_switch
- arming_small_angle
- vtx_power, vtx_channel, vtx_band, vtx_low_power_disarm

### 11.5 Serial / Scaler / Mode / PID

- serial_0, serial_1, serial_2, serial_soft_0, serial_usb
- scaler_0, scaler_1, scaler_2
- mode_0, mode_1, mode_2, mode_3, mode_4, mode_5, mode_6, mode_7
- pid_sync
- pid_roll_p, pid_roll_i, pid_roll_d, pid_roll_f
- pid_pitch_p, pid_pitch_i, pid_pitch_d, pid_pitch_f
- pid_yaw_p, pid_yaw_i, pid_yaw_d, pid_yaw_f
- pid_level_p, pid_level_i, pid_level_d, pid_level_f
- pid_level_angle_limit, pid_level_rate_limit, pid_level_lpf_type, pid_level_lpf_freq
- pid_mag_p
- pid_althold_pos_p, pid_althold_pos_i, pid_althold_pos_d, pid_althold_pos_f
- pid_althold_vel_p, pid_althold_vel_i, pid_althold_vel_d, pid_althold_vel_f
- pid_althold_throttle_mode, pid_althold_deadband, pid_althold_manual_climb_rate, pid_althold_sensor_fault_disarm_delay
- pid_althold_baro_fallback, pid_althold_iterm_center, pid_althold_iterm_range, pid_althold_baro_p_weight, pid_althold_baro_v_weight, pid_althold_surface_weight
- pid_althold_acc_bias_ground, pid_althold_acc_bias_air
- pid_poshold_pos_p, pid_poshold_pos_i, pid_poshold_vel_p, pid_poshold_vel_i, pid_poshold_vel_d
- poshold_flow_gain_x, poshold_flow_gain_y, poshold_deadband, poshold_angle_limit, poshold_use_gyro_comp, poshold_vel_lpf_type, poshold_vel_lpf_freq
- pid_yaw_lpf_type, pid_yaw_lpf_freq
- pid_dterm_lpf_type, pid_dterm_lpf_freq, pid_dterm_lpf2_type, pid_dterm_lpf2_freq, pid_dterm_notch_freq, pid_dterm_notch_cutoff, pid_dterm_dyn_lpf_min, pid_dterm_dyn_lpf_max
- pid_dterm_weight, pid_iterm_limit, pid_iterm_zero, pid_iterm_relax, pid_iterm_relax_cutoff, pid_tpa_scale, pid_tpa_breakpoint

### 11.6 Mixer / Output / Pin / I2C / Telemetry / Blackbox / WiFi / Mix

- mixer_sync, mixer_type, mixer_yaw_reverse, mixer_throttle_limit_type, mixer_throttle_limit_percent, mixer_output_limit
- output_motor_protocol, output_motor_async, output_motor_rate, output_motor_poles, output_servo_rate
- output_min_command, output_min_throttle, output_max_throttle, output_dshot_idle, output_dshot_telemetry
- output_0, output_1, output_2, output_3, output_4, output_5, output_6, output_7
- pin_input_rx, pin_output_0, pin_output_1, pin_output_2, pin_output_3, pin_output_4, pin_output_5, pin_output_6, pin_output_7
- pin_button, pin_buzzer, pin_led
- pin_serial_0_tx, pin_serial_0_rx, pin_serial_1_tx, pin_serial_1_rx, pin_serial_2_tx, pin_serial_2_rx
- pin_i2c_scl, pin_i2c_sda, pin_input_adc_0, pin_input_adc_1
- pin_spi_0_sck, pin_spi_0_mosi, pin_spi_0_miso, pin_spi_cs_0, pin_spi_cs_1, pin_spi_cs_2
- pin_buzzer_invert, pin_led_invert, pin_led_type
- i2c_speed, rescue_config_delay, telemetry_interval
- blackbox_dev, blackbox_mode, blackbox_rate
- blackbox_log_acc, blackbox_log_alt, blackbox_log_bat, blackbox_log_debug, blackbox_log_gps, blackbox_log_gyro, blackbox_log_gyro_raw, blackbox_log_mag, blackbox_log_motor, blackbox_log_pid, blackbox_log_rc, blackbox_log_rpm, blackbox_log_rssi, blackbox_log_sp
- blackbox_log_nav_sensors, blackbox_log_nav_pid_auto, blackbox_log_nav_pid_force
- model_name, wifi_ssid, wifi_pass, wifi_tcp_port, mix_outputs
- mix_0, mix_1, mix_2, mix_3, mix_4, mix_5, mix_6, mix_7, mix_8, mix_9, mix_10, mix_11, mix_12, mix_13, mix_14, mix_15
- mix_16, mix_17, mix_18, mix_19, mix_20, mix_21, mix_22, mix_23, mix_24, mix_25, mix_26, mix_27, mix_28, mix_29, mix_30, mix_31
- mix_32, mix_33, mix_34, mix_35, mix_36, mix_37, mix_38, mix_39, mix_40, mix_41, mix_42, mix_43, mix_44, mix_45, mix_46, mix_47
- mix_48, mix_49, mix_50, mix_51, mix_52, mix_53, mix_54, mix_55, mix_56, mix_57, mix_58, mix_59, mix_60, mix_61, mix_62, mix_63

## 12) MSP/Configurator нюанс по режимам

- `MSP_MODE_RANGES` и `MSP_SET_MODE_RANGE` поддержаны.
- `MSP_MODE_RANGES_EXTRA` передает `logicMode/linkId`.
- Текущая runtime-активация в `Actuator::updateModeMask()` использует только `id/ch/min/max`.

Следствие:

- В Configurator можно настроить link/logic, но в полетной логике они сейчас не влияют.

## 13) Короткий чек-лист диагностики по вашим режимам

Для `ALTHOLD`:

- `status` -> нет `NO_BARO`
- `baro` -> обновляется/валиден
- `get pid_althold_`

Для `SURFACE`:

- `status` -> нет `NO_RANGEFINDER`
- `rangefinder` -> valid height
- `get pid_althold_surface_weight`

Для `POSHOLD`:

- активны `MODE_POSHOLD` + `MODE_ANGLE` + `MODE_ARMED`
- `flow`/`flow watch 100` -> качество и валидность потока
- `rangefinder` -> valid, высота > 0.05m