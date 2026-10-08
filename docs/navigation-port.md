# Navigation on the summer XSFC baseline

Integrated directly into `main`, based on summer `c637c20`, after the XSFC
rename. No extra branch or worktree is required.

## Included

- MS5611 command transactions, checked reads, PROM CRC and compensation.
- Barometer calibration, validity, measured sample intervals and vertical
  estimation using the existing summer attitude/accelerometer pipeline.
- VL53L0X and external MSP rangefinder profiles, range validation, tilt
  compensation, vertical velocity and stale-sample handling.
- Matek/MTF optical-flow profiles, alignment, scaling, rotation compensation,
  range-dependent velocity and earth-frame position hold.
- ALTHOLD (barometer), RANGEHOLD (rangefinder) and POSHOLD (flow + range).
- Navigation CLI settings/readouts, MSP feeds and mode enumeration.
- Sensor-loss latches: turn the affected hold switch off/on after recovery.
- Disarmed configuration edits require save/restart rather than partially
  resetting running navigation filters and hardware across cores.

## Preserved boundaries

Summer gyro/accelerometer drivers, calibration, attitude fusion, RC processing,
rate PID, ESC drivers, queue and hardware pin defaults remain unchanged except
for the XSFC rename. Navigation data uses snapshots; IMU transport does not.
The filter API adds only nonzero `reset(value)` initialization. Existing filter
coefficients, updates and zero reset are unchanged.

Mixer arithmetic is unchanged. Motor-stop at low pilot throttle is bypassed
only while altitude hold is engaged, so navigation throttle remains effective.
Failsafe landing and ICM20948 timing/bypass changes are **not** included.

## Configuration

EEPROM format is `0x07`. The summer `0x01` layout preserves hardware settings,
AUX assignments and non-navigation tuning. Altitude/position-hold tuning is
initialized to the new defaults because its units changed. New appended flow
settings are initialized without copying old structure tail padding.

Autumn `0x02`/`0x03`/`0x04` navigation layouts are recognized by exact version
and size. Landing/polling tail fields from autumn are not imported. This does
not add autumn IMU features; review gyro and loop settings when using an autumn
configuration. Export a CLI dump before switching. Older firmware cannot read
`0x07`, so restoring it requires restoring the previous exported configuration.

Host regression tests are available in `bin/test_baro.ps1`. Firmware builds,
host tests and hardware validation have not been run during this port.
