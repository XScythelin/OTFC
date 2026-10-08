# MS5611, vertical estimation and altitude hold

## Reference and scope

The reference is INAV **9.1.0**, commit
`e519b69b02e27c8bdc03b4a0889f1baaae211a54`, checked out locally in
`reference/inav` (ignored by XSFC Git). Relevant reference files:

- `src/main/drivers/barometer/barometer_ms56xx.c`
- `src/main/sensors/barometer.c`
- `src/main/common/calibration.c`
- `src/main/navigation/navigation_pos_estimator.c`
- `src/main/navigation/navigation_multicopter.c`
- `src/main/navigation/sqrt_controller.c`

XSFC implements the MS5611 equations and the barometer-based vertical-control
principles in its own C++ architecture. This is not a complete or bit-for-bit
port of INAV navigation: there is no INAV GPS altitude blending, air-cushion
detection, landing detector, temperature-drift calibration, or full navigation
failsafe here. Existing XSFC attitude estimation, PID implementation and mixer
remain in use. Default INAV vertical gains are converted from cm/s and PWM
correction to m/s and normalized XSFC throttle, including the actual mixer
throttle span.

## Driver

[BaroMS5611](../lib/Xsfc/src/Device/Baro/BaroMS5611.cpp) supports direct I2C
addresses `0x76` and `0x77`, and SPI with an explicitly configured chip select.
It checks reset acknowledgement, PROM reads and CRC, and uses OSR4096.
Each conversion waits 12 ms, retaining a margin for the slower modules
previously supported by XSFC; INAV uses 10 ms.

Pressure compensation uses **first-order** temperature for the below-20 C
and below-minus-15 C corrections. Corrected temperature is reported separately.
Invalid/short ADC reads and failed conversion commands produce invalid samples,
not repeated old pressure. Runtime bus error callbacks remain enabled.
SPI command transactions do not add the register-read bit or strip command bits;
other SPI devices retain their existing register transaction behavior.

Set `baro_dev MS5611` to require that device rather than merely include it in
autodetection. Direct I2C/SPI is supported; MS5611 is not probed through a gyro's
auxiliary/slave I2C bus.

## Ground zero, height and vario

[BaroAltitude](../lib/Xsfc/src/Sensor/BaroAltitude.hpp) requires a continuous
two-second window of valid pressure samples while disarmed. Like INAV 9.1.0,
calibration rejects pressure standard deviation exceeding the equivalent of
1.5 m at sea level. An unstable window restarts calibration. Calibration cannot
complete while armed, and detected barometer calibration blocks arming except
when an independent rangefinder-only mode is requested.
Requesting ALTHOLD also blocks arming if the barometer is unavailable.

Pressure is converted to standard-atmosphere altitude using exponent 0.190295.
While disarmed, the ground reference follows the current pressure and relative
height and vario are zero. Arming freezes that reference: low throttle during
flight does **not** re-zero altitude.

In flight, relative barometric height is filtered with the INAV-style 1 Hz PT1.
Vario is the change in filtered height divided by the **measured** interval
between valid samples. Positive means up; height is in metres, vario in m/s.
The first sample or a gap over 200 ms resets the derivative rather than
producing a speed spike. Timestamp subtraction supports `micros()` wraparound.
The legacy `baro_lpf_*` fields do not configure this fixed reference filter.

[Altitude](../lib/Xsfc/src/Control/Altitude.hpp) removes gravity from world-frame
acceleration, learns the remaining bias while disarmed, and integrates
`h += v*dt + a*dt*dt/2` before `v += a*dt`. In-flight bias learning uses
barometer residuals, not real climb/descent acceleration or low-stick detection.
Barometer corrections are applied once per valid barometer update and scaled
by its actual interval, independently of IMU loop rate. Recovery from a long
barometer gap resets vertical position/speed to the recovered measurements.
Without an accelerometer the estimate follows barometric height/vario directly.
No rangefinder measurements are blended into the barometric estimate.
RANGEHOLD/POSHOLD use a separate rangefinder-only estimate; see
[rangefinder-flow.md](rangefinder-flow.md).

`baro` in the CLI shows calibration, sample health, sample interval, data age,
pressure, barometric height and vario. MSP altitude/vario use the selected
barometric or rangefinder estimate;
Blackbox's barometric altitude remains the relative barometer measurement.

## Stick and throttle behavior

With `pid_althold_throttle_mode MID_STICK`:

1. Wait for a calibrated/ready barometer, enable ALTHOLD and arm with low throttle.
2. Moving the stick up to the center/deadband **does not initiate automatic
   takeoff**. The controller remains at motor idle until an upward command
   outside the deadband (or a detected lift above 0.5 m).
3. Above the deadband, stick displacement requests proportional climb speed.
4. Returning to center captures the current estimated height and holds it.
5. Below the deadband requests proportional descent speed. Returning to center
   again captures the current height.

The explicit ground wait until an upward command is an XSFC adaptation requested
for this workflow. INAV itself pre-biases the controller once for takeoff;
XSFC keeps the ground gate active until the upward command.

The position controller uses the linear/square-root braking law. Vertical-speed
changes are acceleration-limited; the velocity PID adjusts throttle around the
configured hover baseline. Throttle corrections are filtered at 4 Hz and
bounded by the mixer's idle/full-throttle limits, with conditional anti-windup.
Disarming or leaving the mode clears engagement; rearming initializes it again.

`STICK` and `HOVER` remain supported as alternative **stick-neutral** choices.
They no longer change the physical hover-throttle baseline:
`pid_althold_iterm_center` sets that baseline as a percentage of the usable
mixer throttle span. `pid_althold_iterm_range` limits the velocity integral.
Position I/D/F settings are retained for configuration compatibility; as in
INAV, the altitude-to-speed controller uses only position P.

MOTOR_STOP's low-stick rule does not stop motors while altitude hold is engaged,
because low stick then requests descent. Disarming still stops motors.

By default (`pid_althold_baro_fallback 1`), an invalid sample or data older than
200 ms exits automatic throttle control and logs the transition. Throttle
returns to **manual stick control**, not an automatic landing. The legacy
fallback-disabled option remains available, but continuing inertial prediction
without valid barometer data is not recommended.

## Existing saved configurations

Flashing does not replace saved settings. Gain units/defaults have changed;
do not reuse old ALTHOLD tuning unchanged. For the new reference starting point:

```text
set baro_bus I2C
set baro_dev MS5611
set pid_althold_throttle_mode MID_STICK
set pid_althold_deadband 5
set pid_althold_manual_climb_rate 100
set pid_althold_baro_fallback 1
set pid_althold_baro_p_weight 35
set pid_althold_baro_v_weight 35
set pid_althold_acc_bias_ground 100
set pid_althold_acc_bias_air 5
set pid_althold_pos_p 50
set pid_althold_pos_i 0
set pid_althold_pos_d 0
set pid_althold_pos_f 0
set pid_althold_vel_p 100
set pid_althold_vel_i 50
set pid_althold_vel_d 10
set pid_althold_vel_f 0
save
```

Configure the board's SDA/SCL pins and mode AUX range separately.
For SPI use `baro_bus SPI` and the barometer chip-select pin instead.
A deadband of 5 is +/-5% of half-stick travel, approximately +/-25 us for a
1000/1500/2000 input. `manual_climb_rate 100` means a maximum of +/-1 m/s.
Weights 35 mean 0.35/s, not a 35% correction each loop.

The hover baseline defaults to 50%, **not** an automatically measured hover
throttle. Set it for the actual aircraft, then tune vertical gains as needed.
Do not reset all firmware defaults merely to update these settings.

## Verification

Native regression tests exercise the production MS5611 driver, actual SPI
transport, calibration/vario estimator, fusion kinematics, stick state machine,
velocity PID and throttle filter. They include exact integer compensation
vectors, invalid ADC and command failures, irregular sampling, clock wrap,
ground drift, rate independence, anti-windup, and a simplified closed-loop
climb/hold/descent simulation.

From PowerShell with a C++17 MinGW compiler:

```powershell
.\bin\test_baro.ps1 -Compiler "C:\path\to\mingw\bin\g++.exe"
```

Build both targets with the existing PlatformIO installation:

```powershell
platformio run -e esp32 -e esp32s3
```

These checks verify software calculations and an idealized plant, not flight
safety or performance on a real aircraft. First verify sensor connection,
calibration and throttle behavior with propellers removed. Before flight,
calibrate the accelerometer, check axis signs and receiver center, set the
actual hover baseline, and protect the barometer from propwash/light. Ground
and flight testing remain necessary.
