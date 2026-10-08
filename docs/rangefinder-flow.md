# Rangefinder, optical flow and navigation modes

The algorithm reference is the local INAV 9.1.0 checkout in `reference/inav`:
`sensors/opflow.c`, `io/opflow_msp.c`, `io/rangefinder_msp.c`,
`navigation/navigation_pos_estimator_flow.c`,
`navigation/navigation_pos_estimator_agl.c` and
`navigation/navigation_multicopter.c`.
This is an adaptation to XSFC, not a complete INAV navigation/failsafe port.

## Modes

| Name | Height source | Horizontal control | Barometer required |
|---|---|---|---|
| ALTHOLD | Barometer + accelerometer | Pilot | Yes |
| RANGEHOLD | Rangefinder + accelerometer only | Pilot | No |
| POSHOLD | Rangefinder + accelerometer only | Flow + rangefinder; angle stabilization included | No |

RANGEHOLD and POSHOLD share the center-stick climb/hold/descent behavior
documented in [barometer.md](barometer.md). They hold the **distance above the
observed surface**, not a barometric ground-zero altitude. Landing-gear height
is not subtracted. A rising surface therefore moves the aircraft's height
target in earth coordinates. The ground takeoff gate requires an upward stick
command before automatic climb; the rangefinder must be able to see the floor
before arming.

Rangefinder-only modes take priority over simultaneously requested barometric
modes. Do not intentionally configure overlapping height-mode AUX ranges.
RANGEHOLD keeps ID 13. Full POSHOLD uses ID 14 (formerly FLOWHOLD), preserving
existing full-hold AUX assignments. SURFACE ID 11 and horizontal-only POSHOLD
ID 12 are retired and ignored; reassign those AUX conditions explicitly.
MSP mode names, IDs and status-bit ordering have been updated together.

Readiness requires three consecutive valid range/flow samples, fresh data
(at most 200 ms old), finite estimates, and an active accelerometer.
Optical flow additionally requires at least 80 mm surface height by default.
POSHOLD supplies angle stabilization automatically. When either roll/pitch
stick is outside its deadband, horizontal hold releases to the pilot; centering
both captures a new local anchor. The anchor is integrated in earth coordinates,
so changing yaw does not rotate the stored displacement.

### Sensor loss

In RANGEHOLD/POSHOLD, invalid/stale range measurements stop automatic throttle
and return **manual stick throttle**, with a log and buzzer event. They do not
switch to the barometer. The loss is latched until the range modes are switched
off/on, or the aircraft is disarmed, preventing unexpected automatic reengagement.

Low-quality/stale optical flow releases horizontal hold to pilot angle control,
also with a latched loss indication. Valid range-based height control continues
if only optical flow is lost. Disarming always stops the motors. MOTOR_STOP
does not stop motors merely because the throttle stick requests descent while
automatic height control is engaged.

## Supported modules and range limits

| `rangefinder_dev` | Transport | Profile limit |
|---|---|---|
| VL53L0X | Direct I2C, address 0x29 | 2000 mm |
| MATEK3901 | Matek 3901-L0X, MSP/UART | 2000 mm nominal hardware ceiling |
| MTF02 | MicoAir MTF-02 in MSP mode | 2500 mm |
| MTF02P | MicoAir MTF-02P in MSP mode | 6000 mm |
| MSP | Generic external MSP rangefinder | No additional profile cap |

**MTF-02 and MTF-02P are not the same sensor.** The profile limits above follow
the [Betaflight MicoAir driver](https://github.com/betaflight/betaflight/blob/master/src/main/drivers/rangefinder/rangefinder_lidarmt.c).
The [ArduPilot Matek guide](https://github.com/ArduPilot/ardupilot_wiki/blob/master/common/source/docs/common-mateksys-optflow-3901L0X.rst)
recommends a **1.2 m** maximum for the 3901-L0X's short-range onboard lidar and
warns about poor outdoor performance. A software limit cannot increase its
physical reliable range.

`rangefinder_max_range` is in **cm**. Zero (the new default) uses the profile;
a positive value imposes an additional, smaller user limit.
Generic MSP with zero uses the sensor's validity report without an extra cap.
Previously the default was 150 cm; saved configurations retain that limit until
changed. This can explain an apparent range cutoff, but hardware and surface
conditions must also be checked.

Range samples use signed millimetres on the wire. Zero/negative distances,
short driver reads, range-limit violations and tilt beyond 25 degrees invalidate
the measurement. Invalid samples no longer inject zero into the filter.
First samples/recovery seed the filter at the actual height; height and vario
use measured timestamps and handle clock wrap. Three new valid samples are
required after recovery. Sensor diagnostics retain the last value with an
explicit invalid status instead of presenting it as a fresh reading.

## UART/MSP setup

Both modules send rangefinder (`0x1F01`, five payload bytes) and optical flow
(`0x1F02`, nine payload bytes) on a **115200 baud** UART. Set MTF-02's output
protocol to **MSP/INAV** using its configuration tool; MAVLink/Micolink are not
decoded by this implementation.

Connect module TX to the selected FC UART RX, with common ground and the correct
module supply voltage. Set that UART's function to MSP, not serial receiver/GPS,
and its baud to 115200. Do not share a sensor UART with a configurator or receiver.
Native MSPv2 and MSPv2-in-v1 packets are decoded and CRC checked; sensor push
messages accept `<` or `>` direction without replies. Malformed payloads are
logged and do not refresh the sensor timestamp.

For **Matek 3901-L0X**:

```text
set feature_rangefinder 1
set rangefinder_dev MATEK3901
set rangefinder_max_range 120
set flow_dev MATEK3901
set flow_scale 1050
set flow_align CW0_FLIP
```

For **MTF-02**:

```text
set feature_rangefinder 1
set rangefinder_dev MTF02
set rangefinder_max_range 0
set flow_dev MTF
set flow_align CW0_FLIP
```

For MTF-02P substitute `rangefinder_dev MTF02P`.
`flow_dev AUTO` selects MTF scaling for MTF02/MTF02P range profiles and
Matek/INAV scaling otherwise. For a generic MSP rangefinder paired with an MTF
flow source, explicitly use `flow_dev MTF`.

### Flow units and alignment

Matek uses INAV's counts-per-degree scale (`flow_scale 1050` = 10.5
counts/degree). The MTF profile uses the MicoAir protocol conversion of
1/200 rad per reported integrated motion unit; `flow_scale` affects Matek,
not MTF. `poshold_flow_gain_x/y` are now signed **percentage corrections**
(100 = unity), not the old arbitrary x0.0001 gain.

Downward sensor alignment is applied before gyro subtraction. Gyro angular
rates are averaged over the flow integration interval rather than sampling
only the latest gyro value. `flow_gyro_delay_ms` shifts that interval for sensor
latency; default zero matches the INAV approach. Do not guess a delay from a
different module; verify rotation-only motion with `flow watch`. Body flow is
cross-coupled into forward/right velocity, multiplied by surface distance, then
rotated into earth coordinates with the current roll/pitch/yaw.

Start with:

```text
set poshold_flow_gain_x 100
set poshold_flow_gain_y 100
set poshold_use_gyro_comp 1
set flow_gyro_delay_ms 0
set flow_min_range_mm 80
set poshold_deadband 5
set poshold_angle_limit 20
set poshold_max_speed 200
set pid_poshold_pos_p 65
set pid_poshold_pos_i 0
set pid_poshold_vel_p 40
set pid_poshold_vel_i 15
set pid_poshold_vel_d 100
set pid_poshold_vel_f 40
set pid_althold_throttle_mode MID_STICK
save
```

Horizontal PID scales follow INAV's position-to-speed and speed-to-acceleration
gains. Acceleration is converted to bounded lean angles with gravity, rather
than interpreting velocity PID output directly as an arbitrary bank angle.
Keep [barometric/vertical settings](barometer.md#existing-saved-configurations)
and the actual hover-throttle baseline tuned for the aircraft.

Existing version-1 settings are loaded through a compatible prefix migration;
new navigation settings receive defaults. `save` now writes version 3. Migration
does not replace old user gains/range limits: apply the relevant settings above,
and back up the configuration before flashing.

## Verification and diagnostics

### Transition and failure audit

RANGEHOLD and POSHOLD share the range estimate, height target, velocity demand
and throttle trim. Adding/removing either mode while the other remains active
does not reset that vertical state. POSHOLD adds horizontal hold and angle
stabilization. A real source change or re-entry after leaving hold resets the
vertical controller. Airborne entry seeds the throttle correction filter from
the previous controller output and seeds I-term within configured bounds;
this reduces the initial hover-baseline step but is not a guarantee of constant
thrust or a hardware-validated bumpless transition.

Flow-only loss releases horizontal hold with its existing off/on latch while
healthy range throttle continues. Range loss releases automatic throttle and
horizontal hold; recovery cannot clear the vertical latch by changing only
RANGEHOLD/POSHOLD bits while a range-hold request remains. Turn both requests
off to reset it. Disarm clears the latch. Manual throttle is restored directly,
not ramped: the pilot's stick can differ from the previous automatic output.

Native tests cover same-source reset decisions, bounded airborne entry,
nonfinite gyro rejection, both scale formulas, axis cross-coupling, rotation
cancellation, and a frame-readiness -> vertical loss-latch scenario. These are
production-helper tests, not a full Controller/Actuator/mixer integration test.
The Matek scale conversion agrees with local INAV `sensors/opflow.c`.
The retained MTF 1/200 scale is still a module/profile assumption: equal-angle
synthetic tests cannot establish the real module's units, signs or latency.
Do not treat software tests as calibration of either physical module.

Propeller-free checks still required for **each** module:

1. At known height above a textured surface, translate forward/right without
   tilt: reported forward/right velocity must have the same sign as motion.
2. Rotate roll/pitch without translation: compensated velocity should approach
   zero. Check alignment before changing signed gains; verify measured scale.
3. Check yaw and tilt transforms and the actual sensor integration delay.
4. Switch RANGEHOLD -> both -> POSHOLD -> both -> RANGEHOLD; inspect target,
   velocity PID trim and throttle output for reset-induced steps.
5. Interrupt flow only, then range only; inspect release, recovery latch and
   off/on behavior. Include low quality, overrange, stale and malformed feeds.
   Do not use bench motor output as evidence of stable airborne thrust.

`rangefinder` shows the working limit, rejection reason, sample age, height,
vario and RANGEHOLD loss latch. `flow`/`flow watch` show quality, integration
interval, flow/gyro rates, earth velocity and position-hold loss latch.
MSP altitude/vario report the selected barometric or range estimate.
Sensor feeds and processed frames are published as coherent cross-core snapshots.

Run the combined native regression suite with:

```powershell
.\bin\test_baro.ps1 -Compiler "C:\path\to\mingw\bin\g++.exe"
```

It covers MS5611, range/flow protocols, all CRC byte values, malformed packets,
module range profiles, filter recovery, vario timing, gyro compensation,
yaw/tilt transformation, sensor-loss policies and simplified closed-loop height
and position simulations.

No optical-flow system guarantees a fixed GPS-like position indefinitely:
integrated flow drifts, and performance depends on illumination, textured
surfaces, alignment, vibrations and reliable ranging. Verify wiring, signs,
rotation-only compensation, readiness and mode-loss behavior with propellers
removed before any controlled flight testing.
