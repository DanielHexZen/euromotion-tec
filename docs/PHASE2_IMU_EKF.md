# Phase 2 — IMU and EKF

Gate for `FIRMWARE_SPEC.md` §15 phase 2: H7, H8 (stationary part), `t_att` and `t_ekf_*` green. Vehicle work starts only after H6 (phase 1); host-track work (`att`, `ekf`, tests) may run ahead (D-29).

## Status

**Code written (2026-11-18), not flashed, not run on the vehicle.** Breakout board identified (below), not yet wired or probed. Branch `phase2-imu-ekf`.

| Item | State |
|---|---|
| Host code | `att`, `ekf`, `imu_yaw` (pure); `t_att`, `t_ekf_static/slip/gate/sf_q/jacobians/backshift/line/pred`, `t_imu_yaw` |
| Vehicle code | `imu`, `imu_lsm6ds3` (SPI1 DMA, TIM2 capture), IWDG (`sys_wdg_*`), `F_CSS` callback, console `IMU`/`EKF`, `W` with correction turns (D-54); parameters 400-404, 410-411, 500-522, 905 |
| Host tests | passed, 56764 checks, 0 failed (`make -C test`) |
| Build | `Core/Src` compiles with arm-none-eabi-gcc 13.2 at `-Og`/`-O2`, `FEAT_IMU` 0 and 1, 0 warnings. Not the CubeIDE build |
| Vehicle | nothing flashed; `FEAT_IMU` defaults to 0. EKF, `att_init` and IWDG run in every build (D-61): this branch is phase-2 firmware, phase-1 calibration keeps `phase0`/main until H6 passes |
| H7, H8 (stationary part) | not done |

Decisions: spec D-54 to D-62. Open items (spec OQ-11):
- Capture sets 0/1 deferred (needs `tlm`, 48 KB buffer, phase-4 protocol).
- REQ-IMU-07 "EKF q_θ x100 on saturation" not wired (`ekf_odo_t` has no saturation flag).
- Spec §14 `t_att` "rejects 0.3 g lateral" contradicts `att_acc_gate` 0.1 (0.3 g lateral gives |a| = 1.044 g, passes the gate); the test uses 0.5 g, spec to decide.
- `IMU` and `EKF` print one snapshot; spec "(1 s)" unclear.
- `[VERIFY]` at first power-up: all §7.11.1 register values, `DRDY_PULSE_CFG_G` and CTRL4_C `I2C_disable` (not set), temperature raw/16 + 25 °C, WHO_AM_I 0x69 vs 0x6A.
- `IMU_AXIS_MAP` identity until mounted `[MEASURE]`; no roll/pitch mounting-offset parameter in the spec (`att` assumes the board level).

## 1. IMU breakout board

GODIYMODULES "LSM6DS3" module (Amazon MX `B0FKT9ZR2X`), purple, 12 pins, mounting holes, board print "LSM 6DS3" with axis arrows. From the product photos (2026-11-18): a 3-pin regulator (marking "LLVB"), a few 0603 resistors, no level shifter visible. The chip marking is not readable in the photos, so the variant is still open: plain LSM6DS3 reads WHO_AM_I 0x69, the LSM6DS3TR-C reads 0x6A (spec §7.11.1, OQ-1). Read WHO_AM_I first (section 3) before anything else.

## 2. Wiring (SPI1 4-wire, spec §4.3)

Board pins in the order printed: top row `VIN 3V3 GND SCL SDA CS SAO`, bottom row `DCS INT2 INT1 SCX SDX`. Chip-pin names from the LSM6DS3 datasheet table 2 (mode 1).

| Board pin | LSM6DS3 function (SPI) | NUCLEO-F446RE pin | Morpho | Arduino header | Note |
|---|---|---|---|---|---|
| 3V3 | VDD / VDDIO | 3V3 | CN7-16 | CN6-4 | supply 3.3 V; all logic then runs at 3.3 V like the STM32 |
| VIN | regulator input | — | — | — | leave open (3V3 is fed directly). Alternative: 5 V here and 3V3 open, never both |
| GND | GND | GND | CN7-20 | CN6-6 | |
| SCL | SPC (SPI clock) | PB3, SPI1_SCK | CN10-31 | D3 | |
| SDA | SDI (MOSI) | PB5, SPI1_MOSI | CN10-29 | D4 | |
| SAO | SDO (MISO) | PB4, SPI1_MISO | CN10-27 | D5 | |
| CS | CS (low = SPI) | PA4, `IMU_CS`, GPIO out, init high | CN7-32 | A2 | high at boot keeps the part idle |
| INT1 | INT1, DRDY gyro, push-pull active high | PA1, TIM2_CH2 capture, rising | CN7-30 | A1 | one rising edge per sample (833 Hz) |
| INT2 | INT2 | — | — | — | open, not used |
| SCX | SCx (aux I²C master clock) | — | — | — | datasheet mode 1: connect to VDDIO or GND. If the board has no pull resistor on it `[VERIFY]` with a multimeter, tie to GND |
| SDX | SDx (aux I²C master data) | — | — | — | as SCX |
| DCS | aux SPI CS (OCS_Aux on the TR-C; NC on the plain LSM6DS3) | — | — | — | open |

Wires short (≤ 15 cm, SPI mode 3 at about 5.6 MHz), GND wire next to SCL. Mounting: rigid, at the drive-axle midpoint, away from the BTS7960 drivers (REQ-IMU-04). Note the board orientation (printed X/Y/Z arrows against vehicle x forward, y left, z up); it sets `IMU_AXIS_MAP` (REQ-IMU-02, `[MEASURE]` after mounting).

## 3. First power-up checks

- [ ] Before connecting to the Nucleo: 3V3 to GND not shorted; SCX/SDX level measured (pulled or floating).
- [ ] WHO_AM_I (0x0F) read over SPI: 0x69 → plain LSM6DS3, as the spec assumes. 0x6A → LSM6DS3TR-C: deviation entry in spec §18.2 and re-check every §7.11.1 register before `imu_lsm6ds3.c` is used.
- [ ] Record the result here and close OQ-1 in the spec.

## 4. First power-up on the bench (after H6)

In order; stop at the first failure.
1. [ ] Section 3 checks done (wiring, WHO_AM_I read standalone).
2. [ ] Build with `FEAT_IMU` = 1 (CubeIDE Debug, 0 warnings), flash.
3. [ ] Boot log shows the imu step OK (WHO_AM_I accepted).
4. [ ] `IMU` at rest: z about 9.81 m/s2, gyro rates near 0; sample count rises at about 833/s with no drops.
5. [ ] H7: DRDY rate = ODR +/- 0.1 %, Allan variance logged, 10-turn spin residual < 0.5 deg after `imu_gz_scale`.
6. [ ] H8 stationary part: bias converges (`ekf_bg_conv`), `EKF` command shows it; record the logs under `tools/data/<date>/`.
