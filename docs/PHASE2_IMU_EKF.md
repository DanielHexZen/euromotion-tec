# Phase 2 — IMU and EKF

Gate for `FIRMWARE_SPEC.md` §15 phase 2: H7, H8 (stationary part), `t_att` and `t_ekf_*` green. Vehicle work starts before H6 by team decision (spec D-56: H6 failed only on caster slip, phase 1 closed for now); host-track work (`att`, `ekf`, tests) may run ahead (D-29).

## Status

**Gate passed 2026-11-25** (H7 with D-78, H8 stationary part, host tests green); one follow-up: the final noise parameters from the overnight `IMUS` log with the glitch filter (below). Phase 3 vehicle work may start. Branch `phase2-imu-ekf`.

| Item | State |
|---|---|
| Host code | `att`, `ekf`, `imu_yaw` (pure, incl. the glitch filter D-77); `t_att`, `t_ekf_static/slip/gate/sf_q/jacobians/backshift/line/pred`, `t_imu_yaw` |
| Vehicle code | `imu`, `imu_lsm6ds3` (SPI1 DMA, TIM2 capture), IWDG (`sys_wdg_*`), `F_CSS` callback, console `IMU`/`EKF`/`IMUS` (D-74), `W` with correction turns (D-57, D-68), glitch filter (D-77); parameters 400–405, 410–411, 500–522, 905 |
| Host tests | `make -C test` green (58178 checks on `c7ab8b9`) |
| Vehicle | flashed and running since 2026-11-20; `FEAT_IMU` 1 committed (`9f89270`); `imu_gz_scale` 0.9907 compiled |
| H7 | **passed**: WHO_AM_I 0x69; DRDY 820.85 Hz, stable, deviation accepted (D-78); Allan logged (`tools/data/2026-11-25/imus.log`, `allan.txt`); 10-turn spin CCW/CW 0.53° / 0.30° with 0.9908, confirming pair with 0.9907 passed |
| H8 stationary part | **passed**: `EKF` at standstill `STILL`, `bg` −2.754 °/s against raw z −2.63…−2.77 °/s, `sigma bg` 0.003–0.004 °/s < `ekf_bg_conv` 0.02 °/s (`tools/data/2026-11-25/spin_H7.log`, before and after the spins). The pushed part of H8 (10 m, ≤ 10 cm) belongs to phase 5 |
| Noise parameters | **done 2026-11-26**: 40 min standstill without gaps (`tools/data/2026-11-26/allan.txt`): DRDY 820.87 Hz (same as 2026-11-25, D-78), z N 14.8 mdps/√Hz → `ekf_gyro_nd` 2.6e-4 (the 25th gave 8.5 mdps/√Hz; noisier place or vibration, the larger value is taken), bias instability 4.9 mdps at 62 s, floor 2.6 mdps → `ekf_zupt_floor` 4.6e-5, K² 9.8e-11 confirms `ekf_q_bg` 1e-10. With the glitch filter 5 spike windows remain (4 y, 1 z, was 47); the temperature also shows corrupted values (−33 / 1279 °C, unused while `imu_tc1/2` = 0): byte corruption on the SPI read persists, cause open (D-77). Watch `EKF` ZUPT acc/rej with the new values |
| Before H6 repeat | set `FEAT_IMU` 0 (encoder-only UMBmark, D-72) |

**Watchdog (IWDG, D-64) — read before the first flash of this branch.** It starts at the end of `app_init()` in every vehicle build (not in `APP_CALIB`) and cannot be stopped until the next reset. Timeout prescaler 32 × reload 100 on the LSI ≈ 100 ms (LSI 17–47 kHz: 68–188 ms). It is refreshed only while the main loop runs at least every 5 ms and `S_CTRL` completed within the last 30 ms; one late iteration skips one refresh, about 100 ms without refresh resets the part (boot then prints `reset by watchdog`, fault `F_WDG_RESET`). `sys_wdg_start()` sets `DBGMCU` IWDG freeze, so a breakpoint or a halt in the debugger does not reset the board. `wdg_enable` (905) is read once at start: `P905=0` in RAM has no effect, disabling needs a rebuild with default 0. NMI (HSE loss) and fault paths that spin rely on the IWDG to reset after it has started.

Decisions: spec D-57 to D-65 (code), D-66 to D-69 (commissioning). Open items (spec OQ-11):
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
- [x] WHO_AM_I (0x0F) read over SPI: 0x69 → plain LSM6DS3, as the spec assumes. 0x6A → LSM6DS3TR-C: deviation entry in spec §18.2 and re-check every §7.11.1 register before `imu_lsm6ds3.c` is used. *(2026-11-20: **0x69**, read in the debugger at `imu_lsm6ds3.c:53`; plain LSM6DS3)*
- [x] Record the result here and close OQ-1 in the spec.

## 4. First power-up on the bench (before H6, D-56)

In order; stop at the first failure.
1. [x] Section 3 checks done (wiring, WHO_AM_I read standalone).
   *Before, with `FEAT_IMU` 0 on `83eb182` (2026-11-20): boot OK, `I` S_CTRL max 71 µs, overruns 0, period 20000.0 µs, cpu 0.6 %; `EKF` STILL, step 61 µs; reset cause PIN, no watchdog reset.*
2. [x] Build with `FEAT_IMU` = 1 (CubeIDE Debug, 0 warnings), flash.
3. [x] Boot log shows the imu step OK (WHO_AM_I accepted). *(2026-11-20, `83eb182-dirty` Debug: `8 imu OK`)*
4. [x] `IMU` at rest: z about 9.81 m/s2, gyro rates near 0; sample count rises at about 833/s with no drops.
   *2026-11-20 (`83eb182-dirty`): alive, drop 0, sat 0, T 20 °C, about 808 samples/s (manual 10 s timing, exact rate in H7). Gyro at rest 5.3 / −4.6 / −2.4 °/s (x/y/z): EKF bias stayed 0 because every ZUPT was rejected → fixed by D-66, to be re-checked. After D-66 the first ZUPT after boot learnt −5.6 °/s (cold) and −6.8 °/s (warm reset) against a raw −2.75 / −2.33 °/s and locked the filter out again; `W90` then over-estimated (95.2 of 90) and turned back: fixed by D-67 (IMU settle 0.5 s, bias reopen after 5 rejected ZUPTs), to be re-checked. After D-67 (2026-11-20): `W90` twice → gyro 90.6 of 90.0°, two fine corrections, the second about 2 s later (full ramp); ZUPT acc 36 rej 27 with σ_bg 0.002 °/s (`ekf_zupt_floor` too optimistic, live test with `P508` pending, value from the Allan variance in H7). D-68: correction tolerance 1.0°, 0.3 s ramp. After D-68 (2026-11-20): first ZUPT after boot −2.61 °/s against a raw −2.42 (D-67 settle works); `P508=2e-4` live: one reopen, then 44 accepted, 0 rejected. Learnt bias −3.0 against raw −2.4: the wrong roll (gyro x/y bias in `att`) leaks w_y into the yaw rate → D-69. UMBmark with the gyro turns (`W90`, team report): 3 runs, return error below 100 mm each (values not recorded). On `2a7f495` (D-68, D-69 flashed; `tools/data/2026-11-20/phase2_imu.log`): at rest roll 1.17° / pitch −0.09° (was 6.7° / −5.1°), bg −2.42 °/s against a raw z −2.28 (was −3.0), ZUPT 11 rejected right after boot then only accepted (acc 6 → 21); `W90` gyro 89.8 / 89.9 of 90.0; `W360` 360.9 and 360.4 of 360.0, the correction turn now 0.62 s. Visually good, a small residual remains (not measured; 10-turn spin in H7 gives `imu_gz_scale`). Note for H6: with `FEAT_IMU` 1 the `W` turns are gyro-corrected, so a UMBmark no longer tests the encoder odometry alone; repeat H6 with a `FEAT_IMU` 0 build. Axis signs by hand: nose up → pitch negative, left side up → roll positive (FLU, `IMU_AXIS_MAP` identity is right); `W90` gyro sign right. Finding: roll/pitch at rest 6.2° / −5.0° are the x/y gyro bias times `att_tau` (att.c integrated the gyro without bias), not the mounting; fixed by D-69. Hard-fault record `msp 0x20000BB0` with PC 0x20000000 appears after every flash and is gone after a plain reset: the ST-LINK flash loader running from SRAM, not the application.*
5. [x] H7: DRDY rate = ODR +/- 0.1 % (D-78: stable, no gaps, deviation documented), Allan variance logged, 10-turn spin residual < 0.5 deg after `imu_gz_scale`.
   **a) Standstill log (DRDY rate, Allan, noise parameters; D-74).** Vehicle powered >= 5 min, on the floor, motors off, nobody touching it or the table. Console on USB with logging (`screen -L -Logfile tools/data/<date>/imus.log /dev/cu.usbmodem… 115200`). `IMUS` → `IMUS ON`, wait 30 min (at least 10), `IMUS` → `IMUS OFF`. Then `python3 tools/allan.py tools/data/<date>/imus.log | tee tools/data/<date>/allan.txt`. It prints the DRDY rate against 833 Hz (PASS/FAIL ±0.1 %), the Allan table, and `P503`/`P508`/`P505` proposals. The `P508` floor from the curve covers the sensor noise only; the live test of D-69 needed 2·10⁻⁴ rad/s, so take the larger value unless the ZUPT acceptance (`EKF`) stays good with the smaller one. `python3 tools/allan.py --selftest` must print `SELFTEST PASSED` first.
   *Result 2026-11-25 (`9f89270-dirty`, log `tools/data/2026-11-25/imus.log`, `allan.txt`): 37 min captured, 4 gaps in the capture (Mac asleep), longest piece 1524 s analysed. **DRDY rate 820.85 Hz, −1.46 % against 833 Hz: accepted by spec D-78** (LSM6DS3 internal oscillator; integration is timestamp-based; H7 rate criterion now: stable, no gaps, deviation documented). **Finding: 31 y and 16 z windows (0 x) carry one corrupted IMU sample each, about +17 000 LSB (+300 °/s)**, single axis, single 20 ms window: not motion. Only axes with a negative rest value are hit, which points to corrupted high bytes on the SPI read (signal integrity or a read race), to be diagnosed. Effect: each z event adds about +0.35° to the yaw integral and spoils one ZUPT window. With the spikes replaced (`--despike`, provisional): z angle random walk 8.5 mdps/√Hz (`ekf_gyro_nd` 1.48e-4 rad/s/√Hz, default 1.0e-4), bias instability 4.3 mdps at 309 s, ZUPT floor from the curve 5.0e-5 rad/s (live value 2e-4 kept until the spikes are gone), no rate random walk visible. Temperature span 0.6 K: no coefficient. **Glitch filter D-77** added (P405, `IMU` shows `glitch`); repeat the standstill log with it (overnight, Mac kept awake) for final `ekf_gyro_nd` / `ekf_zupt_floor` without `--despike`.*
   **b) 10-turn spin (`imu_gz_scale`).** Warm, bias settled (>= 30 s still, `EKF` STILL). Laser or long ruler on the vehicle, a mark on a wall >= 3 m away (1° = 52 mm at 3 m). `W3600`, note `Turn (gyro) x of 3600.0 deg` and the overshoot o+ in the turning direction (too short = negative, o = atan(offset / wall distance)). 30 s still, `W-3600`, overshoot o- in the turning direction. New value: `imu_gz_scale_new = imu_gz_scale_old * (1 + (o+ + o-) / 7200)`; (o+ - o-)/2 is the bias part (> 1°: bias not settled). `P402=<new>` at standstill, repeat both turns: |o| < 0.5° each passes. Enter the value as the default in `par_table.h`.
   *Result 2026-11-25 (`fe22ea5-dirty`, `FEAT_IMU` 1, log `tools/data/2026-11-25/spin_H7.log`): wall at 4000 mm, overshoot o = atan(offset / 4000). P402 1.0: W3600 32–36° short, W-3600 25° short (gyro reads about 0.8 % too much; the encoder turn alone was nearly exact, the gyro correction turned the error in). 0.9918: 262 / 247 mm short (3.75° / 3.53°). 0.9908: 37 / 21 mm short (0.53° / 0.30°). Default set to **0.9907** (both ≈ ±0.1° predicted, the step is below the measurement resolution; confirming pair with 0.9907: **passed**, team report 2026-11-25, offsets not recorded). Bias part (o+ − o-)/2 ≤ 0.1° in the last two pairs. **Finding:** the first W3600 after a flash ended 230 mm (3.3°) off, against 37 mm a few minutes later: the bias is not settled right after boot (σ_bg starts at 5 °/s, D-66; cold gyro). Rule for runs: ≥ 5 min powered and ≥ 30 s still before a turn; for AUTO check that `msn` waits for `ekf_bg_conv` at start.*
6. [x] H8 stationary part: bias converges (`ekf_bg_conv`), `EKF` command shows it; record the logs under `tools/data/<date>/`.
   *2026-11-25: `STILL`, `bg` −2.754 °/s (raw z −2.63…−2.77), `sigma bg` 0.003 °/s before and 0.004 °/s after the spin runs, < `ekf_bg_conv` 0.02 °/s. Log `tools/data/2026-11-25/spin_H7.log` lines 91–94, 173–176.*
