# Phase 2 — IMU and EKF

Gate for `FIRMWARE_SPEC.md` §15 phase 2: H7, H8 (stationary part), `t_att` and `t_ekf_*` green. Vehicle work starts only after H6 (phase 1); host-track work (`att`, `ekf`, tests) may run ahead (D-29).

## Status

**Not started.** Breakout board identified (below), not yet wired or probed.

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
