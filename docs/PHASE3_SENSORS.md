# Phase 3 — ToF sensors and guidance

Gate for `FIRMWARE_SPEC.md` §15 phase 3: H9, H9b, H10, SIL on `rect10`, `t_gnd`, `t_fwd`, `t_map`, `t_nav_*`, `t_msn`, `t_lat_tilt` green. Vehicle work starts only after H7/H8 (phase 2); host-track work may run ahead (D-29).

## Status (2026-11-21)

| Item | State |
|---|---|
| Sensor set | decided, spec D-70: F1, F2, S2, optional L1; G1–G4 and S1 not fitted |
| CubeMX | done since `f126577`: I2C1/I2C2 400 kHz IT, all XSHUT outputs low, EXTI PA15/PA12. No regeneration needed for D-70 |
| Host code | `gnd` (S2 classifier, right side state, edge events) and `fwd` (reflex, governor, staleness) with `t_gnd`, `t_fwd`; `vl.h` types only |
| Vehicle code | `vl.c` not written: needs the ST VL53L1X ULD (STSW-IMG009, `VL53L1X_api.c/h`) in the repository first (spec §7.13.2) |
| Not started | `map`, `nav`, `msn`, `drv` VEL mode, `lat` (phase 5), SIL, `course_gen.py` |
| Wiring | not wired |

Open decisions (spec OQ-12): interlock replacement (left side blind, right side one footprint); observation point and σ of S2 edge events; L1 geometry at 45°; L1 distance mode and ROI.

## 1. Sensor set (D-70)

| ID | Name | Mount | View | Purpose |
|---|---|---|---|---|
| 4 | F1 | front left | forward, horizontal | reflex, speed governor (`fwd`) |
| 5 | F2 | front right | forward, horizontal | reflex, speed governor (`fwd`) |
| 7 | S2 | right side, `lat_h` above the path | right, tilted down by `lat_tilt` (wedge bracket, §7.13.4) | ground class, edge events (`gnd`), right side state |
| 6 | L1 | on top of the vehicle, optional | horizontal, yaw ≈ 45° from +x towards the path | landmarks (`lat`, phase 5) |

IDs 0–3 (G1–G4) are not fitted and are reported absent at init.

## 2. Wiring (all on I2C2, spec §4.3)

All modules share the bus. Breakout type `[VERIFY]`: most VL53L1X boards have `VIN GND SCL SDA GPIO1 XSHUT`, a regulator and pull-ups; check before connecting.

| Signal | Board pin | NUCLEO-F446RE | Morpho | Note |
|---|---|---|---|---|
| Supply | VIN | 3V3 | CN7-16 | all sensors; with 3V3 the pull-ups on the boards pull to 3.3 V |
| Ground | GND | GND | CN7-20 | |
| I2C2 SCL | SCL | PB10 | CN10-25 | all sensors |
| I2C2 SDA | SDA | PC12 | CN7-3 | all sensors |
| F1 XSHUT | XSHUT | PB14 | CN10-28 | GPIO1 open |
| F2 XSHUT | XSHUT | PB15 | CN10-26 | GPIO1 open |
| S2 XSHUT | XSHUT | PB1 | CN10-24 | |
| S2 data ready | GPIO1 | PA12 | CN10-12 | EXTI, falling, pull-up |
| L1 XSHUT (optional) | XSHUT | PB0 | CN7-34 | CubeMX label `XSHUT_S1` |
| L1 data ready (optional) | GPIO1 | PA15 | CN7-17 | CubeMX label `VL_S1_INT` |

- Pull-ups: the spec wants 2.2–4.7 kΩ in total per bus. Several boards in parallel add up (four boards with 10 kΩ each give 2.5 kΩ). Measure SCL and SDA to 3V3 with the sensors connected and the board unpowered.
- Keep SCL/SDA short and away from the motor cables; at 400 kHz a long ribbon to the front sensors is the usual failure.
- I2C1 (PB8/PB9) and `XSHUT_G1..4` (PC0–PC3) stay unconnected.

## 3. First bench checks (after H7/H8, once `vl.c` exists)

1. [ ] One sensor at a time: only its XSHUT high, address 0x29 answers (boot log), then the full sequence of §7.13.2.
2. [ ] Boot log: F1, F2, S2 (and L1) present, G1–G4 absent.
3. [ ] F1/F2 against a box at 0.3, 0.5, 1.0 m; ranges within ±2 cm.
4. [ ] S2 on the wedge on flat floor: `LATCAL` gives `θ_meas` within 1° of `lat_tilt` (REQ-VL-10).
5. [ ] Then H9b (tilt test), H9 (S2 outdoors), H10 (reflex).
