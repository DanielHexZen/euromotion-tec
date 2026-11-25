# Phase 3 — ToF sensors and guidance

Gate for `FIRMWARE_SPEC.md` §15 phase 3: H9, H9b, H10, SIL on `rect10`, `t_gnd`, `t_fwd`, `t_map`, `t_nav_*`, `t_msn`, `t_lat_tilt` green. Vehicle work starts only after H7/H8 (phase 2); host-track work may run ahead (D-29).

## Status (2026-11-21)

| Item | State |
|---|---|
| Sensor set | decided, spec D-70: F1, F2, S2, optional L1; G1–G4 and S1 not fitted |
| CubeMX | done since `f126577`: I2C1/I2C2 400 kHz IT, all XSHUT outputs low, EXTI PA15/PA12. No regeneration needed for D-70 |
| Host code | `gnd` (S2), `fwd`; `map` with `course_rect10` (hand table), `nav` (follow, final, turn), `msn` (modes, AUTO without evasion, RECOVER, start conditions), `drv` VEL mode, `prof_plan_v0` (D-75). Tests `t_gnd`, `t_fwd`, `t_map`, `t_nav_line/final/turn`, `t_msn`, `t_drv_vel`; `t_nav_turn` asserts 0.35° instead of 0.2° (D-75 item 9). Not wired into the app |
| Vehicle code | `vl.c` complete for D-70 (D-73): init with the ST ULD v3.5.5 (XSHUT sequence, address, short mode, TB 20 ms, ROI, start ranging, F2 10 ms after F1); run time polled, interrupt-driven I2C2 state machine (ready → 17-byte result → clear), REQ-VL-03 timestamps, bus recovery (REQ-VL-04). S2 feeds `gnd`; console `VL` shows range, status, signal, ambient, age and the S2 class; `VLS` stream and `LATCAL` (D-79). `fwd` is not fed yet. Not run on the vehicle |
| Not started | app integration of `msn`/`nav`/`fwd` (vehicle track), `lat` (phase 5), SIL (`test/sim_vehicle.c`), `course_gen.py`; interlock replacement (OQ-12) |
| Wiring | F1, F2, S2 connected (team, 2026-11-24) |

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

## 3. First bench checks

1. [ ] Unpowered: SCL→3V3 and SDA→3V3 2.2–4.7 kΩ with all sensors connected; 3V3–GND no short.
2. [ ] Boot log (wiring check, works now):
   ```
    9 vl G1    absent      (G1–G4 not fitted, D-70)
    ...
    9 vl F1    OK
    9 vl F2    OK
    9 vl L1    absent      (optional, not connected)
    9 vl S2    OK
   ```
   `absent` on a connected sensor: its XSHUT wire or the sensor's supply; it never booted or did not answer at 0x29. `FAIL 3` (bus) / `FAIL 2` (timeout): SDA/SCL, pull-ups, or a second sensor still answering at 0x29. `FAIL 4` (range): wrong model ID, not a VL53L1X. Swap two XSHUT wires and the names swap: that proves the assignment.
3. [ ] `VL` at rest: every fitted sensor `st 0`, age < 30 ms, range plausible. Then F1/F2 against a box at 0.3, 0.5, 1.0 m; ranges within ±2 cm.
4. [ ] S2 on its 15° bracket (D-79): measure the optical centre height above the floor → `P671=<m>`; on flat floor `LATCAL` gives `tilt` within 1° of `lat_tilt` (REQ-VL-10). Expected flat slant range `lat_h / sin 15°` (270 mm at 7 cm).
5. [ ] **H9b** (D-79), console log on USB (`screen -L -Logfile tools/data/<date>/h9b.log …`), vehicle standing:
   - `VLS flat` on bare flat path, wait ≥ 20 s (1000 results at 50 Hz), `VLS` → flat-path check ±5 %.
   - Edge at D = 15, 20, 25, 30, 40 cm from the sensor (measured horizontally): `VLS d15` … `VLS` per distance and per edge type (path/grass, edging, kerb); label e.g. `grass_d20`, `edge_d25`.
   - The same flat and one edge run on a ±2° roll wedge (`VLS roll+2`, `VLS roll-2`): z changes < 1 cm with roll compensation.
   - A post and a wall inside the footprint (`VLS post`, `VLS wall`); one flat run with F1/F2 aimed at a near wall (crosstalk).
   - Evaluate: `python3 tools/vls_eval.py tools/data/<date>/h9b.log --h <lat_h> --tilt-deg 15 --expect grass_d20=GRASS --expect edge_d25=EDGE_UP …` (95 % between `D_near` 16.9 and `D_far` 53.2 cm).
6. [ ] H9 (S2 outdoors, noon and afternoon, `--min-correct 99`), H10 (reflex, needs the `fwd` hook, not wired yet).
