# Phase 0 Commissioning

Gate for `FIRMWARE_SPEC.md` §15 phase 0: H1–H5 pass and the `D` regression matches the old firmware.
Do the steps in order. Stop at the first failure and write it down; do not skip ahead.
Mark each box and keep the console log of every step (`screen`/CubeIDE terminal capture).

Console: USB virtual COM port, 115200 8N1, CR or LF terminated. Radio (HC-12, 9600) mirrors it.

## 0. Preparation (no vehicle power)

- [ ] Host tests green: `make -C test` → `0 failed`.
- [ ] Baseline recorded **with the old firmware** (spec §16.3 step 1), on the floor, before anything else is flashed:
      `git tag pre-spec-v1`, then `r` + `D300`, `D1000`, `D5000` × 10 each. Note the end value `L=… R=…` of each run in the table below.
- [ ] `git commit` the current tree (CubeMX protocol: commit before regenerating).
- [ ] CubeMX regeneration, values entered in CubeMX, never patched in code:

  | Item | Value |
  |---|---|
  | TIM1 | ARR 8999 (PSC 0) |
  | TIM2 | PSC 89, ARR 0xFFFFFFFF |
  | TIM6 | PSC 89, ARR 999 |
  | ADC1 | IN0 on PA0, 12 bit, 480 cycles, software start (optional for phase 0; without it the battery reads "absent") |
  | PC8 | GPIO out, label `PROBE` (needed for H2) |
  | Project | min stack 0x1000, float printf off |

- [ ] `git diff Core/Src/main.c .cproject Core/Inc/stm32f4xx_hal_conf.h`: check TIM6 (89 / 999), TIM2 (89), PLLM 4 in the generated code, and that the USER CODE hooks survived.
- [ ] CubeIDE: add pre-build step `../../tools/gen_build_info.sh`; warnings `-Wall -Wextra -Wdouble-promotion -Wshadow`; no `-ffast-math`.
- [ ] Build **Debug**: no warnings. (Release is built in H3b.)

## 1. Bench power-up (USB only, wheels in the air, motor supply OFF)

- [ ] Flash Debug, open the console. Expected boot log:
  ```
  HELLO MR2006B fw 0.1-phase0 <hash> Debug reset=0x.. calib=0
   1 sys_time OK
   2 mot      OK
   3 enc      OK
   4 par      OK
   6 clock    OK
   7 bat      absent      (or OK with ADC1 and a pack)
  12 sched    OK
  ready, ? for help
  ```
- [ ] Any `FAIL` or `FAULT`: `2 mot FAIL 4` = TIM1 ARR wrong, `1 sys_time FAIL 4` = TIM2 PSC wrong, `12 sched FAIL 4` = TIM6 wrong, `6 clock FAIL` = HSE not running (HSE bypass needs the ST-Link powered). Fix in CubeMX, not in code.
- [ ] Time from reset to `ready` < 2 s (REQ-NFR-09).
- [ ] Type `?`, `k`, `S`, `L`: output complete, no lost characters (`k` and `L` arrive in chunks).
- [ ] Open the same console on the radio port: HELLO appears there too; type `p`, answer returns on the radio only.

## 2. H1 – tick and clock

- [ ] Wait 3 s after reset, type `I`. Required: `tick 1000.0 Hz` within 999.0–1001.0, `cpu` < 50 %.
- [ ] Boot log showed `clock OK` (TIM2 µs over 100 ms SysTick = 100 000 ± 100).
- [ ] No `WARN tick rate` line appears during a 5-minute idle.

## 3. H2 – slot timing (oscilloscope on PC8, Debug build)

- [ ] PC8 is a 50 Hz pulse train (period 20.000 ms mean, jitter < 100 µs) – REQ-NFR-05.
- [ ] Pulse width = `S_CTRL` execution ≤ 600 µs, also while a `D1000` runs and while `a` (auto print) is on – REQ-NFR-02.
- [ ] `I` shows `S_CTRL max … us` ≤ 600, `overruns 0`, `S_FAST max` well below 150 µs.
- [ ] Tick-ISR ≤ 2 µs (REQ-NFR-01): not visible on PC8; accept if S_FAST/S_CTRL numbers are small and no overruns. Measure separately only if overruns appear.

## 4. H4 – truth of signs (wheels in the air, motor supply ON)

- [ ] Both EN lines low at boot: PB12/PB13 read 0 V before the first command.
- [ ] `r`, then spin each wheel by hand forward: `p` shows **positive** counts and positive mm for **both** wheels (`ENC_SIGN_L=+1`, `ENC_SIGN_R=-1`). Wrong sign → stop, correct the constant, rerun the unit test, re-flash.
- [ ] `10` (open loop 10 %): both wheels turn **forward**; ramp is slow (2 /s). `s` stops them (EN low a moment later).
- [ ] `-10`: both reverse. `PWM 20 0` / `PWM 0 20`: only the named wheel turns (check L/R wiring).
- [ ] Reversal test: `20`, then `-20`: duty passes through zero for ~50 ms, no jump.
- [ ] Dead-man: `20`, then send nothing: motors stop after 2 s and `DEADMAN` is printed.
- [ ] Runaway detector: flip `MOT_SIGN_L` temporarily (Debug build only), `D500`: `FAULT … first fault 0x06`, motors off within ~0.6 s. **Restore the sign** and re-flash.
- [ ] Stall detector: hold one wheel firmly during `D500` (hand brake, short): `FAULT` with 0x04/0x05 (stall) or 0x08/0x09 (track) after ~0.6 s. `CLEAR` returns to IDLE.

## 5. H5 – faults (Debug build only)

For each of 0x04, 0x06, 0x08, 0x13 run `20` (wheels in the air), then `FAULTINJ <code>`:

| Code | Name | Expected |
|---|---|---|
| 0x04 | stall L (R3) | `FAULT`, wheels brake, EN low ≤ 1.5 s |
| 0x06 | runaway L (R4) | PB12/PB13 low **at once** (scope), `FAULT` |
| 0x08 | track L (R3) | as 0x04 |
| 0x13 | numeric (R4) | as 0x06 |

- [ ] Each: motors stop within one tick (≤ 20 ms), console prints `FAULT, motors stopped. first fault 0x..`.
- [ ] While in FAULT: `D100`, `20`, `PWM …` answer `ERR state FAULT`; `S`, `FAULTS`, `EV`, `P…?` still work.
- [ ] `CLEAR` (or `r`) → back to IDLE, `D100` works again. `FAULTS`/`EV` list the fault.
- [ ] `Error_Handler`/HardFault path: temporarily trigger a fault (e.g. call `Error_Handler()` after boot, or write through a null pointer, Debug only). EN lines go low (scope), output stays off. After reset the HELLO shows `last hard fault: code 0x02 …` with stack words. Remove the trigger. Spec marks the backup-register access `[VERIFY]`: if no record appears, note it as an open finding.
- [ ] Release build: `FAULTINJ 4` answers `ERR state Release build`.

## 6. H3 – `D` regression on the floor (battery, vehicle on the ground)

Same surface and battery state as the baseline. `r` before each series. Speed and ramps at defaults (`V300`, `T1000`).

- [ ] Each of `D300`, `D1000`, `D5000` × 10. Each run prints `DONE L=… R=… sync_max=… t=…` (mm, ms).
- [ ] Pass for every run: both wheels end within **2 mm** of the target, no reversal during the settle phase (listen/watch; the wheels never turn backwards after the profile ends), no `TIMEOUT`, no `FAULT`.
- [ ] Compare with the baseline: mean and spread of the end values equal to the old firmware within ~1 mm. A systematic shift means a parameter default is wrong (check `k`, spec Appendix 19.2).
- [ ] `D-500` and `W90` (turn): ends ±2 mm per wheel; `sync_max` < 3 mm.
- [ ] `D2000`, then `s` at ~1 s: smooth ramp to rest (~`0.3/0.5` s), no jerk, ends in IDLE.

| Run | D300 L / R | D1000 L / R | D5000 L / R | baseline (old) |
|---|---|---|---|---|
| 1…10 | | | | |

### H3b – Release build

- [ ] Build **Release** (`-O2`), flash, repeat 3 runs of `D1000` and `D5000` plus H1 (`I`). Behaviour identical to Debug (this catches missing `volatile`).

## 7. Console and radio checks

- [ ] Legacy commands unchanged: `D`, `V`, `T`, `F`, `G`, `Y`, `-100…100`, `s`, `r`, `p`, `a`, `?` behave as before (units mm, mm/s, %).
- [ ] `V500` → `ERR range 20..450`; `D3` → `ERR range 5..20000`; `IMU` → `ERR unknown, ? for help`.
- [ ] `P113=3`, `P113?`, `Pwhl_kp_x?` work; `P201=1.01` while moving → `ERR state …`; `L` dumps `#define PAR_…` lines.
- [ ] Concurrent input on USB and radio does not garble either line.
- [ ] Unplug/replug the radio while idle: the console still answers (UART errors re-arm reception).
- [ ] Send `s` from the radio during a USB-started `D1000`: the vehicle stops.

## 8. Sign-off

- [ ] H1, H2, H3, H3b, H4, H5 all passed; logs and the filled table saved under `tools/data/<date>/` with the firmware hash (`I`) and a `L` dump.
- [ ] Open findings written down (backup-register record, any parameter changed during H3).
- [ ] Commit: `Phase 0: restructure complete, H1–H5 passed (REQ-ARC-01..04, REQ-EXE-*, REQ-MOT-*, REQ-ENC-*, REQ-WHL-*, REQ-DRV-*, REQ-PAR-*, REQ-CON-*)`.
- [ ] `git tag phase0`. Delete the `#if 0` blocks in `main.c` (spec §16.3 step 3), rebuild, repeat one `D1000`.
- [ ] Only then start phase 1 (calibration).
