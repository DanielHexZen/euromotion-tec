# Phase 0 Commissioning

Gate for `FIRMWARE_SPEC.md` §15 phase 0: H1–H5 pass and the `D` regression matches the old firmware.
Do the steps in order. Stop at the first failure and write it down; do not skip ahead.
Mark each box and keep the console log of every step (`screen`/CubeIDE terminal capture).

## Status (2026-11-09, end of session 1)

**Phase 0 is NOT closed.** H3, H3b and the scope part of H2 are open. Everything else on the checklist below is ticked or annotated. Nothing is committed since `681af00`.

| Gate | State |
|---|---|
| Host tests | passed, 54601 checks (`make -C test`) |
| H1 tick/clock | passed |
| H2 slot timing | firmware numbers passed (`S_CTRL max` 15 µs, `S_FAST max` 6 µs, overruns 0, tick 1000.0 Hz, cpu 0.4 %); **scope part open** (no instrument) |
| H4 signs, dead-man, runaway, stall | passed (user report); EN-0 V-at-boot and reversal test not explicitly confirmed |
| H5 faults | `FAULTINJ`, FAULT-state answers, `CLEAR` passed; deliberate HardFault trigger not done; Release `FAULTINJ 4` waits for H3b |
| Radio | HELLO/`p`, concurrent input, replug, `s` over radio passed; `I` over radio passed after the change below |
| **H3 `D` regression** | **open**: baseline invalid (old mechanics), `D1000`/`D5000` not run, `D300` has 7 runs |
| **H3b Release build** | **open**: Release lacks the warning flags; not built or flashed |

### Next steps, in order
1. **H3 baseline on the same mechanics.** Check out tag `old_firmware`, build, flash, run `r` + `D300`, `D1000`, `D5000` × 10 on the NEW casters and record `L=… R=…`. (Alternative: run the new firmware on the old casters and compare with the existing baseline.) Then the same series with the current firmware. Compare mean and spread (≤ ~1 mm) and check "no reversal / no TIMEOUT".
2. **Release configuration:** in CubeIDE set `-Wextra`, `-Wdouble-promotion`, `-Wshadow` for Release (the Debug-only entry is the only one in `.cproject`), optionally `-Wno-unused-parameter` for the `Drivers` folder; build Release (H3b), flash, 3 × `D1000`/`D5000`, `I`, `FAULTINJ 4` → `ERR state Release build`.
3. **Leftovers:** boot-to-`ready` < 2 s, `?`/`k`/`S`/`L`, legacy-command/range/`P…` checks of §7, deliberate HardFault trigger (§5).
4. **Close-out (§8):** delete the `#if 0` blocks in `main.c` (also remove the unused legacy `BTS7960`/`Wheel_*` code), rebuild, one `D1000`; enter the Drivers-warning exception (if used) and the H2 scope gap in spec §18.2; commit with REQ IDs; `git tag phase0`; save logs under `tools/data/<date>/`.
5. **Phase 1** only after that. **Blocker for phase 1:** `calib.c`/`calib.h` are not in the repository (`main.c` includes `calib.h` only under `APP_CALIB`, spec D-27). The calibration firmware cannot be built until they are supplied. CLAUDE.md says leave `calib.c` unchanged, so do not rewrite it.

### Changed this session (uncommitted)
- `Core/Inc/console.h`, `Core/Src/console.c`, `Core/Src/app_cmd.c`, `test/t_con.c`: `I` is now a chunked job (`CON_JOB_INFO`, `con_info_line()`), spec D-28, REQ-CON-01. Reason: `I` printed four lines at once and the 128-byte radio TX buffer dropped lines 3 and 4. Host test added; verified over the radio.
- `Challenge_TestHC/.cproject`, `.settings/language.settings.xml`, `.settings/stm32cubeide.project.prefs`: CubeIDE rewrote them (pre-build step, warning flags, scanner attributes). Review before committing.
- `FIRMWARE_SPEC.md` (D-28, REQ-CON-01 text), this file, `CLAUDE.md` (paths, pointer).

### Setup facts a new session needs
- Console: USB VCP `/dev/cu.usbmodem11403` (number can change: `ls /dev/cu.usbmodem*`), `screen /dev/cu.usbmodem11403 115200`; radio terminal at 9600. The board is powered from E5V (JP5); USB can still be plugged in for the console. `I` maxima are since boot: run first, read `I` after; unplugging USB does not reset the board.
- Launch with **Run → Debug Configurations → STM32 C/C++ Application → `Challenge_TestHC Debug`**. A generic "C/C++ Application" config (`… Debug (1)`) calls plain `gdb` and fails. `Challenge_TestHC Debug.launch` still holds an old absolute log path (`/Users/danielappel/Desktop/Microcontrollers/A2.4/…`), harmless.
- Pre-build step `../../tools/gen_build_info.sh` works (`build_info.h` is git-ignored, hash shows `-dirty`).
- `7 bat absent` is expected: ADC1 is generated but `main.c` does not pass `.adc1` in `app_hw_t`, and the 47k/10k divider is not wired to PA0. Not needed for the phase-0 gate.
- Spec and this file live in the repository root (CLAUDE.md formerly said `docs/`).

Console: USB virtual COM port, 115200 8N1, CR or LF terminated. Radio (HC-12, 9600) mirrors it.

## 0. Preparation (no vehicle power)

- [x] Host tests green: `make -C test` → `0 failed`. *(54601 checks, 0 failed, 2026-11-09, after the `I` job change)*
- [ ] Baseline recorded **with the old firmware** (spec §16.3 step 1), on the floor, before anything else is flashed: **Status: exists, but taken on the OLD mechanics (old casters).** The tag is `old_firmware`, not `pre-spec-v1`. Not valid against the new damped casters; see Status.
      `git tag pre-spec-v1`, then `r` + `D300`, `D1000`, `D5000` × 10 each. Note the end value `L=… R=…` of each run in the table below.
- [x] `git commit` the current tree (CubeMX protocol: commit before regenerating). *(c1dcf1d … 681af00)*
- [x] CubeMX regeneration, values entered in CubeMX, never patched in code: *(verified in `.ioc` and generated `main.c`: TIM1 ARR 8999, TIM2 PSC 89, TIM6 89/999, ADC1 IN0 480 cycles, PC8 `PROBE`, stack 0x1000, heap 0x400)*

  | Item | Value |
  |---|---|
  | TIM1 | ARR 8999 (PSC 0) |
  | TIM2 | PSC 89, ARR 0xFFFFFFFF |
  | TIM6 | PSC 89, ARR 999 |
  | ADC1 | IN0 on PA0, 12 bit, 480 cycles, software start (optional for phase 0; without it the battery reads "absent") |
  | PC8 | GPIO out, label `PROBE` (needed for H2) |
  | Project | min stack 0x1000, float printf off |

- [x] `git diff Core/Src/main.c .cproject Core/Inc/stm32f4xx_hal_conf.h`: check TIM6 (89 / 999), TIM2 (89), PLLM 4 in the generated code, and that the USER CODE hooks survived. *(PLLM 4, TIM values and USER CODE hooks checked)*
- [ ] CubeIDE: add pre-build step `../../tools/gen_build_info.sh`; warnings `-Wall -Wextra -Wdouble-promotion -Wshadow`; no `-ffast-math`. **Partly done:** pre-build step is set in Debug and Release. `-Wextra -Wdouble-promotion -Wshadow` are set in ONE configuration only (presumably Debug); Release still lacks them. `-Wno-unused-parameter` for `Drivers/` is NOT set.
- [ ] Build **Debug**: no warnings. (Release is built in H3b.) **Open:** the build runs with 0 errors; the warning count was not reported. Three `-Wunused-parameter` warnings from `Drivers/…/stm32f4xx_hal_flash_ex.c` (HAL code, `Banks`) were seen with `-Wextra`.

## 1. Bench power-up (USB only, wheels in the air, motor supply OFF)

- [x] Flash Debug, open the console. Expected boot log: *(boot log all OK, `bat absent`)*
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
- [x] Any `FAIL` or `FAULT`: `2 mot FAIL 4` = TIM1 ARR wrong, `1 sys_time FAIL 4` = TIM2 PSC wrong, `12 sched FAIL 4` = TIM6 wrong, `6 clock FAIL` = HSE not running (HSE bypass needs the ST-Link powered). Fix in CubeMX, not in code. *(none)*
- [ ] Time from reset to `ready` < 2 s (REQ-NFR-09).
- [ ] Type `?`, `k`, `S`, `L`: output complete, no lost characters (`k` and `L` arrive in chunks).
- [x] Open the same console on the radio port: HELLO appears there too; type `p`, answer returns on the radio only. *(per user report 2026-11-09)*

## 2. H1 – tick and clock

- [x] Wait 3 s after reset, type `I`. Required: `tick 1000.0 Hz` within 999.0–1001.0, `cpu` < 50 %. *(tick 1000.0 Hz, cpu 0.4 %)*
- [x] Boot log showed `clock OK` (TIM2 µs over 100 ms SysTick = 100 000 ± 100).
- [x] No `WARN tick rate` line appears during a 5-minute idle. *(per user report: H1 done)*

## 3. H2 – slot timing (oscilloscope on PC8, Debug build)

- [ ] PC8 is a 50 Hz pulse train (period 20.000 ms mean, jitter < 100 µs) – REQ-NFR-05. **OPEN: no oscilloscope or logic analyser available (2026-11-09).** Period, jitter and pulse width are unmeasured. Known gap; do it when an instrument is at hand.
- [x] Pulse width = `S_CTRL` execution ≤ 600 µs, also while a `D1000` runs and while `a` (auto print) is on – REQ-NFR-02. *(via `I` only: after 2 × `D1000` with `a` on, `S_CTRL max` 15 µs. Scope not measured)*
- [x] `I` shows `S_CTRL max … us` ≤ 600, `overruns 0`, `S_FAST max` well below 150 µs. *(15 µs, overruns 0, `S_FAST max` 6 µs)*
- [x] Tick-ISR ≤ 2 µs (REQ-NFR-01): not visible on PC8; accept if S_FAST/S_CTRL numbers are small and no overruns. Measure separately only if overruns appear. *(accepted: no overruns)*

## 4. H4 – truth of signs (wheels in the air, motor supply ON)

- [ ] Both EN lines low at boot: PB12/PB13 read 0 V before the first command. *(not explicitly reported)*
- [x] `r`, then spin each wheel by hand forward: `p` shows **positive** counts and positive mm for **both** wheels (`ENC_SIGN_L=+1`, `ENC_SIGN_R=-1`). Wrong sign → stop, correct the constant, rerun the unit test, re-flash. *(per user report: signs correct)*
- [x] `10` (open loop 10 %): both wheels turn **forward**; ramp is slow (2 /s). `s` stops them (EN low a moment later).
- [x] `-10`: both reverse. `PWM 20 0` / `PWM 0 20`: only the named wheel turns (check L/R wiring).
- [ ] Reversal test: `20`, then `-20`: duty passes through zero for ~50 ms, no jump. *(not explicitly reported)*
- [x] Dead-man: `20`, then send nothing: motors stop after 2 s and `DEADMAN` is printed.
- [x] Runaway detector: flip `MOT_SIGN_L` temporarily (Debug build only), `D500`: `FAULT … first fault 0x06`, motors off within ~0.6 s. **Restore the sign** and re-flash. *(per user report; `MOT_SIGN_L` is back to −1.0f in `mot.h`, checked 2026-11-09. Re-flash if the Debug build with the flipped sign is still on the board)*
- [x] Stall detector: hold one wheel firmly during `D500` (hand brake, short): `FAULT` with 0x04/0x05 (stall) or 0x08/0x09 (track) after ~0.6 s. `CLEAR` returns to IDLE.

## 5. H5 – faults (Debug build only)

For each of 0x04, 0x06, 0x08, 0x13 run `20` (wheels in the air), then `FAULTINJ <code>`:

| Code | Name | Expected |
|---|---|---|
| 0x04 | stall L (R3) | `FAULT`, wheels brake, EN low ≤ 1.5 s |
| 0x06 | runaway L (R4) | PB12/PB13 low **at once** (scope), `FAULT` |
| 0x08 | track L (R3) | as 0x04 |
| 0x13 | numeric (R4) | as 0x06 |

- [x] Each: motors stop within one tick (≤ 20 ms), console prints `FAULT, motors stopped. first fault 0x..`. *(per user report: `FAULTINJ` behaves as expected; the "at once" PB12/PB13 check of 0x06 needs a scope/meter and was not measured)*
- [x] While in FAULT: `D100`, `20`, `PWM …` answer `ERR state FAULT`; `S`, `FAULTS`, `EV`, `P…?` still work.
- [x] `CLEAR` (or `r`) → back to IDLE, `D100` works again. `FAULTS`/`EV` list the fault.
- [ ] `Error_Handler`/HardFault path: temporarily trigger a fault (e.g. call `Error_Handler()` after boot, or write through a null pointer, Debug only). EN lines go low (scope), output stays off. After reset the HELLO shows `last hard fault: code 0x02 …` with stack words. Remove the trigger. Spec marks the backup-register access `[VERIFY]`: if no record appears, note it as an open finding. **Not done deliberately.** Finding: on first boot HELLO printed a stale `last hard fault: code 0x02 msp 0x20000BA8` (stack words `3F800000 20000BA8 00000000 FFFFFFF9 00000001 00000200 0000AAAA 00000008 00000000 2000053D 20000000 21000200`). It did not come back after the next reset, so the backup-register record works and the entry was left by an earlier flash state. Origin unknown; the PC/LR guess from the raw words is unreliable (D-26). Watch every reset; if it returns, break in `HardFault_Handler`.
- [ ] Release build: `FAULTINJ 4` answers `ERR state Release build`. *(needs the Release build, H3b)*

## 6. H3 – `D` regression on the floor (battery, vehicle on the ground)

Same surface and battery state as the baseline. `r` before each series. Speed and ramps at defaults (`V300`, `T1000`).

- [ ] Each of `D300`, `D1000`, `D5000` × 10. Each run prints `DONE L=… R=… sync_max=… t=…` (mm, ms).
- [ ] Pass for every run: both wheels end within **2 mm** of the target, no reversal during the settle phase (listen/watch; the wheels never turn backwards after the profile ends), no `TIMEOUT`, no `FAULT`.
- [ ] Compare with the baseline: mean and spread of the end values equal to the old firmware within ~1 mm. A systematic shift means a parameter default is wrong (check `k`, spec Appendix 19.2).
- [ ] `D-500` and `W90` (turn): ends ±2 mm per wheel; `sync_max` < 3 mm.
- [ ] `D2000`, then `s` at ~1 s: smooth ramp to rest (~`0.3/0.5` s), no jerk, ends in IDLE.

**Partial data (new firmware, NEW damped casters, 2026-11-09, console `DONE` lines, target 300 mm):**
L/R = 298.0/298.0, 298.2/298.6, 298.1/298.1, 298.0/298.3, 298.1/298.3, 298.0/298.0, 298.3/298.2; `sync_max` 2.4–3.4 mm; `t` 2500–2600 ms. Every run ends 1.4–2.0 mm SHORT, at the gate edge: `SETTLE` brakes a wheel as soon as the rest is under `drv_pos_tol` (2 mm) and never drives backwards, so this is the design, not a controller fault. Only seven `D300` runs; no `D1000` or `D5000` yet; no reversal/TIMEOUT observations recorded.
User observation: the vehicle physically drives slightly FARTHER than commanded while the encoder frame ends at 298 mm, i.e. more ground distance per count than `odo_m_per_cnt` assumes (effective wheel diameter larger, not smaller; or slip). Unmeasured; belongs to phase 1 calibration, not to H3.

| Run | D300 L / R | D1000 L / R | D5000 L / R | baseline (old) |
|---|---|---|---|---|
| 1…10 | see above (7 runs) | | | **re-record, see Status** |

### H3b – Release build

- [ ] Build **Release** (`-O2`), flash, repeat 3 runs of `D1000` and `D5000` plus H1 (`I`). Behaviour identical to Debug (this catches missing `volatile`).

## 7. Console and radio checks

- [ ] Legacy commands unchanged: `D`, `V`, `T`, `F`, `G`, `Y`, `-100…100`, `s`, `r`, `p`, `a`, `?` behave as before (units mm, mm/s, %).
- [ ] `V500` → `ERR range 20..450`; `D3` → `ERR range 5..20000`; `IMU` → `ERR unknown, ? for help`.
- [ ] `P113=3`, `P113?`, `Pwhl_kp_x?` work; `P201=1.01` while moving → `ERR state …`; `L` dumps `#define PAR_…` lines.
- [x] Concurrent input on USB and radio does not garble either line. *(per user report 2026-11-09)*
- [x] Unplug/replug the radio while idle: the console still answers (UART errors re-arm reception).
- [x] Send `s` from the radio during a USB-started `D1000`: the vehicle stops.

## 8. Sign-off

- [ ] H1, H2, H3, H3b, H4, H5 all passed; logs and the filled table saved under `tools/data/<date>/` with the firmware hash (`I`) and a `L` dump.
- [ ] Open findings written down (backup-register record, any parameter changed during H3).
- [ ] Commit: `Phase 0: restructure complete, H1–H5 passed (REQ-ARC-01..04, REQ-EXE-*, REQ-MOT-*, REQ-ENC-*, REQ-WHL-*, REQ-DRV-*, REQ-PAR-*, REQ-CON-*)`.
- [ ] `git tag phase0`. Delete the `#if 0` blocks in `main.c` (spec §16.3 step 3), rebuild, repeat one `D1000`.
- [ ] Only then start phase 1 (calibration).
