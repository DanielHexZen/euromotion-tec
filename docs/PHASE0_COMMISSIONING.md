# Phase 0 Commissioning

Gate for `FIRMWARE_SPEC.md` §15 phase 0: H1–H5 pass and the `D` regression matches the old firmware.
Do the steps in order. Stop at the first failure and write it down; do not skip ahead.
Mark each box and keep the console log of every step (`screen`/CubeIDE terminal capture).

## Status (2026-11-12; vehicle results are from session 1, 2026-11-09)

**Phase 0 is NOT closed.** H3, H3b and the period/jitter part of H2 are open. Everything else on the checklist below is ticked or annotated. Session 1 is committed as `f551dd3`. The 2026-11-10 code and test changes listed below are in the WIP checkpoint commit `dfde781` (a checkpoint, not a reviewed commit with REQ IDs); They **have not been flashed**. Commit `f126577` (2026-11-12) adds the complete spec §4 `.ioc` target state (CubeMX 6.18.1) and the calibration firmware `calib.c`/`calib.h` (spec D-47, `CALIB_README.md`); all of `Core/Src` compiles and links with the CubeIDE `arm-none-eabi-gcc` 14.3 at `-Og`/`-O2`, `APP_CALIB` 0 and 1, 0 warnings (see below). Not yet built in CubeIDE.

| Gate | State |
|---|---|
| Host tests | passed, 54621 checks, 0 failed (`make -C test`, repeated 2026-11-12) |
| H1 tick/clock | passed |
| H2 slot timing | execution times passed on 2026-11-09 with the previous build (`S_CTRL max` 15 µs, `S_FAST max` 6 µs, overruns 0, tick 1000.0 Hz, cpu 0.4 %). **Period, jitter and tick-ISR time open:** firmware support added 2026-11-10 (self-measurement in `I`, decision D-32), **run on the vehicle pending**. The oscilloscope on PC8 is now an optional cross-check |
| H4 signs, dead-man, runaway, stall | passed (user report); EN-0 V-at-boot and reversal test not explicitly confirmed |
| H5 faults | `FAULTINJ`, FAULT-state answers, `CLEAR` passed; deliberate HardFault trigger not done; Release `FAULTINJ 4` waits for H3b |
| Radio | HELLO/`p`, concurrent input, replug, `s` over radio passed; `I` over radio passed with four lines (2026-11-09). `I` has five lines since 2026-11-10: read it once over the radio again |
| **H3 `D` regression** | **open**: baseline invalid (old mechanics), `D1000`/`D5000` not run, `D300` has 7 runs |
| **H3b Release build** | **open**: Release lacks the warning flags; not built or flashed |

### Next steps, in order
0. **H2 self-measurement (new, 2026-11-10; bench, about 20 min).** Build Debug in CubeIDE from commit `f126577` or later (first CubeIDE build of the regenerated `.ioc` and the 2026-11-10 code: must show 0 warnings), flash; the boot log must show `clock OK` and `12 sched OK`. Run §3 below. The main loop sleep changed, so also re-read H1 (`tick 1000.0 Hz`, no `WARN tick rate`) on this build. H2 and H3 are run on this build, not on the previous one.
1. **H3 baseline on the same mechanics.** Check out tag `old_firmware`, build, flash, run `r` + `D300`, `D1000`, `D5000` × 10 on the NEW casters and record `L=… R=…`. (Alternative: run the new firmware on the old casters and compare with the existing baseline.) Then the same series with the current firmware. Compare mean and spread (≤ ~1 mm) and check "no reversal / no TIMEOUT".
2. **Release configuration:** in CubeIDE set `-Wextra`, `-Wdouble-promotion`, `-Wshadow` for Release (the Debug-only entry is the only one in `.cproject`), optionally `-Wno-unused-parameter` for the `Drivers` folder; build Release (H3b), flash, 3 × `D1000`/`D5000`, `I`, `FAULTINJ 4` → `ERR state Release build`.
3. **Leftovers:** boot-to-`ready` < 2 s, `?`/`k`/`S`/`L`, legacy-command/range/`P…` checks of §7, deliberate HardFault trigger (§5).
4. **Close-out (§8):** delete the `#if 0` blocks in `main.c` (also remove the unused legacy `BTS7960`/`Wheel_*` code), rebuild, one `D1000`; enter the Drivers-warning exception (if used) in spec §18.2 (H2 by self-measurement is D-32); commit with REQ IDs; `git tag phase0`; save logs under `tools/data/<date>/`.
5. **Phase 1 (vehicle track)** only after that. Phase 1 is not blocked by `calib.c` (spec D-31): follow `PHASE1_CALIBRATION.md` with the normal console and the host tools in `tools/`. The calibration firmware (`calib.c`/`calib.h`, `APP_CALIB=1`, `CALIB_README.md`, spec D-47) is an optional second route; `main.c` includes `calib.h` only under `APP_CALIB` (spec D-27). It has not run on the vehicle. Host-track work (pure modules, host tests, SIL, MATLAB design) does not wait for this step (spec §15, D-29).

### Changed 2026-11-12 (commit `f126577`; not built in CubeIDE, not flashed)
- `Challenge_TestHC/Challenge_TestHC.ioc`, generated code: complete spec §4 target state (spec §4, D-48 to D-50). SPI1 mode 3 /16, I2C1/I2C2 400 kHz (PC12 as I2C2_SDA accepted), XSHUT_* outputs low, VL_S1_INT/VL_S2_INT EXTI, DMA for SPI1 and USART1 (USART1 still by interrupt, DMA unused until phase 4), IWDG (init call not generated: `app_init()` starts it last in phase 2), NVIC priorities (TIM2 IRQ 0 → 2), `HSE_STARTUP_TIMEOUT` 1000 ms. Labels kept (`LD2`, `HC12_Set`, `hadc1`). **CSS is not set** (headless CubeMX drops it): tick it in the CubeMX GUI in phase 2 together with `HAL_RCC_CSSCallback`. Verified: PLLM 4, TIM2 PSC 89, TIM6 89/999, TIM1 ARR 8999, USER CODE hooks intact. `.cproject`: trailing space only.
- `Core/Src/calib.c`, `Core/Inc/calib.h` (new, by request): calibration firmware, spec D-47; commands and procedure in `CALIB_README.md`. Not part of the phase-0 gate.
- Build check: `Core/Src` compiled and linked with `arm-none-eabi-gcc` 14.3 at `-Og` and `-O2`, `APP_CALIB` 0 and 1, 0 warnings; `make -C test` 54621 checks, 0 failed. Not the CubeIDE build (step 0).

### Changed in session 1 (2026-11-09; committed since)
- `Core/Inc/console.h`, `Core/Src/console.c`, `Core/Src/app_cmd.c`, `test/t_con.c`: `I` is now a chunked job (`CON_JOB_INFO`, `con_info_line()`), spec D-28, REQ-CON-01. Reason: `I` printed four lines at once and the 128-byte radio TX buffer dropped lines 3 and 4. Host test added; verified over the radio.
- `Challenge_TestHC/.cproject`, `.settings/language.settings.xml`, `.settings/stm32cubeide.project.prefs`: CubeIDE rewrote them (pre-build step, warning flags, scanner attributes). Review before committing.
- `FIRMWARE_SPEC.md` (D-28, REQ-CON-01 text), this file, `CLAUDE.md` (paths, pointer).

### Changed 2026-11-10 (WIP checkpoint `dfde781`; not built for the target, not flashed)
- `Core/Src/app.c`: race-free sleep in `app_loop()` (REQ-NFR-05). Before, a TIM6 tick that fired between `sys_sched_take()` and `__WFI()` left its slots pending until SysTick or a UART interrupt woke the core, up to 1 ms late. Now interrupts are masked, `sys_sched_pending()` is checked, and `__WFI()` runs only if nothing is pending (WFI wakes on a pending interrupt with PRIMASK set; the ISR runs at the restore). Found by review, never observed on the vehicle.
- `Core/Inc/sys_sched.h`, `Core/Src/sys_sched.c`: H2 self-measurement (REQ-EXE-11, REQ-NFR-01, REQ-NFR-05), integer only. `S_CTRL` start-to-start period from TIM2 (1 µs): min, max, sum, count. Tick-ISR body time from `DWT->CYCCNT`: max. New `sys_sched_pending()`, `sys_sched_stats_peek()`, `sys_sched_stats_reset()`. The `SLOT_CTRL` phase comment is reworded (phase 1 kept, D-30); no behaviour change there.
- `Core/Src/app_cmd.c`: `I` has a fifth line (the fourth in the output), `S_CTRL period min … max … mean … us n …; tick isr max … us`. `I` reads the statistics with `sys_sched_stats_peek()`; before, every `I` cleared the CPU-load sums of the running second, so `cpu` read low once. `r` (the `Counters reset` case) now also restarts the `I` maxima and the period statistics.
- `test/t_sched.c` (new), `test/Makefile`, `test/test_main.c`, `test/t_con.c`: host test of the period arithmetic and the `I` timing line. `make -C test`: 54621 checks, 0 failed.
- Target build: no `arm-none-eabi-gcc` on the machine that made the change. The four changed files pass a `clang --target=armv7em-none-eabi -fsyntax-only -Wall -Wextra -Wdouble-promotion -Wshadow` check at `-Og` and `-O2`; that is a syntax and warning check with host libc headers, not a build. **2026-11-12:** all 23 files of `Core/Src` compile with `arm-none-eabi-gcc` 13.2.1 (`-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -std=gnu11 -Wall -Wextra -Wdouble-promotion -DSTM32F446xx -DUSE_HAL_DRIVER -DDEBUG`, include paths of `.cproject`) at `-Og` and at `-O2` with 0 warnings and 0 errors. Compile only (`-c`): not linked, not the CubeIDE toolchain, without `build_info.h`, not flashed. Step 0 above (first CubeIDE build, 0 warnings) stays open; the 2026-11-12 check used gcc 14.3 with link (see the 2026-11-12 entry).
- `FIRMWARE_SPEC.md`: D-32 and the matching text changes are applied separately.

### Setup facts a new session needs
- Console: USB VCP `/dev/cu.usbmodem11403` (number can change: `ls /dev/cu.usbmodem*`), `screen /dev/cu.usbmodem11403 115200`; radio terminal at 9600. The board is powered from E5V (JP5); USB can still be plugged in for the console. `I` maxima and the period statistics are since boot **or since the last `r`** that answered `Counters reset` (2026-11-10; before that: since boot only). `n` in the period line is the number of 20 ms intervals in the window. Run first, read `I` after; unplugging USB does not reset the board. Overrun counts are always since boot.
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

## 3. H2 – slot timing (firmware self-measurement through `I`, Debug build)

Decision D-32 (2026-11-10): H2 is closed by the firmware measuring itself; the oscilloscope is an optional cross-check. **Not run yet.** The 2026-11-09 ticks below were taken with the previous build and cover execution times only.

What `I` prints (line 4 is new):
```
S_CTRL max 15 us, overruns 0; S_FAST max 6 us
S_CTRL period min 19996 max 20004 mean 20000.0 us n 45000; tick isr max 0.6 us
```
(example values, not a measurement). Period = time between two consecutive `S_CTRL` starts, from TIM2 (1 µs). `n` = intervals counted; 50 per second. The first interval after boot is not counted. `tick isr max` = body of `sys_sched_tick_isr()` from the DWT cycle counter, truncated to 0.1 µs.

Steps:
- [ ] Flash the Debug build that contains the 2026-11-10 changes; `I` shows the `S_CTRL period` line. Note the hash.
- [ ] **Idle run, 15 min** (spec §1.4): type `r` (answers `Counters reset`, restarts the statistics), leave the vehicle idle for 15 min, type `I`. Record the two lines. `n` ≥ 45000.
- [ ] **Loaded run** (wheels in the air or on the floor): `r`, `a` (auto print on), 3 × `D1000`, `a` off, `I`. Record the two lines. Do not type `r` between the moves and the `I`.
- [ ] Repeat the loaded run once with the console on the radio port (5 lines of `I` arrive complete).

Pass, for every run:
- [ ] Period mean 20.000 ms ± 0.01 %: `mean` within 19998.0 … 20002.0 us – REQ-NFR-05.
- [ ] Jitter < 100 µs: `max` − `min` < 100 us (this also bounds each of them within 100 µs of 20000) – REQ-NFR-05.
- [ ] `tick isr max` ≤ 2.0 us – REQ-NFR-01.
- [ ] `S_CTRL max` ≤ 600 us – REQ-NFR-02; `overruns 0`; `S_FAST max` well below 150 us.
- [ ] `cpu` < 50 % – REQ-NFR-04 (line 2 of `I`).

Limits of the self-measurement (write them next to the result):
- TIM2, TIM6 and the CPU clock come from the same oscillator (HSE). `mean` therefore proves that no tick was lost or shifted and that the dividers are right; it cannot see an oscillator frequency error. Only an external instrument can. The boot `clock OK` check has the same limit.
- `tick isr max` excludes exception entry/exit and the HAL dispatch (`TIM6_DAC_IRQHandler` → `HAL_TIM_IRQHandler` → callback) in front of the body. Estimate, not measured: 0.5–1.5 µs more.
- A failure: note the three numbers and `n`, type `r`, repeat. A single outlier in `max` or `min` that does not come back is still a failure of the run; find the cause (which command or mode was active) before repeating.

Earlier results (2026-11-09, previous build, `I` only):
- [x] `S_CTRL` execution ≤ 600 µs, also while a `D1000` runs and while `a` (auto print) is on – REQ-NFR-02. *(after 2 × `D1000` with `a` on, `S_CTRL max` 15 µs)*
- [x] `I` shows `S_CTRL max … us` ≤ 600, `overruns 0`, `S_FAST max` well below 150 µs. *(15 µs, overruns 0, `S_FAST max` 6 µs)*
- Tick-ISR ≤ 2 µs (REQ-NFR-01) was accepted on 2026-11-09 without a measurement ("no overruns"). That acceptance is replaced by the `tick isr max` criterion above and is open until it is read.

Optional cross-check with an instrument (not required for the gate):
- [ ] Oscilloscope or logic analyser on PC8 (`PROBE`, Debug build): 50 Hz pulse train, period 20.000 ms mean, jitter < 100 µs, pulse width = `S_CTRL` execution. This is the only check of the absolute frequency.

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

- [ ] Build **Release** (`-O2`), flash, repeat 3 runs of `D1000` and `D5000` plus H1 and the H2 loaded run of §3 (`I`). Behaviour identical to Debug (this catches missing `volatile`).

## 7. Console and radio checks

- [ ] Legacy commands unchanged: `D`, `V`, `T`, `F`, `G`, `Y`, `-100…100`, `s`, `r`, `p`, `a`, `?` behave as before (units mm, mm/s, %).
- [ ] `V500` → `ERR range 20..450`; `D3` → `ERR range 5..20000`; `IMU` → `ERR unknown, ? for help`.
- [ ] `P113=3`, `P113?`, `Pwhl_kp_x?` work; `P201=1.01` while moving → `ERR state …`; `L` dumps `#define PAR_…` lines.
- [x] Concurrent input on USB and radio does not garble either line. *(per user report 2026-11-09)*
- [x] Unplug/replug the radio while idle: the console still answers (UART errors re-arm reception).
- [x] Send `s` from the radio during a USB-started `D1000`: the vehicle stops.

## 8. Sign-off

- [ ] H1, H2 (self-measurement, §3), H3, H3b, H4, H5 all passed; logs and the filled table saved under `tools/data/<date>/` with the firmware hash (`I`) and a `L` dump.
- [ ] Open findings written down (backup-register record, any parameter changed during H3).
- [ ] Commit: `Phase 0: restructure complete, H1–H5 passed (REQ-ARC-01..04, REQ-EXE-*, REQ-MOT-*, REQ-ENC-*, REQ-WHL-*, REQ-DRV-*, REQ-PAR-*, REQ-CON-*)`.
- [ ] `git tag phase0`. Delete the `#if 0` blocks in `main.c` (spec §16.3 step 3), rebuild, repeat one `D1000`.
- [ ] Only then start phase 1 (calibration) on the vehicle (`PHASE1_CALIBRATION.md`); host-track work may run ahead (spec §15, D-29).
