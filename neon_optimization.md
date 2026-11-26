# MR2006B System Analysis & Optimization Suggestions

> **Repository:** `euromotion-tec` (MR2006B Autonomous Differential-Drive Vehicle — STM32F446RE)  
> **Scope:** Repository structure, core firmware/software logic summary, and optimization suggestions only (no code modified).  
> **Baseline Snapshot:** Branch `phase2-imu-ekf`, host test suite passing (58,076 checks, 0 failed).

---

## 1. Repository Overview & Architecture

This repository contains the embedded firmware, simulation/verification models, ground-station communication stack, calibration tools, and engineering documentation for the **MR2006B autonomous differential-drive vehicle** running on an **STM32F446RE (NUCLEO-F446RE, 180 MHz Cortex-M4F)**.

The firmware adheres to strict embedded engineering constraints (`C11`, no RTOS, no `malloc`, `float`-only math, non-blocking post-init execution, SI units internally, and pure hardware-independent logic modules testable on the host).

### Directory Layout

| Path | Role |
|---|---|
| `Challenge_TestHC/Core/` | Active STM32 HAL firmware (`Inc/`, `Src/`, `Startup/`). `main.c` holds only CubeMX code + 4 USER CODE hooks; all HAL callbacks live in `app_isr.c`. Includes optional calibration firmware (`calib.c`, `APP_CALIB=1`). |
| `docs/` | Normative spec (`FIRMWARE_SPEC.md`, v1.17) and phase commissioning docs (`PHASE0_COMMISSIONING.md` through `PHASE3_SENSORS.md`, `BENCH_README.md`, `CALIB_README.md`). |
| `test/` | Host C unit tests (`make -C test`) compiling all 12 pure modules (`par`, `prof`, `whl`, `drv`, `fmt`, `proto`, `console`, `att`, `ekf`, `imu_yaw`, `gnd`, `fwd`) with `-Werror`, ASan, and UBSan (58,076 checks). |
| `Matlab/ekf_design/` | MATLAB/Octave EKF reference model (`core/`), multirate simulator (`sim/`), observability/sensitivity/gate studies (`design/`), verification suite (`verify/`), and Simulink model builder (`simulink/`). |
| `ground_station/` | Python binary ground-station stack: COBS + CRC-16/CCITT codec, schema-driven `protocol.yaml`, C code generator (`tools/gen_protocol.py`), raw byte session recorder, simulator, and pytest suite. |
| `tools/` | Standard-library Python Phase-1 calibration tools (`straight_scale.py`, `umbmark.py`, `ffit.py`, `calib_host.py`), `gen_build_info.sh`, and dated hardware test logs (`tools/data/`). |
| `MR2006B_Vehicle_Technical_Reference_source/` | LaTeX source & PDF explaining physical models, control/estimation rationale, and course geometry. |
| `Activity 2.6/`, `Assessment 1/`, `motortesting/`, `reports/`, `Matlab/assessment1/` | Coursework reports, legacy experiments, and early motor test projects. |

### Current Commissioning Status

- **Phase 0 (Restructure & Timing):** Closed (`phase0` tag).
- **Phase 1 (Feedforward & Odometry):** Feedforward (`whl_ks/kv/ksb/tau_m`) and odometry (`odo_m_per_cnt`, `odo_scale_l/r`, `odo_wheelbase`) defaults compiled in `par_table.h`. Gate H6 (UMBmark < 10 mm) remains open due to mechanical caster swivel drag.
- **Phase 2 (IMU & EKF):** In commissioning on `phase2-imu-ekf` (LSM6DS3 on SPI1 DMA + TIM2 CH2 capture, `att`, `imu_yaw`, 5-state `ekf`, IWDG, `W` gyro-corrected turns).
- **Phase 3 (ToF & Guidance):** Reduced sensor set decided (D-70: F1, F2, tilted S2, optional L1; G1–G4 and S1 removed). Pure host modules `gnd.c` and `fwd.c` are implemented and unit-tested; `vl.c` (awaiting ST VL53L1X ULD), `map`, `nav`, `evd`, `msn`, and `lat` are not yet started.

---

## 2. Core System Logic & Data Flow

### Firmware Execution Model

1. **Interrupt Layer (`app_isr.c`, `sys_sched.c`, `enc.c`, `imu.c`, `com_uart.c`)**
   - **TIM6 @ 1 kHz (`sys_sched_tick_isr`):** Captures TIM2 1 µs timestamp, snapshots `TIM3->CNT` and `TIM8->CNT` via a lock-free sequence counter (`seqlock` in `enc.c`), and sets pending slot bitflags (`S_FAST` 1 kHz, `S_CTRL` 50 Hz, `S_TLM` 10 Hz, `S_SLOW` 1 Hz).
   - **TIM2 CH2 Input Capture (`imu_drdy_capture_isr`):** Hardware-timestamps LSM6DS3 833 Hz `DRDY` rising edges and initiates a 15-byte SPI1 DMA read (`imu_lsm6ds3.c`); DMA completion raises CS and publishes to an 8-slot SPSC ring.
   - **USART1/USART2 IT (`com_uart.c`):** Pushes received bytes into 256-byte rings and clears double-buffered TX busy flags.

2. **Cooperative Main Loop (`app.c`) with Race-Free `__WFI()` Sleep**
   - **`S_FAST` (1 kHz):** Unwraps 16-bit encoder snapshots to 32-bit (`enc_update`), drains UART RX/TX (`com_uart_poll`), drains raw IMU DMA samples (`imu_drain` → axis mapping, temperature compensation, complementary roll/pitch filter in `att.c`, body-to-world yaw rate projection into the 32-entry yaw-integral ring in `imu_yaw.c`), and polls non-blocking battery ADC conversions (`bat_service`).
   - **`S_CTRL` (50 Hz):**
     - **5-State EKF (`ekf.c`, `x = [x, y, θ, b_g, s]`):** Queries `imu_yaw_integral_at()` at exact encoder timestamps, runs standstill + ZUPT accumulation (with auto-reopen after 5 consecutive rejections), detects differential wheel slip, performs state + Joseph-form covariance prediction, and records a 64-entry correction-free pose history ring (`hist_p = x - corr`) for delayed-measurement back-shifting.
     - **Motion Cascade (`prof.c` → `drv.c` → `whl.c` → `mot.c`):** Trapezoid profile generator → P-position + L/R cross-coupled synchronization → breakaway/Coulomb/viscous/inertia feedforward + anti-windup PI velocity loop + stall/runaway/tracking fault persistence counters → safe PWM output (`mot_apply` zeroes the inactive H-bridge channel before writing the active channel).
   - **`S_TLM` (10 Hz) & `S_SLOW` (1 Hz):** Status LED blink, auto-print, CPU load (`cpu_permille`), TIM6 tick-rate self-check, and `S_CTRL` overrun escalation.

3. **Safety, Faults & Watchdog (`sys_fault.c`, `sys_sched.c`, `stm32f4xx_it.c`)**
   - Faults map to reaction levels `R0`–`R4`. `R4` and all Cortex-M exception handlers (`NMI`, `HardFault`, `MemManage`, `BusFault`, `UsageFault`) invoke `sys_fault_hard_stop()` immediately (`GPIOB->BSRR` EN low + `TIM1->CCR1..4 = 0`) and record crash stack frames to RTC backup registers (`RTC->BKP0R..`).
   - IWDG (~100 ms timeout) is started at the end of `app_init()` and refreshed only when the main loop ran within 5 ms and `S_CTRL` completed within 30 ms.

---

## 3. Suggested System Optimizations

All items below are **recommendations only** and respect the repository's C11 / no-RTOS / no-`malloc` / `float`-only / two-track rules.

---

### A. Embedded Firmware — CPU, Timing & Peripheral Efficiency

#### A1. Exploit Matrix Sparsity & Rank-1 Joseph Updates in `ekf.c`
- **Observation:**
  - `congruence()` (`Challenge_TestHC/Core/Src/ekf.c:26-45`) performs dense $5 \times 5 \times 5$ matrix multiplications (250 MACs + two 100-byte stack buffers `T`, `R`) on every predict step—including when `e->still == true` and $F = I$.
  - `upd1()` (`ekf.c:230`) calls `h_s(e, H, R)` to compute innovation variance $S$ for chi-square gating, and then `kal1()` (`ekf.c:195-198`) recomputes `h_s()` and $P H^T$ from scratch before running another dense 250-MAC `congruence()`.
- **Suggestion:**
  - In `ekf_predict()`, when `e->still` is true ($F = I$), bypass `congruence()` completely and add the diagonal process noise directly (`P[i][i] += Q[i][i]`). When moving, unroll $F P F^T$ using only the 7 non-zero off-diagonal entries of $F$ (`F[0][2..4]`, `F[1][2..4]`, `F[2][3]`).
  - Compute `PHt` and `S` once in `upd1()` and pass them into `kal1()`.
  - Replace the dense $O(n^3)$ `congruence(P, I - K H, R K K^T)` in `kal1()` with a two-step $O(n^2)$ exact Joseph-form update:
    1. $P_1 = P - K (P H^T)^T$ (25 MACs)
    2. $P_{\text{new}} = P_1 - (P_1 H^T) K^T + R K K^T$ (50 MACs + symmetrize)
  - For single-element Jacobians (`ZUPT` with $H = e_3$, `HEADING` with $H = e_2$), read the column of $P$ directly without dot-product loops.
- **Impact:** Cuts EKF prediction/update CPU cost by ~3–5× and eliminates 200 bytes of stack pressure while maintaining exact float32 Joseph-form positive-definiteness—creating comfortable headroom for Phase 3–5 multi-sensor updates (`gnd`, `lat`, `nav`, `vl`).

#### A2. Upgrade `com_uart.c` from Per-Byte IT to DMA + Circular RX
- **Observation:**
  - `Challenge_TestHC/Core/Src/com_uart.c` currently uses `HAL_UART_Receive_IT(..., 1U)` and `HAL_UART_Transmit_IT(...)`, generating up to ~11,500 IRQs/s at 115200 baud (USB) and ~1,920 IRQs/s at 19200 baud (HC-12).
  - `hdma_usart1_rx` and `hdma_usart1_tx` are already configured in CubeMX (`stm32f4xx_it.c:65-66`) but unused.
  - When TX finishes in `com_tx_cplt_isr()`, any data waiting in the second ping-pong buffer sits idle up to 1 ms until the next `S_FAST` poll (`com_uart_poll()`).
- **Suggestion:**
  - Switch RX to circular DMA (`HAL_UART_Receive_DMA`) and advance the ring head in `S_FAST` from `RX_N - __HAL_DMA_GET_COUNTER(hdma_rx)`—zero RX interrupts per byte.
  - Switch TX to `HAL_UART_Transmit_DMA` and optionally chain the waiting double-buffer transfer cleanly on completion.
- **Impact:** Eliminates per-byte UART interrupt jitter (improving H2 timing margin), prevents RX overruns during critical sections, and prepares USART1/USART2 for Phase 4 binary telemetry and 48 KB RAM capture downloads.

#### A3. Replace Modulo Divisions in the 1 kHz Tick ISR (`sys_sched.c`)
- **Observation:** `sys_sched_tick_isr()` (`Challenge_TestHC/Core/Src/sys_sched.c:49-54`) executes 4 modulo (`%`) operations every 1 ms.
- **Suggestion:** Replace `%` with per-slot countdown counters (`uint16_t rem[SLOT_N]`, decremented each tick and reloaded with `SLOT[i].div` at 0).
- **Impact:** Removes all 4 `UDIV` instructions from the highest-frequency ISR, tightening worst-case tick ISR latency.

#### A4. Reduce 833 Hz Trigonometry & Scaling Overhead in `imu.c` and `att.c`
- **Observation:** On every 833 Hz IMU sample in `S_FAST`:
  - `att_update()` (`Challenge_TestHC/Core/Src/att.c:19-20`) computes `atan2f` twice and `sqrtf` twice unconditionally, even when `att_acc_gate` (line 44) rejects the sample.
  - `imu.c:130` recomputes `sinf(phi)`, `cosf(phi)`, and `cosf(th)` immediately after `att.c:37` already computed `sinf(a->phi)`, `cosf(a->phi)`, and `tanf(a->theta)`.
  - `imu.c:115-118` multiplies by `gyro_lsb_radps` and `acc_lsb_mps2` inside the inner $3 \times 3$ axis-mapping loop (18 float muls instead of 6).
- **Suggestion:**
  - Check squared acceleration magnitude `an_sq = ax*ax + ay*ay + az*az` against precomputed squared gate bounds `[(1 ± att_acc_gate) * G_MPS2]²` first; compute `atan2f`/`sqrtf` only when inside the gate (or during `t_init_s < ATT_INIT_S`).
  - Cache/expose `sinf(phi)`, `cosf(phi)`, `cosf(theta)` from `att_t` so `imu.c` reuses them without redundant trig calls.
  - Apply integer signed permutation for `IMU_AXIS_MAP` first, then scale each of the 3 axes once.
- **Impact:** Noticeably reduces `S_FAST` worst-case execution time as IMU runs at 833 Hz alongside future I2C ToF servicing.

#### A5. Small Robustness & Utility Improvements (`console.c`, `sys_fault.c`, `proto.c`)
- **Multi-line console input (`console.c:36-45`, `app.c:342`):** `con_rx_byte()` drops incoming characters while `c->ready` is true, and `app_loop()` drains at most one `con_cmd_t` per loop iteration. Adding a 2–4 line ring buffer or draining `con_poll()` in a bounded `while` loop prevents silent command drops when host calibration scripts send consecutive lines.
- **Fault clearing (`sys_fault.c:74-81`):** If an un-clearable fault (`F_CLOCK` or `F_PARAM`) is latched after a clearable fault, `sys_fault_clear()` clears the first fault from `s_latched` without updating `s_first`. Scanning `s_latched` to update `s_first` when non-zero keeps fault reporting consistent.
- **CRC-16 speed (`proto.c:4-14`):** Replacing the bit-by-bit loop with a 16-entry nibble lookup table (32 bytes of flash) speeds up CRC-16/CCITT by ~4× ahead of Phase 4 binary framing and bulk capture downloads.

---

### B. Estimation, Sensing & Control Accuracy

#### B1. Prioritize Physical & Prior Bottlenecks (Highest Competition ROI)
- **Evidence:** The MATLAB waypoint lever study (`Matlab/ekf_design/README.md:263-293`) demonstrates that filter gain tuning cannot meet the ≤ 10–15 cm waypoint target with default initial uncertainties:
  - **Caster mechanics:** Caster swivel drag/sticking caused H6 UMBmark failures (190–494 mm vs. 10 mm target). Servicing, lubricating, or upgrading the casters (and pre-aligning them before runs) delivers the largest immediate real-world odometry improvement.
  - **Start heading alignment jig ($\sigma_{\theta_0}: 0.5^\circ \to 0.1^\circ$):** Cuts worst-case simulated cross-track error at WP1 from ~121 mm to ~26 mm.
  - **Outdoor surface-specific odometry scale ($\sigma_s: 0.5\% \to 0.1\%$):** Re-measuring `odo_m_per_cnt` on the actual outdoor stone tiles (not indoor lecture tiles) cuts along-track error at WP1 from ~76 mm to ~21 mm. Combined with the start jig, simulated worst-case waypoint error drops from **12.2 cm → 3.0 cm**.

#### B2. Upgrade IMU Yaw Integration, Thermal Bias Tracking & Saturation Handling
- **Trapezoidal yaw integration (`imu_yaw.c:16`):** Replace backward Euler (`rate_radps * dt`) with trapezoidal integration (`0.5f * (p->rate + rate_radps) * dt`) to eliminate first-order phase lag during turn accelerations.
- **Online x/y gyro bias tracking (`att.c:22-34`):** `att.c` currently estimates `bw[0]` and `bw[1]` only during the first 0.5 s after boot. Over a 15-minute outdoor run, thermal bias drift shifts roll/pitch by $\Delta b \cdot \tau_{\text{att}}$ and leaks `w[1]` into the projected world yaw rate (`imu.c:130`). Slowly updating `bw[0]`/`bw[1]` during EKF standstill windows (e.g., during the three 10 s waypoint holds) prevents long-run attitude drift.
- **Wire `F_IMU_SAT` into EKF (`REQ-IMU-07`):** Add a `bool sat` flag to `ekf_odo_t` so gyro saturation inflates `Q[2][2]` by 100× as specified (currently an open item in `docs/PHASE2_IMU_EKF.md`).
- **Multi-turn heading scale-factor covariance (Finding F-9):** If the 150 m course accumulates $> 2\pi$ net rotation in one direction without heading resets, the current `ekf_sf_corr = 2π` model becomes overconfident. Tightly calibrating `imu_gz_scale` via the H7 10-turn spin—and/or adopting `mx_sf_mode = 2` (tracking net angle $\Theta$)—prevents NEES overconfidence.

#### B3. Drivetrain & Turning Control Refinements
- **Sampled M/T speed estimator (`WHL_SPEED_EST = EST_MT`):** At 50 Hz, 1 encoder count = 3.18 mm/s quantization step (`drv.c:285`). Because `enc_snapshot_isr()` already records 1 µs `TIM2` timestamps at 1 kHz, computing wheel speed via timestamped count edges (M/T method) will yield much cleaner low-speed estimates during `SETTLE`, crawl, and waypoint final approach.
- **Enable battery voltage feedforward normalization (`whl_ff_norm_en`):** Once `bat_ratio` (`P300`) is verified against a multimeter, enabling `P128=1` compensates for the ~15% voltage drop across a 3S LiPo discharge cycle, keeping feedforward accurate throughout the run.
- **Smooth breakaway Coulomb transition (`whl.c:62-66`):** `whl.c` jumps discontinuously from `ff = 0` to `±ksb` (0.11–0.135 duty) at `whl_v_zero` (2 mm/s). A narrow linear blend region around zero reduces low-speed chattering and caster perturbation.
- **Direct closed-loop gyro turning (Phase 3 `nav`):** `W` currently executes an encoder turn, stops, and performs up to 2 stop-start correction turns (`app.c:233-246`). Closing `nav_turn` directly on `turn_acc_rad` / EKF $\hat{\theta}$ inside `S_CTRL` eliminates repeated breakaway starts, shortens turn time, and reduces caster scrub.

#### B4. Harden ToF Sensor Classifiers (`fwd.c`, `gnd.c`)
- **`fwd.c` staleness timer bug (`Challenge_TestHC/Core/Src/fwd.c:30-35`):** `s->t_last_us` is updated *before* `if (jump) { return; }`, so continuous crosstalk spikes keep resetting the timeout and prevent `fwd_stale()` from triggering. Separate `t_rx_us` (used for jump $\Delta t$) from `t_valid_us` (used for staleness).
- **`fwd.c` startup crosstalk vulnerability (`fwd.c:45`):** When `s->n == 2`, `med()` returns `fminf(r0, r1)`, allowing a single spurious reading from `INFINITY` on the second sample after boot to trip `reflex`. Requiring `n == 3` (or gating single-sample transitions from `INFINITY`) prevents false startup stops.
- **`fwd.c` smooth speed governor (`fwd.c:79`):** Replace the hard step from `INFINITY` to `fwd_v_slow` at `fwd_slow_range` (1.0 m) with a linear ramp between `d_stop` and `fwd_slow_range` to prevent abrupt deceleration jerks and pitch transients.
- **`gnd.c` variance check without `sqrtf` (`Challenge_TestHC/Core/Src/gnd.c:16-23`):** Compare squared spread `v >= (float)n * sqf(q->gnd_sd_grass)` to avoid `sqrtf()` on every S2 measurement.
- **Hybrid corridor interlock for D-70 / OQ-12 (`gnd.c:98-100`):** With G1–G4 removed, `gnd_interlock()` currently returns `false` and leaves the left side blind. Before running `AUTO`, consider pairing S2 right-side grass/edge detection with a virtual EKF corridor interlock (checking cross-track margin minus $2\sigma_\perp$ against the `map` corridor boundary on the blind left side).

---

### C. Communications & Ground Station (`ground_station/`)

1. **Reconcile Wire Protocol IDs Before Phase 4**
   - `ground_station/protocol.yaml:51-54` diverges from `FIRMWARE_SPEC.md §8.4` (`0x03` is `DRIVE` in `protocol.yaml` vs. `TELEM_SLOW` in the spec; `veh2pc` IDs start at `0x80` in `protocol.yaml`). Unify both into a single authoritative `protocol.yaml` and wire `gen_protocol.py` into the build/verification flow so firmware and ground station cannot drift.
2. **Coalesce `HEARTBEAT` with `DRIVE` on the Half-Duplex HC-12 Link**
   - In `ground_station/groundstation/link/command_manager.py:127-128`, `tick()` transmits all periodic messages every `TELEM_FAST` slot. Skipping `HEARTBEAT` whenever `DRIVE` is sent in the same slot (since `DRIVE` proves operator liveness) cuts uplink airtime nearly in half and reduces collision/FIFO-overrun risk at 19200 baud FU3.
3. **Reduce Ground-Station I/O & Decoding Overhead**
   - **`recorder.py:30`:** Currently flushes to the OS on every serial `read()` chunk. Flushing per decoded frame or on a 50–100 ms interval cuts syscall overhead during sustained logging and capture downloads while retaining crash resilience.
   - **`link.py:38-42`:** Replace the 20 ms `time.sleep()` busy-poll in `wait_hello()` with a `threading.Event` signaled when `HELLO` arrives.
   - **`codec.py:53`:** Replace the Python byte-by-byte loop in `Codec.feed()` with C-level `bytes.split(b"\x00")` for significantly faster replay of multi-megabyte session logs.

---

### D. Build, Verification & Repository Hygiene

1. **Generate Parameter ID Enums via `PAR_TABLE(X)`**
   - `Challenge_TestHC/Core/Src/app_cmd.c:23-32` hardcodes parameter IDs (`#define ID_V_NOM 161U`, etc.). Generating a `PAR_ID_<name>` enum from `PAR_TABLE(X)` in `par.h` eliminates manual ID duplication and prevents silent mismatches as more parameter groups are added.
2. **Avoid Editing `app_config.h` for Routine Field Tests**
   - Switching `FEAT_IMU` in `app_config.h` to run encoder-only UMBmark tests leaves tracked files dirty and risks accidental commits. Either pass `-DFEAT_IMU=0` via a dedicated CubeIDE build configuration (`app_config.h` already guards with `#ifndef FEAT_IMU`) or add a runtime console command (e.g., `WE<deg>`) for encoder-only turns.
3. **Unify Cross-Language Verification & Add C-vs-MATLAB Replay**
   - Add a single root-level verification script or `Makefile` target that executes:
     1. Host unit tests (`make -C test`)
     2. Phase-1 tool selftests (`python3 tools/{straight_scale,umbmark,ffit}.py --selftest`)
     3. Ground-station tests (`pytest` in `ground_station/`)
     4. Headless ARM cross-compile checks (`arm-none-eabi-gcc` at `-Og` and `-O2`, `FEAT_IMU=0/1`, `APP_CALIB=0/1`)
   - Implement the planned `test/replay.c` harness (`Matlab/ekf_design/README.md:346`) to continuously verify `ekf.c` against the MATLAB reference on identical sensor traces.
   - **Simulink base-step optimization (`Matlab/ekf_design/simulink/build_ekf_model.m`):** Switching from 960 Hz to 833 Hz forced `lcm(1000, 833)` to set the base step to `1/1,666,000 s` (35× slower). Using an asynchronous/triggered subsystem or simulating the IMU at `833.33 Hz` (`2500/3 Hz`) with a 2.5 kHz / 5 kHz base clock eliminates the 35× slowdown.
4. **Tighten `.gitignore` for Build Outputs & Intermediates**
   - Exclude compiled outputs (`Challenge_TestHC/Debug/`, `Activity 2.6/STM-Project/Debug/`, `test/t_all`, `test/t_all.dSYM/`), `tools/__pycache__/`, LaTeX build artifacts (`.aux`, `.fls`, `.fdb_latexmk`, `.synctex.gz`, `.log`), and regenerated MATLAB figures (`Matlab/ekf_design/results/*.png`) to keep `git status` and diffs clean.
