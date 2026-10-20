# MR2006B Vehicle Firmware — Implementation Specification

> **Status:** Draft for review
> **Version:** 1.0
> **Last Updated:** 2026-10-14 (course week 6 of 16)
> **Authors:** Dani (system design, goals), Claude (specification)
> **Scope:** all code that runs on the NUCLEO-F446RE of the MR2006B vehicle, and the host-side tools the firmware depends on (course generator, unit tests)
> **Companion documents:** `MR2006B_Vehicle_Technical_Reference.tex/.pdf` (physics, models, rationale — *why*), `CALIB_README.md` (calibration firmware), `BENCH_README.md` (bench firmware), `MR2006B_CubeMX_Konfiguration.xlsx` (current `.ioc` state), `CONCEPT.md` Rev. E + `CONCEPT_REV_F_CHANGES.md`, `TOOLCHAIN.md`

---

## 0. How to read this document

This document is **normative**. It says *what* the firmware must do, with which interfaces, numbers and tolerances. The technical reference says *why*. Where the two disagree, this document wins for code and the technical reference must be corrected.

**Keywords.** MUST / MUST NOT = requirement, a deviation is a defect. SHOULD = recommended, a deviation needs a one-line reason in the code. MAY = optional.

**Requirement IDs.** Every testable requirement has an ID of the form `REQ-<MODULE>-<nn>` (e.g. `REQ-WHL-04`). Tests in §14 reference these IDs. Commit messages that implement a requirement SHOULD name it.

**Value tags** (same convention as `CONCEPT.md`):

| Tag | Meaning for the implementer |
|---|---|
| `[MEASURE]` | The default is a placeholder. Implement it as a parameter (§10) so the measured value can be entered without a code change. |
| `[VERIFY]` | The value comes from a datasheet or memory and has not been checked. Check it against the cited document before relying on it. |
| `[DECIDE]` | A team decision is pending. Implement the default; the alternative must remain possible without restructuring. |
| `[PHASE n]` | Not needed before implementation phase *n* (§15). |

**Units.** Internally all quantities are SI with `float` (32 bit): metre, second, radian, volt, duty as a fraction in [−1, 1]. Console commands keep the legacy units of the teammate's code (mm, mm/s, %) and convert at the boundary (§6.4, §9).

---

## 1. Overview

### 1.1 Problem statement

The vehicle must drive a closed circuit of roughly 150 m on the paved paths of the Jardín de las Carreras autonomously. It must stop for 10 s at each of three waypoints, avoid obstacles, never touch grass and return to the start. The STM32F446RE is the only computer allowed, and the run must finish within 15 min. Speed is not graded. Completion, waypoint accuracy, obstacle handling and STM32-only operation are.

Today the firmware consists of the teammate's single-file `main.c`: two BTS7960 drivers, two encoders, a trapezoid-profile → P-position → feedforward + PI speed cascade at 50 Hz, L/R synchronisation, a `D<mm>` command, and a text console on USART2 and the HC-12 (USART1). There is no IMU, no ground or obstacle sensing, no state estimation, no path following and no mission logic. The code is one file with module-level globals, which will not scale to the remaining eleven modules.

This specification defines the complete target firmware. It also defines a migration path from the existing code that keeps the working drivetrain behaviour intact at every step (§15, §16).

### 1.2 Goals

| ID | Goal | Measurable target |
|---|---|---|
| G1 | Complete the circuit autonomously | ≥ 4 of 5 practice runs complete, no grass contact, ≤ 15 min |
| G2 | Waypoint accuracy | every waypoint within 20 cm (grade ≥ 1.5 of 2); stretch: 10 cm |
| G3 | Obstacle handling | a static obstacle on the path is passed by lateral rerouting (2 points); a temporary obstacle is waited out |
| G4 | STM32 only | no co-processor executes navigation, estimation or control code |
| G5 | Graded milestones | week 8 sensors logged with timestamps; week 10 telemetry for 15 min at 10 Hz; week 14 manual remote control with failsafe; week 15 autonomous circuit; week 16 complete system |
| G6 | Maintainability | each module is unit-testable on the host without hardware; CubeMX regeneration never deletes or silently changes user code |

### 1.3 Non-goals (out of scope)

- Ground-station GUI and analysis tools. Only the wire protocol (§8) is specified here, as the contract between both sides. The ground station has its own spec.
- SLAM, absolute positioning (GNSS/RTK), magnetometer heading, optical flow, camera processing. All were rejected, see technical reference §2.4.
- An RTOS, C++, dynamic memory allocation.
- Persisting parameters to flash (v1). Parameters live in RAM and are dumped as `#define` blocks (D-7 in §18.2).
- The calibration firmware itself. It exists and is specified in `CALIB_README.md`. This spec only defines how it coexists with the vehicle firmware (§7.27).
- Optimising for speed. Every trade-off between time and accuracy goes to accuracy.

### 1.4 Success criteria

| Criterion | Measurement | Target |
|---|---|---|
| Control-loop timing | period of the 50 Hz control slot, measured with the DWT cycle counter over 15 min | mean 20.000 ms ± 0.01 %, max jitter < 100 µs |
| CPU load | sum of slot execution times / wall time (§5.6) | < 50 % worst case |
| Stop accuracy, encoder frame | `D300`, `D1000`, `D5000`, 10 runs each | end within 2 mm of target, never reversing |
| Straight-line odometry after UMBmark | return error on a 2 m square, both directions | < 10 mm |
| EKF consistency | normalised estimation error squared over the logged runs of the rectangle test | within the 95 % χ² band ≥ 90 % of the time |
| Path following | 10 × 10 m rectangle | closes within 30 cm (week 13) |
| Waypoint accuracy | full circuit | ≤ 40 cm (week 15), ≤ 20 cm (week 16) |
| Telemetry | 15-min radio log at 10 Hz | < 1 % lost frames at ≤ 30 m range |
| Failsafe | manual mode, radio switched off | vehicle stops within 0.6 s |
| Robustness | no unexplained reset or hang during 5 full circuits | 0 occurrences |

---

## 2. Users and stakeholders

### 2.1 Personas

| Persona | Description | Primary need |
|---|---|---|
| Test operator | Team member on the course with laptop, radio and the vehicle | Start runs, drive manually, stop instantly, see what the vehicle thinks |
| Firmware developer | Team member implementing and tuning modules | Clear module boundaries, testable logic, parameters changeable at run time, high-rate data capture |
| Examiner | Professor during the graded demonstrations | Visible control loop with user reference, telemetry, manual and automatic modes, safe behaviour |

### 2.2 User stories

- As a test operator, I want to start an autonomous run with one button press (B1) or one command, so that the run is reproducible.
- As a test operator, I want any `STOP` command, from either port and in any mode, to stop the vehicle, so that I can always intervene.
- As a test operator, I want the vehicle to stop by itself when the radio link is lost in manual mode, so that a dropped link cannot cause a runaway.
- As a test operator, I want live pose, mode, waypoint index and fault codes at 10 Hz, so that I can tell a sensor problem from a control problem on the spot.
- As a firmware developer, I want to change any gain at run time and print all parameters as a `#define` block, so that tuning does not need a rebuild per iteration.
- As a firmware developer, I want to record 100–200 Hz data in RAM and download it afterwards, so that I can tune loops that the 10 Hz radio cannot show.
- As a firmware developer, I want every logic module to compile and run on my Mac, so that I can test the EKF, the path follower and the state machine before the hardware exists.
- As an examiner, I want to set a speed or distance reference and see the loop follow it, so that requirement R3 of the brief is demonstrated.

### 2.3 Operating modes (instead of roles)

The firmware has no user accounts. Authority comes from the operating mode:

| Mode | Who commands motion | Allowed commands | Leaves the mode by |
|---|---|---|---|
| `IDLE` | nobody, motors disabled | everything except `DRIVE` | `MODE`, console command, B1 |
| `BENCH` | console `D`, `W`, open-loop duty | all console commands | end of move, `s`, `STOP`, fault |
| `MANUAL` | `DRIVE(v, ω)` from radio or console `M` | `DRIVE`, `STOP`, `MODE`, parameter read | `MODE IDLE`, `STOP`, heartbeat loss → `IDLE` |
| `AUTO` | mission state machine | `STOP`, `MODE IDLE`, parameter read, telemetry | finish, `STOP`, fault |
| `FAULT` | nobody, motors disabled | `STATUS`, `CLEAR` (= `r`), parameter read | `CLEAR` after the cause has gone |

`STOP` is accepted in every mode from every port (REQ-COM-01).

---

## 3. Architecture

### 3.1 Layers

```
 ┌─────────────────────────────────────────────────────────────────────────┐
 │  APPLICATION     app  (init sequence, main loop, slot dispatch)          │
 ├─────────────────────────────────────────────────────────────────────────┤
 │  MISSION         msn  (mode + state machine)    evd (1-D evasion)        │
 ├─────────────────────────────────────────────────────────────────────────┤
 │  GUIDANCE        nav  (path follower, turn controller, approach)         │
 │                  map  (course data: segments, corridors, landmarks)      │
 ├─────────────────────────────────────────────────────────────────────────┤
 │  ESTIMATION      ekf  (5-state EKF)   att (roll/pitch)                   │
 │                  gnd  (ground classes, edge events)                      │
 │                  lat  (face fit, post notch)   fwd (reflex, free space)  │
 ├─────────────────────────────────────────────────────────────────────────┤
 │  MOTION          drv  (v,ω / move / turn → wheel references, sync)       │
 │                  whl  (per-wheel P-position + FF + PI speed)  prof       │
 ├─────────────────────────────────────────────────────────────────────────┤
 │  DEVICES         mot  enc  imu  vl  bat  com_uart                        │
 ├─────────────────────────────────────────────────────────────────────────┤
 │  SYSTEM          sys_time  sys_sched  sys_fault  par  tlm  con  proto    │
 ├─────────────────────────────────────────────────────────────────────────┤
 │  VENDOR          CubeMX HAL, CMSIS, VL53L1X ULD, (IMU vendor regs)       │
 └─────────────────────────────────────────────────────────────────────────┘
```

**Dependency rule (REQ-ARC-01).** A module may call only modules in its own layer or below. SYSTEM modules may be called from any layer. Upward communication happens through return values, status structs that the upper layer reads, or event flags. Callbacks pointing upwards are not allowed. The single exception is `sys_fault_raise()`, which every layer may call.

**Pure vs. hardware modules (REQ-ARC-02).** These modules MUST NOT include `main.h`, `stm32f4xx_hal*.h` or any CMSIS header, so they compile on the host: `prof`, `whl`, `drv`, `att`, `gnd`, `lat`, `fwd`, `ekf`, `map`, `nav`, `evd`, `msn`, `proto`, `par`. Their inputs are plain structs; their outputs are plain structs. Only the device and system modules touch hardware.

### 3.2 Data flow per control period

```
 1 kHz tick ISR ──► enc snapshot (cntL, cntR, t_us) ──┐
 IMU DRDY IC ISR ─► SPI DMA ─► imu ring (t_us, ω, a, T)│
 VL I²C ISRs ────► vl results (range, σ, S, t_us)      │
                                                       ▼
 200 Hz  ekf_predict( Δs_L, Δs_R, ∫ψ̇ dt )  ◄── imu_yaw_integral_at (uses att φ, ϑ)
         ekf_update( ZUPT | edge | face | post )  ◄── gnd, lat
                                                       │ pose x̂, P
 50 Hz   fwd_step (reflex) ─► msn_step ─► nav_follow/turn/final ─► drv_step ─► whl_step ─► mot_apply
                                 ▲            ▲
                               evd          map
 10 Hz   tlm_fast ─► proto ─► com_uart (USART1, USART2)
```

### 3.3 Source layout

All user code lives in `Core/Inc` and `Core/Src`. CubeIDE always compiles these folders, and they survive regeneration. Adding source folders to the `.cproject` is not allowed (REQ-ARC-03), because CubeMX 2.x rewrites `.cproject` and would drop them silently.

```
Challenge_TestHC/
├── Challenge_TestHC.ioc                  CubeMX, owned by CubeMX (§4)
├── Core/
│   ├── Inc/
│   │   ├── main.h                        CubeMX
│   │   ├── app.h        app_config.h     build switches, rates, feature flags
│   │   ├── units.h                       constants, conversions, wrap_pi, clampf
│   │   ├── sys_time.h   sys_sched.h   sys_fault.h   par.h   par_table.h
│   │   ├── mot.h  enc.h  imu.h  imu_lsm6dsv.h  vl.h  bat.h  com_uart.h
│   │   ├── prof.h  whl.h  drv.h
│   │   ├── att.h  gnd.h  lat.h  fwd.h  ekf.h
│   │   ├── map.h  nav.h  evd.h  msn.h
│   │   ├── proto.h  con.h  tlm.h
│   │   ├── build_info.h                  generated by the pre-build step (git hash)
│   │   └── calib.h  calib_config.h       existing calibration firmware
│   └── Src/
│       ├── main.c                        CubeMX + 4 USER CODE hooks only (§16.2)
│       ├── stm32f4xx_it.c  stm32f4xx_hal_msp.c   CubeMX (+ fault hooks in USER CODE)
│       ├── app.c  sys_*.c  par.c  …      one .c per header above
│       ├── course_jardin.c               generated by tools/course_gen.py
│       ├── course_rect10.c               test course, 10 × 10 m rectangle
│       ├── calib.c                       existing
│       └── vl53l1x_uld/                  ST Ultra Lite Driver: VL53L1X_api.c,
│                                         VL53L1X_calibration.c, vl53l1_platform.c
├── test/                                 host unit tests (not in the CubeIDE build)
│   ├── Makefile   test_main.c   t_*.c    plant models: sim_plant.c
└── tools/
    ├── course_gen.py   course_jardin.yaml   course_rect10.yaml
    ├── calib_host.py                     existing
    └── gen_build_info.sh                 pre-build step
```

The ULD folder sits inside `Core/Src`, so it is compiled without changing `.cproject`. **[VERIFY]** that CubeIDE picks up the sub-folder automatically. If it does not, flatten the files into `Core/Src` with prefix `vl53l1x_`.

### 3.4 Build variants

| Switch (`app_config.h`) | Values | Effect |
|---|---|---|
| `APP_CALIB` | 0 / 1 | 1 = calibration firmware (`Calib_Loop`, existing). 0 = vehicle firmware. |
| `FEAT_IMU` | 0 / 1 | 0 = no IMU; EKF uses encoder heading (§7.17.7). For work before the IMU arrives. |
| `FEAT_VL` | 0 / 1 | 0 = no VL53L1X; ground interlock and reflex disabled, AUTO refuses to start unless `msn_allow_nosensors` is set. |
| `FEAT_BIN_PROTO` | 0 / 1 | 0 = text console only on both ports (current behaviour). |
| `FEAT_CAPTURE` | 0 / 1 | RAM capture buffer (48 KB). |
| `WHL_SPEED_EST` | `EST_DIFF` / `EST_MT` | Count difference (current) or sampled M/T method (§7.6.4). |
| `COURSE` | `COURSE_RECT10` / `COURSE_JARDIN` | Which generated course table is linked. |

Build configurations: **Debug** `-Og -g3`, **Release** `-O2 -g`. The competition build is Release. It MUST be built and run on the vehicle for the first time no later than week 13 (REQ-ARC-04), so that missing `volatile` qualifiers surface early.

### 3.5 Technology stack

| Layer | Technology | Rationale |
|---|---|---|
| MCU | STM32F446RET6, Cortex-M4F, 180 MHz, single-precision FPU | given by the course |
| HAL | STM32Cube HAL (F4), generated by STM32CubeMX standalone, edited in STM32CubeIDE 2.2 on macOS | team toolchain; LL only where noted |
| Language | C11 (`-std=gnu11`) | ISR linkage and hidden allocation pitfalls of C++ avoided |
| Compiler | `arm-none-eabi-gcc` from CubeIDE; host tests with Apple clang / gcc | — |
| Libraries | newlib-nano (no float printf), CMSIS, VL53L1X ULD | ULD < 10 KB vs. ~90 KB full API |
| Host tools | Python 3 + numpy, pyserial, matplotlib, PyYAML | already used by `calib_host.py` |
| Version control | Git in Terminal, IDE for editing only | regeneration diffs must be reviewable |

---

## 4. Platform configuration (CubeMX target state)

The `.ioc` belongs to CubeMX. Every value below is entered in CubeMX, not patched in generated code. After every regeneration: `git diff Core/Src/main.c .cproject Core/Inc/stm32f4xx_hal_conf.h` (REQ-PLT-01). The start-up self-check (§5.8) measures the tick rate and the clock tree at run time and reports any mismatch.

### 4.1 Clock tree (unchanged from the current `.ioc`)

| Setting | Value |
|---|---|
| HSE | **BYPASS**, 8 MHz from the ST-Link MCO; PH1 free |
| PLL | source HSE, M = **4**, N = 180, P = 2 → SYSCLK 180 MHz; VCO input 2 MHz |
| AHB / APB1 / APB2 | /1 / /4 / /2 → HCLK 180, PCLK1 45 (timer clock 90), PCLK2 90 (timer clock 180) MHz |
| Voltage scale / over-drive / flash latency | VOS1 / on / 5 wait states |
| LSE / LSI | off / on (IWDG) |
| HSE start-up timeout | **1000 ms** in `stm32f4xx_hal_conf.h` (`HSE_STARTUP_TIMEOUT`) — open item O-10 |
| CSS | **on** `[PHASE 2]`; `HAL_RCC_CSSCallback` implemented in `sys_fault.c` (§7.3) |

### 4.2 Timers

| Timer | Function | Clock | PSC | ARR | Result | Changes vs. today |
|---|---|---|---|---|---|---|
| TIM1 | PWM CH1–CH4 → BTS7960 | 180 MHz | 0 | **8999** | 20 kHz, 9000 steps | ARR set in CubeMX instead of overwritten by `PWM_Setup()` |
| TIM2 | 32-bit time base + CH2 input capture (IMU DRDY) | 90 MHz | **89** | 0xFFFFFFFF | 1 MHz, wraps every 71.6 min | PSC 0 → 89; started at boot |
| TIM3 | encoder L (PA6/PA7) | 90 MHz | 0 | 65535 | encoder mode TI1+TI2 (×4), IC filter 15, CKD /1 | — |
| TIM8 | encoder R (PC6/PC7) | 180 MHz | 0 | 65535 | as TIM3, CKD /2 | — |
| TIM6 | scheduler tick | 90 MHz | **89** | **999** | **1 kHz** | was PSC 8999 / ARR 119 (83 Hz, wrong) or 199 (50 Hz) |
| SysTick | HAL time base | HCLK | — | — | 1 kHz | stays SysTick (TIM6 must not be the HAL time base) |
| DWT | cycle counter | 180 MHz | — | — | slot timing, calibration firmware | enabled in `sys_time_init()` |

`APP_CALIB = 1` writes TIM6 ARR = 19999 at start (PSC 89 → 50 Hz), so the calibration firmware keeps the tick rate it was verified with (REQ-PLT-02, §7.27).

TIM1 output configuration: PWM mode 1, polarity high, idle state reset, no dead time (the BTS7960 handles it internally), break disabled. With MOE cleared, all four outputs are low.

### 4.3 Pin allocation (complete)

Pins marked **new** are not in the current `.ioc`. XSHUT assignments follow `MR2006B_Pinout.xlsx` of 7 Oct. If that file has been changed since, reconcile it with this table before wiring, and update both.

| Pin | Morpho | Function | Peripheral / mode | Init | Pull | Target |
|---|---|---|---|---|---|---|
| PA8 | CN10-23 | PWM L forward | TIM1_CH1 | — | none | BTS7960 L RPWM |
| PA9 | CN10-21 | PWM L reverse | TIM1_CH2 | — | none | BTS7960 L LPWM |
| PA10 | CN10-33 | PWM R forward | TIM1_CH3 | — | none | BTS7960 R RPWM |
| PA11 | CN10-14 | PWM R reverse | TIM1_CH4 | — | none | BTS7960 R LPWM |
| PB12 | CN10-16 | `EN_L` | GPIO out PP | **low** | pull-down (+ ext. 10 kΩ) | BTS7960 L R_EN+L_EN |
| PB13 | CN10-30 | `EN_R` | GPIO out PP | **low** | pull-down (+ ext. 10 kΩ) | BTS7960 R R_EN+L_EN |
| PA6 / PA7 | CN10-13 / -15 | encoder L A/B | TIM3_CH1/CH2 | — | none (3 kΩ on encoder) | motor L |
| PC6 / PC7 | CN10-4 / -19 | encoder R A/B | TIM8_CH1/CH2 | — | none | motor R |
| PA2 / PA3 | internal | console | USART2 TX/RX | — | none | ST-Link VCP |
| PB6 | CN10-17 | radio TX | USART1_TX | — | none | HC-12 RXD |
| PB7 | CN7-21 | radio RX | USART1_RX | — | **pull-up** (O-15) | HC-12 TXD |
| PC9 | CN10-1 | `HC12_SET` | GPIO out PP | **high** | none | HC-12 SET |
| PB3 / PB4 / PB5 | CN10-31 / -27 / -29 | IMU SPI | SPI1 SCK/MISO/MOSI | — | none | IMU `[PHASE 2]` |
| PA4 | CN7-32 | `IMU_CS` | GPIO out PP, very high speed | **high** | none | IMU CS `[PHASE 2]` |
| PA1 | CN7-30 | IMU DRDY | TIM2_CH2 input capture, rising | — | none | IMU INT1 `[PHASE 2]` |
| PB8 / PB9 | CN10-3 / -5 | ground bus | I2C1 SCL/SDA, 400 kHz | — | ext. 2.2–4.7 kΩ | G1–G4 `[PHASE 3]` |
| PB10 / PC12 | CN10-25 / CN7-3 | horizontal bus | I2C2 SCL/SDA, 400 kHz | — | ext. 2.2–4.7 kΩ | F1, F2, S1, S2 `[PHASE 3]` — PC12 as I2C2_SDA **[VERIFY]** in CubeMX; fallback: all eight on I2C1 |
| PC0 | CN7-38 | `XSHUT_G1` | GPIO out PP | low | none | ground left outer |
| PC1 | CN7-36 | `XSHUT_G2` | GPIO out PP | low | none | ground left inner |
| PC2 | CN7-35 | `XSHUT_G3` | GPIO out PP | low | none | ground right inner |
| PC3 | CN7-37 | `XSHUT_G4` | GPIO out PP | low | none | ground right outer |
| PB14 | CN10-28 | `XSHUT_F1` | GPIO out PP | low | none | forward left |
| PB15 | CN10-26 | `XSHUT_F2` | GPIO out PP | low | none | forward right |
| PB0 | CN7-34 | `XSHUT_S1` | GPIO out PP | low | none | lateral left |
| PB1 | CN10-24 | `XSHUT_S2` | GPIO out PP | low | none | lateral right |
| PA15 | CN7-17 | `VL_S1_INT` **new** | GPIO EXTI15, falling | — | pull-up | S1 GPIO1 (data ready) `[PHASE 5]` |
| PA12 | CN10-12 | `VL_S2_INT` **new** | GPIO EXTI12, falling | — | pull-up | S2 GPIO1 (data ready) `[PHASE 5]` |
| PA0 | CN7-28 | battery voltage | ADC1_IN0, 480 cycles | — | none | divider 47 k / 10 k, 100 nF at the node |
| PC4 / PC5 | CN10-34 / -6 | reserve analog | ADC1_IN14/IN15 | — | — | unused |
| PC8 | CN10-2 | `PROBE` | GPIO out PP, very high speed | low | none | oscilloscope: slot timing |
| PA5 | CN10-11 | `LED` | GPIO out PP | low | none | LD2 status |
| PC13 | CN7-23 | `START_BTN` | GPIO EXTI13, falling | — | none (board pull-up) | B1 |
| PA13 / PA14 | CN7-13 / -15 | SWD | SYS Serial Wire | — | — | ST-Link |
| PH0 | CN7-29 | HSE in | RCC_OSC_IN (BYPASS) | — | — | MCO |

**EXTI rule (REQ-PLT-03).** EXTI lines are shared by pin *number*. In use: 12, 13, 15 (all in `EXTI15_10_IRQHandler`). Any new interrupt input must use a free line number.

**Free after full allocation:** PB2, PC10, PC11 (USART3 reserve), PC14, PC15, PD2, PH1.

### 4.4 Peripherals

| Peripheral | Configuration | Phase |
|---|---|---|
| USART2 | 115200 8N1, TX+RX, oversampling 16, NVIC on. RX by interrupt (1 byte re-armed), TX by interrupt from a double buffer | 1 |
| USART1 | 9600 8N1 (19200 after the range test, `[DECIDE]` D-11), RX **DMA circular** 256 B + IDLE-line interrupt, TX **DMA normal** from a double buffer, NVIC on | 1 (IT) → 4 (DMA) |
| SPI1 | full-duplex master, 8 bit, MSB first, CPOL 1 / CPHA 2nd edge (mode 3), software NSS, prescaler **/16** → 5.625 MHz (LSM6DSV max 10 MHz **[VERIFY]**), RX+TX DMA | 2 |
| I2C1, I2C2 | fast mode 400 kHz, duty 2, 7-bit addressing, **interrupt mode, no DMA**, NVIC EV+ER on | 3 |
| ADC1 | 12 bit, right aligned, single conversion, software start, channel 0, 480 cycles; no DMA | 1 |
| IWDG | LSI, prescaler 32, reload 100 → 100 ms nominal (68–188 ms over the LSI tolerance); started last in `app_init()` | 2 |
| DBGMCU | `__HAL_DBGMCU_FREEZE_IWDG()` in Debug builds | 2 |

### 4.5 DMA allocation

**[VERIFY]** each stream/channel against RM0390 Table 29 (DMA2 request mapping) before generating.

| Request | Controller / stream / channel | Mode | Priority |
|---|---|---|---|
| SPI1_RX | DMA2 S0 ch3 | normal | very high |
| SPI1_TX | DMA2 S3 ch3 | normal | high |
| USART1_RX | DMA2 S2 ch4 | circular | medium |
| USART1_TX | DMA2 S7 ch4 | normal | low |
| (reserve) ADC1 | DMA2 S4 ch0 | — | not used, ADC is polled |

DMA1 stays entirely free (I²C reserve).

### 4.6 NVIC

Priority group 4 (4 bits pre-emption, 0 sub-priority). Lower number = higher priority.

| Interrupt | Pre-emption | Why |
|---|---|---|
| SysTick | 0 | `HAL_GetTick()` must advance while any ISR waits on a HAL timeout during init |
| TIM6 (tick) | 1 | encoder snapshot timing |
| TIM2 (IMU DRDY capture) | 2 | hardware latches the timestamp; the ISR only starts the SPI DMA |
| DMA2 S0 / S3 (SPI1) | 2 | completes the IMU read before the next sample |
| USART1, USART2, DMA2 S2 / S7 | 3 | byte rates ≤ 11.5 kHz |
| I2C1 / I2C2 EV + ER | 4 | VL53L1X transactions |
| EXTI15_10 | 4 | start button, lateral data-ready (timestamp read in the ISR, µs latency acceptable) |

### 4.7 Project settings

| Setting | Value |
|---|---|
| Generate peripheral initialisation as .c/.h pairs | off (as today) |
| Keep user code when regenerating | on |
| Minimum stack size | **0x1000** (vsnprintf + EKF locals) |
| Minimum heap size | 0x400 (newlib internal only; user code never calls `malloc`) |
| Use float with printf | **off** |
| C standard | gnu11 |
| Optimisation | Debug `-Og`, Release `-O2` |
| Warnings (global) | `-Wall -Wextra -Wdouble-promotion -Wshadow` |
| Forbidden flag | `-ffast-math` (changes IEEE semantics the EKF relies on) |
| Pre-build step | `../tools/gen_build_info.sh` → `Core/Inc/build_info.h` (git short hash, dirty flag, build date) |
| Post-build | `arm-none-eabi-size`, `-Wl,--print-memory-usage` |

---

## 5. Execution model

### 5.1 Principle

There is no RTOS (cyclic executive). One 1 kHz interrupt (TIM6) is the only scheduling source. The interrupt itself does almost nothing: it takes snapshots and sets flags. All computation runs in the main loop, in a fixed order, from flags. Every measurement carries a 1 µs timestamp from TIM2, taken at acquisition. Integrations use these timestamps, never nominal periods (REQ-EXE-01). *Timestamps beat sample rate.*

### 5.2 Tick ISR (`sys_sched_tick_isr`, called from `HAL_TIM_PeriodElapsedCallback` for TIM6)

In this order, and nothing else (REQ-EXE-02):

1. `t = TIM2->CNT` (µs).
2. `enc_snapshot_isr(t)`: read `TIM3->CNT`, `TIM8->CNT` into the double-buffered snapshot (§7.6).
3. `tick++` (`uint32_t`).
4. For each slot *i*: if `(tick - phase_i) % div_i == 0`: if `pending & bit_i`, then `overrun[i]++`; set `pending |= bit_i`.
5. `PROBE` pin toggle (Debug builds only).

Budget: ≤ 2 µs at 180 MHz. No floating point, no HAL call except the register reads above.

### 5.3 Slots

| Slot | Rate | Divider | Phase (tick) | Work, in this order | Budget |
|---|---|---|---|---|---|
| `S_FAST` | 1 kHz | 1 | 0 | `enc_update()`, `com_uart_poll()` (drain RX into parsers), `imu_drain()` → `att`, `vl_service()` (advance I²C state machines, hand new results to `gnd`/`lat`/`fwd`), `bat_service()` | 150 µs |
| `S_EST` | 200 Hz | 5 | 0 | `app_est()`: build `ekf_odo_t` from the encoder reading of this tick and `imu_yaw_integral_at()`, `ekf_predict()`, then ZUPT and the queued aiding measurements (edge events, face, post) | 300 µs |
| `S_CTRL` | 50 Hz | 20 | **1** | `gnd_step()`, `fwd_step()` (reflex, staleness) → `msn_step()` (calls `nav_*`, `evd_*`) → `drv_step()` (runs both `whl_step()`) → `mot_set()`/`mot_apply()` | 600 µs |
| `S_TLM` | 10 Hz | 100 | 7 | `tlm_fast()` or every 10th call `tlm_slow()`; console auto-print; LED | 400 µs |
| `S_SLOW` | 1 Hz | 1000 | 13 | statistics, CPU load, `sys_fault` housekeeping | 100 µs |

Phase 1 of `S_CTRL` means the control chain always runs one tick *after* an EKF step and uses a pose at most 1 ms old (REQ-EXE-03). Phase 7 and 13 keep slow work out of the control tick.

**Deviation from the concept (D-1, §18.2).** The concept had wheel PI at 200 Hz, path follower at 100 Hz and mission at 20 Hz. This spec runs *mission, guidance and wheel control in one 50 Hz slot*:
- With count-difference speed estimation, 50 Hz is the correct wheel rate: one count per sample is 3.18 mm/s at 50 Hz, 12.7 mm/s at 200 Hz.
- A path follower faster than the loop that consumes its output adds nothing.
- 20 Hz is not harmonic with 50 Hz (ratio 2.5), and a mission running between control ticks would see half-updated state.

The guidance bandwidth (1.5–2 rad/s) is a factor ≥ 150 below 50 Hz. Raising the wheel loop to 200 Hz is possible later with `WHL_SPEED_EST = EST_MT` (§7.6.4) by changing `S_CTRL` to divider 5; the cascade code reads Δt from timestamps and is rate-independent (REQ-EXE-04).

### 5.4 Main loop

```c
void app_loop(void)                  /* called forever from main() USER CODE 3 */
{
    uint32_t p = sys_sched_take();   /* atomically read and clear `pending` */
    if (p & S_FAST) app_fast();
    if (p & S_EST)  app_est();
    if (p & S_CTRL) app_ctrl();
    if (p & S_TLM)  app_tlm();
    if (p & S_SLOW) app_slow();
    con_poll();                      /* text command execution, non-blocking */
    sys_wdg_service();               /* refresh only if the conditions of §5.7 hold */
    __WFI();                         /* sleep until the next interrupt */
}
```

Every slot function MUST return within its budget and MUST NOT block (REQ-EXE-05). "Block" includes `HAL_Delay`, polling HAL calls with a timeout, busy-waiting on a flag, and blocking UART transmission. Exceptions are allowed only during `app_init()`.

### 5.5 Concurrency rules

| Rule | Requirement |
|---|---|
| REQ-EXE-06 | Data written in an ISR and read in the main loop is either (a) a single aligned 32-bit word declared `volatile`, or (b) a struct protected by a sequence counter (ISR increments `seq` before and after writing; reader retries while `seq` is odd or changed), or (c) a single-producer/single-consumer ring buffer with `volatile` head/tail indices. |
| REQ-EXE-07 | Short critical sections (`__disable_irq()` … `__enable_irq()`) are allowed only for copying ≤ 64 bytes; they MUST restore the previous PRIMASK state (`sys_irq_save()` / `sys_irq_restore()`). |
| REQ-EXE-08 | No floating-point arithmetic in ISRs. Float lazy stacking is on by default, but keeping the FPU out of ISRs keeps their timing bounded and the rule simple. |
| REQ-EXE-09 | No HAL function with an internal timeout (`HAL_I2C_Master_Transmit`, `HAL_UART_Transmit`, `HAL_SPI_TransmitReceive`, `HAL_Delay`) is called from an ISR or from a slot function. Use `_IT` / `_DMA` variants. |
| REQ-EXE-10 | Each module owns its state. Other modules read it only through `const` accessors or by copying a published struct. No module writes another module's variables. |

### 5.6 Timing measurement

Each slot is wrapped by `SYS_SLOT_BEGIN(i)` / `SYS_SLOT_END(i)`, which read `DWT->CYCCNT` and keep `max_cycles[i]` and `sum_cycles[i]`. `S_SLOW` computes the CPU load = Σ slot cycles / 180·10⁶ per second and publishes it in `TELEM_SLOW`. `PROBE` (PC8) is high during `S_CTRL` in Debug builds, so the control slot can be checked with an oscilloscope (REQ-EXE-11).

**Overrun policy (REQ-EXE-12).** `overrun[i] > 0` for `S_EST` or `S_CTRL` raises `F_OVERRUN` (warning, logged). More than 10 overruns of `S_CTRL` within 1 s raises it to a controlled stop.

### 5.7 Watchdog

The IWDG is refreshed in `sys_wdg_service()` only if **all** of the following hold (REQ-EXE-13):
- `S_CTRL` completed within the last 30 ms;
- `S_EST` completed within the last 10 ms (only when `FEAT_IMU`);
- the main loop has run within the last 5 ms.

A watchdog reset is detected at the next boot (`RCC_CSR.IWDGRSTF`) and reported as `F_WDG_RESET` in `HELLO` and on the console. The flags are cleared with `__HAL_RCC_CLEAR_RESET_FLAGS()` after reading.

### 5.8 Start-up sequence (`app_init`)

Order is mandatory (REQ-EXE-14). Each step reports `OK` / `FAIL <code>` on USART2. The steps marked *fatal* go to `FAULT` on failure; the others mark the device unavailable.

| # | Step | Fatal |
|---|---|---|
| 1 | `sys_time_init()`: start TIM2 at 1 MHz, enable DWT. Read and clear the reset cause. | yes |
| 2 | `mot_init()`: EN_L/EN_R low, CCR1–4 = 0, start PWM on all four channels | yes |
| 3 | `enc_init()`: start TIM3/TIM8 encoder mode, zero counts | yes |
| 4 | `par_init()`: load compiled defaults, validate ranges | yes |
| 5 | `com_uart_init()`: arm USART2 RX, USART1 RX; print `HELLO` (version, git hash, reset cause, build flags) | no |
| 6 | Clock self-check: count TIM2 µs across 100 SysTick ms. Must be 100 000 ± 100 µs; else `F_CLOCK` (HSE probably not running). | yes |
| 7 | `bat_init()`, first reading. Below cut-off → warning (bench supply allowed). | no |
| 8 | `imu_init()` (if `FEAT_IMU`): WHO_AM_I, configure, start DRDY capture | no |
| 9 | `vl_init()` (if `FEAT_VL`): XSHUT sequence, address assignment, configuration, start ranging | no |
| 10 | `map_init()`: validate course table (§7.18.4) | no (AUTO then unavailable) |
| 11 | `ekf_init()`, `nav_init()`, `msn_init()` | yes |
| 12 | `sys_sched_start()`: start TIM6 interrupt | yes |
| 13 | After 1 s: report the measured tick rate (expect 1000.0 Hz ± 0.1 %) | no |
| 14 | `sys_wdg_start()` (Release always; Debug if `par.wdg_enable`) | — |

`HAL_Delay` is allowed only in steps 1–11.

---

## 6. Coding rules and conventions

### 6.1 Language rules

| ID | Rule |
|---|---|
| REQ-COD-01 | C11 (`gnu11`). No C++, no VLAs, no recursion, no `malloc`/`free`/`calloc`. All buffers are static and sized at compile time. |
| REQ-COD-02 | **Single precision only.** Every floating literal has the `f` suffix (`0.5f`). Use `sinf`, `cosf`, `atan2f`, `sqrtf`, `fabsf`, `fminf`, `fmaxf`. No `double` anywhere in user code. `-Wdouble-promotion` must be warning-free. |
| REQ-COD-03 | No `printf`/`sprintf` with `%f`. Format numbers as integers in scaled units (`fmt_milli()`, §6.5). |
| REQ-COD-04 | Fixed-width integer types (`uint8_t` … `int32_t`) for everything that has a size on the wire, in a register or in an array index. `int` only for loop counters bounded below 32 767. |
| REQ-COD-05 | Variables shared with an ISR are `volatile` and follow REQ-EXE-06. |
| REQ-COD-06 | Every `switch` has a `default`; every state machine handles every state explicitly. |
| REQ-COD-07 | Every loop has a compile-time or checked run-time bound. |
| REQ-COD-08 | `static` for everything that is file-local. Public symbols carry the module prefix. |
| REQ-COD-09 | No magic numbers in logic. Tunables are parameters (§10); physical constants are `#define` in `units.h` or the module header with unit and source in a comment. |
| REQ-COD-10 | Each float result that feeds an actuator or the EKF state is checked with `isfinite()` at the module boundary. A non-finite value raises `F_NUMERIC` (§11). |
| REQ-COD-11 | Angles are wrapped to (−π, π] with `wrap_pi()` after every addition. Differences of angles always go through `wrap_pi()`. |
| REQ-COD-12 | Timestamps are `uint32_t` µs; differences are computed as `(uint32_t)(t1 - t0)`, which is correct across the 71.6-min wrap. |
| REQ-COD-13 | Encoder counts are accumulated as `int32_t` from `int16_t` differences of the 16-bit counters. This is correct while `\|Δ\| < 32768` between two snapshots; the actual maximum is ≈ 11 counts per 1 ms snapshot at no-load speed (10.8·10³ counts/s), and still < 2000 if the main loop were to miss a full second. |

### 6.2 Naming

| Item | Convention | Example |
|---|---|---|
| Public function | `<prefix>_<verb>[_object]` | `whl_step()`, `ekf_update_edge()` |
| Type | `<prefix>_<name>_t` | `whl_state_t`, `proto_frame_t` |
| Enum constant | `<PREFIX>_<NAME>` upper case | `MSN_ST_APPROACH` |
| Macro | upper case with prefix | `WHL_SIDE_L` |
| Unit suffix on every physical variable | `_m`, `_mps`, `_mps2`, `_rad`, `_radps`, `_s`, `_us`, `_ms`, `_v`, `_hz`, `_cnt`, `_d` (duty fraction), `_pct` | `v_ref_mps`, `t_us`, `duty_d` |
| Side index | `WHL_L = 0`, `WHL_R = 1` | arrays `[2]` |

Parameter names (§10) are the exception: they carry no unit suffix, because their unit is part of the parameter descriptor and the catalogue.

Module prefixes: `app`, `sys`, `par`, `mot`, `enc`, `prof`, `whl`, `drv`, `bat`, `imu`, `att`, `vl`, `gnd`, `lat`, `fwd`, `ekf`, `map`, `nav`, `evd`, `msn`, `com`, `con`, `proto`, `tlm`, `cal` (existing `Calib_*` keeps its names).

### 6.3 Status and error returns

```c
typedef enum {
    ST_OK = 0,
    ST_BUSY,        /* operation in progress, call again        */
    ST_TIMEOUT,     /* peripheral did not respond in time        */
    ST_BUS,         /* I²C NACK / bus error / SPI error          */
    ST_RANGE,       /* argument out of the allowed range         */
    ST_STATE,       /* not allowed in the current mode/state     */
    ST_CRC,         /* frame or data integrity error             */
    ST_NODEV,       /* device absent or disabled                 */
    ST_REJECT       /* measurement rejected by a gate            */
} st_t;
```

Functions that can fail return `st_t`. Functions that cannot fail return `void` or the value. A caller MUST handle or explicitly ignore (`(void)`) every `st_t` (REQ-COD-14).

### 6.4 Units and conversions (`units.h`)

```c
#define PI_F          3.14159265f
#define TWO_PI_F      6.28318531f
#define DEG2RAD_F     0.0174532925f
#define RAD2DEG_F     57.2957795f
#define G_MPS2        9.80665f

static inline float wrap_pi(float a);          /* → (−π, π]          */
static inline float clampf(float x, float lo, float hi);
static inline float signf(float x);            /* −1, 0, +1          */
static inline float sqf(float x) { return x * x; }
```

**Console boundary.** Console commands (§9) use mm, mm/s, %, degrees and ms. `con.c` converts on entry and exit; no other module sees legacy units. The table in Appendix 19.2 converts every existing constant of the teammate's `main.c` into the SI parameter it becomes.

### 6.5 Text formatting

`con_printf(port, fmt, ...)` uses `vsnprintf` into a 128-byte stack buffer and hands the result to the port's TX ring (§7.22), dropping on overflow (counted, never blocking). Float values are printed through helpers that convert to scaled integers:

```c
/* fmt_fixed(buf, value, decimals): 1.2345f, 3 → "1.235" ; −0.0004f, 3 → "-0.000" */
int fmt_fixed(char *buf, size_t n, float value, uint8_t decimals);
```

### 6.6 Assertions

`SYS_ASSERT(cond)` → in Debug builds `sys_fault_raise(F_ASSERT, __LINE__)` plus the file ID; in Release builds the same, but only for asserts marked `SYS_ASSERT_SAFETY`. Plain asserts compile to nothing in Release. Assertions MUST NOT have side effects (REQ-COD-15).

### 6.7 Comments and headers

Every public function in a header has a comment with: purpose, units of every argument and return value, which context it may be called from (`ISR`, `main loop`, `init only`), and whether it is pure (REQ-COD-16). Every module `.c` starts with a header block naming the spec section it implements.

### 6.8 Memory budget

| Item | Budget | Expected |
|---|---|---|
| Flash total | < 192 KB (37 %) | 60–100 KB |
| ULD | < 10 KB | — |
| SRAM total | < 112 KB of 128 KB | ~70 KB |
| Capture buffer | 48 KB, static, `FEAT_CAPTURE` | — |
| UART buffers | USART1 RX 256 B + TX 2 × 128 B; USART2 RX 256 B + TX 2 × 256 B | — |
| IMU ring | 32 samples × 24 B | — |
| Event log | 64 × 12 B | — |
| Stack | 0x1000 (4 KB); high-water mark measured at boot by painting (`0xA5`) and reported in `TELEM_SLOW` | < 2.5 KB |

---

## 7. Module specifications

**Conventions for this section.** Pure modules (§3.1) expose `xxx_init(xxx_t *ctx, …)` and `xxx_step(xxx_t *ctx, inputs, outputs)`. `app.c` owns the one instance of each context (`static xxx_t g_xxx;`), so unit tests can create as many as they need. Hardware modules are singletons with plain functions. API listings are normative for names, arguments, units and semantics. Private helpers are free.

### 7.1 `sys_time` — time base

**Responsibility.** 1 µs monotonic time (TIM2), CPU cycle counter (DWT), reset cause.

```c
void     sys_time_init(TIM_HandleTypeDef *htim2);   /* init only: PSC must be 89 */
uint32_t sys_time_us(void);                          /* any context; TIM2->CNT    */
uint32_t sys_cycles(void);                           /* any context; DWT->CYCCNT  */
uint32_t sys_ms(void);                               /* = HAL_GetTick()           */
uint8_t  sys_reset_cause(void);                      /* bit field, read once at boot */
```

- REQ-TIM-01 `sys_time_init` MUST verify `htim2->Init.Prescaler == 89` and the timer clock at run time (§5.8 step 6).
- REQ-TIM-02 Reset cause bits: POR/BOR, pin, software, IWDG, WWDG, low-power. Reported in `HELLO`.

### 7.2 `sys_sched` — tick and slots

```c
void     sys_sched_start(TIM_HandleTypeDef *htim6);  /* starts TIM6 IT                  */
void     sys_sched_tick_isr(void);                   /* ISR: §5.2                       */
uint32_t sys_sched_take(void);                       /* main: atomically fetch+clear    */
uint32_t sys_sched_tick(void);                       /* ticks since start               */
void     sys_sched_stats(sys_sched_stats_t *out);    /* overruns, max/avg cycles, load  */
```

- REQ-SCH-01 Slot table (§5.3) is a `const` array; dividers and phases are compile-time constants.
- REQ-SCH-02 At 1 s after start, `tick` must equal 1000 ± 1 relative to `sys_time_us()`. Otherwise print a warning (TIM6 misconfigured — the O-1 class of error).

### 7.3 `sys_fault` — faults and events

**Responsibility.** Central registry of fault codes (§11), latching, the reaction class, an event log ring buffer, and the low-level stop path used by `Error_Handler`, `HardFault_Handler`, `NMI_Handler` (CSS) and `HAL_RCC_CSSCallback`.

```c
void        sys_fault_raise(fault_t code, int32_t arg);     /* any context              */
void        sys_fault_clear(void);                          /* main: only if causes gone */
bool        sys_fault_active(fault_t code);
reaction_t  sys_fault_reaction(void);                       /* highest active reaction  */
uint16_t    sys_fault_first(void);                          /* first latched fault      */
void        sys_event(event_t ev, int32_t arg);             /* log + telemetry EVENT    */
void        sys_fault_hard_stop(void);                      /* register-level: EN low, CCR=0 */
```

- REQ-FLT-01 `sys_fault_hard_stop()` uses only direct register writes (`GPIOB->BSRR = (GPIO_PIN_12 | GPIO_PIN_13) << 16; TIM1->CCR1..4 = 0;`). It is safe from any ISR and from fault handlers.
- REQ-FLT-02 `Error_Handler()` (USER CODE in `main.c`), `HardFault_Handler`, `MemManage_Handler`, `BusFault_Handler`, `UsageFault_Handler` and `NMI_Handler` (USER CODE in `stm32f4xx_it.c`) call `sys_fault_hard_stop()` as their first statement. (Open item O-12.)
- REQ-FLT-03 `sys_fault_raise` with reaction ≥ `R_BRAKE` takes effect in the same control tick: `app_ctrl()` checks `sys_fault_reaction()` before `msn_step()`.
- REQ-FLT-04 The event log holds the last 64 events `{t_ms, code, arg}` and survives a fault (not a reset). It is downloadable with `EVLOG` (§9).

### 7.4 `par` — parameters

**Responsibility.** All tunables in one RAM struct, compiled defaults, run-time get/set by ID with range checks, a dump in `#define` format.

```c
typedef struct { /* one field per row of §10, SI units */ } par_t;
extern const par_t PAR_DEFAULT;
void         par_init(void);                       /* copy defaults, validate          */
const par_t *par(void);                            /* read access for everyone         */
st_t         par_set(uint16_t id, float value);    /* range check, apply, log event    */
st_t         par_get(uint16_t id, float *value);
const par_desc_t *par_desc(uint16_t id);           /* name, unit, min, max, flags      */
void         par_dump_defines(com_port_t port);    /* prints `#define PAR_xxx value`   */
```

- REQ-PAR-01 The descriptor table (`par_table.h`) is the single source for ID, name, unit, default, min, max and the flags `LIVE` (may change while moving) / `STOPPED` (only in `IDLE`/`BENCH` with the vehicle stationary). Parameters are declared once in an X-macro list, and the struct, the defaults and the table are generated from it (REQ-PAR-02).
- REQ-PAR-03 `par_set` outside the range or in the wrong mode returns `ST_RANGE` / `ST_STATE` and changes nothing.
- REQ-PAR-04 Modules read parameters through `par()` at every step. They do not cache them, so a live change takes effect at the next tick. Derived constants (e.g. SIMC gains) are recomputed by the owning module when `par_generation()` changes.
- REQ-PAR-05 IDs are stable. A removed parameter's ID is never reused.

### 7.5 `mot` — motor actuation (BTS7960)

**Hardware.** One BTS7960 module per motor. RPWM = forward, LPWM = reverse, R_EN + L_EN bridged on one GPIO per module. With both PWM inputs low and EN high, both low-side switches conduct: the winding is short-circuited (**brake**). With EN low, all switches are off (**coast**, high impedance).

```c
void  mot_init(TIM_HandleTypeDef *htim1);            /* EN low, CCR=0, start 4 PWM ch.   */
void  mot_enable(bool on);                           /* both EN lines                    */
void  mot_set(uint8_t side, float duty_d);           /* command for the next mot_apply() */
void  mot_apply(void);                               /* single writer of CCR1..4         */
void  mot_brake(void);                               /* duty 0 both, EN high             */
float mot_duty(uint8_t side);                        /* last applied, after sign/limits  */
```

| ID | Requirement |
|---|---|
| REQ-MOT-01 | `mot_apply()` is the **only** code that writes CCR1–CCR4 (besides `sys_fault_hard_stop()`). |
| REQ-MOT-02 | Mapping: `m = duty_d · MOT_SIGN[side]`; `m > 0` → forward channel = `round(\|m\| · (ARR+1))`, reverse channel = 0; `m < 0` → mirrored; `m = 0` → both 0. The inactive channel is written **before** the active one. Both channels can never be non-zero at the same time, not even for one instruction. |
| REQ-MOT-03 | `\|duty_d\|` is clamped to `mot_d_max` (default 1.0). Values with `0 < \|m\| < mot_d_min` (driver output floor, `[MEASURE]`, default 0.0) are written as 0. |
| REQ-MOT-04 | `MOT_SIGN_L = −1`, `MOT_SIGN_R = +1` (measured on the vehicle harness 8 Oct). Positive `duty_d` drives the wheel forward. |
| REQ-MOT-05 | EN is high only while the drive layer is in a closed-loop or open-loop mode. `IDLE` and `FAULT` keep EN low. |
| REQ-MOT-06 | Open-loop duty from the console (`BENCH`) is slew-limited to `mot_slew_ol` (2.0 /s = 0→100 % in 0.5 s), and a sign change passes through 50 ms of zero. Closed-loop duty is **not** slew-limited. The loops assume an unlimited actuator. |

### 7.6 `enc` — encoders and odometry increments

#### 7.6.1 Interface

```c
typedef struct {
    int32_t  cnt[2];          /* accumulated counts, wheel-forward positive      */
    uint32_t t_us;            /* TIM2 timestamp of the snapshot                   */
} enc_reading_t;

void enc_init(TIM_HandleTypeDef *htim3, TIM_HandleTypeDef *htim8);
void enc_snapshot_isr(uint32_t t_us);   /* ISR (tick): latch 16-bit CNT + time     */
void enc_update(void);                  /* S_FAST: accumulate int32 totals         */
enc_reading_t enc_read(void);           /* latest accumulated reading              */
void enc_history(uint8_t side, enc_hist_t *out); /* last 64 snapshots (M/T)        */
void enc_zero(void);                    /* only when stationary                    */
```

- REQ-ENC-01 Sign: `ENC_SIGN_L = +1`, `ENC_SIGN_R = −1` (vehicle harness). Forward motion gives positive counts on both wheels.
- REQ-ENC-02 Totals are accumulated from `(int16_t)(cnt_now − cnt_prev)` (REQ-COD-13).
- REQ-ENC-03 Consumers (`drv`, `ekf`, `tlm`) keep their own previous `enc_reading_t` and form differences themselves. `enc` never resets a consumer's view.

#### 7.6.2 Distance per count

`m_per_cnt[side] = odo_m_per_cnt · odo_scale_l/r` with `odo_m_per_cnt = π · 0.0726 m / 3584 = 6.3638·10⁻⁵ m` (calibrated mean, 8 Oct) and `odo_scale_L/R = 1.0` until UMBmark (`[MEASURE]`). Only the ratio D/C is calibrated. If per-wheel CPR values are ever entered, D must rise to ≈ 73.06 mm to keep the mean (open item O-7).

#### 7.6.3 Backlash dead band `[PHASE 5]`

For odometry (EKF input only, not the wheel loops): per wheel, track the direction of the last count change. After a reversal, the first `odo_backlash_l` / `odo_backlash_r` counts (from calibration test `L`, default 0) are not converted to distance. REQ-ENC-04.

#### 7.6.4 Speed estimation

`EST_DIFF` (default, current behaviour). At each control step with snapshot pair (n₀, t₀) → (n₁, t₁):
`v_raw = m_per_cnt · (n₁ − n₀) / ((t₁ − t₀)·10⁻⁶)`, then `v̂ ← v̂ + α (v_raw − v̂)` with `whl_speed_alpha` = 0.5 (τ ≈ 29 ms at 50 Hz). Resolution 3.18 mm/s per count at 50 Hz.

`EST_MT` `[PHASE 5]`. From the 1 kHz history: within the window W = control period, find the first and last snapshot at which the count changed (`k_f`, `k_l`). If at least two changes exist: `v = m_per_cnt · (n_kl − n_kf) / (t_kl − t_kf)`. If fewer and no change for > `enc_mt_zero_s` (0.1 s): v = 0. Otherwise: `v = ±m_per_cnt / (t_now − t_last_change)` (upper bound, decays). This improves low-speed resolution: at 10 mm/s (≈ 3 counts per 20 ms) the count difference is uncertain by ±1 count = ±33 %, the M/T estimate by about ±8 % (1 ms time quantisation at both ends of a ≈ 13 ms interval). REQ-ENC-05: both estimators are unit-tested against the same synthetic edge trains (§14.1).

### 7.7 `prof` — trapezoidal profiles (pure)

```c
typedef struct { float D, V, ta, td, T, t1, t2; int8_t dir; } prof_t;
st_t  prof_plan(prof_t *p, float dist, float v_nom, float t_acc_s, float t_dec_s);
void  prof_eval(const prof_t *p, float t_s, float *x, float *v, float *a);
float prof_duration(const prof_t *p);
```

Units are generic (m or rad).

- Full trapezoid: `T = |D|/V + (ta + td)/2`, t₁ = ta, t₂ = T − td.
- Triangle if `|D| < V (ta + td)/2`: `V' = 2|D|/(ta + td)`, `T = ta + td`.
- REQ-PRF-01 `x(T) = D` exactly (to 1e-6 relative), `v` continuous, `a` piecewise constant. `prof_eval` for `t ≥ T` returns `(D, 0, 0)`.
- REQ-PRF-02 `prof_plan` rejects `V ≤ 0`, `ta < 0.05 s`, `td < 0.05 s`, `|D| > 20 m` with `ST_RANGE`.

The same file provides a rate limiter for online references:
`float ramp_step(ramp_t *r, float target, float rate_up, float rate_down, float dt);`

### 7.8 `whl` — per-wheel controller (pure)

**Responsibility.** For one wheel: P position loop around a reference, model feedforward, PI speed loop, and fault detectors. It contains no mode logic. `drv` decides the references.

```c
typedef struct {
    float x_ref_m, v_ref_mps, a_ref_mps2;  /* reference from drv                  */
    float v_sync_mps;                      /* L/R coupling term from drv          */
    float x_m, v_mps;                      /* measured (enc)                      */
    float u_bat_v;                         /* filtered battery voltage (bat)      */
    float dt_s;                            /* from timestamps                     */
    bool  pos_loop;                        /* false: pure speed control (OPEN-like) */
} whl_in_t;

typedef struct {
    float duty_d;                          /* to mot_set()                        */
    float v_cmd_mps, e_x_m, e_v_mps, integ_d, ff_d;
    uint8_t flags;                         /* WHL_ROLLING, WHL_SAT, WHL_STALL, WHL_RUNAWAY, WHL_TRACK */
} whl_out_t;

void whl_init(whl_t *w, uint8_t side);
void whl_reset(whl_t *w);                  /* integrator 0, detectors cleared     */
void whl_step(whl_t *w, const whl_in_t *in, whl_out_t *out);
```

**Control law (REQ-WHL-01).** With side-specific parameters (§10, group WHL):

```
e_x    = x_ref − x                                          (if pos_loop, else 0)
v_cmd  = v_ref + sat(Kp_x · e_x, ±v_corr_max) + v_sync
e_v    = v_cmd − v̂
rolling: set when |v̂| > v_moving (8 mm/s), cleared when |v̂| < v_stopped (3 mm/s)
Ks*    = rolling ? Ks : Ks_break
ff     = 0                                            if |v_cmd| ≤ v_zero (2 mm/s)
       = sgn(v_cmd)·(Ks* + Kv·|v_cmd|) + Kv·τ_m·a_ref  otherwise
ff     ← ff · (U_ref / clamp(U_bat, 9.5, 12.8))       if ff_norm_en and battery present
u      = ff + Kp_v·e_v + I
I      ← clamp(I + Ki_v·e_v·dt, ±I_max)    only if |u| < 1 (conditional integration)
I      ← 0                                  if |v_cmd| ≤ v_zero and |v_ref| ≤ v_zero
duty   = clamp(u, ±1)
```

- REQ-WHL-02 The kick mechanism of the old code (40 % for up to 5 ticks) is **not** implemented. The `Ks_break` switching replaces it, because the kick acted per wheel independently and let one wheel break away first.
- REQ-WHL-03 Defaults (converted from the identification of 9 Oct, see Appendix 19.2): Kp_v = 0.45 /(m/s), Ki_v = 8.98 /m (SIMC "balanced", τ_c = 2θ), I_max = 0.30, Kp_x = 2.0 /s, v_corr_max = 0.10 m/s, τ_m = 0.050 s; feedforward L: Ks = 0.098, Kv = 1.275 /(m/s); R: Ks = 0.041, Kv = 1.361 /(m/s); Ks_break L/R = 0.141 / 0.169 (bench). All `[MEASURE]` with calibration `G` and `B` on the floor.
- REQ-WHL-04 **Detectors.** Each detector needs its condition for `whl_fault_time` (0.6 s) without interruption:
  - `WHL_STALL`: `|duty| ≥ 0.5` and `|v̂| < 5 mm/s`;
  - `WHL_RUNAWAY`: `duty > 0.1` and `v̂ < −20 mm/s`, or mirrored (wrong motor or encoder sign);
  - `WHL_TRACK`: `|e_x| > whl_track_max` (0.04 m) while `pos_loop`.

  They only *report*. `drv` raises the fault.
- REQ-WHL-05 `dt_s` outside [0.5, 2]·nominal (timing glitch): the step uses the nominal period for the integrator and increments a statistics counter.

### 7.9 `drv` — drive modes and wheel references (pure)

**Responsibility.** Converts the motion command of the upper layers into per-wheel references `(x_ref, v_ref, a_ref)` and runs the two `whl` controllers. It implements the end-of-move rules and L/R synchronisation and raises drivetrain faults.

**Modes.**

| Mode | Command source | References |
|---|---|---|
| `DRV_DISABLED` | — | EN low |
| `DRV_HOLD` | after any stop | EN high, duty 0 (brake); references frozen at the current position |
| `DRV_OPEN` | console open-loop duty | duty set directly, slew-limited (REQ-MOT-06) |
| `DRV_MOVE` | `drv_move(D, V, ta, td)` | straight line: both wheels follow `x0 + dir·x_prof(t)` |
| `DRV_TURN_ENC` | `drv_turn_enc(Δθ, ω, ta)` | turn in place by encoder angle: wheels follow `∓ (b/2)·θ_prof(t)`. Fallback without IMU. |
| `DRV_VEL` | `drv_vel(v, ω)` every control tick | `v_ref_L/R = v ∓ ω·b/2`, `x_ref += v_ref·dt`, `a_ref` from the rate limiter |

```c
void drv_init(drv_t *d);
st_t drv_move(drv_t *d, float dist_m, float v_mps, float t_acc_s, float t_dec_s);
st_t drv_move_from(drv_t *d, float dist_m, float v0_mps, float v_mps, float a_acc, float a_dec);
                                /* MOVE starting at the current speed (§7.19)   */
st_t drv_turn_enc(drv_t *d, float dtheta_rad, float w_radps, float t_acc_s);
void drv_vel(drv_t *d, float v_mps, float w_radps);       /* call every tick in VEL  */
void drv_open(drv_t *d, float duty_l, float duty_r);
void drv_hold(drv_t *d);        /* HOLD at the current position (EN high, duty 0)     */
void drv_stop(drv_t *d);        /* controlled: ramp v,ω → 0 at stop limits, then HOLD */
void drv_brake(drv_t *d);       /* immediate: duty 0, EN high, HOLD                   */
void drv_disable(drv_t *d);     /* EN low                                             */
void drv_step(drv_t *d, const drv_meas_t *m, drv_out_t *out);  /* S_CTRL            */
drv_status_t drv_status(const drv_t *d); /* mode, phase, event, sync max, errors     */
```

| ID | Requirement |
|---|---|
| REQ-DRV-01 | **MOVE** keeps the semantics of the current `D<mm>` command. Phase `MOVE` follows the profile with both `whl` in `pos_loop`. When the profile ends, phase `SETTLE`: a wheel with `dir·e_x < drv_pos_tol` (2 mm) is "reached" and braked (duty 0, integrator 0); a wheel not yet reached gets `v_cmd = clamp(Kp_x·e_x, ±drv_v_settle)` (20 mm/s) and is **never commanded backwards**. Done when both wheels are reached and `\|v̂\| < 10 mm/s` → event `DRV_EV_DONE`. After `drv_settle_timeout` (1.0 s) → `DRV_EV_TIMEOUT` (open item O-6). Then `HOLD`. |
| REQ-DRV-02 | **Synchronisation** in MOVE, TURN_ENC and VEL: `Δ = e_x,L − e_x,R`; `v_sync,L = +½·K_sync·Δ`, `v_sync,R = −½·K_sync·Δ` (K_sync = 3.0 /s, 0 = off). `max \|Δ\|` per move is reported in the status (`sync_max_m`). |
| REQ-DRV-03 | **VEL limits** (in this order): `\|v\| ≤ drv_v_max` (0.45 m/s), `\|ω\| ≤ drv_w_max` (2.5 rad/s), rate limits `drv_a_max` (0.3 m/s²) / `drv_a_stop` (0.5 m/s²) on v and `drv_alpha_max` (3.0 rad/s²) on ω. If `\|v\| + \|ω\|·b/2 > drv_v_whl_max` (0.45 m/s), reduce `\|v\|` and keep ω, so that curvature is preserved where possible. |
| REQ-DRV-04 | **Reference anti-windup** in VEL: `x_ref − x` is clamped to ±`drv_e_ref_max` (0.05 m) per wheel. A blocked wheel therefore cannot accumulate a large position debt that would make it jump when released. |
| REQ-DRV-05 | Mode changes between closed-loop modes are bumpless: the new references start at the measured positions, `a_ref = 0`, and integrators are kept. Changes into `DRV_OPEN` or `DRV_DISABLED` reset them. |
| REQ-DRV-06 | `whl` detector flags → `sys_fault_raise(F_STALL_L/R, F_RUNAWAY_L/R, F_TRACK_L/R)`; `drv` goes to `HOLD` (stall, track) or `DISABLED` (runaway). |
| REQ-DRV-07 | `drv_stop()` is the normal stop. It ramps down with `drv_a_stop`, and is used by STOP commands, mission ends and controlled-stop faults. `drv_brake()` is the reflex and fault stop. |
| REQ-DRV-08 | Wheelbase for the kinematics: `odo_wheelbase` (0.22325 m geometric; effective from UMBmark `[MEASURE]`). |

### 7.10 `bat` — battery voltage

```c
void  bat_init(ADC_HandleTypeDef *hadc1);
void  bat_service(void);   /* S_FAST: start a conversion every 20 ms, read the previous */
float bat_v(void);         /* filtered pack voltage [V], 0 if absent                    */
bool  bat_present(void);
```

- REQ-BAT-01 `U = adc / 4095 · 3.3 V · bat_ratio` with `bat_ratio` = 5.7 nominal (47 k/10 k, `[MEASURE]` with a multimeter). IIR with τ = `bat_tau` (0.2 s).
- REQ-BAT-02 `U < 3.0 V` → battery absent (USB bench). Feedforward normalisation and the cut-off are inactive.
- REQ-BAT-03 `U < bat_warn_v` (10.5 V) for 2 s → event `EV_BAT_WARN`. `U < bat_cut_v` (10.0 V) for 2 s → `F_BAT_LOW` (controlled stop, then `IDLE`; AUTO cannot start). The 2 s filter keeps a stall sag from tripping it.
- REQ-BAT-04 The ADC input has 100 nF to ground at the divider node. Source impedance 8.3 kΩ needs the 480-cycle sampling time.

### 7.11 `imu` — inertial sensor `[PHASE 2]`

**Selection.** The ICM-42688-P is unobtainable. The default backend is the **LSM6DSV** (board STEVAL-MKI239A), with the ISM330DHCX as backup (decision D-1 in the technical reference, still `[DECIDE]`). The module is split into a device-independent front end (`imu.c`) and a backend (`imu_lsm6dsv.c`). A different part needs only a new backend.

```c
/* backend interface (imu_xxx.c) */
typedef struct {
    st_t (*probe)(void);                       /* WHO_AM_I                        */
    st_t (*configure)(void);                   /* ODR, ranges, filters, DRDY      */
    void (*start_read_dma)(uint8_t *rx, uint8_t *tx, uint16_t n); /* from DRDY ISR */
    void (*decode)(const uint8_t *rx, int16_t g[3], int16_t a[3], int16_t *t);
    float gyro_lsb_radps, acc_lsb_mps2, temp_lsb_c, temp_off_c;
    float odr_hz;
} imu_backend_t;

/* front end */
st_t  imu_init(SPI_HandleTypeDef *hspi1, TIM_HandleTypeDef *htim2);
void  imu_drdy_capture_isr(uint32_t t_capture_us);  /* TIM2 CC2 ISR: start DMA       */
void  imu_dma_done_isr(void);                       /* SPI RX DMA complete: push ring */
void  imu_drain(att_t *att);                        /* S_FAST: convert, att, integral */
st_t  imu_yaw_integral_at(uint32_t t_us, float *psi_rad); /* ∫ψ̇ dt up to t (§7.11.3) */
bool  imu_alive(void);                              /* sample within imu_timeout    */
float imu_temp_c(void);
```

#### 7.11.1 LSM6DSV configuration `[VERIFY]` every register value against the LSM6DSV datasheet

| Item | Value | Note |
|---|---|---|
| WHO_AM_I (0x0F) | 0x70 | probe fails otherwise → `ST_NODEV` |
| Gyro | ODR 960 Hz, high-performance mode, FS ±500 dps (17.50 mdps/LSB) | ±250 dps would leave too little margin over the 2.7 rad/s (154 dps) maximum spin |
| Accelerometer | ODR 960 Hz, FS ±4 g (0.122 mg/LSB) | — |
| BDU | on | high/low bytes of one sample are never mixed |
| Auto-increment | on | burst read |
| INT1 | gyro data-ready, **pulsed** mode, push-pull, active high | the TIM2 capture sees one edge per sample |
| Burst read | 14 bytes from OUT_TEMP_L (0x20): temperature, gyro XYZ, accel XYZ | 15 SPI bytes incl. address, ≈ 21 µs at 5.6 MHz |

The design does not depend on the ODR. All integration uses timestamps, and 960 Hz is not an integer multiple of the 200 Hz EKF (REQ-IMU-01).

#### 7.11.2 Axis mapping and calibration

- REQ-IMU-02 A constant 3 × 3 signed permutation `IMU_AXIS_MAP` (in `imu.h`) maps sensor axes to body axes (x forward, y left, z up). It is set after mounting `[MEASURE]`: a positive (counter-clockwise) turn must give positive `ω_z`, and a nose-down tilt positive pitch.
- REQ-IMU-03 Temperature model on the z rate: `ω_z ← ω_z − (imu_tc1·ΔT + imu_tc2·ΔT²)` with `ΔT = T − 25 °C`. Coefficients come from a warm-up calibration run, default 0 `[MEASURE]`. The constant bias is *not* compensated here; the EKF estimates it.
- REQ-IMU-04 Mounting: rigid, at the drive-axle midpoint, away from the drivers. A mechanical requirement, recorded here because the kinematics assume it.

#### 7.11.3 Yaw integral

For every sample i (timestamp tᵢ from the input capture, rates in rad/s after mapping), `imu_drain` computes the projected yaw rate with the current attitude from `att` (§7.12):

`ψ̇ᵢ = (q·sin φ + r·cos φ) / cos ϑ`

and accumulates `Ψᵢ = Ψᵢ₋₁ + ψ̇ᵢ · (tᵢ − tᵢ₋₁)`. A ring of the last 32 `(tᵢ, Ψᵢ, ψ̇ᵢ)` is kept.

`imu_yaw_integral_at(t)` returns Ψ at an arbitrary time t inside the ring by linear interpolation. Beyond the newest sample, it extrapolates by at most `imu_extrap_max` (2 ms) with the newest rate. Older or newer t → `ST_RANGE` (REQ-IMU-05).

- REQ-IMU-06 No sample for `imu_timeout` (20 ms) → `F_IMU_TIMEOUT`.
- REQ-IMU-07 Saturated samples (|raw| = 32767/−32768 on any gyro axis) are counted and flagged; three in a row → `F_IMU_SAT` (warning). The EKF inflates `q_θ` × 100 for that step.

### 7.12 `att` — roll and pitch (pure) `[PHASE 2]`

Complementary filter, updated for every IMU sample:

```
φ̇ = p + (q sin φ + r cos φ) tan ϑ          ϑ̇ = q cos φ − r sin φ
φ_a = atan2(a_y, a_z)                       ϑ_a = atan2(−a_x, √(a_y² + a_z²))
if | |a| − g | < att_acc_gate · g:   φ += k (φ_a − φ),  ϑ += k (ϑ_a − ϑ),  k = Δt / att_tau
```

with `att_tau` = 1.0 s and `att_acc_gate` = 0.1. Initialised to the mean accelerometer angles over the first 0.5 s at standstill.

```c
void att_init(att_t *a);
void att_update(att_t *a, const float w_radps[3], const float acc_mps2[3], float dt_s);
float att_roll(const att_t *a);   float att_pitch(const att_t *a);
```

- REQ-ATT-01 On level ground at rest, |φ|, |ϑ| < 0.5° after 2 s. On a 6° ramp, ϑ is within 0.5° of the true pitch after 3 s.

### 7.13 `vl` — VL53L1X manager `[PHASE 3]`

#### 7.13.1 Sensor table

`const vl_cfg_t VL_CFG[8]` in `vl.c` (REQ-VL-01):

| ID | Name | Bus | XSHUT | 7-bit addr | Role | Mount (body, m) | View | Mode | ROI | Data ready |
|---|---|---|---|---|---|---|---|---|---|---|
| 0 | G1 | I2C1 | PC0 | 0x30 | ground left outer | `x = gnd_x, y = +gnd_y_out` | down | short | 4×4 | polled |
| 1 | G2 | I2C1 | PC1 | 0x31 | ground left inner | `x = gnd_x, y = +gnd_y_in` | down | short | 4×4 | polled |
| 2 | G3 | I2C1 | PC2 | 0x32 | ground right inner | `x = gnd_x, y = −gnd_y_in` | down | short | 4×4 | polled |
| 3 | G4 | I2C1 | PC3 | 0x33 | ground right outer | `x = gnd_x, y = −gnd_y_out` | down | short | 4×4 | polled |
| 4 | F1 | I2C2 | PB14 | 0x34 | forward left | `x = fwd_x, y = +fwd_y` | +x | short | 16×16 | polled |
| 5 | F2 | I2C2 | PB15 | 0x35 | forward right | `x = fwd_x, y = −fwd_y` | +x | short | 16×16 | polled |
| 6 | S1 | I2C2 | PB0 | 0x36 | lateral left | `x = lat_x, y = +lat_y` | +y | short | 4×4 + baffle | EXTI PA15 |
| 7 | S2 | I2C2 | PB1 | 0x37 | lateral right | `x = lat_x, y = −lat_y` | −y | short | 4×4 + baffle | EXTI PA12 |

Mounting positions are parameters (§10, group GEO) `[MEASURE]` on the finished chassis. Ground sensor lateral offsets default to 0.15 m (inner) and 0.25 m (outer) from the centre line, `[DECIDE]` D-4.

#### 7.13.2 Initialisation (init only, blocking allowed)

1. All XSHUT low (GPIO init state); wait 2 ms.
2. For each sensor in ID order: XSHUT high; poll `VL53L1X_BootState` every 2 ms for up to 100 ms; `VL53L1X_SetI2CAddress(0x29 → addr)`; `VL53L1X_SensorInit`; `SetDistanceMode(short)`; `SetTimingBudgetInMs(vl_tb_ms)`; `SetInterMeasurementInMs(vl_imp_ms)`; `SetROI` + `SetROICenter` per table; for S1/S2 `SetInterruptPolarity(active low)`. A sensor that does not boot is marked absent; the next sensor continues.
3. Start ranging in ID order. F2 starts 10 ms after F1, so the two forward emitters are out of phase.

Defaults `vl_tb_ms = 20`, `vl_imp_ms = 20` (50 Hz). **[VERIFY]** in the ULD documentation (UM2510) whether the inter-measurement period may equal the timing budget; otherwise 24 ms.

ULD platform functions (`vl53l1_platform.c`) are implemented with blocking `HAL_I2C_Mem_Read/Write` and a 5 ms timeout. They are used **only** during initialisation (exception to REQ-EXE-09).

#### 7.13.3 Run-time acquisition (non-blocking)

Each bus has its own state machine, advanced by `vl_service()` in `S_FAST` and by the HAL I²C completion callbacks. One transaction is in flight per bus at any time.

| Step | Transaction | Registers `[VERIFY]` against the ULD source |
|---|---|---|
| 1. ready? | 1-byte read | `GPIO__TIO_HV_STATUS` 0x0031; ready when bit 0 equals the inverted interrupt polarity (as `VL53L1X_CheckForDataReady`) |
| 2. result | 17-byte read from `RESULT__RANGE_STATUS` 0x0089 | decode exactly as `VL53L1X_GetResult` (range status mapping table, distance, ambient, signal per SPAD, SPAD count) |
| 3. clear | 1-byte write 0x01 to `SYSTEM__INTERRUPT_CLEAR` 0x0086 | — |

- REQ-VL-02 Polled sensors are checked every `vl_poll_ms` (5 ms) round-robin on their bus. EXTI sensors skip step 1: the EXTI ISR stores `t_ready = sys_time_us()` and sets a flag.
- REQ-VL-03 **Timestamp** of a result = middle of its integration window: `t_meas = t_ready − vl_tb_ms/2`. For polled sensors `t_ready` = the time the ready bit was first seen minus `vl_poll_ms/2`.
- REQ-VL-04 Transaction timeout 5 ms → abort, bus recovery (de-init, SCL as GPIO, 9 clock pulses, STOP, re-init), retry. Three recoveries within 1 s → `F_VL_BUS` for that bus.
- REQ-VL-05 Per-sensor timeout: no valid result for `vl_timeout` (0.2 s) → the sensor is marked stale. The consumer decides the reaction (§11).
- REQ-VL-06 Output per sensor, published with a sequence counter:

```c
typedef struct {
    float    range_m;       /* raw distance                                    */
    uint8_t  status;        /* ULD range status (0 = valid)                    */
    float    signal_kcps;   /* signal rate per SPAD                            */
    float    ambient_kcps;
    uint32_t t_us;          /* t_meas, REQ-VL-03                               */
    uint16_t seq;
} vl_meas_t;
const vl_meas_t *vl_get(uint8_t id);
```

The ULD result contains no per-measurement sigma value. The spread feature of the classification (§7.14) is therefore computed as the rolling standard deviation of the range over the last `gnd_sd_n` samples (REQ-VL-07). This replaces the "sigma" of the concept.

### 7.14 `gnd` — ground classification and edge events (pure) `[PHASE 3]`

**Per sensor, at every new result:**

1. Height corrected for tilt: `h = range · cos φ · cos ϑ`.
2. Spread `σ_h` = standard deviation of the last `gnd_sd_n` (5) heights.
3. Raw class:

| Class | Condition (first match wins) |
|---|---|
| `UNKNOWN` | `status ≠ 0` and not "signal fail at long range", or sensor stale |
| `DROP` | `h > gnd_h0 + gnd_d_drop` (default h0 = 0.12 m, d_drop = 0.03 m) |
| `GRASS` | `σ_h ≥ gnd_sd_grass` (0.008 m) or `signal < gnd_sig_grass` |
| `EDGE_UP` | `h < gnd_h0 − gnd_d_up` (0.02 m) and `σ_h < gnd_sd_grass` |
| `PATH` | otherwise |

4. Debounce: the published class changes only after `gnd_n_deb` (2) consecutive samples of the new raw class. The event timestamp is that of the **first** of those samples.

All thresholds are `[MEASURE]` (outdoor test, noon and afternoon). The classifier is table-driven so the thresholds can be tuned live (REQ-GND-01).

**Per side (inner, outer):**

| Inner | Outer | Side state |
|---|---|---|
| PATH | PATH | `GND_FAR` — boundary further out than the outer footprint |
| PATH | not PATH | `GND_NOMINAL` — boundary between the footprints |
| not PATH (except UNKNOWN) | any | `GND_TOO_CLOSE` → **interlock** |
| UNKNOWN | any | `GND_UNKNOWN` |

```c
void gnd_init(gnd_t *g);
void gnd_on_meas(gnd_t *g, uint8_t id, const vl_meas_t *m, float roll, float pitch);
void gnd_step(gnd_t *g, uint32_t now_us);              /* staleness, side states    */
gnd_side_t gnd_side(const gnd_t *g, uint8_t side);
bool gnd_interlock(const gnd_t *g);                    /* any side TOO_CLOSE        */
bool gnd_pop_edge_event(gnd_t *g, gnd_edge_ev_t *ev);  /* {sensor id, t_us, from, to} */
```

- REQ-GND-02 **Interlock.** `gnd_interlock()` true → `msn` brakes immediately (`R_BRAKE`) in every mode except `BENCH` with `gnd_interlock_bench = 0`. The interlock is never disabled in `AUTO`, including during evasion.
- REQ-GND-03 Every published transition between `PATH` and {`EDGE_UP`, `DROP`, `GRASS`} creates an edge event (both directions). `UNKNOWN` transitions create none.
- REQ-GND-04 A stale inner sensor in `AUTO` → controlled stop (`F_GND_STALE`). Without it the interlock is blind.

### 7.15 `lat` — lateral landmarks (pure) `[PHASE 5]`

Consumes S1/S2 results, the encoder distance `s_enc` and the EKF pose history. Active only inside the along-track windows of landmarks that `map` lists for the current segment (§7.18).

**Face fit (`LM_FACE`).**
- Collect `(s_enc,i, r_i)` with `status = 0` and `lat_r_min ≤ r ≤ lat_r_max` (0.05–1.2 m) inside the window.
- At the window end, or when `lat_n_max` (400) samples are collected: least-squares line `r = a + b·s`. Accept if N ≥ `lat_n_min` (50), span ≥ `lat_span_min` (1.0 m), residual RMS ≤ `lat_rms_max` (0.03 m), and the EKF heading changed by < 0.5° over the window.
- Heading relative to the face: left sensor `δ = asin(−b)`, right sensor `δ = asin(b)` (CCW positive). Measured heading `θ_m = ψ_face + δ`, variance `σ_δ² = 12·rms² / (N·span²)`, floored at `(lat_sig_face_min)²` (0.05°)².
- Range at the last sample: `r_end = a + b·s_end`, variance rms².
- Output: one `LAT_HEADING` and one `LAT_FACE_RANGE` measurement to the EKF queue (REQ-LAT-01).

**Post notch (`LM_POST`).**
- Background `r_bg` = median of the last 10 samples outside a notch. A notch starts when `r < r_bg − lat_notch_depth` (0.15 m) for 2 samples and ends when it rises again.
- Flank positions `s_in`, `s_out`; centre `s_c = (s_in + s_out)/2`; `r_min` = minimum range inside.
- Accept if the notch width in s is within [0.5, 2.0]·(post diameter + spot width at `r_min`).
- Output: `LAT_POINT` with the landmark id, `t_c` (time of the sample closest to `s_c`), and the body-frame offset `q = r_s + (r_min + r_post)·u_s` (REQ-LAT-02).

```c
void lat_init(lat_t *l);
void lat_begin_segment(lat_t *l, const seg_t *sg, const lm_t *lm, uint16_t n_lm);
void lat_on_meas(lat_t *l, uint8_t id, const vl_meas_t *m, float s_enc_m);
bool lat_pop(lat_t *l, lat_out_t *out);   /* LAT_HEADING | LAT_FACE_RANGE | LAT_POINT */
```

- REQ-LAT-03 All landmark outputs are suppressed while `|y_off| > 0.1 m` (evasion active) or while the mission is not in `DRIVE`/`APPROACH`.

### 7.16 `fwd` — forward reflex and free space (pure)

**Reflex (REQ-FWD-01, `[PHASE 3]`).**
- Per forward sensor, the filtered range is the median of the last 3 valid samples. A sample is rejected as implausible if it differs from the previous one by more than `|v|·Δt + 0.3 m` (crosstalk from other vehicles' 940 nm emitters).
- Stopping distance:
  `d_stop = |v|·fwd_t_lat + v²/(2·fwd_a_brake) + fwd_margin` with defaults 0.1 s, 0.5 m/s², 0.15 m. At 0.3 m/s that gives 0.27 m.
- `REFLEX` is set when any filtered forward range < `d_stop` while moving forward. It clears only after both ranges are > `d_stop + 0.1 m` for 0.5 s.
- Speed governor: any filtered range < `fwd_slow_range` (1.0 m) → `v_limit = fwd_v_slow` (0.25 m/s).
- Range status "signal fail" (nothing in range) counts as free. Other invalid states count as unknown. Both forward sensors unknown or stale for > 0.2 s in `AUTO` → `F_FWD_STALE` (controlled stop).

**Free-space profile (REQ-FWD-02, `[PHASE 6]`).** Each forward sensor cycles its ROI centre through three zones (left, centre, right; ROI 8 columns × 16 rows). Per zone it keeps the newest range, ≤ 70 ms old for the reflex. Output: up to 6 obstacle points `(bearing, range)` in the body frame, used by `evd`. While the zones cycle, the reflex uses the minimum over all zones of a sensor.

```c
void  fwd_on_meas(fwd_t *f, uint8_t id, const vl_meas_t *m, uint8_t zone);
void  fwd_step(fwd_t *f, float v_mps, uint32_t now_us);
bool  fwd_reflex(const fwd_t *f);
float fwd_v_limit(const fwd_t *f);            /* +inf if no limit                   */
uint8_t fwd_obstacles(const fwd_t *f, fwd_pt_t *pts, uint8_t max);
```

### 7.17 `ekf` — state estimation (pure) `[PHASE 2]`

#### 7.17.1 State and conventions

`x = [x, y, θ, b_g, s]ᵀ`: position of the axle midpoint in the world frame (m), heading (rad, CCW from world x), bias of the projected yaw rate (rad/s), odometry scale (–, ≈ 1). The world frame is the course frame: origin and axes as defined by the course start pose (§7.18). Covariance `P` is 5 × 5 `float`, stored full and symmetrised after every operation (REQ-EKF-01).

```c
void ekf_init(ekf_t *e);                                    /* P0, x = course start  */
void ekf_reset_pose(ekf_t *e, float x, float y, float th);  /* keeps b_g, s          */
void ekf_predict(ekf_t *e, const ekf_odo_t *u);             /* §7.17.2               */
st_t ekf_update_zupt(ekf_t *e, float rate_mean_radps, float T_s);
st_t ekf_update_edge(ekf_t *e, const ekf_line_t *ln, float sx, float sy, const ekf_back_t *bk);
st_t ekf_update_face_range(ekf_t *e, const ekf_line_t *ln, float sx, float sy,
                           float ux, float uy, float r_m, float var, const ekf_back_t *bk);
st_t ekf_update_heading(ekf_t *e, float th_meas, float var);
st_t ekf_update_point(ekf_t *e, float lx, float ly, float qx, float qy, float var,
                      const ekf_back_t *bk);
void ekf_get(const ekf_t *e, ekf_out_t *out);   /* state, P, stats, flags            */
st_t ekf_pose_at(const ekf_t *e, uint32_t t_us, float *x, float *y, float *th);
```

`ekf_odo_t` holds `ds_l, ds_r` (m, after backlash), `dpsi` (rad, from `imu_yaw_integral_at`, or NAN without IMU), `dt` (s, from encoder timestamps), `t_us`, and `still` (standstill flag, §7.17.4).

`ekf_back_t` is the *back-shift* for delayed measurements: `{dx, dy, dth}`, the estimated motion between the measurement time and now, taken from the EKF's own pose history (`ekf_pose_at`, ring of 256 entries = 1.28 s). The measurement is then evaluated at the past pose `p − Δp`, `θ − Δθ`, and Δ is treated as exact (REQ-EKF-02). Measurements older than 1.0 s are discarded.

#### 7.17.2 Prediction

```
Δs_e = (ds_l + ds_r) / 2
Δθ   = still ? 0 : (dpsi − b_g·dt)                          (IMU mode)
Δθ   = still ? 0 : (ds_r − ds_l) / b                        (encoder mode, §7.17.7)
Δs   = still ? 0 : s·Δs_e
φ    = θ + Δθ/2
x   += Δs·cos φ          y += Δs·sin φ          θ = wrap(θ + Δθ)          b_g, s unchanged
```

Jacobian `F` (IMU mode, `c = cos φ`, `n = sin φ`):

```
      ⎡ 1  0  −Δs·n   +½Δs·n·dt   Δs_e·c ⎤
      ⎢ 0  1  +Δs·c   −½Δs·c·dt   Δs_e·n ⎥
F  =  ⎢ 0  0    1        −dt        0    ⎥
      ⎢ 0  0    0         1         0    ⎥
      ⎣ 0  0    0         0         1    ⎦
```

`P ← F P Fᵀ + Q`, with Q built from (REQ-EKF-03):

| Source | Contribution |
|---|---|
| distance noise | `σ²_ds = ekf_kd·\|Δs_e\|·(slip ? ekf_slip_infl : 1)`, mapped with `g = [s·c, s·n, 0, 0, 0]ᵀ`: `Q += g σ²_ds gᵀ` |
| gyro white noise | `Q_θθ += ekf_gyro_nd²·dt` (0 when `still`) |
| gyro scale-factor error | `Q_θθ += (ekf_gyro_sf·Δθ)²` |
| bias random walk | `Q_bb += ekf_q_bg·dt` |
| scale random walk | `Q_ss += ekf_q_s·dt` |

Defaults: `ekf_kd` = 1.0·10⁻⁴ m, `ekf_slip_infl` = 100, `ekf_gyro_nd` = 1.0·10⁻⁴ rad/s/√Hz, `ekf_gyro_sf` = 0.003, `ekf_q_bg` = 1.0·10⁻¹⁰ (rad/s)²/s, `ekf_q_s` = 1.0·10⁻⁹ /s. All `[MEASURE]`: Allan variance of the chosen IMU, repeated `D5000` runs, 10-turn spins.

**Slip detector (REQ-EKF-04).** In IMU mode: `r_slip = (ds_r − ds_l)/(b·dt) − (dpsi/dt − b_g)`. If `|r_slip| > ekf_slip_thr` (0.15 rad/s) for ≥ 3 consecutive steps, `slip` = true for those steps and while it persists. Each slip episode is counted and logged.

#### 7.17.3 Measurement updates

Common procedure for every update (REQ-EKF-05):

1. Innovation `ν = z − h(x̂)` (angles through `wrap_pi`), `S = H P Hᵀ + R`.
2. Gate: `d² = νᵀ S⁻¹ ν`; reject if `d² > ekf_gate1` (9.0, scalar) or `> ekf_gate2` (9.21, 2-D). Return `ST_REJECT` and count it per type.
3. `K = P Hᵀ S⁻¹`; `x̂ += K ν`; `θ` wrapped.
4. Joseph form `P = (I − K H) P (I − K H)ᵀ + K R Kᵀ`, then `P = ½(P + Pᵀ)`.
5. Health: all elements finite, diagonal > 0, `P_xx, P_yy < 25 m²`, `P_θθ < 1 rad²`. Otherwise `F_EKF` (§11).
6. Accumulate the normalised innovation squared `d²` per type for consistency checks (§14).

2-D updates gate on the joint `d²`, then apply both rows sequentially. R is diagonal.

With `R(θ)` the 2-D rotation, `R'(θ) = dR/dθ`, a body vector `r = (r_x, r_y)`:
`R(θ) r = (c r_x − n r_y, n r_x + c r_y)` and `R'(θ) r = (−n r_x − c r_y, c r_x − n r_y)` with `c = cos θ`, `n = sin θ`.

| Update | `z`, `h(x)` | Non-zero `H` | `R` |
|---|---|---|---|
| **ZUPT** | `z` = mean of `dpsi/dt` over the standstill window of length T; `h = b_g` | `H_bg = 1` | `ekf_gyro_nd²/T + ekf_zupt_floor²` (floor 3.5·10⁻⁵ rad/s) |
| **Edge** | line `{n, d}`: `nᵀq = d`, n points into the path. Footprint `f = p + R(θ)r_s`; `h = nᵀf − d`; `z = 0` | `H_p = nᵀ`, `H_θ = nᵀR'(θ)r_s` | `ekf_sig_edge`² (0.02 m)² |
| **Face range** | sensor ray `f + r·R(θ)u_s`; `g = nᵀR(θ)u_s`, `D = d − nᵀf`; `h = D/g`; `z = r_meas` | `H_p = −nᵀ/g`, `H_θ = (−nᵀR'r_s·g − D·nᵀR'u_s)/g²` | from `lat` (rms²) |
| **Heading** | `h = θ`, `z = θ_m` | `H_θ = 1` | from `lat` (`σ_δ²`) |
| **Point (post)** | `h = p + R(θ)q`, `q` = body offset of the post centre; `z = (l_x, l_y)` surveyed | `H = [I₂ \| R'(θ)q \| 0 \| 0]` | `ekf_sig_post`² (0.03 m)² + survey σ² per axis |

- REQ-EKF-06 Face range is rejected without update if `|g| < 0.3` (ray too oblique).
- REQ-EKF-07 Edge and face updates use the boundary line of the **current** segment on the sensor's side (from `map`). They are skipped within `ekf_edge_skip` (0.25 m) of the segment start or end, and in the first 0.25 m after a TURN (casters still aligning).
- REQ-EKF-08 Landmark, face and heading updates are suppressed while `|y_off| > 0.1 m`. Edge updates remain active.

#### 7.17.4 Standstill and ZUPT

- REQ-EKF-09 `still` = no count change on either wheel for ≥ `ekf_still_s` (0.2 s) **and** the drive layer is in `HOLD`, `DISABLED` or `IDLE` (no motion commanded).
- While `still`: Δθ = 0 and no θ noise is added (the vehicle cannot rotate). The ZUPT accumulator sums `dpsi` and `dt`. Every `ekf_zupt_T` (1.0 s) one ZUPT update is applied with the mean over that period, and the accumulator restarts. Leaving `still` discards a partial window shorter than 0.5 s.

#### 7.17.5 Initial covariance

`P₀ = diag((0.01 m)², (0.01 m)², (0.5°)², (0.5 °/s)², (0.005)²)`, all parameters. `INIT` is complete when `√P_bg < ekf_bg_conv` (0.02 °/s = 3.5·10⁻⁴ rad/s) and at least `msn_init_min_s` (3 s) of standstill have passed.

#### 7.17.6 Outputs

`ekf_out_t`: state, `P` diagonal, `P_xy`, the cross-track σ with respect to the current segment (`σ_⊥ = √(n_segᵀ P_pp n_seg)`), counters (accepted / rejected per type, slip episodes), mean NIS per type, and flags (`EKF_STILL`, `EKF_SLIP`, `EKF_DEGRADED`).

#### 7.17.7 Encoder-heading mode (`FEAT_IMU = 0`)

Δθ from the wheel difference with `b = odo_wheelbase`. `b_g` is frozen (its row and column of F are identity, no ZUPT). `Q_θθ += ekf_q_th_enc·|ds_r − ds_l|` with default 1·10⁻⁴ rad²/m. The slip detector is off. This mode lets path following and mission logic be developed before the IMU arrives (REQ-EKF-10).

### 7.18 `map` — course representation (pure)

#### 7.18.1 Data structures (`map.h`)

```c
typedef enum { SEG_LINE, SEG_ARC, SEG_TURN, SEG_WAYPOINT, SEG_PAUSE, SEG_FINISH } seg_type_t;
typedef enum { BND_NONE, BND_GRASS, BND_EDGING, BND_KERB_DROP, BND_STEP_DROP, BND_WALL } bnd_t;
enum { SEG_F_STOP_AT_END = 1, SEG_F_NO_EVADE = 2, SEG_F_RAMP = 4, SEG_F_NO_LANDMARKS = 8 };

typedef struct {
    seg_type_t type;
    union {
        struct { float x0, y0, x1, y1; }                          line;  /* m          */
        struct { float cx, cy, r, phi0, phi1; int8_t dir; }       arc;   /* dir +1 CCW */
        struct { float theta; }                                   turn;  /* absolute   */
        struct { uint8_t id; float hold_s; }                      wp;
        struct { float t_s; }                                     pause;
    } g;
    float    v_max;                 /* m/s, ≤ drv limit                              */
    float    w_left, w_right;       /* path line → boundary, m (corridor)            */
    bnd_t    bnd_left, bnd_right;
    int8_t   ref_side;              /* +1 left, −1 right, 0 none (edge tracked)      */
    uint8_t  flags;
    uint16_t lm_first, lm_count;    /* landmarks valid on this segment               */
} seg_t;

typedef enum { LM_FACE, LM_POST } lm_type_t;
typedef struct {
    lm_type_t type;
    int8_t    side;                 /* sensor side that sees it: +1 S1, −1 S2        */
    float     s_from, s_to;         /* along-segment window, m                       */
    float     px, py;               /* post centre, or a point on the face           */
    float     tx, ty;               /* face direction, unit (travel direction)       */
    float     r_post;               /* post radius, m                                */
    float     sigma;                /* survey σ, m                                   */
} lm_t;

typedef struct {
    const char *name; const char *gen_hash;
    float start_x, start_y, start_theta;
    const seg_t *seg; uint16_t n_seg;
    const lm_t  *lm;  uint16_t n_lm;
} course_t;

const course_t *map_course(void);
st_t map_validate(const course_t *c);                         /* §7.18.4 */
void map_seg_frame(const seg_t *sg, float x, float y, float th,
                   map_frame_t *out);   /* s, L, e_y, e_psi, kappa, t̂, n̂ for LINE/ARC */
st_t map_boundary(const seg_t *sg, int8_t side, ekf_line_t *out);  /* edge line, n into path */
```

#### 7.18.2 Geometry

- **LINE** from `p₀` to `p₁`: `t̂ = (p₁ − p₀)/L`, left normal `n̂ = (−t_y, t_x)`. `s = (p − p₀)·t̂`, `e_y = (p − p₀)·n̂` (left positive), `ψ_path = atan2(t_y, t_x)`, `e_ψ = wrap(θ − ψ_path)`, κ = 0.
- **ARC** (dir = +1 CCW, −1 CW), centre c, radius R: `ρ = |p − c|`, `φ = atan2(p − c)`; `ψ_path = φ + dir·π/2`; `e_y = dir·(R − ρ)`; `κ = dir/R`; `s = R·dir·wrap(φ − φ₀)` (unwrapped across ±π by tracking).
- **Boundary lines** for LINE: left boundary through `p₀ + w_left·n̂` with inward normal `−n̂`; right boundary through `p₀ − w_right·n̂` with inward normal `+n̂`; `d = nᵀ·(point)`. ARC boundaries are not used for edge updates in v1 (REQ-MAP-01).

#### 7.18.3 Path placement rule

When a segment has `ref_side ≠ 0`, the generator places the path line at the derived distance ½(`gnd_y_in` + `gnd_y_out`) = 0.20 m from that boundary (the generator reads both values from the YAML `geometry` block, which must match the parameters; `map_validate` prints a warning if they differ), so the nominal ground state is `GND_NOMINAL` (boundary between inner and outer footprint). The other side's corridor width follows from the survey. REQ-MAP-02.

#### 7.18.4 Validation (`map_validate`, at boot)

REQ-MAP-03, all of:
- consecutive geometric segments connect within 1 mm, and a `TURN` sits between two LINEs whose headings differ by the turn angle within 0.1°;
- every `WAYPOINT` and `TURN` is preceded by a segment with `STOP_AT_END`;
- the last segment is `FINISH`, preceded by a segment ending at the start point within 1 mm;
- `0 < v_max ≤ drv_v_max`; `w_left, w_right ≥ 0.25 m`; landmark windows lie within their segment.

A failed validation makes `AUTO` unavailable and prints the first violated rule.

#### 7.18.5 Course generator (`tools/course_gen.py`)

Input YAML, output `Core/Src/course_<name>.c`. The C file is generated, never edited by hand (REQ-MAP-04). Schema:

```yaml
name: rect10
start: {x: 0.0, y: 0.0, theta_deg: 0.0}
defaults: {v_max: 0.30, w_left: 0.60, w_right: 0.60,
           bnd_left: none, bnd_right: none, ref_side: none}
legs:                                   # executed in order
  - line:     {to: [10.0, 0.0]}
  - waypoint: {id: 1, hold_s: 10.5}
  - turn:     {to_deg: 90}
  - line:     {to: [10.0, 10.0], ref_side: right, bnd_right: grass, w_right: 0.20}
  - waypoint: {id: 2, hold_s: 10.5}
  # …
  - finish:   {}
landmarks:
  - {type: face, leg: 4, side: left, from: 2.0, to: 5.0, p: [9.2, 3.0], dir_deg: 90, sigma: 0.02}
  - {type: post, leg: 4, side: right, from: 6.0, to: 8.0, p: [10.6, 7.1], r: 0.05, sigma: 0.03}
```

The generator computes segment geometry, sets `STOP_AT_END` automatically before `waypoint`, `turn` and `finish`, applies the placement rule, runs the same validation as `map_validate`, and writes the git hash of the YAML into `gen_hash`. **The Jardín course cannot be generated before the survey** (open items F-1 … F-14). Until then `course_rect10.yaml` is the only course (§15).

### 7.19 `nav` — guidance (pure) `[PHASE 3]`

```c
void nav_init(nav_t *n);
void nav_begin_segment(nav_t *n, const seg_t *sg, const ekf_out_t *est, float s_enc);
nav_cmd_t nav_follow(nav_t *n, const ekf_out_t *est, float v_limit, float y_off, float dt);
nav_cmd_t nav_turn(nav_t *n, const ekf_out_t *est, float w_meas, float dt);
nav_cmd_t nav_final(nav_t *n, const ekf_out_t *est, float s_enc);
const nav_dbg_t *nav_debug(const nav_t *n);   /* s, L, e_y, e_psi, v_ref, w_ref        */
```

`nav_cmd_t` is one of `{VEL(v, ω), MOVE_FROM(D, v0), STOP, NONE}` plus `done`, `aligned`, `s_rem`.

**Line and arc following (REQ-NAV-01).**

```
speed:   v_ref = min( v_max_seg, v_limit,
                      √(v_end² + 2·nav_a_dec·max(L − s, 0)),
                      v_prev + nav_a_acc·dt )
         v_end = 0 if STOP_AT_END, else v_max of the next segment
gains:   k_ψ = 2·ζ·ω_n          k_y = ω_n² / max(|v_ref|, nav_v_min)
turn:    ω = κ·v_ref − k_ψ·e_ψ − k_y·(e_y − y_off)
limits:  |ω| ≤ nav_w_max;  drv applies the wheel-speed limit (REQ-DRV-03)
```

Defaults: ω_n = 1.5 rad/s, ζ = 0.7, `nav_v_min` = 0.05 m/s, `nav_a_acc` = 0.3 m/s², `nav_a_dec` = 0.3 m/s², `nav_w_max` = 1.0 rad/s. ω_n stays a factor ≥ 4 below the speed-loop crossover (6.8 rad/s), so the cascade separates (technical reference §6.6). The wheel position loop (2 /s) does not limit the path bandwidth: the reference speed is fed forward, and the position loop only trims the integral of the speed error. `t_nav_line` and the SIL run verify the damping with the real cascade in the loop.

**Final stop (REQ-NAV-02).** With `STOP_AT_END`: when `L − s ≤ nav_final_dist` (0.30 m), `|e_y| < 0.05 m` and `|e_ψ| < 3°`, `nav_follow` returns `MOVE_FROM(D = L − s, v0 = current v_ref)` once. The distance target is now frozen in the **encoder** frame. EKF corrections that arrive during the last 0.30 m no longer move the stop point, so the stop is jump-free and never reverses (REQ-DRV-01). This needs the profile variant `prof_plan_v0(p, D, v0, v_nom, a_acc, a_dec)`: start at v0 ≥ 0, optional cruise, stop at D. If `v0²/(2·a_dec) > D`, it raises `a_dec` up to `drv_a_stop` and otherwise stops as early as possible. The residual `L − s` from the EKF after `DRV_EV_DONE` is reported as the waypoint miss estimate. If the alignment condition is not met at 0.30 m, follow continues and the final stop starts as soon as it is met, or at `L − s ≤ 0.10 m` regardless.

**Turn (REQ-NAV-03).** Target heading `θ_t` absolute (from the TURN segment). At the start: `Δ = wrap(θ_t − θ̂)`, angle profile `prof_plan(Δ, nav_turn_w, nav_turn_ta, nav_turn_ta)` (0.8 rad/s, 0.4 s). Each tick:
`ω = θ̇_prof + nav_turn_k·wrap(θ₀ + θ_prof − θ̂)` (k = 2.0 /s), `v = 0`.
Done when `|wrap(θ_t − θ̂)| < nav_turn_tol` (0.2°) and `|ω_gyro| < 0.02 rad/s` for 0.2 s, or 2 s after the profile ended (`done` with the residual reported). Inside 0.5° no reversal is commanded: the residual is accepted rather than paying gearbox backlash. Without IMU: `MOVE`-type `drv_turn_enc(Δ)`.

### 7.20 `evd` — 1-D evasion (pure) `[PHASE 6]`

```c
void evd_init(evd_t *v);
evd_state_t evd_plan(evd_t *v, const fwd_pt_t *pts, uint8_t n, const ekf_out_t *est,
                     const seg_t *sg, const map_frame_t *fr);
float evd_step(evd_t *v, const map_frame_t *fr, gnd_side_t left, gnd_side_t right,
               float dt);                                 /* returns y_off, m         */
bool  evd_active(const evd_t *v);
```

**Plan (REQ-EVD-01).**
1. Transform obstacle points to the path frame `(s_o, y_o)` using the EKF pose.
2. Obstacle band: `[min y_o − m, max y_o + m]` with `m = veh_w/2 + evd_margin` (0.175 + 0.10 m), extended by `evd_obs_extra` (0.10 m) on each side because the sensor sees only the near face.
3. Corridor: `[−w_right + veh_w/2 + evd_edge_margin, +w_left − veh_w/2 − evd_edge_margin]` (`evd_edge_margin` 0.05 m).
4. Free set = corridor minus band (0, 1 or 2 intervals). `y_cmd` = the free value closest to 0. No free value → `EVD_BLOCKED`.
5. `s_clear = max s_o + evd_obs_depth (0.5 m) + veh_rear (0.20 m) + evd_margin`.

**Execution (REQ-EVD-02).** `y_off` follows `y_cmd` with a trapezoidal lateral profile: outward 0.15 m/s and 0.3 m/s², return 0.08 m/s. When `s > s_clear`, `y_cmd = 0`. There is no separate return manoeuvre: the same rule with a free nominal offset brings the vehicle back. If, while moving towards a side, that side's ground state becomes `GND_NOMINAL` or worse (outer footprint on the boundary), `y_off` is frozen at its current value: **the search ends at the measured edge, not the estimated one.** If the frozen offset does not clear the band, return `EVD_BLOCKED`.

- REQ-EVD-03 No evasion on segments flagged `NO_EVADE` or within `evd_wp_guard` (1.5 m) of a waypoint. Waypoint accuracy beats rerouting there.
- REQ-EVD-04 `evd_active()` is true while `|y_off| > 0.01 m` or `s < s_clear`.

### 7.21 `msn` — modes and mission state machine (pure)

#### 7.21.1 Interface

```c
void msn_init(msn_t *m);
void msn_request(msn_t *m, msn_req_t req);       /* from con/proto/B1: IDLE, BENCH,
                                                    MANUAL, AUTO, STOP, CLEAR       */
void msn_manual(msn_t *m, float v, float w, uint32_t t_rx_ms);  /* DRIVE frame      */
void msn_step(msn_t *m, const msn_in_t *in, msn_out_t *out);    /* S_CTRL           */
msn_mode_t  msn_mode(const msn_t *m);
msn_state_t msn_state(const msn_t *m);
```

`msn_in_t` bundles everything the decision needs, copied once per tick: EKF output, `gnd` side states and interlock, `fwd` reflex and speed limit, `evd` status, `drv` status and events, battery, fault reaction, sensor health, `s_enc`, time. `msn_out_t` is the motion command for `drv` (via `nav` where applicable), `y_off`, the current segment index and the events to log. The state machine is a transition table, not nested `if` cascades (REQ-MSN-01). Every state has entry, cycle and exit actions.

#### 7.21.2 Mode transitions

| From | Request / condition | To | Action |
|---|---|---|---|
| any | `STOP` | (same mode) | `drv_stop()`; in AUTO → `AUTO_STOPPED` (§7.21.3) |
| any | fault reaction ≥ `R_STOP` | `FAULT` | `drv_stop()` or `drv_brake()` per reaction, then `drv_disable()` |
| `IDLE` | `BENCH` (any console motion command) | `BENCH` | `drv_hold()` |
| `IDLE` | `MANUAL` | `MANUAL` | `drv_hold()` |
| `IDLE` | `AUTO` (B1, `RUN`, `MODE AUTO`) and **start conditions** | `AUTO` (`AUTO_INIT`) | reset EKF pose to course start |
| `BENCH`/`MANUAL`/`AUTO` | `IDLE` | `IDLE` | `drv_stop()` then `drv_disable()` |
| `FAULT` | `CLEAR` and no active fault with reaction ≥ `R_STOP` | `IDLE` | — |

**AUTO start conditions (REQ-MSN-02):** course valid; no active fault; battery present and above `bat_warn_v` (or `msn_allow_bench_supply` = 1); IMU alive (if `FEAT_IMU`); G1–G4, F1, F2 fresh (if `FEAT_VL`, unless `msn_allow_nosensors` = 1); vehicle stationary. A refused start prints and logs the first unmet condition.

**MANUAL (REQ-MSN-03).** A `DRIVE(v, ω)` frame or console `M` sets the command, clamped to `msn_man_v_max` (0.40 m/s) and `msn_man_w_max` (1.5 rad/s), and `drv_vel()` runs with it. No valid command for `msn_man_timeout` (0.5 s) → `drv_stop()` and event `EV_MAN_TIMEOUT`. The next valid command resumes. The reflex blocks only forward motion (v > 0 → 0); turning and reversing remain possible. The ground interlock brakes and allows only commands that move away (v ≤ 0 or ω away from the tripped side).

#### 7.21.3 AUTO states

| State | Entry | Cycle | Exits (first match wins) |
|---|---|---|---|
| `AUTO_INIT` | `drv_hold()`; start timer | wait (ZUPT runs in the EKF) | converged (§7.17.5) → `AUTO_SEG`; `t > msn_init_max_s` (30 s) → `FAULT(F_INIT)` |
| `AUTO_SEG` | `nav_begin_segment()`; dispatch on segment type | — | LINE/ARC → `AUTO_DRIVE`; TURN → `AUTO_PRETURN`; WAYPOINT → `AUTO_HOLD`; PAUSE → `AUTO_PAUSE`; FINISH → `AUTO_FINISHED` |
| `AUTO_DRIVE` | — | `nav_follow()` with `v_limit = min(fwd, degraded)`, `y_off` from `evd` | interlock → `AUTO_RECOVER`; reflex → `AUTO_OBST_STOP`; `MOVE_FROM` issued → `AUTO_FINAL`; segment end without stop → next segment (`AUTO_SEG`) |
| `AUTO_FINAL` | `drv_move_from(D, v0)` | monitor | `DRV_EV_DONE`/`TIMEOUT` → next segment; reflex → `AUTO_OBST_STOP` (on resume, re-plan the final with the new remaining distance); interlock → `AUTO_RECOVER` |
| `AUTO_PRETURN` | `drv_hold()` | wait for EKF `still` + `msn_preturn_s` (1.0 s), so a ZUPT precedes every turn | done → `AUTO_TURN` |
| `AUTO_TURN` | — | `nav_turn()` | `done` → next segment; interlock → `AUTO_RECOVER` |
| `AUTO_HOLD` | `drv_hold()`; event `EV_WP_ARRIVED(id, x̂, ŷ, σ)` | start the hold timer when EKF `still` is first true | `t_hold ≥ hold_s` (10.5 s) → event `EV_WP_DONE` → next segment |
| `AUTO_PAUSE` | `drv_hold()` | timer | elapsed → next segment |
| `AUTO_OBST_STOP` | `drv_brake()`; event | wait for standstill | still → `AUTO_OBST_WAIT` |
| `AUTO_OBST_WAIT` | timer | `evd_plan()` each tick | reflex clear for 0.5 s → back to the interrupted state (`DRIVE`/`FINAL`); `t ≥ msn_obst_wait_s` (3 s) and plan ≠ BLOCKED and segment allows evasion → `AUTO_EVADE`; otherwise keep waiting (event every 10 s) |
| `AUTO_EVADE` | — | as `AUTO_DRIVE`, with `y_off` from `evd_step()` and `v_limit = fwd_v_slow` | reflex → `AUTO_OBST_STOP`; interlock → `AUTO_RECOVER`; `evd_active()` false → `AUTO_DRIVE` |
| `AUTO_RECOVER` | `drv_brake()`; count++ | 1. wait still; 2. `drv_move(−msn_rec_back_m)` (0.10 m); 3. set lateral bias `msn_rec_bias_m` (0.05 m) away from the tripped side for the next 2 m | done → back to `AUTO_DRIVE`; interlock persists after backing → `FAULT(F_GND_INTERLOCK)`; count > 3 on one segment → `FAULT(F_GND_INTERLOCK)` |
| `AUTO_STOPPED` | `drv_stop()` | wait | `AUTO` request → resume the interrupted state; `IDLE` → `IDLE` |
| `AUTO_FINISHED` | `drv_hold()`; event `EV_FINISHED(total time)` | — | after 2 s → mode `IDLE` |

- REQ-MSN-04 **Degraded flag.** If no edge, face or landmark update was accepted for `msn_noref_dist` (30 m) **and** the EKF cross-track σ exceeds `msn_sig_lat_max` (0.30 m), the speed limit drops to `msn_v_degraded` (0.20 m/s) until the next accepted reference. No state change.
- REQ-MSN-05 **Safety precedence** in every AUTO state, checked before the state's own logic: fault reaction → ground interlock → reflex → state logic.
- REQ-MSN-06 Every state change emits `EV_STATE(old, new)`.

### 7.22 `com_uart` — serial ports

```c
typedef enum { COM_USB = 0, COM_RADIO = 1 } com_port_t;
void   com_uart_init(UART_HandleTypeDef *h2, UART_HandleTypeDef *h1);
void   com_uart_poll(void);                         /* S_FAST: move RX bytes to parsers */
size_t com_write(com_port_t p, const uint8_t *d, size_t n);   /* non-blocking, drops   */
bool   com_tx_idle(com_port_t p);
com_stats_t com_stats(com_port_t p);                /* rx bytes, overruns, drops, errors */
```

| ID | Requirement |
|---|---|
| REQ-COM-02 | **TX double buffer.** Each port has two TX buffers. Writes go into the *filling* buffer; when the hardware is idle, the buffers swap and the full one is transmitted (`HAL_UART_Transmit_IT` on USART2, `_DMA` on USART1). The buffer being transmitted is never written. Reason: HAL sets `gState = READY` when the last byte reaches the data register, one character time *before* it leaves the shift register (reproduced on the bench). |
| REQ-COM-03 | `com_write` never blocks. If the filling buffer cannot take all bytes, the write is dropped **whole** (no partial frames) and counted. |
| REQ-COM-04 | **RX USART2**: 1-byte `HAL_UART_Receive_IT`, re-armed in `HAL_UART_RxCpltCallback`, into a 256-byte SPSC ring. **RX USART1** `[PHASE 4]`: DMA circular into 256 B, IDLE-line interrupt; `com_uart_poll` reads up to the DMA write index (NDTR). |
| REQ-COM-05 | `HAL_UART_ErrorCallback` (overrun, framing, noise) clears the error, re-arms reception, and counts the error. Reception must never stay disarmed (a known failure of the first HC-12 version). |
| REQ-COM-06 | Each port has a protocol mode `TEXT` or `BINARY` (§8.1). The mode decides which parser the received bytes go to. |

### 7.23 `con` — text console

Line-based, terminated by CR or LF, max 48 characters. Echo `> <line>` before execution, so a corrupted radio command is visible. Command reference in §9.

- REQ-CON-01 The console runs in the main loop (`con_poll`), never in an ISR. Long outputs (`k`, `L`, help) are emitted in chunks of ≤ 200 bytes per main-loop pass, so TX buffers never overflow.
- REQ-CON-02 Each port has its own line buffer. The current firmware shares one buffer between both ports, which garbles concurrent input.
- REQ-CON-03 Every received byte on a port counts as a sign of life for the open-loop dead-man on that port (2 s, as today).
- REQ-CON-04 Unknown commands answer `ERR unknown, ? for help`; range errors answer `ERR range <min>..<max>`; mode errors `ERR state <mode>`.

### 7.24 `proto` — binary protocol codec (pure) `[PHASE 4]`

Encoder/decoder for §8: COBS framing, CRC-16, header, message structs with explicit little-endian serialisation (no `memcpy` of structs; padding and endianness are explicit, REQ-PRO-01).

```c
size_t proto_encode(uint8_t type, uint8_t seq, const uint8_t *payload, size_t n,
                    uint8_t *out, size_t out_max);         /* returns frame length   */
void   proto_rx_byte(proto_rx_t *rx, uint8_t b);           /* feeds the decoder      */
bool   proto_rx_frame(proto_rx_t *rx, proto_frame_t *f);   /* complete, CRC-checked  */
uint16_t crc16_ccitt(const uint8_t *d, size_t n);          /* CCITT-FALSE            */
/* one pack/unpack function per message in §8.4 */
```

### 7.25 `tlm` — telemetry, events, capture

- `tlm_fast()` / `tlm_slow()` assemble the frames of §8.4 from the published module outputs and send them on every port in `BINARY` mode.
- In `TEXT` mode with auto-print on (`a`), every 250 ms a human-readable status line goes to that port (current behaviour, extended by mode, state and pose).
- **Events.** `sys_event` entries are sent as `EVENT` frames immediately on binary ports and printed on text ports.
- **Capture** (`FEAT_CAPTURE`, REQ-TLM-01). A static buffer of 48 KB. `CAPTURE(set, decim, duration)` starts recording one of the record sets below at the slot rate divided by `decim`. Recording stops when the buffer is full or the duration has passed. `CAP_GET` streams the buffer as `CAP_DATA` frames (index, up to 4 records per frame). Capture continues across mode changes and stops on `FAULT`, so the run-up to a fault is preserved.

| Set | Rate | Record (little-endian, packed) | Size |
|---|---|---|---|
| 0 `WHEEL` | `S_CTRL` (50 Hz) | `u32 t_us; i16 v_ref_L, v_L, v_ref_R, v_R [mm/s]; i16 e_x_L, e_x_R [0.1 mm]; i16 duty_L, duty_R [1e-4]; i16 integ_L, integ_R [1e-4]` | 24 B → 2048 rec = 41 s |
| 1 `EST` | `S_EST` (200 Hz) | `u32 t_us; i32 x, y [0.1 mm]; i16 th [1e-4 rad]; i16 bg [1e-6 rad/s]; i16 ds_L, ds_R [µm]; i16 dpsi [1e-5 rad]; u8 flags; u8 reserved` | 24 B → 2048 rec = 10 s |
| 2 `NAV` | `S_CTRL` | `u32 t_us; i16 s [cm]; i16 e_y [0.1 mm]; i16 e_psi [1e-4 rad]; i16 v_ref [mm/s]; i16 w_ref [mrad/s]; i16 y_off [mm]; u8 seg, state; u16 gnd (4 × 4-bit class); u8 reserved[4]` | 24 B |
| 3 `VL` | per new result | `u32 t_us; u8 id, status; u16 range [mm]; u16 signal [kcps/8]; u16 ambient; i16 h [0.1 mm]; u8 class; u8 pad` | 16 B → 3072 rec |

### 7.26 `app` — composition

`app.c` owns all contexts, implements `app_init()` (§5.8), `app_loop()` (§5.4) and the slot functions, and contains the only code that moves data between modules (REQ-APP-01). It is the only place where hardware modules and pure modules meet. `main.c` contains exactly four user hooks (§16.2).

### 7.27 Calibration firmware coexistence

- REQ-CAL-01 `APP_CALIB = 1` compiles the existing `calib.c` unchanged and calls `Calib_Init`/`Calib_Loop` from the same hook position as today. `app.c` is not initialised in this build.
- REQ-CAL-02 Because the `.ioc` now runs TIM6 at 1 kHz, `main.c` (USER CODE 2, `APP_CALIB` branch) writes `htim6.Instance->ARR = 19999` before `Calib_Init`. That gives 50 Hz, the rate the calibration firmware was verified at. Its timing uses the DWT, but its control ticks are counted in TIM6 periods.
- REQ-CAL-03 Results of the calibration (`calib_result.h` from `calib_host.py eval`) map onto parameters as listed in Appendix 19.2, so they can be entered with `P` commands or pasted as defaults.

---

## 8. Binary protocol `[PHASE 4]`

The protocol is the contract between the firmware and the ground station. Both transports (USB VCP on USART2, radio on USART1) carry the same frames. The radio module is a transparent byte pipe, so the protocol does not change if the HC-12 is replaced (D-2 of the technical reference).

### 8.1 Port modes

| Port | Default mode | Switch to binary | Switch to text |
|---|---|---|---|
| USART2 (USB) | `TEXT` | console command `BIN` | frame `MODE_TEXT`, or the ASCII line `#TEXT` followed by CR/LF |
| USART1 (radio) | `TEXT` until phase 4, then `com1_mode` (default `BINARY`) | as above | as above |

The binary receiver additionally scans the raw byte stream for the literal `#TEXT` + CR/LF, so an operator can always get the text console back (REQ-PRO-02).

### 8.2 Frame format

```
on the wire:   COBS( vid | type | seq | payload[0..96] | crc_lo | crc_hi )  0x00
```

| Field | Size | Meaning |
|---|---|---|
| `vid` | 1 | vehicle ID (`com_vid`, default 0x42). Frames with another ID are dropped, except `STOP` with `vid` 0xFF (broadcast emergency stop) |
| `type` | 1 | message type (§8.4) |
| `seq` | 1 | sender's frame counter, wraps at 255; used for loss statistics and duplicate suppression |
| `payload` | 0–96 | message-specific, little-endian, packed, no padding |
| `crc` | 2 | CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection, xorout 0) over `vid … payload`, little-endian. Check value: `"123456789"` → 0x29B1 |

COBS (Consistent Overhead Byte Stuffing) removes all zero bytes, so 0x00 delimits frames unambiguously. Maximum frame: 101 bytes before encoding, ≤ 103 after, plus the delimiter. The decoder drops frames that exceed this length, fail COBS decoding, or fail the CRC, and counts each case (REQ-PRO-03). Floats are IEEE-754 binary32, little-endian. All other quantities are scaled integers with the units given below.

### 8.3 Radio timing (half-duplex)

The HC-12 cannot receive while it transmits. The **vehicle is timing master** (REQ-PRO-04):

```
 t = 0 ms            ≤ 55 ms                          100 ms
 |── vehicle TX ─────|── PC may send one frame ──────|── next period
   [EVENT/ACK…] [TELEM]   start ≥ 5 ms after the delimiter it received; ≤ 30 ms airtime
```

- Period `tlm_period_ms` = 100 ms. In each period the vehicle sends at most `tlm_air_max_ms` = 55 ms of frames, in this priority: pending `ACK`/`PARAM_VAL`, pending `EVENT`s, then `TELEM_FAST` (`TELEM_SLOW` in every 10th period). A telemetry frame that does not fit is skipped, not delayed. Events that do not fit wait for the next period.
- The PC sends only after it has received a vehicle frame in the current period, at most one command frame per period, ≤ 30 ms airtime (≈ 28 bytes at 9600 Bd).
- At 9600 Bd one byte takes 1.04 ms. On the wire (header 3 + payload + CRC 2, + 1 COBS byte, + delimiter): `TELEM_FAST` 39 bytes ≈ 40.6 ms, `TELEM_SLOW` 41 bytes ≈ 42.7 ms, `ACK` 10 bytes ≈ 10.4 ms, `EVENT` 17 bytes ≈ 17.7 ms; uplink `DRIVE` 12 bytes ≈ 12.5 ms, `PARAM_SET` 13 bytes ≈ 13.5 ms.
- **[VERIFY]** HC-12 turnaround (FU3) with the radio range test. If the 5 ms guard is too short, raise it. At 19200 Bd (D-11) all airtimes halve.
- On the USB port no timing rules apply; telemetry is sent at the same rate, and `CAP_DATA` streams as fast as the TX buffer allows.

### 8.4 Message catalogue

**Vehicle → PC**

| Type | Name | Payload | Bytes | When |
|---|---|---|---|---|
| 0x01 | `HELLO` | `u8 proto_ver (=1); u8 fw_major, fw_minor; u8 reset_cause; u32 git_hash; u8 dirty; u8 features (bit0 IMU, 1 VL, 2 BIN, 3 CAPTURE, 4 EST_MT); char course[8]; u32 last_fault_code; u32 last_fault_pc` | 26 | at boot and on `PING` |
| 0x02 | `TELEM_FAST` | see below | 32 | 10 Hz |
| 0x03 | `TELEM_SLOW` | see below | 34 | 1 Hz |
| 0x04 | `EVENT` | `u32 t_ms; u16 code; i32 arg` | 10 | on event |
| 0x05 | `ACK` | `u8 type; u8 seq; u8 result (st_t)` | 3 | after every uplink command except `DRIVE` |
| 0x06 | `PARAM_VAL` | `u16 id; f32 value; u8 result` | 7 | after `PARAM_GET`/`PARAM_SET` |
| 0x07 | `CAP_DATA` | `u16 index; u8 set; u8 n; n × record` | 4 + n·size ≤ 96 | after `CAP_GET` |
| 0x08 | `TEXT` | ASCII, not terminated | ≤ 96 | debug messages |

`TELEM_FAST` (32 bytes):

| Offset | Field | Type | Unit / encoding |
|---|---|---|---|
| 0 | `t_ms` | u32 | ms since boot |
| 4 | `mode` | u8 | `msn_mode_t` |
| 5 | `state` | u8 | `msn_state_t` |
| 6 | `flags` | u16 | bit 0 reflex, 1 interlock, 2 still, 3 slip, 4 degraded, 5 evade active, 6 battery warn, 7 fault active, 8 IMU ok, 9 VL ok, 10 manual timeout, 11 capture running, 12–15 reserved |
| 8 | `x` | i32 | mm (EKF) |
| 12 | `y` | i32 | mm |
| 16 | `th` | i16 | 0.01° (−18000 … 18000) |
| 18 | `v` | i16 | mm/s, measured (mean of the wheels) |
| 20 | `w` | i16 | mrad/s, gyro (encoder without IMU) |
| 22 | `duty_l` | i16 | 1/1000 |
| 24 | `duty_r` | i16 | 1/1000 |
| 26 | `seg` | u8 | current segment index |
| 27 | `wp` | u8 | next waypoint ID |
| 28 | `e_y` | i16 | mm, cross-track error to the current segment |
| 30 | `gnd` | u8 | bits 0–2 left side state, bits 3–5 right side state |
| 31 | `fwd` | u8 | minimum forward range in cm, 255 = none |

`TELEM_SLOW` (34 bytes):

| Offset | Field | Type | Unit |
|---|---|---|---|
| 0 | `t_ms` | u32 | ms |
| 4 | `bat` | u16 | mV |
| 6 | `sig_x`, `sig_y` | 2 × u16 | mm (√P) |
| 10 | `sig_th` | u16 | 0.01° |
| 12 | `bg` | i16 | 0.001 °/s |
| 14 | `scale` | i16 | ppm, (s − 1)·10⁶ |
| 16 | `imu_t` | i16 | 0.01 °C |
| 18 | `fault_first` | u16 | fault code |
| 20 | `n_faults` | u8 | active faults |
| 21 | `cpu` | u8 | % load (§5.6) |
| 22 | `ovr_ctrl` | u16 | `S_CTRL` overruns since boot |
| 24 | `rx_err` | u16 | radio frame errors since boot |
| 26 | `ekf_acc` | u16 | accepted aiding updates since boot |
| 28 | `ekf_rej` | u16 | rejected aiding updates since boot |
| 30 | `stack_free` | u16 | bytes, high-water mark |
| 32 | `vl_alive` | u8 | bit i = sensor i fresh |
| 33 | `sens` | u8 | bit 0 IMU alive, 1 battery present, 2 course valid |

**PC → vehicle**

| Type | Name | Payload | Bytes | Allowed in | Reply |
|---|---|---|---|---|---|
| 0x20 | `PING` | — | 0 | any | `ACK`, `HELLO` |
| 0x21 | `STOP` | — | 0 | **any** | `ACK` |
| 0x22 | `MODE` | `u8 mode` (0 IDLE, 1 BENCH, 2 MANUAL, 3 AUTO) | 1 | per §7.21.2 | `ACK` (`ST_STATE` + event with the reason if refused) |
| 0x23 | `DRIVE` | `i16 v [mm/s]; i16 w [mrad/s]; u8 flags (reserved)` | 5 | `MANUAL` | none (effect visible in telemetry) |
| 0x24 | `PARAM_GET` | `u16 id` | 2 | any | `PARAM_VAL` |
| 0x25 | `PARAM_SET` | `u16 id; f32 value` | 6 | per parameter flag | `PARAM_VAL` |
| 0x26 | `CAPTURE` | `u8 set; u8 decim; u16 duration [0.1 s]` | 4 | any | `ACK` |
| 0x27 | `CAP_GET` | `u16 from; u16 count` | 4 | not `AUTO` | `CAP_DATA` stream |
| 0x28 | `CLEAR` | — | 0 | `FAULT`, `IDLE` | `ACK` |
| 0x2F | `MODE_TEXT` | — | 0 | any | `ACK`, then the port switches to text |

- REQ-PRO-05 Commands are **absolute and idempotent**. A frame repeated with the same `type` and `seq` within 1 s is acknowledged again but not executed again.
- REQ-PRO-06 **Security.** The link has no authentication. Other teams on the same channel can be filtered (vehicle ID, CRC) but not prevented. Accepted consequence: `STOP` (also broadcast) is honoured from anyone, because a wrong stop is safe and a lost stop is not. `MODE AUTO` and `DRIVE` are honoured only with the own vehicle ID. Agree channels with the other teams (open item O-13).

### 8.5 Failsafe by mode

| Mode | Link lost (no valid frame for `msn_man_timeout`) | `STOP` received |
|---|---|---|
| `MANUAL` | `drv_stop()`, event, stays `MANUAL`; the next `DRIVE` resumes | `drv_stop()` |
| `AUTO` | **no effect**: autonomy must not depend on the link | `AUTO_STOPPED` |
| `BENCH` | text dead-man 2 s for open-loop duty, as today | stop |
| `IDLE` / `FAULT` | — | — |

---

## 9. Text console reference

Line-based, CR or LF terminated, case-sensitive. Values in **legacy units** (mm, mm/s, %, °, ms) unless stated. The first block is identical to the current firmware (REQ-CON-05: existing commands keep their syntax and meaning). Parsing: multi-letter keywords are matched first (longest match), then single-letter commands followed by a number, sign or space.

| Command | Mode | Action |
|---|---|---|
| `?` | any | help |
| `k` | any | print the control parameters (short list) |
| `s` | any | stop / abort (= `STOP`) |
| `r` | `IDLE`, `BENCH` | reset encoder counters (stationary only); in `FAULT`: `CLEAR` |
| `p` | any | print status once |
| `a` | any | auto-print every 250 ms on/off |
| `D<mm>` | `IDLE`/`BENCH` | straight move, e.g. `D1000`, `D-500` (enters `BENCH`) |
| `V<mm/s>` | any | nominal speed for `D`/`W` (≤ `drv_v_max`) |
| `T<ms>[ <ms>]` | any | acceleration [and deceleration] ramp time |
| `F<Ks %> <Kv %/(mm/s)>` | `IDLE`/`BENCH` | feedforward both wheels |
| `FL…`, `FR…` | `IDLE`/`BENCH` | feedforward one wheel |
| `G<Kp %/(mm/s)> <Ki %/(mm/s·s)>` | any | speed PI gains |
| `Y<K_sync (mm/s)/mm>` | any | L/R synchronisation, 0 = off |
| `-100` … `100` | `IDLE`/`BENCH` | open-loop duty both wheels in %, dead-man 2 s |
| **New** | | |
| `W<deg>` | `IDLE`/`BENCH` | turn in place, CCW positive: gyro-controlled if IMU present, else encoder |
| `PWM <l %> <r %>` | `IDLE`/`BENCH` | open-loop duty per wheel, dead-man 2 s |
| `M <v mm/s> <w mrad/s>` | `IDLE`/`MANUAL` | manual velocity; enters `MANUAL`; console dead-man 2 s |
| `MODE IDLE\|BENCH\|MANUAL\|AUTO` | per §7.21.2 | mode request |
| `RUN` | `IDLE` | = `MODE AUTO` (same as button B1) |
| `STOP` | any | = `s` |
| `CLEAR` | `FAULT`, `IDLE` | clear latched faults whose cause is gone |
| `P<id>=<value>` / `P<name>=<value>` | per flag | set parameter, **SI units** (§10) |
| `P<id>?` / `P<name>?` | any | read parameter |
| `L` | any | dump all parameters as `#define PAR_<NAME> <value>` |
| `S` | any | status: mode, state, segment, faults, sensors, battery, EKF σ |
| `I` | any | info: version, git hash, build flags, reset cause, tick rate, memory, last hard fault |
| `EV` | any | event log (last 64) |
| `FAULTS` | any | active and latched faults with reaction |
| `VL` | any | VL53L1X live table: id, range, status, signal, class (1 s, any key ends) |
| `IMU` | any | rates, temperature, roll, pitch, bias (1 s) |
| `EKF` | any | state, σ, NIS per update type, counters |
| `SEG` | any | course name, segment list, current segment |
| `ZERO` | `IDLE` | reset the EKF pose to the course start |
| `CAP <set> <decim> <s>` | any | start capture |
| `CAPDUMP` | `IDLE` | print the capture buffer as CSV (USB; slow over radio) |
| `BIN` | any | switch this port to binary (§8.1) |
| `FAULTINJ <code>` | `IDLE`/`BENCH`, **Debug builds only** | raise a fault for test H5 |

---

## 10. Parameter catalogue

All values are SI. Flag **L** = may change at any time (live), **S** = only while stationary in `IDLE`/`BENCH`. "Source" names where the default comes from: **M** measured, **C** catalogue, **D** design value, **E** estimate. Every `[MEASURE]` default is a placeholder until the named test has run. This table is the content of the X-macro list in `par_table.h` (REQ-PAR-02).

### 10.1 Actuation and wheel control (100–199)

| ID | Name | Default | Unit | Range | Flag | Src | Note |
|---|---|---|---|---|---|---|---|
| 100 | `mot_d_max` | 1.0 | – | 0.5–1.0 | L | D | duty clamp |
| 101 | `mot_d_min` | 0.0 | – | 0–0.2 | L | D | driver output floor `[MEASURE]` oscilloscope |
| 102 | `mot_slew_ol` | 2.0 | 1/s | 0.5–10 | L | D | open loop only |
| 110 | `whl_kp_v` | 0.45 | 1/(m/s) | 0–5 | L | D | SIMC balanced (0.045 %/(mm/s)) |
| 111 | `whl_ki_v` | 8.98 | 1/m | 0–100 | L | D | SIMC balanced (0.898 %/(mm/s·s)) |
| 112 | `whl_i_max` | 0.30 | – | 0–1 | L | D | integrator clamp |
| 113 | `whl_kp_x` | 2.0 | 1/s | 0–10 | L | D | position loop |
| 114 | `whl_v_corr_max` | 0.10 | m/s | 0–0.3 | L | D | position correction clamp |
| 115 | `whl_tau_m` | 0.050 | s | 0–0.5 | L | M | plant time constant (in air) `[MEASURE]` floor |
| 116 | `whl_ks_l` | 0.098 | – | 0–0.5 | L | M | BTS7960 in air, 9 Oct `[MEASURE]` `G` |
| 117 | `whl_kv_l` | 1.275 | 1/(m/s) | 0.5–3 | L | M | idem |
| 118 | `whl_ks_r` | 0.041 | – | 0–0.5 | L | M | idem; plausibility `[VERIFY]` (§5.2 tech. ref.) |
| 119 | `whl_kv_r` | 1.361 | 1/(m/s) | 0.5–3 | L | M | idem |
| 120 | `whl_ksb_l` | 0.141 | – | 0–0.6 | L | M | breakaway, bench `[MEASURE]` `B` on BTS7960 |
| 121 | `whl_ksb_r` | 0.169 | – | 0–0.6 | L | M | idem |
| 122 | `whl_v_moving` | 0.008 | m/s | 0.002–0.05 | L | D | rolling detection on |
| 123 | `whl_v_stopped` | 0.003 | m/s | 0.001–0.02 | L | D | rolling detection off |
| 124 | `whl_v_zero` | 0.002 | m/s | 0–0.01 | L | D | zero command band |
| 125 | `whl_speed_alpha` | 0.5 | – | 0.05–1 | L | D | IIR speed filter |
| 126 | `whl_fault_time` | 0.6 | s | 0.2–3 | L | D | detector persistence |
| 127 | `whl_track_max` | 0.04 | m | 0.01–0.2 | L | D | tracking fault threshold |
| 128 | `whl_ff_norm_en` | 0 | bool | 0/1 | S | D | battery normalisation of the feedforward |
| 129 | `whl_ff_u_ref` | 12.0 | V | 9–13 | S | E | voltage at which the FF was identified `[VERIFY]` |
| 150 | `drv_pos_tol` | 0.002 | m | 0.0005–0.02 | L | D | MOVE reached tolerance |
| 151 | `drv_v_settle` | 0.020 | m/s | 0.005–0.1 | L | D | settle speed clamp |
| 152 | `drv_settle_timeout` | 1.0 | s | 0.2–5 | L | D | → `DRV_EV_TIMEOUT` |
| 153 | `drv_k_sync` | 3.0 | 1/s | 0–20 | L | D | L/R synchronisation |
| 154 | `drv_v_max` | 0.45 | m/s | 0.1–0.6 | L | M | ≈ 75 % of the weaker wheel at low battery |
| 155 | `drv_w_max` | 2.5 | rad/s | 0.5–4 | L | D | |
| 156 | `drv_a_max` | 0.3 | m/s² | 0.05–1 | L | D | |
| 157 | `drv_a_stop` | 0.5 | m/s² | 0.1–2 | L | D | controlled stop |
| 158 | `drv_alpha_max` | 3.0 | rad/s² | 0.5–10 | L | D | |
| 159 | `drv_v_whl_max` | 0.45 | m/s | 0.1–0.6 | L | D | per-wheel limit |
| 160 | `drv_e_ref_max` | 0.05 | m | 0.01–0.2 | L | D | reference anti-windup |
| 161 | `drv_v_nom` | 0.30 | m/s | 0.02–0.6 | L | D | console `V` default |
| 162 | `drv_t_acc` | 1.0 | s | 0.05–5 | L | D | console `T` default |
| 163 | `drv_t_dec` | 1.0 | s | 0.05–5 | L | D | |

### 10.2 Odometry, battery, IMU (200–499)

| ID | Name | Default | Unit | Range | Flag | Src | Note |
|---|---|---|---|---|---|---|---|
| 200 | `odo_m_per_cnt` | 6.3638e-5 | m | 5e-5–8e-5 | S | M | π·0.0726/3584, 8 Oct |
| 201 | `odo_scale_l` | 1.0 | – | 0.95–1.05 | S | D | `[MEASURE]` UMBmark |
| 202 | `odo_scale_r` | 1.0 | – | 0.95–1.05 | S | D | `[MEASURE]` UMBmark |
| 203 | `odo_wheelbase` | 0.22325 | m | 0.15–0.35 | S | M | geometric; effective `[MEASURE]` UMBmark |
| 204 | `odo_backlash_l` | 0 | counts | 0–200 | S | D | `[MEASURE]` calibration `L` |
| 205 | `odo_backlash_r` | 0 | counts | 0–200 | S | D | idem |
| 206 | `enc_mt_zero_s` | 0.1 | s | 0.02–1 | L | D | M/T zero-speed timeout |
| 300 | `bat_ratio` | 5.7 | – | 4–8 | S | D | divider `[MEASURE]` multimeter |
| 301 | `bat_tau` | 0.2 | s | 0.05–5 | L | D | |
| 302 | `bat_warn_v` | 10.5 | V | 9.5–12 | L | D | |
| 303 | `bat_cut_v` | 10.0 | V | 9.5–11.5 | L | D | 3.33 V/cell |
| 400 | `imu_tc1` | 0 | rad/s/K | ±1e-3 | S | D | `[MEASURE]` temperature run |
| 401 | `imu_tc2` | 0 | rad/s/K² | ±1e-4 | S | D | idem |
| 402 | `imu_gz_scale` | 1.0 | – | 0.95–1.05 | S | D | `[MEASURE]` 10-turn spin against marks |
| 403 | `imu_timeout` | 0.020 | s | 0.005–0.2 | L | D | |
| 404 | `imu_extrap_max` | 0.002 | s | 0–0.005 | L | D | |
| 410 | `att_tau` | 1.0 | s | 0.1–10 | L | D | complementary filter |
| 411 | `att_acc_gate` | 0.1 | g | 0.02–0.5 | L | D | |

### 10.3 Estimator (500–599)

| ID | Name | Default | Unit | Range | Flag | Src | Note |
|---|---|---|---|---|---|---|---|
| 500 | `ekf_kd` | 1.0e-4 | m | 1e-6–1e-2 | L | E | distance noise `[MEASURE]` repeated D5000 |
| 501 | `ekf_slip_infl` | 100 | – | 1–1e4 | L | D | |
| 502 | `ekf_slip_thr` | 0.15 | rad/s | 0.02–1 | L | D | |
| 503 | `ekf_gyro_nd` | 1.0e-4 | rad/s/√Hz | 1e-6–1e-2 | L | E | `[MEASURE]` Allan variance |
| 504 | `ekf_gyro_sf` | 0.003 | – | 0–0.05 | L | E | gyro scale-factor uncertainty |
| 505 | `ekf_q_bg` | 1.0e-10 | (rad/s)²/s | 1e-14–1e-6 | L | E | `[MEASURE]` Allan variance |
| 506 | `ekf_q_s` | 1.0e-9 | 1/s | 0–1e-5 | L | E | |
| 507 | `ekf_q_th_enc` | 1.0e-4 | rad²/m | 1e-6–1e-2 | L | E | encoder-heading mode |
| 508 | `ekf_zupt_floor` | 3.5e-5 | rad/s | 0–1e-3 | L | D | |
| 509 | `ekf_zupt_T` | 1.0 | s | 0.2–5 | L | D | |
| 510 | `ekf_still_s` | 0.2 | s | 0.05–2 | L | D | |
| 511 | `ekf_gate1` | 9.0 | – | 1–100 | L | D | χ²₁ 99.7 % |
| 512 | `ekf_gate2` | 9.21 | – | 1–100 | L | D | χ²₂ 99 % |
| 513 | `ekf_sig_edge` | 0.02 | m | 0.005–0.2 | L | E | `[MEASURE]` outdoor |
| 514 | `ekf_sig_post` | 0.03 | m | 0.005–0.3 | L | E | |
| 515 | `ekf_edge_skip` | 0.25 | m | 0–1 | L | D | segment ends, after turns |
| 516 | `ekf_bg_conv` | 3.5e-4 | rad/s | 1e-5–1e-2 | L | D | 0.02 °/s |
| 517 | `ekf_p0_xy` | 0.01 | m | — | S | D | σ |
| 518 | `ekf_p0_th` | 0.0087 | rad | — | S | D | 0.5° |
| 519 | `ekf_p0_bg` | 0.0087 | rad/s | — | S | D | 0.5 °/s |
| 520 | `ekf_p0_s` | 0.005 | – | — | S | D | |

### 10.4 Sensors and geometry (600–699)

| ID | Name | Default | Unit | Range | Flag | Src | Note |
|---|---|---|---|---|---|---|---|
| 600 | `vl_tb_ms` | 20 | ms | 20–100 | S | D | timing budget |
| 601 | `vl_imp_ms` | 20 | ms | 20–200 | S | D | inter-measurement `[VERIFY]` ≥ TB rule |
| 602 | `vl_poll_ms` | 5 | ms | 2–20 | S | D | |
| 603 | `vl_timeout` | 0.2 | s | 0.05–1 | L | D | stale |
| 610 | `gnd_x` | 0.15 | m | −0.3–0.3 | S | D | footprint ahead of the axle `[MEASURE]` |
| 611 | `gnd_y_in` | 0.15 | m | 0–0.3 | S | D | `[DECIDE]` D-4 |
| 612 | `gnd_y_out` | 0.25 | m | 0–0.4 | S | D | `[DECIDE]` D-4 |
| 613 | `gnd_h0` | 0.12 | m | 0.05–0.3 | S | D | mounting height over paving `[MEASURE]` |
| 614 | `gnd_d_drop` | 0.03 | m | 0.01–0.1 | L | D | `[MEASURE]` |
| 615 | `gnd_d_up` | 0.02 | m | 0.01–0.1 | L | D | `[MEASURE]` edging height |
| 616 | `gnd_sd_grass` | 0.008 | m | 0.001–0.05 | L | E | `[MEASURE]` |
| 617 | `gnd_sig_grass` | 0 | kcps/SPAD | 0–100 | L | D | 0 = criterion off until measured |
| 618 | `gnd_sd_n` | 5 | – | 3–15 | S | D | |
| 619 | `gnd_n_deb` | 2 | – | 1–5 | L | D | |
| 620 | `gnd_interlock_bench` | 0 | bool | 0/1 | L | D | interlock in `BENCH` |
| 630 | `fwd_x` | 0.20 | m | 0–0.4 | S | D | `[MEASURE]` |
| 631 | `fwd_y` | 0.08 | m | 0–0.2 | S | D | `[MEASURE]` |
| 632 | `fwd_t_lat` | 0.10 | s | 0.02–0.5 | L | D | |
| 633 | `fwd_a_brake` | 0.5 | m/s² | 0.1–3 | L | D | `[MEASURE]` braking test |
| 634 | `fwd_margin` | 0.15 | m | 0.05–0.5 | L | D | |
| 635 | `fwd_slow_range` | 1.0 | m | 0.3–2 | L | D | |
| 636 | `fwd_v_slow` | 0.25 | m/s | 0.05–0.45 | L | D | |
| 650 | `lat_x` | 0.0 | m | −0.3–0.3 | S | D | `[MEASURE]` |
| 651 | `lat_y` | 0.175 | m | 0–0.3 | S | D | `[MEASURE]` |
| 652 | `lat_r_min` | 0.05 | m | 0.04–0.5 | L | D | |
| 653 | `lat_r_max` | 1.2 | m | 0.3–3 | L | D | |
| 654 | `lat_n_min` | 50 | – | 10–400 | L | D | |
| 655 | `lat_n_max` | 400 | – | 50–1000 | S | D | buffer size |
| 656 | `lat_span_min` | 1.0 | m | 0.3–5 | L | D | |
| 657 | `lat_rms_max` | 0.03 | m | 0.005–0.1 | L | D | |
| 658 | `lat_notch_depth` | 0.15 | m | 0.05–1 | L | D | |
| 659 | `lat_sig_face_min` | 8.7e-4 | rad | 0–0.01 | L | D | 0.05° |
| 660 | `veh_w` | 0.35 | m | 0.2–0.5 | S | D | `[MEASURE]` |
| 661 | `veh_rear` | 0.20 | m | 0.05–0.4 | S | D | axle → rear |
| 662 | `veh_front` | 0.20 | m | 0.05–0.4 | S | D | axle → front |

### 10.5 Guidance, evasion, mission, communication (700–999)

| ID | Name | Default | Unit | Range | Flag | Src | Note |
|---|---|---|---|---|---|---|---|
| 700 | `nav_wn` | 1.5 | rad/s | 0.3–3 | L | D | path-loop natural frequency |
| 701 | `nav_zeta` | 0.7 | – | 0.3–1.5 | L | D | |
| 702 | `nav_v_min` | 0.05 | m/s | 0.01–0.2 | L | D | gain scheduling floor |
| 703 | `nav_a_acc` | 0.3 | m/s² | 0.05–1 | L | D | |
| 704 | `nav_a_dec` | 0.3 | m/s² | 0.05–1 | L | D | |
| 705 | `nav_w_max` | 1.0 | rad/s | 0.2–2.5 | L | D | |
| 706 | `nav_final_dist` | 0.30 | m | 0.1–1 | L | D | encoder-frame final stop |
| 707 | `nav_turn_w` | 0.8 | rad/s | 0.1–2.5 | L | D | |
| 708 | `nav_turn_ta` | 0.4 | s | 0.1–2 | L | D | |
| 709 | `nav_turn_k` | 2.0 | 1/s | 0–10 | L | D | |
| 710 | `nav_turn_tol` | 0.0035 | rad | 0.0005–0.05 | L | D | 0.2° |
| 750 | `evd_margin` | 0.10 | m | 0–0.5 | L | D | |
| 751 | `evd_obs_extra` | 0.10 | m | 0–0.5 | L | D | |
| 752 | `evd_edge_margin` | 0.05 | m | 0–0.3 | L | D | |
| 753 | `evd_obs_depth` | 0.5 | m | 0.1–2 | L | D | assumed obstacle depth |
| 754 | `evd_vy_out` | 0.15 | m/s | 0.02–0.3 | L | D | |
| 755 | `evd_vy_back` | 0.08 | m/s | 0.02–0.3 | L | D | |
| 756 | `evd_ay` | 0.3 | m/s² | 0.05–1 | L | D | |
| 757 | `evd_wp_guard` | 1.5 | m | 0–5 | L | D | no evasion near waypoints |
| 800 | `msn_init_min_s` | 3.0 | s | 1–60 | L | D | |
| 801 | `msn_init_max_s` | 30 | s | 5–300 | L | D | |
| 802 | `msn_preturn_s` | 1.0 | s | 0–5 | L | D | ZUPT before turns |
| 803 | `msn_obst_wait_s` | 3.0 | s | 0–60 | L | D | wait before evasion |
| 804 | `msn_rec_back_m` | 0.10 | m | 0.02–0.5 | L | D | |
| 805 | `msn_rec_bias_m` | 0.05 | m | 0–0.2 | L | D | |
| 806 | `msn_noref_dist` | 30 | m | 5–200 | L | D | degraded detection |
| 807 | `msn_sig_lat_max` | 0.30 | m | 0.05–2 | L | D | |
| 808 | `msn_v_degraded` | 0.20 | m/s | 0.05–0.45 | L | D | |
| 809 | `msn_man_v_max` | 0.40 | m/s | 0.05–0.6 | L | D | |
| 810 | `msn_man_w_max` | 1.5 | rad/s | 0.2–2.5 | L | D | |
| 811 | `msn_man_timeout` | 0.5 | s | 0.2–2 | L | D | radio heartbeat |
| 812 | `msn_allow_nosensors` | 0 | bool | 0/1 | S | D | development only |
| 813 | `msn_allow_bench_supply` | 0 | bool | 0/1 | S | D | development only |
| 814 | `msn_wp_hold` | 10.5 | s | 10–30 | S | D | default hold (course may override) |
| 900 | `com_vid` | 66 (0x42) | – | 1–254 | S | D | vehicle ID |
| 901 | `com1_mode` | 1 | enum | 0 text / 1 binary | S | D | radio default from phase 4 |
| 902 | `tlm_period_ms` | 100 | ms | 50–1000 | L | D | |
| 903 | `tlm_air_max_ms` | 55 | ms | 20–90 | L | D | |
| 904 | `con_deadman_s` | 2.0 | s | 0.5–10 | L | D | |
| 905 | `wdg_enable` | 1 | bool | 0/1 | S | D | Debug builds only; Release always on |

---

## 11. Fault catalogue

**Reaction classes** (REQ-FLT-05):

| Class | Meaning |
|---|---|
| `R0` | log + event only |
| `R1` | degrade: feature off, warning, mission continues if safe |
| `R2` | controlled stop (`drv_stop`), then `FAULT` mode |
| `R3` | immediate brake (`drv_brake`), then `FAULT` |
| `R4` | hard stop (`sys_fault_hard_stop`: EN low, CCR 0) at once, then `FAULT` |

All faults with class ≥ R2 latch until `CLEAR` and the condition has gone.

| Code | Name | Source | Condition | Reaction |
|---|---|---|---|---|
| 0x01 | `F_CLOCK` | app | boot clock check failed (§5.8 step 6) | R4, boot stops in `FAULT` |
| 0x02 | `F_HARDFAULT` | handlers | HardFault/MemManage/BusFault/UsageFault/NMI | R4, then spin until the IWDG resets. Code, PC and LR are stored in RTC backup registers BKP0R–BKP3R (survive a reset, no linker change needed; requires PWR clock + DBP, **[VERIFY]** whether RTCEN is needed for register access on the F446) and reported in `HELLO` |
| 0x03 | `F_WDG_RESET` | boot | `RCC_CSR.IWDGRSTF` set | R0 |
| 0x04 / 0x05 | `F_STALL_L` / `_R` | drv | REQ-WHL-04 stall | R3 |
| 0x06 / 0x07 | `F_RUNAWAY_L` / `_R` | drv | REQ-WHL-04 runaway (sign error) | R4 |
| 0x08 / 0x09 | `F_TRACK_L` / `_R` | drv | REQ-WHL-04 tracking | R3 |
| 0x0A | `F_BAT_LOW` | bat | REQ-BAT-03 cut-off | R2 |
| 0x0B | `F_IMU_TIMEOUT` | imu | REQ-IMU-06 | `AUTO`: R3; else R1 (EKF to encoder mode) |
| 0x0C | `F_IMU_SAT` | imu | REQ-IMU-07 | R0 |
| 0x0D / 0x0E | `F_VL_BUS1` / `2` | vl | REQ-VL-04 | `AUTO`: R3; else R1 |
| 0x0F | `F_GND_STALE` | gnd | inner ground sensor stale in `AUTO` | R2 |
| 0x10 | `F_FWD_STALE` | fwd | both forward sensors stale in `AUTO` | R2; `MANUAL`: R1 (forward motion limited to 0.15 m/s) |
| 0x11 | `F_GND_INTERLOCK` | msn | interlock persists after recovery, or > 3 recoveries per segment | R3 |
| 0x12 | `F_EKF` | ekf | non-finite state or P, P bounds exceeded | R3; EKF re-initialised in `IDLE` |
| 0x13 | `F_NUMERIC` | any | non-finite value at an actuator boundary (REQ-COD-10) | R4 |
| 0x14 | `F_OVERRUN` | sched | slot overrun (REQ-EXE-12) | R0; R2 if > 10 `S_CTRL` overruns/s |
| 0x15 | `F_INIT` | msn | `AUTO_INIT` timeout | R2 |
| 0x16 | `F_MAP` | map | course validation failed | R0 (AUTO unavailable) |
| 0x17 | `F_COM` | com | UART error burst (> 100/s) | R0 |
| 0x18 | `F_ASSERT` | any | `SYS_ASSERT` failed | R3 |
| 0x19 | `F_CSS` | RCC | HSE failure (CSS NMI) | R4 |
| 0x1A | `F_STACK` | sys | stack high-water mark < 256 B | R0 |
| 0x1B | `F_PARAM` | par | defaults fail validation at boot | R4 (boot stops) |

---

## 12. Operational flows

**Flow A — Bench move (regression reference for phase 0).**
1. Operator: `D1000` on USB or radio. `con` converts to `drv_move(1.000 m, V, T, T)`; mode `IDLE` → `BENCH`; EN high.
2. `drv` runs the profile; both `whl` track with position loop, feedforward, PI and sync.
3. Profile ends → `SETTLE`; each wheel brakes when reached; never reverses.
4. Both reached and slow → `DRV_EV_DONE`. The console prints `DONE L=<mm> R=<mm> sync_max=<mm> t=<ms>`. After 1 s without reaching → `TIMEOUT …` instead.
5. Stall, track or runaway during the move → fault reaction, `FAULT`, message with the code.

**Flow B — Manual driving over radio.**
1. PC: `MODE MANUAL` → `ACK OK`. Vehicle in `MANUAL`, `HOLD`.
2. PC: `DRIVE(v, ω)` every 100 ms in its uplink window. The vehicle clamps and runs `drv_vel`.
3. Forward obstacle → reflex: v > 0 is zeroed; turning and reversing allowed. Ground interlock → brake; only motion away from the tripped side allowed.
4. Link lost for 0.5 s → `drv_stop`, `EVENT MAN_TIMEOUT`. A new `DRIVE` resumes.
5. `STOP` or `MODE IDLE` → stop, then EN low.

**Flow C — Autonomous circuit.**
1. Vehicle at X, aligned with the start jig (defines the world frame). 10 min warm-up (operational rule, gyro temperature).
2. B1 or `RUN` → start conditions checked → `AUTO_INIT`: EKF pose = course start; standstill ZUPT until the bias has converged (≥ 3 s).
3. Per segment: `AUTO_DRIVE` (follow with edge and landmark aiding) → `AUTO_FINAL` (encoder-frame stop in the last 0.30 m) → `AUTO_HOLD` 10.5 s at a waypoint (ZUPT runs; `EV_WP_ARRIVED` with estimated pose and σ) → `AUTO_PRETURN` 1 s ZUPT → `AUTO_TURN` on the gyro → next `AUTO_DRIVE`.
4. Back at X: `FINISH` → `AUTO_FINISHED` → `IDLE`. Event with the total time.

**Flow D — Static obstacle on the path.**
1. `AUTO_DRIVE`; the forward range falls below 1.0 m → speed limited to 0.25 m/s.
2. Range < `d_stop` → `AUTO_OBST_STOP` (brake) → `AUTO_OBST_WAIT`.
3. After 3 s still blocked: `evd_plan` finds a free lateral offset → `AUTO_EVADE`. The offset moves out at 0.15 m/s; landmark updates are suppressed beyond 0.1 m offset; edge updates continue.
4. If the outer ground sensor on that side reaches the boundary before the offset clears the obstacle, the offset freezes; if it does not clear → `EVD_BLOCKED` → wait.
5. Past `s_clear` → offset returns at 0.08 m/s → `AUTO_DRIVE`.
6. Within 1.5 m of a waypoint: no evasion; wait. If the obstacle stays, the operator decides (stop/continue).

**Flow E — Ground interlock.**
1. An inner ground sensor reports grass, drop or edging → brake at once (`AUTO_RECOVER`).
2. Standstill → back up 0.10 m straight → lateral bias 0.05 m away from that side for 2 m → `AUTO_DRIVE`.
3. Interlock still active after backing, or the fourth recovery on one segment → `FAULT F_GND_INTERLOCK`.

**Flow F — Fault in AUTO.**
1. E.g. IMU timeout → `F_IMU_TIMEOUT` (R3) → brake → `FAULT`. Event with code; `TELEM_FAST` flag 7; `TELEM_SLOW.fault_first`.
2. The operator inspects (`FAULTS`, `EV`), fixes, `CLEAR` → `IDLE`. A new run starts from X.

---

## 13. Non-functional requirements

### 13.1 Timing and performance

| Metric | Requirement | ID |
|---|---|---|
| Tick ISR execution | ≤ 2 µs | REQ-NFR-01 |
| `S_CTRL` execution | ≤ 600 µs worst case (measured, §5.6) | REQ-NFR-02 |
| `S_EST` execution | ≤ 300 µs worst case | REQ-NFR-03 |
| CPU load | < 50 % worst-case 1 s window | REQ-NFR-04 |
| Control period | 20.000 ms mean, jitter < 100 µs | REQ-NFR-05 |
| Reflex latency (sensor data ready → brake applied) | ≤ 30 ms (poll interval 5 ms + I²C read ≈ 1 ms + up to one `S_CTRL` period); covered by `fwd_t_lat` = 0.1 s | REQ-NFR-06 |
| `STOP` latency (last byte received → drive braking) | ≤ 25 ms on USB, ≤ 25 ms after frame reception on radio | REQ-NFR-07 |
| Timestamp resolution | 1 µs; error between physical event and stamp < 50 µs (IMU, encoders), < 3 ms (VL53L1X) | REQ-NFR-08 |
| Boot to ready (`IDLE`) | < 2 s (without VL53L1X boot), < 3 s with | REQ-NFR-09 |

### 13.2 Reliability

- REQ-NFR-10 Five full circuits in a row without an unexplained reset, hang or fault. Every reset has a reported cause.
- REQ-NFR-11 Any single sensor failure is detected within 0.2 s and handled per §11; there is no silent use of stale data.
- REQ-NFR-12 Every UART error re-arms reception; the console never goes deaf.
- REQ-NFR-13 15 minutes of continuous operation with all features: no buffer overflow, no stack high-water mark below 512 B, no growing counters other than statistics.

### 13.3 Safety

Safety-relevant mechanisms and the requirement that defines each:

| Hazard | Mechanism | Requirement |
|---|---|---|
| Both PWM inputs of a half-bridge pair active | single writer, inactive channel first | REQ-MOT-01/02 |
| Undefined outputs during reset or a firmware fault | EN pull-downs; hard stop in all fault handlers | REQ-FLT-01/02 |
| Driving off a kerb or onto grass | inner ground interlock, never disabled in AUTO | REQ-GND-02 |
| Collision | reflex layer, bypasses planning | REQ-FWD-01, REQ-MSN-05 |
| Runaway from a sign error | runaway detector → EN low | REQ-WHL-04, §11 |
| Lost radio in manual mode | 0.5 s heartbeat | REQ-MSN-03 |
| Firmware hang | IWDG with liveness conditions | REQ-EXE-13 |
| Clock failure | CSS → NMI → hard stop | §4.1, REQ-FLT-02 |
| Battery deep discharge / brown-out | cut-off at 10.0 V; buck minimum input `[VERIFY]` | REQ-BAT-03 |
| Firmware-independent stop | **recommended:** mechanical E-stop in series with both EN lines (decision D-6 of the technical reference) | hardware |

### 13.4 Security

There are no secrets, user data or external services. The only attack surface is the radio link: it is unauthenticated, and REQ-PRO-06 defines the accepted behaviour. No compliance regime applies to the firmware. The regulatory status of the 433 MHz band in Mexico is a hardware/radio decision (open question Q-8), not a firmware requirement.

### 13.5 Observability

The firmware is observable without a debugger: `HELLO` with reset cause and last hard fault; 10 Hz `TELEM_FAST`, 1 Hz `TELEM_SLOW`; events; the 64-entry event log; the capture buffer; the console status commands (`S`, `I`, `EKF`, `VL`, `IMU`, `FAULTS`). The live-expression debug views in CubeIDE remain available. All published module outputs are plain structs, ready to be added to Live Expressions.

---

## 14. Testing and verification

### 14.1 Host unit tests (`test/`, `make test` on the Mac)

Pure modules compile with the host compiler using `-std=gnu11 -Wall -Wextra -Wdouble-promotion -Wconversion -Wshadow -Werror -fsanitize=address,undefined` (REQ-TST-01). The framework is a 50-line assertion header (`test.h`), with no external dependency. Every test names the requirement it covers.

| Test | Covers | Pass criterion |
|---|---|---|
| `t_crc` | proto | `crc16("123456789") == 0x29B1` |
| `t_cobs` | proto | round-trip of 10⁴ random frames 0–101 B incl. all-zero and no-zero; decoder rejects corrupted frames |
| `t_proto_msgs` | §8.4 | pack/unpack of every message matches the byte offsets of the tables |
| `t_prof` | REQ-PRF-01/02 | x(T) = D within 1e-6 relative for trapezoid and triangle; v continuous; `prof_plan_v0` stops at D from v0 ∈ {0, 0.1, 0.3} |
| `t_whl_law` | REQ-WHL-01..05 | feedforward values equal hand-computed ones; integrator clamps; conditional integration; breakaway switching with hysteresis; detectors fire after exactly 0.6 s |
| `t_drv_move` | REQ-DRV-01/02 | with `sim_plant` (first-order wheels K = 7.6, τ = 50 ms, Coulomb friction, ±10 % L/R mismatch): `D300`, `D1000`, `D5000` end within 2 mm, never reverse; sync max < 3 mm |
| `t_drv_vel` | REQ-DRV-03/04/05 | limits respected; curvature preserved under wheel saturation; reference anti-windup; bumpless mode switches |
| `t_enc_est` | REQ-ENC-05 | synthetic edge trains 5–500 mm/s: DIFF error ≤ one count/period; MT error ≤ 10 % at ≥ 10 mm/s |
| `t_att` | REQ-ATT-01 | synthetic IMU at 6° static pitch converges within 0.5° in 3 s; rejects 0.3 g lateral acceleration |
| `t_ekf_static` | REQ-EKF-01/05/09 | 60 s stationary with bias 0.5 °/s + noise: σ_bg < 0.02 °/s after 3 s; heading drift < 0.01°; P symmetric, positive definite after 10⁶ steps |
| `t_ekf_line` | §7.17 | simulated 40 m straight with scale error 0.5 %, gyro bias drift, edge events every 0.5 m: cross-track error < 3 cm; NIS within the 95 % band ≥ 90 % |
| `t_ekf_gate` | REQ-EKF-05 | a 10σ edge outlier is rejected; a 2σ one is accepted |
| `t_ekf_jacobians` | §7.17.3 | every analytic H and F matches central finite differences to 1e-3 relative |
| `t_map` | REQ-MAP-01..03 | frame quantities for LINE/ARC; boundary lines; validation catches each rule violation (one test course per rule) |
| `t_nav_line` | REQ-NAV-01 | closed loop with unicycle + first-order wheels: 0.2 m initial offset converges with overshoot < 10 % (ζ = 0.7); steady cross-track error < 2 mm on an ARC (curvature feedforward) |
| `t_nav_final` | REQ-NAV-02 | final stop in the encoder frame is unaffected by an EKF jump of 5 cm injected at 0.2 m before the end |
| `t_nav_turn` | REQ-NAV-03 | 90° turns end within 0.2°, no reversal inside 0.5° |
| `t_gnd` | REQ-GND-01..03 | synthetic height/σ sequences produce the expected classes, debounce, events, side states and interlock |
| `t_lat` | REQ-LAT-01/02 | face fit recovers a 0.5° heading within 0.05° from 250 noisy samples (σ_r 2 cm, span 2 m); notch centre within 1 cm |
| `t_fwd` | REQ-FWD-01 | d_stop formula; reflex set/clear hysteresis; implausible jumps rejected |
| `t_evd` | REQ-EVD-01..04 | free-set cases: obstacle centre/left/right/full width; edge freeze; waypoint guard |
| `t_msn` | §7.21 | every row of the transition tables of §7.21.2–3 is exercised by a scripted input sequence |
| `t_par` | REQ-PAR-01..05 | every default is inside its range; IDs unique; set/get round trip; flags enforced |

### 14.2 Software-in-the-loop

`test/sim_vehicle.c` links **all pure modules** with a vehicle model: unicycle kinematics, first-order wheels with friction and backlash, encoder quantisation, gyro with bias and noise, VL53L1X ground sensors over a synthetic course with boundaries and noise. It replays `course_rect10` and the Jardín course (once surveyed) end to end (REQ-TST-02). Pass: course completed, every waypoint estimate within 5 cm of truth, every true waypoint error < 15 cm, no interlock trip at nominal tracking. Run before every hardware session in phases 3–7. The existing calibration simulator (`sim/` of `CALIB_README.md`) stays separate.

### 14.3 Hardware-in-the-loop and vehicle tests

| ID | Test | Set-up | Pass | Phase |
|---|---|---|---|---|
| H1 | Tick and clock self-check | bench, USB | 1000.0 Hz ± 0.1 %; clock check OK; `I` prints rates | 0 |
| H2 | Slot timing | oscilloscope on PC8, all features on | REQ-NFR-01..05 | 0, 3, 6 |
| H3 | Regression `D` | floor | `D300`, `D1000`, `D5000` × 10 each: end within 2 mm (encoder), no reversal; same as the old firmware | 0 |
| H4 | Truth of signs | wheels in the air | calibration `C` passes; runaway detector trips when a sign is flipped deliberately | 0 |
| H5 | Faults | bench | each R3/R4 fault injected via a debug command (`FAULTINJ <code>`, Debug builds only) stops within one tick; `Error_Handler` and HardFault paths drive EN low (verified with a scope) | 0 |
| H6 | Calibration chain | stand, floor | `A`, `L`, `G`, `D5000`×3, UMBmark: return error < 10 mm after correction | 1 |
| H7 | IMU | bench | DRDY capture rate = ODR ± 0.1 %; Allan variance logged; 10-turn spin residual < 0.5° after `imu_gz_scale` | 2 |
| H8 | EKF stationary + pushed | bench, floor | bias converges; hand-pushed 10 m straight: estimate within 10 cm | 2 (week 10) |
| H9 | Ground sensors outdoors | course, noon + afternoon | classes correct on paving/edging/grass/kerb ≥ 99 % of samples; thresholds recorded | 3 |
| H10 | Reflex | floor, box obstacle | stops before contact at 0.1, 0.3, 0.45 m/s; stopping distance logged | 3 |
| H11 | Telemetry | 15 min, radio | < 1 % frame loss at ≤ 30 m; link map recorded along the course | 4 (week 10) |
| H12 | Rectangle | 10 × 10 m | closes within 30 cm, waypoint estimates within 10 cm of tape | 3 (week 13) |
| H13 | Manual + failsafe | radio | drives on command; radio off → stops within 0.6 s | 6 (week 14) |
| H14 | Full circuit, no obstacles | course | completes; waypoints within 40 cm | 7 (week 15) |
| H15 | Full system | course, static obstacle | completes; waypoints within 20 cm; obstacle passed by rerouting | 7 (week 16) |
| H16 | Release build | course | H3, H12 repeated with `-O2`; identical behaviour | 7 (week 13) |

### 14.4 Test data

Synthetic data is generated by the host tests themselves (seeded PRNG, deterministic). Recorded vehicle data (captures, logs) is kept under `tools/data/<date>/` with a README naming firmware hash, parameters (`L` dump) and conditions. A recorded log can be replayed through `ekf`, `gnd` and `lat` on the host (`test/replay.c`) to tune thresholds offline (REQ-TST-03).

---

## 15. Implementation order

One layer at a time. Each phase ends with a gate. The next phase starts only when its gate passes; a failed gate is a finding, not a delay to hide. Weeks refer to the 16-week course; today is week 6.

| Phase | Weeks | Content | Gate (all must pass) |
|---|---|---|---|
| **0 — Restructure** | 6–7 | CubeMX target state §4 except phase-tagged peripherals (TIM6 1 kHz, TIM2 1 MHz, TIM1 ARR 8999, build flags). Modules `sys_*`, `par`, `units`, `mot`, `enc` (DIFF), `prof`, `whl`, `drv` (MOVE, TURN_ENC, OPEN, HOLD), `bat`, `com_uart` (IT both ports), `con` with all legacy commands, `app`, `app_isr`. Fault handlers. `test/` with `t_prof`, `t_whl_law`, `t_drv_move`, `t_par`, `t_crc`. | H1, H2, H3, H4, H5. The `D` regression matches the old firmware. Host tests green. |
| **1 — Calibrate** | 7 | Run calibration firmware `A`, `L`, `G`, odometry sequence. Enter results as defaults. Per-wheel scale; feedforward normalisation if the battery divider is fitted. | H6. Parameter defaults updated and committed with the calibration logs. |
| **2 — IMU and EKF** | 7–9 | `imu` (LSM6DSV backend), `att`, `ekf` (prediction, ZUPT, encoder fallback), TIM2 capture, SPI DMA, IWDG, capture buffer (sets 0, 1). Console `IMU`, `EKF`, `W` with gyro. | H7, H8 (stationary part). `t_att`, `t_ekf_*` green. **Week-8 milestone:** IMU and encoders logged with timestamps. |
| **3 — Sensing and guidance** | 8–11 | `vl`, `gnd`, `fwd` (reflex, governor), edge updates in `ekf`, `map` + `course_rect10`, `nav` (follow, final, turn), `msn` (AUTO without evasion, RECOVER), `drv` VEL mode. SIL simulation. | H9, H10, SIL pass on `rect10`, `t_gnd`, `t_fwd`, `t_map`, `t_nav_*`, `t_msn` green. **Week-8 milestone** extended by ToF logging. |
| **4 — Telemetry** | 9–10 | `proto`, `tlm` (FAST, SLOW, EVENT), USART1 DMA, port modes, `HELLO`, radio timing. Ground station logging (separate spec). | H11. `t_cobs`, `t_proto_msgs` green. **Week-10 milestone.** |
| **5 — Course aiding** | 10–12 | Course survey → `course_jardin.yaml`. `lat` (face, post), landmark updates, backlash model, optional `EST_MT`. Replay tool. | H8 (pushed), H12 (rectangle ≤ 30 cm, week 13 at the latest). `t_lat`, `t_enc_est` green. |
| **6 — Obstacles and manual** | 12–14 | `evd`, `fwd` ROI profile, `AUTO_EVADE`; `MANUAL` via `DRIVE` with heartbeat. | H13. `t_evd` green. **Week-14 milestone.** |
| **7 — Integration** | 13–16 | Release build first in week 13 (H16). Full-circuit runs, tuning with captures, offline optimisation of scale, wheelbase and landmark offsets from recorded runs. | H14 (**week 15**), H15 (**week 16**). |

**Critical path.** Course survey (phase 5 input, needed early for wheel/caster size and the error budget) → IMU delivery (phase 2) → ground sensors (phase 3). Phases 2 and 3 can overlap if the IMU is late: `FEAT_IMU = 0` runs the EKF in encoder mode, so guidance and mission logic do not wait for the IMU.

---

## 16. Migration from the current `main.c`

### 16.1 Function mapping

| Current (`main.c`, 10 Oct) | Target |
|---|---|
| `Wheel_t` (`enc`, channels, signs, `ks`, `kv`, `ks_break`, `total`, `mmpc`, `speed`, `integ`, `duty`, `moved`, `kick_ticks`, `fault_cnt`) | split: hardware fields → `enc`/`mot` tables; controller state → `whl_t`; parameters → `par` |
| `Wheel_UpdateEncoder` | `enc_update` (1 kHz accumulation) + speed estimate in `drv_step` input preparation |
| `Wheel_SetDuty` | `mot_set` + `mot_apply` |
| `Wheel_Control` | `whl_step` (without the kick; REQ-WHL-02) |
| `Wheel_Faulty` | `whl` detectors (REQ-WHL-04) + `drv` fault raising |
| `Profile_Eval`, move planning in `Move_Start` | `prof_plan`, `prof_eval` |
| `Control_Tick` (TIM6 ISR, 50 Hz) | `app_ctrl` in the main loop (`S_CTRL`), body in `drv_step` |
| `Move_Start`, `Move_Abort`, phases `PH_MOVE/SETTLE/IDLE/FAULT` | `drv_move`, `drv_stop`/`drv_brake`, `drv` phases |
| `Motors_Stop` | `drv_brake` (normal), `sys_fault_hard_stop` (faults) |
| `Car_Init` | `app_init` |
| `Car_Loop` | `app_loop` |
| `Serial_HandleCommand`, `Serial_Print`, `rx_line`, `line_ready`, `rx_byte`, `rx_byte1` | `con` (per-port line buffers) + `com_uart` (rings, double-buffered TX) |
| `PWM_Setup` | TIM1 ARR 8999 in CubeMX; `mot_init` verifies ARR |
| `#define`s in USER CODE PD | `par_table.h` defaults (converted, Appendix 19.2) |
| `open_loop`, `t_last_rx`, dead-man in `Car_Loop` | `drv` OPEN mode + `con` dead-man per port |

### 16.2 `main.c` after migration

Only these USER CODE blocks contain code (REQ-MIG-01):

```c
/* USER CODE BEGIN Includes */
#include "app.h"
#include "calib.h"
/* USER CODE END Includes */

/* USER CODE BEGIN 2 */
#if APP_CALIB
  htim6.Instance->ARR = 19999U;                  /* 50 Hz for the calibration firmware */
  Calib_Init(&htim1, &htim3, &htim8, &htim6, &huart2, &huart1);
  for (;;) { Calib_Loop(); }
#else
  app_init(&(app_hw_t){ .tim1 = &htim1, .tim2 = &htim2, .tim3 = &htim3, .tim6 = &htim6,
                        .tim8 = &htim8, .uart1 = &huart1, .uart2 = &huart2,
                        .adc1 = &hadc1, .spi1 = &hspi1, .i2c1 = &hi2c1, .i2c2 = &hi2c2 });
#endif
/* USER CODE END 2 */

/* USER CODE BEGIN 3 */               /* inside while (1) */
  app_loop();
/* USER CODE END 3 */

/* USER CODE BEGIN Error_Handler_Debug */
  sys_fault_hard_stop();
  sys_fault_record_hard(F_HARDFAULT, __get_MSP());
/* USER CODE END Error_Handler_Debug */
```

All HAL callbacks (`HAL_TIM_PeriodElapsedCallback`, `HAL_TIM_IC_CaptureCallback`, `HAL_UART_RxCpltCallback`, `HAL_UART_TxCpltCallback`, `HAL_UART_ErrorCallback`, `HAL_UARTEx_RxEventCallback`, `HAL_SPI_TxRxCpltCallback`, `HAL_I2C_MemRxCpltCallback`, `HAL_I2C_MemTxCpltCallback`, `HAL_I2C_ErrorCallback`, `HAL_GPIO_EXTI_Callback`, `HAL_RCC_CSSCallback`) are defined in exactly one file, `app_isr.c`, and dispatch on the instance. In the `APP_CALIB` build, `app_isr.c` forwards TIM6 to `Calib_Tick()` instead (REQ-MIG-02). The `.ioc` handle names above must exist; phase-tagged handles are added to `app_hw_t` when their peripheral is generated.

### 16.3 Migration procedure

1. Tag the current working state in Git (`pre-spec-v1`). Record `D300`/`D1000`/`D5000` results × 10 with that firmware as the regression baseline.
2. CubeMX changes of phase 0 only; regenerate; `git diff`; verify TIM6 and TIM2 values in the generated code.
3. Move code module by module. After each module: build, flash, `D1000`. Keep the old `main.c` code compiled out with `#if 0` until H3 passes, then delete it.
4. Unit tests for each moved pure module before moving the next one.
5. H1–H5. Tag `phase0`.

---

## 17. Dependencies and risks

### 17.1 External dependencies

| Dependency | Purpose | If unavailable |
|---|---|---|
| STM32Cube HAL F4 | peripheral access | none (fundamental) |
| VL53L1X ULD (ST, BSD-3) | sensor init | port of the documented register sequence; hot path is already own code |
| LSM6DSV (or replacement) | heading | `FEAT_IMU = 0`: encoder-heading EKF; accuracy budget then relies on boundary and face updates only |
| HC-12 (or certified replacement) | telemetry, remote control | USB cable for development; radio required only for the week-10 and week-14 demonstrations |
| Python 3, numpy, pyserial, PyYAML | host tools | — |
| Course survey | `course_jardin.yaml`, thresholds, landmarks | rectangle and simulated course only; no AUTO run on the real circuit |

### 17.2 Implementation risks

| Risk | Likelihood | Impact | Mitigation |
|---|---|---|---|
| CubeMX regeneration reverts TIM6/TIM2 values or build flags | high | wrong rates, lost warnings | REQ-PLT-01 diff; boot self-check of tick and clock rates (§5.8) |
| HSE BYPASS without USB fails on battery | medium | no autonomous run | boot clock check reports it; open item O-3 (crystal at X3 or power bank) |
| IMU not delivered before week 9 | medium | late heading aiding | encoder-heading mode; phase overlap |
| I²C bus lock-up by one VL53L1X | medium | blind interlock | non-blocking transactions, timeouts, bus recovery, split buses (REQ-VL-04) |
| Restructuring breaks the working drivetrain | medium | lost week | regression baseline, module-by-module migration (§16.3) |
| `-O2` exposes missing `volatile` late | medium | heisenbugs in week 16 | Release build in week 13 (REQ-ARC-04), REQ-EXE-06 |
| EKF numerical drift in float32 | low | divergence | Joseph form, symmetrisation, health checks (REQ-EKF-05) |
| Half-duplex collisions on the radio | high without timing rules | manual mode looks broken | vehicle as timing master (§8.3) |
| Thresholds of the ground classifier wrong in sun | medium | grass contact → invalid run | outdoor test at noon and afternoon (H9); live-tunable parameters; replay tool |
| Course furniture moved on exam day | unknown | landmark updates wrong | gate rejects; no fix is the normal case (P3); question Q-7 |

---

## 18. Open questions and decision log

### 18.1 Open questions that affect the code

*These must be answered before the named phase.*

| # | Question | Owner | Needed by |
|---|---|---|---|
| OQ-1 | Final IMU part (LSM6DSV / ISM330DHCX)? Determines the backend and register values of §7.11.1. | team | phase 2 (week 7) |
| OQ-2 | Ground-sensor mounting: height `gnd_h0`, lateral offsets `gnd_y_in/out`, distance ahead of the axle `gnd_x`. | team | phase 3 |
| OQ-3 | Course survey: segment geometry, boundary types and corridor widths, landmark positions, ramp and step (F-1 … F-14 of the technical reference). | team | phase 5 (geometry earlier for wheel size) |
| OQ-4 | Radio: keep the HC-12 at 9600 Bd, raise to 19200 Bd, or replace with a certified module? Only §8.3 timing numbers change. | team / professor (Q-8) | phase 4 |
| OQ-5 | May the PC receive telemetry during graded AUTO runs? If not, `com1` stays silent in AUTO (`tlm_period_ms` = 0). | professor (Q-11) | phase 4 |
| OQ-6 | How many vehicles on the course at once? Affects the reflex plausibility gate and the radio channel plan. | professor (Q-6) | phase 6 |
| OQ-7 | Is waypoint accuracy taken per waypoint, as a mean, or as the worst case? Affects `nav_final_dist` and the hold logic only marginally, but decides tuning priorities. | professor (Q-12) | phase 7 |
| OQ-8 | Does the start-heading jig exist? Without it the world frame is defined by placement accuracy (≈ 1°). | team | phase 3 |
| OQ-9 | Will a mechanical E-stop be wired into the EN lines? | team | phase 0 (wiring) |

### 18.2 Decisions taken in this specification

Each decision has a short rationale. Each can be reversed; the column "To reverse" names what would change.

| # | Decision | Rationale | To reverse |
|---|---|---|---|
| D-1 | Mission, guidance and wheel control in **one 50 Hz slot**; EKF at 200 Hz | count-difference speed is right at 50 Hz; no cross-rate inconsistency; guidance bandwidth ≪ 50 Hz | `S_CTRL` divider 5 + `EST_MT` |
| D-2 | **Unified reference tracking**: every closed-loop mode feeds `(x_ref, v_ref, a_ref)` to the same `whl` controller | one tested controller for D-moves, turns, path following and manual driving | separate speed-only mode (`pos_loop = false` exists) |
| D-3 | **SI units** internally, legacy units only at the console | one unit system for the EKF and guidance; float32 is ample (20 µm at 200 m) | — |
| D-4 | TIM6 at **1 kHz** as the only scheduler; the calibration build forces 50 Hz | encoder snapshots and slot phases need a fast base tick | — |
| D-5 | VL53L1X: lateral sensors on **EXTI** (PA15, PA12) with timestamps, the others **polled** | timing matters for landmarks, not for the ground and forward sensors; saves six pins | GPIO1 of the others on free EXTI lines |
| D-6 | VL53L1X **hot path non-blocking** with own register-level code; ULD only for init | blocking HAL I²C would cost ~25 % of the main loop and violate REQ-EXE-05 | — |
| D-7 | Parameters **in RAM**, no flash persistence; `L` dumps `#define`s | no flash wear or sector-erase stalls; defaults stay reviewable in Git | flash sector 7 with CRC |
| D-8 | **Binary protocol** on the radio, text on USB, both switchable | radio airtime; human console on the cable | — |
| D-9 | Normal stop = **brake** (EN high, duty 0); EN low only in `IDLE`/`FAULT` and hard stops | defined stopping; coast is uncontrolled | — |
| D-10 | IMU through a **backend interface**; LSM6DSV default at 960 Hz, ±500 dps; SPI at 5.6 MHz | part still open; ODR-independent integration | — |
| D-11 | **Final 0.30 m in the encoder frame** (`MOVE_FROM`), target frozen | jump-free stop, reuses the proven `D` end-of-move logic | stop on the EKF estimate |
| D-12 | **Heading frozen at standstill** (zero-rate assumption) | no drift during 10.5 s holds | — |
| D-13 | **NHC not implemented** as an update | already contained in the 5-state process model (technical reference §7.2) | add a lateral-velocity state |
| D-14 | Gyro noise as **q_θ·dt** plus scale-factor term instead of `G M Gᵀ` | equivalent for the θ row; simpler, directly from the Allan variance | — |
| D-15 | Path line placed **0.20 m from the reference edge** (midpoint of the ground footprints) | nominal ground state "edge between sensors" gives continuous edge events | per-segment offset in YAML |
| D-16 | Ground classification uses the **rolling range spread**, not a sigma output | the ULD result has no sigma value | full ST API (+80 KB) |
| D-17 | **ADC polled**, no DMA | one channel at 50 Hz; frees DMA2 S4 | DMA circular if channels are added |
| D-18 | **Kick removed**, breakaway feedforward switching instead | the kick acted per wheel and caused heading errors at start | `KICK` parameters re-added |
| D-19 | Hard-fault record in **RTC backup registers** | survives reset without editing the linker script | `.noinit` section |
| D-20 | Mission frequency **50 Hz instead of 20 Hz** | see D-1 | — |
| D-21 | **Phase-0 API adjustments for pure modules.** `par_dump_defines(com_port_t)` becomes `par_format_define(i, buf, n)`, one line per call; PAR_S gating uses `par_set_stationary(bool)`; the `par_set` event is logged by the caller. `drv` returns the drivetrain fault in `drv_out_t.fault` instead of calling `sys_fault_raise()`; `app` raises it. `con_poll()` returns the parsed command; `app_cmd.c` executes it. | `par`, `drv` and `con`'s callers must not depend on `com_uart`/`sys_fault.c` (REQ-ARC-02) so the host tests link; `app` is the only place that moves data between modules (REQ-APP-01) | `par`/`drv` call the system modules directly and the tests stub them |
| D-22 | `whl`: the integrator is cleared **before** it is used when command and reference are both zero (spec §7.8 lists the clear last) | identical to the old `Wheel_Control`; the other order feeds one control tick of stale integrator into a stopped wheel | move the reset below the `u` computation |
| D-23 | `PROBE` (PC8) is high during `S_CTRL` (§5.6, REQ-EXE-11); the tick-ISR toggle of §5.2 step 5 is not implemented | both cannot drive one pin; H2 measures the slot | add a second probe pin |
| D-24 | `W<deg>` in phase 0 uses `drv_v_nom` and `drv_t_acc` for the wheels (ω = v_nom / (b/2)) | there is no turn-rate parameter before `nav_turn_w` (phase 3) | `W` takes `nav_turn_w`/`nav_turn_ta` once `nav` exists |
| D-25 | Boot-time platform mismatches (TIM1 ARR ≠ 8999, TIM6 PSC/ARR ≠ 89/999, TIM2 PSC ≠ 89, clock check) all raise `F_CLOCK` (R4) | the catalogue has no code for "CubeMX not regenerated"; the reaction wanted is the same | add `F_PLATFORM` |
| D-26 | The hard-fault record in the backup registers holds the code, MSP and the first 12 stack words raw, not PC/LR | the handler's own prologue shifts the exception frame by an unknown amount, so PC/LR cannot be picked reliably from MSP | naked-asm entry that selects the frame from `EXC_RETURN` |
| D-27 | `main.c` includes `calib.h` only under `APP_CALIB`; the pre-spec code is kept under `#if 0` until H3 passes (§16.3 step 3); `app_hw_t` has `adc1` as `void *` until CubeMX generates ADC1 | `calib.c` is not in the repository; the build must not depend on it | delete the `#if 0` blocks after H3; type `adc1` when ADC1 exists |

---

## 19. Appendix

### 19.1 Glossary

| Term | Definition |
|---|---|
| ARR | auto-reload register of a timer: counter period minus one |
| BDU | block data update (IMU): high and low bytes of a sample always belong together |
| CCR | capture/compare register of a timer channel; PWM duty, or a captured timestamp |
| COBS | Consistent Overhead Byte Stuffing: framing that removes zero bytes |
| CSS | clock security system: HSE failure → HSI + NMI |
| DRDY | data-ready output of a sensor |
| DWT | data watchpoint and trace unit; contains the CPU cycle counter |
| EKF | extended Kalman filter |
| EXTI | external interrupt controller; lines are shared by pin number |
| FF | feedforward |
| IWDG | independent watchdog (own LSI clock) |
| M/T method | speed estimation from counts and the time between count changes |
| NIS | normalised innovation squared, `νᵀS⁻¹ν`; consistency measure of the EKF |
| NHC | non-holonomic constraint: zero lateral velocity |
| ROI | region of interest: active SPAD sub-array of the VL53L1X |
| SIMC | Skogestad internal-model-control tuning rules |
| SPSC | single-producer single-consumer ring buffer |
| ULD | VL53L1X Ultra Lite Driver (ST) |
| UMBmark | square-path test separating wheel-diameter and wheelbase errors |
| ZUPT | zero-velocity update: standstill used as a measurement of the gyro bias |

### 19.2 Conversion of the current constants

| `main.c` (10 Oct) | Value | Unit | Parameter | SI value |
|---|---|---|---|---|
| `FF_KS_L_PCT` / `FF_KS_R_PCT` | 9.8 / 4.1 | % | `whl_ks_l` / `whl_ks_r` | 0.098 / 0.041 |
| `FF_KV_L` / `FF_KV_R` | 0.1275 / 0.1361 | %/(mm/s) | `whl_kv_l` / `whl_kv_r` | 1.275 / 1.361 1/(m/s) |
| `KV_L` / `KV_R` (bench) | 0.00492 / 0.00507 | duty/rpm | (reference) | 1.294 / 1.334 1/(m/s) via ·60/(π·0.0726) |
| `KS_L` / `KS_R` (bench) | 0.110 / 0.127 | duty | (reference) | 0.110 / 0.127 |
| `KS_BREAK_L/R_PCT` | 14.1 / 16.9 | % | `whl_ksb_l` / `whl_ksb_r` | 0.141 / 0.169 |
| `TAU_PLANT_S` | 0.050 | s | `whl_tau_m` | 0.050 s |
| `KP_SPEED` / `KI_SPEED` (SIMC) | 0.045 / 0.898 | %/(mm/s), %/(mm/s·s) | `whl_kp_v` / `whl_ki_v` | 0.45 1/(m/s) / 8.98 1/m |
| `KP_SPEED` / `KI_SPEED` (older build) | 0.10 / 0.50 | idem | — | 1.0 / 5.0 |
| `INTEG_MAX_PCT` | 30 | % | `whl_i_max` | 0.30 |
| `KP_POS` | 2 | (mm/s)/mm | `whl_kp_x` | 2.0 1/s |
| `V_CORR_MAX_MM_S` | 100 | mm/s | `whl_v_corr_max` | 0.10 m/s |
| `k_sync` (`Y`) | 3.0 | (mm/s)/mm | `drv_k_sync` | 3.0 1/s |
| `SETTLE_V_MM_S` | 20 | mm/s | `drv_v_settle` | 0.020 m/s |
| `POS_TOL_MM` | 2 | mm | `drv_pos_tol` | 0.002 m |
| `SETTLE_TICKS` | 50 | ticks @ 50 Hz | `drv_settle_timeout` | 1.0 s |
| `FAULT_TICKS` | 30 | ticks @ 50 Hz | `whl_fault_time` | 0.6 s |
| `V_MOVING_MM_S` / `V_STOPPED_MM_S` / `V_ZERO_MM_S` | 8 / 3 / 2 | mm/s | `whl_v_moving` / `_stopped` / `_zero` | 0.008 / 0.003 / 0.002 m/s |
| `SPEED_FILTER_ALPHA` | 0.5 | – | `whl_speed_alpha` | 0.5 |
| `WHEEL_DIAMETER_MM`, `COUNTS_PER_WHEEL_REV` | 72.6, 3584 | mm, counts | `odo_m_per_cnt` | 6.3638·10⁻⁵ m |
| `MM_PER_COUNT_L/R` factor | 1.0000 | – | `odo_scale_l/r` | 1.0 |
| `WHEEL_BASE_MM` | 223.25 | mm | `odo_wheelbase` | 0.22325 m |
| `V_NOM_DEFAULT` | 300 | mm/s | `drv_v_nom` | 0.30 m/s |
| `V_NOM_MAX` | 800 → 450 | mm/s | `drv_v_max` | 0.45 m/s (O-5) |
| `ACC_MS_DEFAULT` / `DEC_MS_DEFAULT` | 1000 / 1000 | ms | `drv_t_acc` / `drv_t_dec` | 1.0 / 1.0 s |
| `DEADMAN_MS` | 2000 | ms | `con_deadman_s` | 2.0 s |
| `PWM_FREQ_HZ` | 20000 | Hz | TIM1 ARR 8999 (CubeMX) | — |
| `CTRL_PERIOD_S` | 0.02 | s | `S_CTRL` divider 20 at 1 kHz | — |
| `KICK_DUTY_PCT` / `KICK_TICKS` | 40 / 5 | % / ticks | removed (D-18) | — |
| `ENC_L/R_SIGN`, `MOT_L/R_SIGN` | +1/−1, −1/+1 | – | `enc`/`mot` constants (harness, not parameters) | — |

Calibration results map as follows: `G` → `whl_kv_*`, `whl_ks_*`; `B` → `whl_ksb_*`; `K`/`G` level changes → `whl_tau_m`; `odo` → `odo_scale_l/r`, `odo_wheelbase`; `L` → `odo_backlash_l/r`; `U` with `A`/`G` → `whl_ff_u_ref` and `whl_ff_norm_en = 1`.

### 19.3 References

- `MR2006B_Vehicle_Technical_Reference.tex/.pdf`, Rev. 1.0, 14 Oct 2026 — models, physics, rationale, error budget, open items.
- `CALIB_README.md` (9 Oct 2026), `BENCH_README.md` (week 5), `CONCEPT_REV_F_CHANGES.md`, `MR2006B_CubeMX_Konfiguration.xlsx` (9 Oct 2026), `MR2006B_Pinout.xlsx` (7 Oct 2026), `TOOLCHAIN.md`.
- Challenge brief *Challenge MR2006B*, Tecnológico de Monterrey, 2026.
- STMicroelectronics: RM0390 (STM32F446 reference manual), UM1724 (Nucleo-64), STM32F446 datasheet, VL53L1X datasheet, UM2510 (VL53L1X ULD), LSM6DSV datasheet.
- Infineon: BTS7960 data sheet.
- J. Borenstein, L. Feng, "Measurement and correction of systematic odometry errors in mobile robots", IEEE Trans. Robotics and Automation 12(6), 1996.
- S. Skogestad, "Simple analytic rules for model reduction and PID controller tuning", J. Process Control 13(4), 2003.
- S. Cheshire, M. Baker, "Consistent Overhead Byte Stuffing", IEEE/ACM Trans. Networking 7(2), 1999.

### 19.4 Revision history

| Version | Date | Author | Changes |
|---|---|---|---|
| 1.0 | 2026-10-14 | Dani / Claude | Initial specification, derived from the technical reference Rev. 1.0 and the project documents |
