# Drivetrain Bench Firmware — Setup and Procedure

**Status:** running as of week 5. Clock tree, 1 kHz tick, slot dispatch and console output verified with nothing connected.

**Purpose:** satisfy the week-5 milestone (*chassis, motor and driver built and working*) and produce the constants §17.4 of `CONCEPT.md` asks for. Open loop only — the velocity PI belongs to week 8 (§22 principle 12).

**Files:** `bench_config.h`, `motor.h/.c`, `encoder.h/.c`, `bench.h/.c`. Sources in `Core/Src`, headers in `Core/Inc`.

---

## 1. Status of the pre-flight checks

| # | Check | Status | Ref |
|---|---|---|---|
| 1 | Driver truth table, motors disconnected, 6 V / 100 mA | **done** — `(1,1)` draws no current, shoot-through ruled out. OUT voltages still to record | §5.2 |
| 2 | Motor connector pinout | **method established** — the ~3 kΩ pull-up signature identifies encoder V+ without power | §13.6 |
| 4 | Encoder direction and sign | **done** — `ENC_SIGN_L = -1`, `ENC_SIGN_R = +1` | §5.3 |
| 5 | Deadband `Ks`, per motor | **done** — L 0.141, R 0.169 (mean of 3–4 runs) | §5.5 |
| 6 | Duty/rpm `Kv`, per motor | **done** — see §8 below | §5.5 |
| 7 | Encoder counts per output revolution | **done** — 3595 (L), 3618 (R), mean of three revolutions each | §5.3 |
| 3 | Harness commissioning order, steps 1–6 | pending, needed before the *vehicle* is powered, not before bench work | §13.7 |

**The PH2.0 blocker does not exist.** The motors ship with a plain 6-pin Dupont connector; the product listing was wrong for this batch.

### What the `(1,1)` measurement does and does not prove

No current at 6 V with the outputs open rules out a bridge shoot-through. It does **not** distinguish brake from coast: with both outputs on the same rail and a motor connected, `(1,1)` short-circuits the motor terminals — a real current path and an unmodelled braking torque, though not destructive to the bridge. The OUT1/OUT2 voltages in that state are the only thing that separates the two cases. Ten seconds with a multimeter at the next power-up.

The software prohibition in `motor.c` stays either way. The measurement covers one supply voltage, at room temperature, with no load; enforcing the state in a single setter costs nothing and removes a failure class from consideration.

---

## 2. CubeMX configuration

Full tables in `TOOLCHAIN_ADDENDUM.md`. The values that have already caused trouble:

| Item | Value | Symptom when wrong |
|---|---|---|
| **TIM6 ARR** | **9** (PSC 8999) | CubeMX defaults it to 65535 → tick at 0.15 Hz → fragmented console output |
| **HSE** | **BYPASS**, PLLM **4**, PLL source **HSE** | Silent fallback to HSI: ±1 % on every timestamp |
| **USART2 NVIC** | enabled, priority 3 | Console goes permanently silent after the first write |
| **TIM6 NVIC** | enabled, priority 1 | Nothing runs at all |
| TIM8 CKD | **Division by 2** | Asymmetric encoder filtering between left and right |
| PB12–PB15 | GPIO out, **pull-down** | Floating direction inputs between reset and `motor_init()` |
| TIM3/TIM8 | Encoder Mode **TI1 and TI2** | TI1 alone gives ×2 instead of ×4 — a quarter of the expected count |

Project settings: **do not** enable "Use float with printf" — the code formats integers only, and it costs 10–15 KB (§15.6).

---

## 3. main.c — what goes in which USER CODE block

```c
/* USER CODE BEGIN Includes */
#include "bench.h"
/* USER CODE END Includes */
```

```c
/* USER CODE BEGIN 2 */
  bench_init(&htim1,    /* PWM        */
             &htim3,    /* encoder L  */
             &htim8,    /* encoder R  */
             &htim2,    /* 1 MHz base */
             &htim6,    /* 1 kHz tick */
             &hadc1,
             &huart2);
  bench_run();          /* never returns */
/* USER CODE END 2 */
```

```c
/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) {
        bench_tick_isr();
    }
}
/* USER CODE END 4 */
```

Keep SysTick as the HAL time base — if CubeMX is set to use TIM6 for it, this callback collides with `HAL_IncTick()`.

**No `_write()` override is needed.** The code never calls `printf`; `out()` formats with `vsnprintf` into a ring buffer and `tx_pump()` sends it with `HAL_UART_Transmit_IT`. A blocking 80-character transmission at 115200 baud is 7 ms inside a 1 ms tick — the §15.1 failure in miniature.

---

## 4. Console

115200 8N1 on the ST-Link VCP. Use either the CubeIDE Serial Terminal or `screen /dev/cu.usbmodem<n> 115200`, never both — the port is exclusive.

Keys are read as single characters, no Enter needed, and there is no echo. **Open the terminal before pressing reset**, otherwise the help text has already drained.

| Key | Action |
|---|---|
| `h` | help |
| `o` / `x` | arm / E-stop |
| `r` | clear faults |
| `w` `s` | common duty ±5 % |
| `a` `d` | differential ±5 % |
| space | stop, stay armed |
| `L` `R` `B` | select left / right / both |
| `c` | zero encoder counts |
| `p` `b` | telemetry on-off, battery supervision on-off |
| `f` | feedforward on-off. On: `w`/`s` command **speed**; off: raw duty |
| `0` | driver truth-table probe (disarmed, motors disconnected) |
| `1` | direction and encoder-sign check |
| `2` | deadband sweep → `Ks` |
| `3` | duty/rpm sweep → `Kv` |
| `4` `5` | straight / turn-in-place demo, 35 %, 3 s |
| `6` | hand-turn readout |

`L` and `R` govern manual duty, the deadband sweep, the Kv sweep and the truth-table probe. Sequences `1`, `4` and `5` always use both motors.

**Feedforward (`f`, on by default).** Manual commands and the two demo sequences are interpreted as output-shaft speed and converted per motor through the measured `Kv` and `Ks` (§5.5). This removes the systematic pairing difference — otherwise the left motor starts one 5 % increment earlier and runs ~12 % faster at the same duty.

It is **not** a control loop. There is no feedback, so a load difference between the two sides still produces a speed difference; that closes in week 8. Say so if asked during the review — it is the stronger answer, because the feedforward is measured and the loop already knows what it sits on.

The deadband and Kv sweeps always command raw duty regardless of `f`, otherwise they would measure the compensation instead of the motor.

Telemetry is CSV at 10 Hz, integer-scaled:

```
T,ms,mode,uL,uR,rpmL,rpmR,cntL,cntR,mV,flags
```

`u` in 1/1000, `rpm` in 1/100. `p` silences it while working through commands.

---

## 5. Wiring, by phase

### Phase 1 — encoder only, no motor, no driver

| Encoder | Nucleo |
|---|---|
| V+ | 3V3 |
| GND | GND |
| Channel A | PA6 (CN10-13) |
| Channel B | PA7 (CN10-15) |

Motor left on TIM3. Right would be PC6/PC7.

Identify V+ first, without power: the wire showing ~3 kΩ to **both** channels is V+, because the pull-ups hang there by definition. Confirm no encoder wire has a low-resistance path to the winding pair (the two wires reading a few ohms between them).

### Phase 2 — control lines to the driver, still no motor

| Driver | Nucleo |
|---|---|
| EN (left channel) | PA8 (CN10-23) |
| IN1 | PB12 (CN10-16) |
| IN2 | PB13 (CN10-30) |
| Logic GND | GND — only if the module has a non-isolated logic side |

Whether a separate logic VCC exists is item V3 of §20.2; with full galvanic isolation there is neither a VCC pin nor a shared ground.

### Phase 3 — motor connected

Motor to OUT1/OUT2, encoder stays on the STM32.

**Dupont jumpers are fine for everything except the motor winding.** Logic and encoder lines carry milliamps. A Dupont contact is rated around 1 A; 4.3 A stall current through one is a heat problem, and an oxidised contact adds a drifting series resistance that `Kv` cannot see. Acceptable on a stand at 180 mA no-load. Before the vehicle drives: crimped or soldered leads, ≥ 20 AWG, and the six-way header keeps only the four encoder wires.

Twist the A/B pair and the V+/GND pair by hand, keep runs under 20 cm, and solder the three 100 nF capacitors to the motor terminals **before** the motor is installed (§13.5) — they are hard to reach afterwards.

---

## 6. Test procedure

Keys are pressed in the serial terminal.

**Preparation.** `h` to confirm the console, `p` to silence telemetry while working.

**Encoder, motor stationary.** `L` → `c` → `6`. Turn the **output** shaft — the front 6 mm one with the D-flat — and read the count. Any key ends the sequence.

Two traps worth naming:

- **The rear encoder shaft gives 64.** That is one revolution of the *motor*, 16 PPR × 4 quadrature, with no gearbox in between. A useful confirmation of the encoder resolution, but not the measurement you came for.
- **Turn slowly**, and turn three revolutions rather than one. Fast turning can lose edges to the timer input filter; three revolutions divided by three spreads the reading error at the mark.

Mark both the shaft and the housing with a felt pen first — by feel alone you are easily ten percent out, which is the same order as what you are trying to measure.

Measured on this pair: **L 3595, R 3618** against a nominal 3603.

**First motor run.** `o` to arm, then `w` four times to reach 20 %. The motor starts around the third press — below that the setpoint is under the duty floor. Check with `p` that the count runs **positive**. Then space, then `x`.

A negative count under positive duty means flipping `ENC_SIGN_L` in `bench_config.h`, not rewiring.

**Measurements**, `r` between each to clear faults. Run each **two to four times per motor** — the breakaway scatters by about ±5 %, so a single value is not a measurement.

- `1` — direction and sign check, 24 s, prints a verdict per motor
- `2` — deadband sweep, 20 s, gives the breakaway duty
- `3` — duty/rpm sweep, 18 s in nine steps, gives `Kv` and the intercept. Fit the nine points offline; the intercept is what goes into `bench_config.h`, not the breakaway from `2`

Let a sequence finish before starting the next. Any key aborts, and an abort mid-sequence leaves the previous output still draining — two overlapping runs are what produced the interleaved lines earlier.

Wheels off the ground, or better still not fitted, so `3` measures a no-load curve rather than a load curve.

**Week-5 acceptance**, once both motors are connected: `B`, then `4` and `5`. Both run at 30 % speed command, ~51 rpm on each shaft.

Worth showing in the review: run `4` once with `f` on and once with `f` off. The count difference printed at the end of each run quantifies the pairing correction directly.

Any key aborts a running sequence.

---

## 7. What the firmware protects you from

Built in:

- `IN1 = IN2 = 1` unreachable outside the probe function — deactivate-before-activate ordering, no transient
- 50 ms enforced coast on every direction reversal
- duty slew-limited to 0→100 % in 500 ms
- duty below the driver's minimum pulse width coerced to zero
- encoder-based stall detection: duty above 15 %, shaft below 5 rpm, 300 ms → fault and stop. **Suspended during the deadband sweep**, where a stationary shaft above the threshold is the measurement rather than a fault — do not remove that exception without re-reading why it is there
- dead-man: no keystroke for 2 s while driving → stop
- battery cutoff at 10.0 V, off by default since bench work runs off the lab supply

Not built in, on purpose:

- **No hardware kill.** PWM occupies the enable pin, so the emergency path is software only (§15.4 decision 7). Keep a hand on the lab supply's output switch.
- **No closed-loop control.** Week 8.
- **No `volatile` audit for `-O2`.** Develop at `-Og`, but do the full `-O2` build in week 13 as §19.2 requires.

---

## 8. Constants to replace with measurements

### Measured, week 5

| | Motor L | Motor R |
|---|---|---|
| `Kv` (duty per output rpm) | 0.00492 | 0.00507 |
| `Ks` (intercept of the running fit) | 0.110 | 0.127 |
| Breakaway duty from rest | 0.141 | 0.169 |
| No-load speed at 100 % | 181 rpm | 177 rpm |
| Linearity over 20–100 % duty | R² > 0.999 | R² > 0.999 |

`Ks` in `bench_config.h` is the **intercept**, not the breakaway. They differ by ~0.03 because static friction exceeds kinetic; the feedforward has to hit the running case, and starting from rest is the integrator's job in week 8.

### Still to replace

| Constant | Status |
|---|---|
| `ENC_CPR_OUTPUT_L_F` / `_R_F` | **done**: 3595 / 3618, gear ratios 56.17 / 56.53 |
| `ENC_SIGN_L`, `ENC_SIGN_R` | **done**: −1 and +1 |
| `KV_*_F`, `KS_*_F` | **done**, see the table above |
| `U_MIN_F` | open — scope the driver output to find the real floor |
| `WHEEL_DIA_M_F` | open — gated on the ramp lip measurement |
| divider ratio in `battery_service_20hz` | open — only needed when the vehicle runs off the pack |

**The bench calibration is complete.** What remains is gated on hardware decisions or on the course survey, not on the bench.
| `U_MIN_F` | scoped driver output | APPROACH duty floor (§5.2) |
| `WHEEL_DIA_M_F` | wheel decision after the ramp lip is measured | display only |
| divider ratio in `battery_service_20hz` | measured, if the resistors are 5 % | feedforward normalisation (§5.5) |

Without a divider fitted, PA0 floats and the telemetry shows 100–240 mV of noise. Battery supervision is off by default, so it does not interfere.

## 9. Feeds back into `CONCEPT.md`

§5.2 (truth table, real duty floor), §5.3 (real CPR and gear ratio per motor), §5.5 (`Ks`, `Kv`), §12.6 (whether 3.4 kgf·cm behaves as rated or stall), §12.8 (measured track width `b`), §13.6 (connector pinout), §20.2 items 1, 3, 4, and §20.4.
