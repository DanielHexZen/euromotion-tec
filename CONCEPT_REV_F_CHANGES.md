# CONCEPT.md — Rev. F Changeset

Apply these edits to `CONCEPT.md` Rev. E. Each block names the target section and states what replaces what. Commit before applying so the diff is readable.

Source: bench bring-up session, week 5 preparation.

---

## §0.1 — replace the Rev. E change table header with this Rev. F table

| Area | Change |
|---|---|
| **Procurement** | **The PH2.0 blocker does not exist.** The motors ship with a plain Dupont connector; the product listing's claim was wrong for this batch. §14.2, §19.1 and §20.4 corrected. |
| **Driver** | §5.2 — truth table measured. The `(1,1)` state does **not** draw current at 6 V with the outputs open, so a bridge shoot-through is ruled out. Risk downgraded from [RISK] to a measured finding; the software prohibition is retained and the reason given. |
| **Encoders** | §13.6 — pull-up resistance measured at ~6 kΩ between A and B, symmetric in both polarities, i.e. two ~3 kΩ pull-ups in series via V+. Gives a non-destructive method for identifying V+ before anything is connected. |
| **Clock tree** | New §15.7 — HSE in **BYPASS** mode, PLL source HSE, PLLM 4. The ST-Link MCO is a clock signal, not a crystal. HSI would cost ±1 % on every timestamp. |
| **Pins** | §15.3 — encoder 2 moves to TIM8 (PC6/PC7) because TIM4 would kill USART1. SPI1 must relocate to PB3/PB4/PB5 in week 8. USART3 must use PC10/PC11 in LQFP64. |
| **Bench firmware** | New §19.3 — open-loop drivetrain bench firmware exists and runs. Console on USART2, test sequences for `Ks`, `Kv`, encoder CPR and the truth table. |
| **Drivetrain, measured** | §5.5 — `Kv` and `Ks` measured per motor over two sweeps each, R² > 0.999. The linear model is confirmed rather than assumed. Static and kinetic friction separated. |
| **Motor pairing** | New §12.9 — the two motors differ by ~12 % in speed and ~20 % in breakaway duty. Measured gearing rules out the gearbox as the cause. |
| **Encoder resolution** | §5.3 — counts per output revolution measured per motor: **3595 (L), 3618 (R)** against a nominal 3603. Gear ratios 56.17 and 56.53. |
| **Telemetry** | §16.4 — a single TX buffer is not safe: HAL clears `gState` when the last byte reaches the data register, not when it leaves the shift register. Confirmed on the bench. |
| **PWM frequency** | §5.2 — the 5 kHz option is now favoured. The measured breakaway sits at 0.14–0.17 against a 0.10 duty floor, leaving little resolution in exactly the APPROACH band. |
| **Wiring** | §13.5 — Dupont contacts are limited to ~1 A. Motor leads must be crimped or soldered, ≥ 20 AWG, before the vehicle drives. |

Keep the Rev. E table, renumbered to §0.2, and shift the older ones down accordingly.

---

## §5.2 — replace the `[RISK]` paragraph

Delete:

> **[RISK]** The vendor description says the module reproduces L298 logic using discrete gates […] A misunderstood mapping that drives both direction inputs active is a half-bridge short.

Replace with:

> **Measured, week 5.** The control logic was verified on the bench with the motors disconnected and the driver fed from a lab supply at 6 V with a 100 mA current limit.
>
> | EN | IN1 | IN2 | OUT1 | OUT2 | Supply current |
> |---|---|---|---|---|---|
> | 0 | 0 | 0 | | | — |
> | PWM | 1 | 0 | | | — |
> | PWM | 0 | 1 | | | — |
> | PWM | 0 | 0 | | | — |
> | PWM | 1 | 1 | | | **no limiting** |
>
> **[MEASURE]** Fill in the OUT columns. The `(1,1)` row is the one that matters: no current at 6 V with the outputs open rules out a bridge shoot-through, but it does **not** distinguish brake from coast. With both outputs tied to the same rail and a motor connected, `(1,1)` short-circuits the motor terminals — not destructive to the bridge, but a real current path and an unmodelled braking torque. The OUT voltages in that state are the only thing that separates the two cases.
>
> **The software prohibition stays regardless.** The measurement covers one supply voltage, at room temperature, with no load. Enforcing the state in a single setter function costs nothing and removes an entire failure class from consideration. The state is reachable only through `motor_logic_probe()`, which exists for this measurement and is never called from the control path.

This answers §20.2 question 4.

### §5.2 — append on the duty floor

> **The duty floor is tighter than Rev. E assumed.** At 10 kHz the floor is 5–10 % of full scale; the measured breakaway duty is **0.141 (L) and 0.169 (R)**, and the APPROACH phase runs at 15–25 %. The usable resolution between "the driver cannot resolve it" and "the motor is turning" is therefore about four percentage points, in the one phase where waypoint accuracy is produced.
>
> **Recommendation: take the 5 kHz option.** Period 200 µs, floor 2.5–5 %, resolution in the APPROACH band doubled. The cost is one prescaler constant and more audible noise, neither of which is graded. §20.3 decision 11 should be closed in favour of 5 kHz once APPROACH behaviour is observed under load.

---

## §5.5 — replace the parameter description with measured values

Insert after the controller equations:

> **Measured, week 5.** Two sweeps per motor, nine duty steps from 20 % to 100 %, no load, linear fit:
>
> | | Motor L | Motor R |
> |---|---|---|
> | `Kv` (duty per output rpm) | **0.00492** | **0.00507** |
> | `Ks` (intercept of the running fit) | **0.110** | **0.127** |
> | Breakaway duty (from rest) | 0.141 | 0.169 |
> | No-load speed at 100 % | 181 rpm | 177 rpm |
> | Linearity R² | > 0.999 | > 0.999 |
>
> **The linear model of this section is confirmed, not assumed.** R² above 0.999 over the full range means `u = Kv·ω + Ks` describes the drivetrain to within the measurement noise, which is what justifies a feedforward term at all.
>
> **`Ks` is the intercept, not the breakaway.** The two differ by ~0.03 on both motors, and that difference is the physical distinction between static and kinetic friction. The feedforward must hit the *running* case: using the higher breakaway value would overcompensate during every normal movement. Starting from rest is the integrator's job, which is precisely the stick-slip mechanism this section describes — now measured rather than argued.
>
> **Motor R breaks away at 0.169, inside the 15–25 % APPROACH band.** The Coulomb feedforward is therefore not a refinement but a requirement: without it, the integrator has to wind through the deadband in the phase that produces the accuracy grade. The Rev. B correction to §12.6 said the deadband compensation was mandatory "for a different reason" than originally stated; this is that reason, with a number on it.
>
> **[VERIFY]** `Kv` scales with the real gear ratio. These constants must be re-derived if the hand-turn check changes `ENC_CPR_OUTPUT_F`, and re-measured once the wheels are fitted.

---

## §12.9 — new section

### 12.9 Motor pairing

The two delivered motors are not matched. Measured at identical duty, no load:

| Quantity | Motor L | Motor R | Difference |
|---|---|---|---|
| Speed at 30 % duty | 39.3 rpm | 35.1 rpm | **+12 % left** |
| `Kv` | 0.00492 | 0.00507 | 3 % |
| Breakaway duty | 0.141 | 0.169 | **+20 % right** |
| No-load speed | 181 rpm | 177 rpm | 2 % |
| **Gear ratio** | **56.17** | **56.53** | **0.64 %** |

**The gearing is not the cause.** The measured ratios differ by 0.64 %, two orders below the speed difference. What remains is motor constant and friction — the latter dominating, since the breakaway difference (20 %) is far larger than the `Kv` difference (3 %).

Visible to the naked eye on the bench: at a common duty the left motor starts at the third 5 % increment and the right only at the fourth.

**This is the systematic error UMBmark exists to correct** (§17.1), and the reason the velocity controller of §5.2 is per wheel rather than shared. Three consequences worth stating:

1. **Open loop, the vehicle would drive an arc.** A shared duty command produces a curvature of roughly `Δv/b` — at 12 % difference and 0.4 m/s nominal, an unacceptable rate.
2. **The per-motor feedforward removes most of it before the loop closes.** With §5.5's constants applied individually the residual is the load difference only.
3. **The closed loop removes the rest**, which is why speed differences are not a hardware selection problem. Matching a pair by purchase would be expensive and would still drift with wear; measuring and compensating them is cheap and self-maintaining.

**[VERIFY]** Re-measure once wheels and vehicle mass are on. Friction under load may not scale the same way on both sides.

---

## §5.3 — append after the resolution table

> **Measured, week 5.** Mean of three output-shaft revolutions per motor, hand-turned:
>
> | | Motor L | Motor R | Nominal |
> |---|---|---|---|
> | Counts per output revolution | **3595** | **3618** | 3603 |
> | Deviation from nominal | −0.22 % | +0.42 % | — |
> | Real gear ratio (÷ 64) | **56.17** | **56.53** | 56.3 |
>
> The catalogue 56.3:1 is confirmed to within half a percent on both units. 16 PPR per channel is confirmed directly and independently: turning the *rear* encoder shaft one revolution gives exactly 64 counts on both motors, which is 16 × 4 quadrature with no gearbox in between.
>
> **What this rules out.** The 12 % speed difference of §12.9 is **not** gearing — the two gearboxes differ by 0.64 %. The difference is motor constant and friction, and that is a cleaner separation of causes than an unmeasured assumption would have allowed.
>
> **What it does not make negligible.** 0.64 % between the two sides sits in the same order as the `ε ≈ 0.5 %` scale error the error budget assumes after UMBmark (§18.1). Over a 40 m segment that is 26 cm of along-track error. The firmware therefore carries **two** constants, not an average — entering them correctly costs nothing now and removes a term that would otherwise have to be hunted later.

---

## §12.6 — append to the selection paragraph

> **Connector correction.** The product listing states *"Requiere conector PH2.0 mm con cable 26 AWG de 6 pines (no incluido)"*. This is wrong for the delivered batch: the motors carry a plain 6-pin Dupont connector. Verify on arrival rather than ordering against the listing — this cost a week of schedule anxiety for a part that was never needed.

---

## §13.5 — append a subsection

### Connector current rating

Dupont contacts are rated at roughly 1 A each, and their contact resistance rises with oxidation and insertion cycles. Consequences:

| Line | Bench, no load | Vehicle |
|---|---|---|
| Encoder A/B, V+, GND | Dupont fine | crimp or solder |
| Motor winding | Dupont acceptable at 180 mA no-load | **crimped or soldered, ≥ 20 AWG** |
| Battery to driver | never Dupont | 14–16 AWG, screw terminal (§13.7) |

Two reasons, and the second is the one that is easy to miss:

1. **Heat.** 4.3 A stall current through a 1 A contact.
2. **A variable series resistance.** After fifty insertion cycles an oxidised contact adds milliohms that drift. That resistance sits in series with the motor and is invisible to `Kv`, so the §5.5 feedforward is calibrated against a constant that no longer holds. This is the same class of unobservable error as a slipping wheel hub (§12.8).

For the bench measurements on a stand the motor draws 180 mA and Dupont is acceptable. Before the vehicle drives, the two motor leads come off the Dupont header and the six-way connector keeps only the four encoder wires.

---

## §13.6 — replace the `[ASK]` paragraph

Delete the paragraph beginning *"**[ASK]** Verify the connector pinout with a multimeter…"*.

Replace with:

> **Measured, week 5.** Between encoder channels A and B: **~6 kΩ, symmetric under probe reversal.** Symmetry rules out a semiconductor junction, so these are real resistors — two pull-ups of roughly 3 kΩ each, in series via the V+ rail. Consistent with open-collector Hall outputs.
>
> **This identifies V+ without powering anything:**
>
> | Wire measured against both A and B | Identification |
> |---|---|
> | ~3 kΩ to **both** channels | **Encoder V+** — the pull-ups hang here by definition |
> | open / high to both | Encoder GND |
> | a few ohms to one other wire | Motor winding — leave alone |
>
> Confirm before connecting: no encoder wire may show a low-resistance path to the winding pair.
>
> **Consequence for the input network.** 3 kΩ against the 1 kΩ / 1 nF RC filter gives a rise time of ~4 µs. The fastest quadrature edge spacing at 10 000 rpm motor speed and 16 PPR is ~94 µs, so the filter costs 4 % of the edge spacing. No external pull-up is needed. If phantom counts appear once the motors run, an added 2.2 kΩ to 3.3 V per channel is the first and cheapest measure, ahead of touching the timer input filter.
>
> **Record the 6 kΩ per motor in the goods-in list.** A later comparison against this figure is the fastest way to spot a blown output transistor or a detached pull-up.

This answers §20.2 question 3.

---

## §14.2 — replace the whole section

### 14.2 Order 2 — outstanding

The PH2.0 line is deleted. The motors ship with a Dupont connector and can be connected as delivered.

| Item | Qty | Where | Blocks |
|---|---|---|---|
| XT60 mating connector / pigtail | 2 | local | Battery cannot be connected |
| Inline fuse holder + 10 A slow-blow fuse | 1 | local | First power-on (§13.7) |
| Inline fuse holder + 2 A fast fuse | 1 | local | Logic rail protection |
| Main power switch, ≥ 10 A | 1 | local | First power-on |
| Buck converter, fixed 5 V output | 1 | local | Sensor supply — §13.7, no trimpot |
| Standoffs for the driver module | 1 set | local | Vendor warns the underside can short |
| Electrolytic capacitor 470–1000 µF, 25 V | 1 | local | Not in the ceramic kit (§13.5) |
| LiPo safety bag | 1 | local | Charging safety (§13.3) |
| Crimp contacts + 20 AWG wire for motor leads | — | local | Vehicle operation (§13.5) |

Nothing here blocks the bench measurements. The Nucleo, the driver on a lab supply, and one motor are sufficient for everything in §19.3.

---

## §15.3 — replace the peripheral allocation table notes

Add above the existing table:

> **Three pin constraints that CubeMX only reports when the last peripheral is added.** All three belong in the reservation table in `TOOLCHAIN.md` before the first CubeMX session.
>
> **a) The second encoder cannot use TIM4.** TIM1_CH1/CH2 exist only on PA8/PA9 in LQFP64, and PA9/PA10 are USART1's default pins. USART1's only alternative mapping is PB6/PB7 — which is also TIM4's only encoder mapping. Putting encoder 2 on TIM4 therefore removes USART1 entirely, and §15.3 needs it for a forward TF-Luna. **Encoder 2 goes on TIM8 (PC6/PC7).**
>
> **b) SPI1 must move for the IMU.** PA6/PA7 are SPI1_MISO/MOSI and are taken by TIM3 encoder mode. SPI1 relocates to **PB3/PB4/PB5**, which costs SWO trace output — acceptable, since debug output goes over the USART2 console. Set the debug interface to *Serial Wire* rather than *Trace Asynchronous Sw* when SPI1 is added in week 8.
>
> **c) USART3 must use PC10/PC11.** PB11 does not exist in LQFP64.

Update the GPIO row of the table to include `PC8 — slot load probe (§15.2)`.

---

## §15.7 — new section

### 15.7 Clock tree

| Setting | Value | Why |
|---|---|---|
| HSE | **BYPASS Clock Source**, 8 MHz | The NUCLEO-F446RE has **no crystal fitted at X3**; the 8 MHz comes from the ST-Link MCO, which is a clock *signal*. BYPASS is the mode for that, and it frees PH1. |
| PLL source | **HSE** | |
| PLLM / PLLN / PLLP | **4 / 180 / 2** | 8/4 = 2 MHz at the VCO input, which is ST's recommended value for minimum jitter. → 180 MHz. |
| AHB | /1 → HCLK 180 MHz | |
| APB1 | /4 → PCLK1 45 MHz, **timer clock 90 MHz** | TIM2, TIM3, TIM6 |
| APB2 | /2 → PCLK2 90 MHz, **timer clock 180 MHz** | TIM1, TIM8 |
| LSE | **disabled** | No RTC, no LSE-clocked peripheral. X2 is fitted on this board but unused — a fitted crystal that nothing needs stays unused. |
| LSI | on | Required for IWDG (§15.4) |
| CSS | **enable** | See below |

**Why not simply disable HSE and run the PLL from HSI.** The PLL would still reach 180 MHz, so nothing visibly breaks — which is what makes this the dangerous option.

| Source | Tolerance over temperature |
|---|---|
| HSI (internal RC) | **±1 %** over 0–85 °C |
| ST-Link MCO | **±30 ppm** |

A 1 % clock error is not noise. It is a constant scale factor on every timestamp, and §15.1 puts every integrated quantity downstream of those timestamps: encoder-derived velocity, gyro bias estimation, ZUPT averaging. Worse, the HSI tolerance is temperature-dependent — so a morning calibration would not hold at midday in the sun, which is exactly the error class §10.4 installs temperature compensation to avoid. There is no reason to accept it when the accurate source is already on the board.

**Crystal/Ceramic mode also "works"** — the oscillator amplifier, overdriven by the MCO's square wave, follows it and HSERDY is set. It is the same class of accident as the 74AHC244 threshold in §13.1: fine on the bench because a level happens to suffice, marginal under heat and noise. Use the specified mode.

**CSS (Clock Security System).** If the external clock fails, the hardware falls back to HSI and raises an NMI. For a vehicle that is the difference between a silent total failure mid-circuit and a logged telemetry event. The NMI handler must stop the motors — same category as the IWDG reset path in §15.4, and it belongs in that section's failsafe list.

---

## §16.4 — append to the TX buffer paragraph

The existing text already requires two alternating buffers plus a busy flag. Add the reason, because the failure was reproduced on the bench and the mechanism is not obvious:

> **Why the busy flag alone is not enough.** `HAL_UART_Transmit_IT` returns `gState` to `READY` when the last byte has been written to the **data register**, not when it has left the shift register. At 115200 baud that leaves one character time — 87 µs — during which a single-buffer implementation will happily overwrite bytes the hardware has not yet transmitted. The symptom is individual characters missing at random positions, which reads like a baud-rate or noise problem and is neither.
>
> At the telemetry's 9600 baud the window is twelve times longer, and a 50 ms transmission against a 100 ms period leaves far less margin than the bench case. Two alternating buffers are mandatory, not defensive.

---

## §19.1 — replace the action table

Order 1 is placed (§14.1) and the PH2.0 blocker turned out not to exist.

| Action | Priority | Blocks |
|---|---|---|
| **Measure the course** (§20.1) — one afternoon with a tape measure | **blocking** | Wheel size, caster size, motor margin, chassis width, error budget |
| **Ask the professor: does 2 driven + 2 casters satisfy "four-wheeled"?** | **blocking** | Week 5 review, drivetrain, §4 kinematics, error budget |
| **Decide build stage 1 vs. 2** (§10.1) | **blocking** | Power sizing, UART and DMA allocation, §10 structure, flash |
| ~~Source the PH2.0 cable~~ | **resolved** | Motors ship with Dupont (§12.6) |
| ~~Verify the driver truth table~~ | **done** | §5.2 — measured, one column outstanding |
| ~~Verify the motor connector pinout~~ | **method established** | §13.6 — pull-up signature identifies V+ |
| Order 2: XT60, fuses, switch, buck, standoffs, electrolytic (§14.2) | high | First power-on of the vehicle |
| Crimp contacts and 20 AWG for the motor leads (§13.5) | high | Vehicle operation |
| Ask the vendor whether 3.4 kgf·cm is rated or stall torque | medium | Ramp margin, wheel size |
| Coordinate HC-12 channels with other teams | medium | Week 10 |
| Pin reservation table in `TOOLCHAIN.md` — including §15.3 notes a–c | high | All wiring, week-8 IMU bring-up |
| **DMA stream allocation** verified against RM0390 (§15.5) | high | First CubeMX session for stage 2 |
| Order the second battery (§13.2) | high | Any three-hour field session |

**The course measurement is now the single remaining blocking action of a technical nature.** The other two blockers are a question and a decision, both answerable this week.

---

## §19.3 — new section

### 19.3 Bench firmware

Open-loop drivetrain bench firmware, running as of week 5. Documented in `BENCH_README.md`.

Scope: manual duty over a serial console, hardware quadrature acquisition, encoder-based stall detection, and the test sequences that produce §17.4's constants — `Ks`, `Kv`, encoder counts per output revolution, and the §5.2 truth table.

Deliberately excluded: the velocity PI (week 8), the EKF, DMA. §22 principle 12 — commission one layer at a time.

**The §5.5 feedforward is commissioned, the feedback is not.** Manual commands are interpreted as output-shaft speed and converted per motor through the measured `Kv` and `Ks`. This removes the systematic pairing difference of §12.9 and nothing else: there is no feedback, so a load difference between the two sides still produces a speed difference. Commissioning the feedforward first is the correct order — the loop in week 8 then closes on a plant whose static behaviour is already known and compensated.

The deadband and `Kv` sweeps deliberately bypass the feedforward and command raw duty, otherwise they would measure the compensation rather than the motor.

**Two defects found and fixed in the bench firmware, both with consequences beyond it:**

- **The stall supervisor aborted the deadband sweep.** The sweep ramps duty deliberately above the stall threshold while the shaft is still stationary — that dwell *is* the measurement. The supervisor cannot distinguish "jammed" from "has not broken away yet", so it is suspended for the duration of that sequence. A safety mechanism and a measurement procedure contradicted each other; worth remembering when the same supervisor guards the vehicle.
- **The single TX buffer race** — see §16.4.

Architecture is the cyclic executive of §15.2 in miniature: a 1 kHz TIM6 tick drives 200 / 100 / 20 / 10 Hz slots, nothing blocks, console output goes through a ring buffer with drop-on-full rather than blocking transmission (§16.4 discipline). `printf` is not used and `_write()` is not overridden — a blocking 80-character transmission at 115200 baud is 7 ms inside a 1 ms tick, which is the §15.1 failure in miniature.

---

## §20.2 — update the question table

| # | Question | Status |
|---|---|---|
| 1 | Is 3.4 kgf·cm the rated or the stall torque? | open — vendor |
| 2 | Restock lead time for HS5141-178 | open — vendor |
| 3 | Connector pinout — which wire is encoder V+? | **method established**, §13.6. Confirm per motor at goods-in |
| 4 | Actual truth table of the H-bridge control logic | **answered**, §5.2. OUT voltages in the `(1,1)` state still to be recorded |
| 5 | **Does two driven wheels plus two casters satisfy "four-wheeled"?** | **open — professor, highest urgency** |
| 6 | How many vehicles on the course simultaneously? | open — professor |
| 7 | Will the furniture be in the same position on exam day? | open — professor |
| 8 | 433 MHz vs. 902–928 MHz in Mexico | open |
| 9 | HC-12 operating mode FU1–FU4 and over-the-air rate | open |

Add to the verification list:

| # | Item | Against what |
|---|---|---|
| V6 | `(1,1)` output state: brake or coast? | Multimeter on OUT1/OUT2, 6 V, 100 mA limit |
| V7 | `.cproject` build flags survive CubeMX regeneration | `git diff` after every regeneration |

---

## §20.3 — decision updates

| # | Decision | Update |
|---|---|---|
| 11 | PWM 10 kHz vs 5 kHz | **Recommend 5 kHz.** Measured breakaway 0.14–0.17 against a 0.10 floor leaves ~4 points of resolution in the APPROACH band (§5.2) |
| — | *(new)* Per-motor feedforward constants | **Measured and adopted** — §5.5 |

---

## §20.4 — risk register edits

Delete:

| Risk | Impact | Mitigation |
|---|---|---|
| ~~PH2.0 cable not sourced in time~~ | ~~Motors unusable, week 5 missed~~ | **Resolved** — the motors ship with a Dupont connector (§12.6) |
| ~~H-bridge truth table differs from assumed~~ | ~~Half-bridge short, destroyed driver~~ | **Resolved** — measured, no current draw in the `(1,1)` state at 6 V (§5.2) |

Add:

| Risk | Impact | Mitigation |
|---|---|---|
| Motor current through Dupont contacts | Heat, and a drifting series resistance invisible to `Kv` | §13.5 — crimped or soldered leads, ≥ 20 AWG, before the vehicle drives |
| CubeMX regeneration silently reverts timer values and build flags | Wrong tick rate, lost `-Wdouble-promotion`; both present as mysterious runtime behaviour | `TOOLCHAIN.md` regeneration protocol: commit first, `git diff .cproject` and `main.c` after |
| PLL left on HSI | ±1 % temperature-dependent scale error on every timestamp | §15.7 — HSE BYPASS, PLLM 4, verify in the generated code |
| Single UART TX buffer overwritten mid-transmission | Corrupted telemetry frames that look like a radio or noise problem | §16.4 — two alternating buffers; `gState` returns to READY before the shift register empties |
| Safety supervisor contradicts a measurement procedure | The mechanism aborts the sequence it exists to protect | §19.3 — scope the stall check by mode; re-examine whenever a new sequence deliberately stalls a wheel |
| Motors differ by 12 % in speed at equal duty | Open-loop driving describes an arc | §12.9 — per-motor feedforward now, per-wheel PI in week 8, UMBmark in week 12 |
| External clock failure mid-run | Silent total failure | §15.7 — enable CSS, NMI handler stops the motors |

---

## §17.4 — calibration status

| Item | Status |
|---|---|
| Encoder counts per revolution | **done**, per motor — 3595 / 3618 (§5.3) |
| Motor deadband `Ks` | **done**, per motor, four and three repeats — §5.5 |
| Motor gain `Kv` | **done**, per motor, two sweeps each, R² > 0.999 — §5.5 |
| Everything else | unchanged — UMBmark, Allan variance, sensor extrinsics all still ahead |

**The bench phase of §17.4 is complete.** Truth table, direction, encoder sign, counts per revolution, `Ks` and `Kv` — all measured, all per motor. `Kv` shifts by the 0.2–0.4 % gearing correction, which is inside the scatter of the sweeps themselves, so no re-measurement is needed.

Worth stating for the defence: the no-load speed landing within 2 % of the catalogue 178 rpm was *not* taken as confirmation of the gear ratio. Two errors can cancel. The ratio was measured separately, and only then did the agreement become evidence.

---

## §22 — append a principle

13. **Measure the constant, do not inherit it.** `Ks`, `Kv` and the gear ratio are all catalogue numbers until an afternoon on the bench turns them into measurements. Two of the three turned out to differ meaningfully between two nominally identical motors — and the third, by turning out not to, is what allowed the other two to be attributed to a cause.
14. **Verify the delivered part, not the listing.** The PH2.0 requirement, the 74AHC244 threshold inside the BTS7960, and the H-bridge truth table were all claims about hardware that only a measurement could settle. Two of the three turned out differently than documented.
