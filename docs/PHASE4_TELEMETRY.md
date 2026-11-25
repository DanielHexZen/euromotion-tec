# Phase 4 — Telemetry and binary protocol

Gate for `FIRMWARE_SPEC.md` §15 phase 4: **H11** (15 min on the radio at 19200 Bd, < 1 % frame loss at ≤ 30 m, link map along the course; on failure repeated at 9600 Bd, D-35), `t_cobs` and `t_proto_msgs` green. Week-10 milestone in the plan (behind schedule). Vehicle work after the phase-3 hardware gates; host-track work may start now (D-29).

## Status (2026-11-25)

| Item | Track | State |
|---|---|---|
| `proto` (§7.24, §8.2, §8.4): COBS, header, CRC, one pack/unpack per message | host | only `crc16_ccitt` exists (`proto.c`, test in `t_crc`) |
| `tlm` (§7.25): `TELEM_FAST` 10 Hz, `TELEM_SLOW` 1 Hz, per-port event queues (D-34), capture buffer 48 KB with sets 0–3, `CAPTURE`/`CAP_GET` (D-33) | mixed: framing and queues pure, sending on the vehicle | not started |
| Port modes `TEXT`/`BINARY` (§8.1, REQ-COM-06), `BIN`, `MODE_TEXT`, `#TEXT` escape (REQ-PRO-02), `HELLO` | vehicle | not started; both ports run the text console |
| USART1 RX by DMA circular + IDLE line, TX by DMA (REQ-COM-02/04) | vehicle | USART1 runs by interrupt at 19200 Bd (D-51); DMA streams are in the `.ioc` but unused |
| Radio timing, vehicle as master (§8.3, REQ-PRO-04): 100 ms period, 55 ms vehicle window, one uplink frame | vehicle | not started |
| Ground station (logging, link map, `CAP_GET` download) | host, Python | **no spec yet**: the firmware spec defines only the wire protocol (§1.3, §8) |
| Radio hardware | vehicle | both HC-12 at 19200 Bd, CH001, P8, vehicle module on 5 V (D-51, D-71); text console over the radio tested |
| H11 | vehicle | not started |

## 1. Why now

- `proto` and the `tlm` framing are pure code with fixed formats in §8: they can be written and tested on the host while phase 3 is commissioned.
- The capture buffer was deferred from phase 2 (OQ-11). Until it exists, the 50 Hz data needs come from console streams (`IMUS`, D-74) or the calibration firmware (`K`).
- `whl_tau_m` is **already measured** (0.04975 s, `K` of the calibration firmware **on blocks**, 2026-11-18, `PHASE1_CALIBRATION.md`). The loaded value on the floor can be taken later with capture set 0 (`WHEEL`, 50 Hz: `v_ref`, `v`, duty per wheel) after a `D` step; it matters only for the dynamic feedforward term `Kv·τ_m·a_ref`.

## 2. Work packages, in order

| # | Package | Track | Depends on | Done when |
|---|---|---|---|---|
| 1 | `proto`: COBS encode/decode, frame header (`vid`, `type`, `seq`), CRC, length and `vid` checks, `#TEXT` scan; pack/unpack of every §8.4 message with explicit little-endian (REQ-PRO-01) | host | — | `t_cobs`, `t_proto_msgs` green |
| 2 | `tlm` pure part: `TELEM_FAST`/`TELEM_SLOW` assembly from module outputs, event queues depth 16 per port (drop new when full, `ev_drop`), radio window planner of §8.3 (priority ACK → TELEM → EVENT, airtime budget `tlm_air_max_ms`) | host | 1 | host tests for the window arithmetic at 9600 and 19200 Bd |
| 3 | Ground station minimal logger (Python, pyserial): decode frames, write CSV, frame-loss statistics from `seq`, `HELLO` and `PING` | host | 1 | replay of a recorded USB stream decodes without loss |
| 4 | USART1 DMA RX/TX, port modes, `BIN`/`MODE_TEXT`/`#TEXT`, `HELLO` at boot and on `PING` | vehicle | 1, phase-3 gates | text console unchanged in `TEXT`; binary frames on USB decode in the logger |
| 5 | Radio timing on USART1, uplink window | vehicle | 2, 4 | logger shows 10 Hz `TELEM_FAST`, uplink `PING`/`PARAM_GET` answered |
| 6 | Capture (`FEAT_CAPTURE`): sets 0–3, `CAPTURE`, `CAP_GET` on USB only (D-33) | vehicle | 4 | set 0 and 1 downloaded and plotted; optional: loaded `whl_tau_m` from set 0 |
| 7 | H11: 15 min along the course at 19200 Bd, link map | vehicle | 5 | < 1 % frame loss at ≤ 30 m |

## 3. Open decisions

- **Ground-station spec.** Who writes it and how much: a minimal logger (package 3) is enough for H11 and the capture download; a GUI is not needed for the gate.
- **Capture before `tlm`?** Package 6 could be pulled ahead of 2–5 with a USB-only text dump (like `IMUS`), if 50 Hz wheel or EKF data are needed for phase 3 tuning before the binary protocol exists.
- **Events at 9600 Bd** stay `[DECIDE]` in §8.3: no event fits behind `TELEM_FAST` in the 55 ms window. Irrelevant if H11 passes at 19200 Bd.
- **`[VERIFY]` HC-12 turnaround** (FU3) against the 5 ms guard, during H11.

## 4. H11 checklist (after packages 1–5)

1. [ ] Both modules at 19200 Bd, CH001, P8 (`AT+RX` on each, record the answers).
2. [ ] Vehicle on the course, laptop with the logger at the start point; walk the vehicle (or drive `MANUAL` once phase 6 exists) along the course out to ≥ 30 m.
3. [ ] 15 min logging; the logger prints frame loss per distance band (`seq` gaps) and the link map.
4. [ ] < 1 % loss at ≤ 30 m: switch confirmed (D-35). Otherwise repeat at 9600 Bd (both modules `AT+B9600`, `.ioc` USART1 back to 9600) and record the decision in spec §18.2.
5. [ ] Logs under `tools/data/<date>/`, result here and in the spec.
