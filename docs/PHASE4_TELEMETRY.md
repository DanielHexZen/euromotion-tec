# Phase 4 — Telemetry and binary protocol

Gate for `FIRMWARE_SPEC.md` §15 phase 4: **H11b** (protocol on the bench, D-80) and **H11** (15 min on the radio at 19200 Bd, < 1 % frame loss at ≤ 30 m, link map along the course; on failure repeated at 9600 Bd, D-35), `t_cobs`, `t_proto_msgs`, `t_tlm`, `t_tlm_cap`, `t_link` green. Week-10 milestone in the plan (behind schedule). Vehicle work runs ahead of the phase-3 hardware gates by team decision (D-80); the phase-3 fields follow when the branches meet and repeat H11b.

## Status (2026-11-26)

Packages 1–6 written (D-76, D-80) on `phase4-telemetry`, which holds phase 2 (`main` merged) but not phase 3. `make -C test` green (`t_all`, `t_link`), `gs_log.py --selftest` ok, ARM build clean in Debug, Release `-O2` and `FEAT_CAPTURE 0`/`FEAT_IMU 0`. **Not flashed, not run on the vehicle.** Next: H11b on the bench.

| Item | Track | State |
|---|---|---|
| `proto` (§7.24, §8.2, §8.4) | host | **done** (package 1): `proto.c`, `t_cobs`, `t_proto_msgs` |
| `tlm` pure part: payloads, event queues, window planner, capture core, record packing, duplicates | host | **done** (packages 2, 6): `tlm_pure.c`, `t_tlm`, `t_tlm_cap` |
| `tlm` vehicle part (`tlm.c`): binary receivers, replies, radio window, events, `HELLO`, `CAP_DATA` stream | vehicle | **written** (packages 4–6), host test `t_link` against stubbed ports |
| Port modes (§8.1, REQ-COM-06): `BIN`, `#TEXT`, `MODE_TEXT`; radio starts in `com1_mode` (default `BINARY`) | vehicle | **written**; text to a binary port is dropped (D-80 item 6) |
| USART1 RX circular DMA, TX DMA (REQ-COM-02/04/05) | vehicle | **written**, read by NDTR in the 1 kHz poll, no IDLE interrupt (D-80 item 4) |
| Binary commands (§8.4) in `app_cmd.c` | vehicle | **written**: PING, STOP (also 0xFF), MODE IDLE/BENCH, PARAM_GET/SET, CAPTURE, CAP_GET (USB only), CLEAR, MODE_TEXT; MANUAL/AUTO `ST_STATE`, DRIVE ignored (phases 3/6) |
| Capture (`FEAT_CAPTURE` 1, 48 KB, REQ-TLM-01): sets 0 `WHEEL`, 1 `EST`; console `CAP` | vehicle | **written**; sets 2/3 `ST_NODEV` until phase 3; set 1 `bg` in 1e-5 rad/s (D-80 item 3) |
| `TELEM_SLOW` `stack_free` (stack painting), `rx_err` (D-80 item 5) | vehicle | **written** |
| Ground station `tools/gs_log.py` | host | logger (package 3) plus `--bin`, `--cmd` (all uplink commands, `WAIT`, `TEXT`), live print of replies/events, `cap_wheel.csv`/`cap_est.csv`. No link map by distance yet (H11) |
| Radio hardware | vehicle | both HC-12 at 19200 Bd, CH001, P8, vehicle module on 5 V (D-51, D-71) |
| H11b | vehicle | not started (§4) |
| H11 | vehicle | not started (§5); needs the link map in `gs_log.py` |

**Follows with phase 3 (D-80 item 2), then H11b again for these:** `TELEM_FAST` `mode`/`state` from `msn`, `seg`, `wp`, `e_y`, `gnd`, `fwd` and the reflex/interlock/evade/VL/manual-timeout flags; `TELEM_SLOW` `vl_alive` and the course bit; `HELLO` `course` and VL bit; capture sets 2 `NAV` and 3 `VL`; `MODE` MANUAL/AUTO. `DRIVE` with phase 6.

**Before the first flash:** CubeIDE must pick up the new file `Core/Src/tlm.c` (it is in `Core/Src`, so a refresh of the project is enough; no `.cproject` change). Note: with `com1_mode` 1 (default) the radio console is binary from boot; `P901=0` (or `#TEXT` over the radio) brings the text console back.

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

## 4. H11b checklist (bench, D-80)

Vehicle on blocks, wheels free, battery connected. Logger: `python3 tools/gs_log.py --port <port> --baud <baud> --out tools/data/<date>/h11b_<step> --duration <s> [--bin] [--cmd ...]`. Record the build (`HELLO` git hash) and every log under `tools/data/<date>/`.

1. [ ] Text console unchanged on USB after boot (`?`, `S`, `I`); boot log ends with `ready`; the radio shows no text (binary from boot).
2. [ ] USB binary: `--port <usb> --baud 115200 --bin --duration 900`: `HELLO` printed, `telem_fast` ≈ 9000 rows, `telem_slow` ≈ 900, frame loss 0.00 %. `bat_mv` within 50 mV of `S`; `x_mm`, `th_cdeg`, `bg_mdps` agree with `EKF`.
3. [ ] Back to text: `--cmd TEXT` (raw `#TEXT`), the console answers `?`; again with `--bin --cmd MODETEXT` (`ack result=0`, then text).
4. [ ] Radio: `--port <hc12> --baud 19200 --duration 900 --ping 10 --cmd PGET=902 --cmd PSET=161:0.25 --cmd PGET=161`: `ack` + `hello` every 10 s, `param_val` lines with `result=0`, loss < 1 %, `rx_err` 0 in `telem_slow.csv`. Reset `P161` afterwards.
5. [ ] STOP: start `D1000` on USB text, then over the radio `--cmd STOP`; again with `--cmd STOPALL` (vid 0xFF). Both stop the wheels and print `ack result=0`.
6. [ ] Modes: `--cmd MODE=1 --cmd MODE=0 --cmd MODE=2 --cmd MODE=3`: results 0, 0, 5, 5 (`ST_STATE`). Duplicate suppression (REQ-PRO-05) is covered by `t_link`, not repeated here.
7. [ ] Events: `P161=0.25` on the USB console while the radio logs: `event` line with code 0x101 (EV_PARAM) inside the next windows; `ev_drop` 0.
8. [ ] Capture: on the USB text console `CAP 0 1 30`, then `D1000`; then `--bin --cmd CAPGET=0,65535 --cmd TEXT --duration 20`: `cap_wheel.csv` with continuous indices from 0, one end frame, the console is text again; plot `v_ref` against `v`. Same with `CAP 1 1 30` (`cap_est.csv`). Over the radio `--cmd CAPGET=0,10` gives `ack result=5` and no data.
9. [ ] `I` with both ports binary for 5 min (switch USB back with `#TEXT` to read it): no new `S_CTRL` overruns; `stack_free` ≥ 2048 in `telem_slow.csv`.
10. [ ] Result here and in spec H11b; failures as findings.

## 5. H11 checklist (after H11b)

1. [ ] Both modules at 19200 Bd, CH001, P8 (`AT+RX` on each, record the answers).
2. [ ] Vehicle on the course, laptop with the logger at the start point; walk the vehicle (or drive `MANUAL` once phase 6 exists) along the course out to ≥ 30 m.
3. [ ] 15 min logging; the logger prints frame loss per distance band (`seq` gaps) and the link map.
4. [ ] < 1 % loss at ≤ 30 m: switch confirmed (D-35). Otherwise repeat at 9600 Bd (both modules `AT+B9600`, `.ioc` USART1 back to 9600) and record the decision in spec §18.2.
5. [ ] Logs under `tools/data/<date>/`, result here and in the spec.
