# Rotation check and UMBmark verification 2026-11-19 (partial)

## W360 check (pure rotation, before the new values)

Values live: P200 6.35444e-05, P201 = P202 = 1.0, P203 0.22325. Overshoot read as the lateral offset of a
line 2 m long (assumed over-rotation in both directions).

| Command | Offset at 2 m | Angle | Per 90 deg | P203 = 0.22325 * 718.36 / (720 + angle) |
|---|---|---|---|---|
| 2 x W360 | 446.5 mm | 12.58 deg | 1.57 deg | 0.21892 |
| 2 x W-360 | 364 mm | 10.31 deg | 1.29 deg | 0.21960 |

718.36 deg = 720 deg minus the SETTLE shortfall (about 1.6 mm per 360 deg arc). Mean P203 = 0.21926.

UMBmark (runs 2-11 of `umb_before_measurements.md`) gave 3.1-4.4 deg per corner and P203 0.2108-0.2135;
the pure rotation shows 1.4 deg. The difference is attributed to the casters: after each turn they stand
across and swing round when the next leg starts, turning the vehicle further while the wheels slip
(encoders blind). No odometry parameter can correct that, so P203 comes from the rotation check.
E_d = 0.996 is the same in every UMBmark subset and matches section 4 (k_r/k_l 0.9944): scales from UMBmark.

## Verification with the new values (4 runs, then stopped)

Values live: P200 6.35444e-05, P201 1.00189, P202 0.99811, P203 0.21926. Turn DONE 170.5-171.2 mm
(commanded 172.2 mm). Commands and DONE lines were pasted into the session, no screen log.

| Run | Dir | Result (estimates, not measured to the mm) | Note |
|---|---|---|---|
| 1 | CW | about 30-50 mm off | |
| 2 | CCW | y error about 20 mm, x small | a W90 sent during the leg was refused (Busy) |
| 3 | CCW | about x +30 mm, y -20 mm | a W-90 sent during the leg was refused (Busy) |
| 4 | CW | L x ~170, y ~320; R x ~220, y ~130 mm (hub points) | a caster hung after turn 1 |

Before: return errors 115-1000 mm. After: 20-50 mm without caster event, about 300 mm with one.
H6 (every run < 10 mm) is not reached and not expected with the current casters. Open: 6 more runs
measured to the mm, caster service (clean, check swivel play).
