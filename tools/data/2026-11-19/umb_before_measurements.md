# UMBmark measurements 2026-11-19 (before correction)

Lecture room, tiled floor. Track 223.25 mm (outer edges 253.5, inner 193). Firmware f7fe0a2-dirty Debug,
P200 = 6.35444e-05 (live), P201 = P202 = 1.0, P203 = 0.22325. Commands and DONE lines: `umb_before.log`.
Hub points in mm, x forward along the first leg, y left, both measured from the axes through O (start: wheels at y = ±111.6 mm). Values are the hub positions, not offsets.

| Run | Direction | R-X | R-Y  | L-X | L-Y  |
| --- | --------- | --- | ---- | --- | ---- |
| 1   | CW        | 550 | 345  | 634 | 549  |
| 2   | CCW       | 127 | -389 | 121 | -163 |
| 3   | CW        | 740 | 408  | 832 | 605  |
| 4   | CCW       | 320 | -426 | 272 | -205 |
| 5   | CCW       | 254 | -328 | 241 | -111 |
| 6   | CW        | 277 | 148  | 327 | 364  |
| 7   | CCW       | 156 | -322 | 133 | -104 |
| 8   | CW        | 399 | 147  | 469 | 360  |
| 9   | CCW       | -17 | -228 | -12 | -5   |
| 10  | CW        | 435 | 246  | 494 | 462  |
| 11  | CCW       | -34 | -222 | -27 | 1    |
| 12  | CW        | 221 | -211 | 226 | 11   |

Log notes: run 1 has the third corner as W90, W-90, W-90 (direction corrected on the floor). One run
was aborted with `s` after its first leg (not in the table). Run 12 started with W90 + W180 instead of
W-90 (a +270 deg turn): excluded. Run 1 excluded as well (corrected corner).

Evaluation (`umbmark.py --wheelbase 0.22325 --scale-l 1 --scale-r 1 --turn-done-mm 173.72`, runs 2-11):
E_b 0.94416, E_d 0.99622, residual 34 mm, P203=0.21078 P201=1.00189 P202=0.99811. Not entered:
the per-pair turn error falls from about 4.7 to 1.7 deg per corner over the session (see W360 check).
