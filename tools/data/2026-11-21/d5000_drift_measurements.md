D5000 Läufe start bei -111.6mm und 111.6mm  

| X   | 5022mm | 20mm    | 17mm  | 11mm  | 12mm  |
| --- | ------ | ------- | ----- | ----- | ----- |
| y_L | 14mm   | -97.6mm | 17mm  | 11mm  | -20mm |
| y_R | 235mm  | 125.6mm | 238mm | 232mm | 214   |
  

## Evaluation (2026-11-21)

Frame: y left (the start values in the title are swapped: L started at +111.6, R at -111.6), drift
always to the left. x measured to the vehicle middle. Firmware as for the UMBmark repeat (`FEAT_IMU 0`,
P200 6.35444e-05, P201 1.00189, P202 0.99811). DONE 4998.0-4998.9 (`d5000_drift.log`, 6 runs).

Lateral offset of the middle: 124.5, 14.0 (outlier), 127.5, 121.5, 97.0 mm; mean without run 2
117.6 mm -> curvature 2y/L^2 = 0.0094 1/m -> the right wheel rolls 0.21 % more per modelled
metre than the left. New scales (mean kept): P201 1.00084, P202 0.99916 (to be verified live).
Distance: x mean 5016.4 mm against 4998.3 encoder (+18 mm). Same lecture-room tiles as section 4
(2026-11-18, -0.15 %, per-wheel hub marks at two lengths); one length to the vehicle middle cannot
separate an offset from a scale: P200 not changed. The task runs outside on stone tiles, never
measured: re-check P200 there.

Check with P201=1.00084, P202=0.99916 set live (computed 1.00086 / 0.99914, difference negligible), 3 x D5000 (`d5000_check.log`), lateral offset of
the vehicle middle: 95 mm right, 21 mm left, 17 mm left. The right run is an outlier of the same
size as run 2 above (about 100 mm right of the rest): caster. Values compiled (spec D-72).
