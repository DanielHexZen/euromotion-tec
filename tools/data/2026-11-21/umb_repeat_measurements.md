# UMBmark Measurements 2   
  

| Run | Direction | R-X  | R-Y  | L-X  | L-Y  |
| --- | --------- | ---- | ---- | ---- | ---- |
| 1   | CW        | -286 | -465 | -364 | -256 |
| 2   | CW        | -56  | -281 | -95  | -67  |
| 3   | CW        | -249 | -372 | -317 | -156 |
| 4   | CW        | -162 | -296 | -115 | -79  |
| 5   | CW        | -197 | -317 | -153 | -10  |
| 6   | CCW       | 437  | -260 | 384  | -46  |
| 7   | CCW       | 314  | -252 | 274  | -37  |
| 8   | CCW       | 340  | -302 | 297  | -81  |
| 9   | CCW       | 403  | -372 | 354  | -159 |
| 10  | CCW       | 343  | -445 | 381  | -226 |
  
  
Erster: Counters reset  
```
> D2000
Move 2000 mm: Vmax 300 mm/s, T 7667 ms
DONE L=1998.1 R=1998.4 sync_max=1.4 t=8280
> W-90
Turn -90 deg
DONE L=170.6 R=-170.7 sync_max=2.6 t=2220
> D2000
Move 2000 mm: Vmax 300 mm/s, T 7667 ms
DONE L=1998.2 R=1998.7 sync_max=1.3 t=8240
> W-90
Turn -90 deg
DONE L=170.6 R=-170.7 sync_max=2.1 t=2220
> D2000
Move 2000 mm: Vmax 300 mm/s, T 7667 ms
DONE L=1998.2 R=1998.6 sync_max=1.3 t=8260
> W-90
Turn -90 deg
DONE L=170.9 R=-170.5 sync_max=2.1 t=2260
> D2000
Move 2000 mm: Vmax 300 mm/s, T 7667 ms
DONE L=1998.0 R=1998.4 sync_max=1.4 t=8240
> W-90
Turn -90 deg
DONE L=170.9 R=-170.5 sync_max=2.0 t=2240
```
Zweiter: Counters reset  
```
> D2000
Move 2000 mm: Vmax 300 mm/s, T 7667 ms
DONE L=1998.0 R=1998.4 sync_max=1.3 t=8260
> W-90
Turn -90 deg
DONE L=170.6 R=-170.6 sync_max=2.5 t=2200
> D2000
Move 2000 mm: Vmax 300 mm/s, T 7667 ms
DONE L=1998.1 R=1998.2 sync_max=1.0 t=8260
> W-90
Turn -90 deg
DONE L=170.9 R=-170.4 sync_max=1.7 t=2240
> D2000
Move 2000 mm: Vmax 300 mm/s, T 7667 ms
DONE L=1998.1 R=1998.1 sync_max=1.3 t=8300
> W-90
Turn -90 deg
DONE L=170.9 R=-170.6 sync_max=2.1 t=2240
> D2000
Move 2000 mm: Vmax 300 mm/s, T 7667 ms
DONE L=1998.0 R=1997.9 sync_max=1.1 t=8260
> W-90
Turn -90 deg
DONE L=170.5 R=-170.6 sync_max=2.0 t=2240
```
Dritter: (ohne counter reset)  
```
> D2000
Move 2000 mm: Vmax 300 mm/s, T 7667 ms
DONE L=1998.1 R=1998.5 sync_max=0.9 t=8300
> W-90
Turn -90 deg
DONE L=170.8 R=-170.7 sync_max=2.6 t=2220
> D2000
Move 2000 mm: Vmax 300 mm/s, T 7667 ms
DONE L=1998.2 R=1998.2 sync_max=1.1 t=8260
> W-90
Turn -90 deg
DONE L=170.7 R=-170.3 sync_max=1.9 t=2220
> D2000
Move 2000 mm: Vmax 300 mm/s, T 7667 ms
DONE L=1998.2 R=1998.6 sync_max=1.3 t=8260
> W-90
Turn -90 deg
DONE L=170.6 R=-170.6 sync_max=2.1 t=2240
> D2000
Move 2000 mm: Vmax 300 mm/s, T 7667 ms
DONE L=1998.1 R=1998.5 sync_max=1.2 t=8260
> W-90
Turn -90 deg
DONE L=170.9 R=-170.7 sync_max=2.5 t=2240
```
Ab jetzt im Logfile  

## Evaluation (2026-11-21)

Firmware `phase2-imu-ekf` with `FEAT_IMU 0` (encoder turns, no gyro), defaults P200 6.35444e-05,
P201 1.00189, P202 0.99811, P203 0.21926. Frame: x along the first D2000, y left, from the axes
through O. Runs 1-2 pasted above, from run 3 on in `umb_repeat.log`. Run 5: wheel points 310 mm apart
(track 223 mm), one value mistyped.

`umbmark.py --wheelbase 0.21926 --scale-l 1.00189 --scale-r 0.99811 --turn-done-mm 170.7`, all 10 runs:
CW cg (-199, -230) mm, CCW cg (+353, -218) mm, single runs 190-494 mm, type B +3.6 deg per leg
(legs curve left; x and y agree), type A +0.5 deg per corner (x and y disagree), residual 18.5 mm.
H6 not passed. A straight D2000 from an aligned start drifted 22 mm left (1.3 deg), a third of the
UMBmark type B: the legs after a turn add caster error. Parameters not changed; next: caster service,
5 x D5000 lateral drift for the wheel ratio.
