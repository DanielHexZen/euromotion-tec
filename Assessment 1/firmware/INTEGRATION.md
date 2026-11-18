# Adding the identification tests to the car firmware

Files: copy `sysid.c` to `Core/Src` and `sysid.h` to `Core/Inc`.
Four small edits in `main.c` (Challenge_TestHC). Nothing else changes.

## 1. Include (USER CODE BEGIN Includes)

```c
#include "sysid.h"
```

## 2. `Control_Tick()` — replace the first two lines

```c
static void Control_Tick(void)
{
  int32_t tl0 = wl.total, tr0 = wr.total;
  Wheel_UpdateEncoder(&wl);
  Wheel_UpdateEncoder(&wr);
  position_l_mm = (float)wl.total * MM_PER_COUNT;
  position_r_mm = (float)wr.total * MM_PER_COUNT;
  speed_l_mms   = wl.speed;
  speed_r_mms   = wr.speed;

  /* identification test: open loop, overrides everything else */
  if (SysId_Active()) {
    float dl, dr;
    SysId_Tick(wl.total - tl0, wr.total - tr0, &dl, &dr);
    Wheel_SetDuty(&wl, dl);
    Wheel_SetDuty(&wr, dr);
    return;
  }

  if (!Moving()) return;
  /* ... rest unchanged ... */
```

## 3. `Serial_HandleCommand()` — new command `X<n>`, and stop also aborts the test

```c
  if (c == 's' || c == 'S') {
    SysId_Abort();                 /* new */
    Move_Abort();
    Serial_Print("STOP\r\n");

  } else if (c == 'x' || c == 'X') {               /* new block */
    if (Moving() || SysId_Active()) { Serial_Print("Busy, send s first\r\n"); return; }
    long n = strtol(cmd + 1, &end, 10);
    if (end == cmd + 1 || *end != '\0' || !SysId_Start((uint8_t)n)) {
      Serial_Print("Use X1..X6: 1 step, 2 PRBS, 3 chirp (lon), 4 PRBS, 5 DLC, 6 SLC (lat)\r\n");
      return;
    }
    open_loop = 0;
    Serial_Print("SYSID start\r\n");
  } else if ...
```

## 4. `Car_Loop()` — send the log when the test is finished

```c
  /* 5) identification log */
  if (SysId_DumpReady()) {
    Motors_Stop();
    SysId_Dump(Serial_Print);
  }
```

## Running the tests

| Cmd | Test | Duty | Duration | Space needed |
|---|---|---|---|---|
| `X1` | Longitudinal multi-level step | 40 % ± 10 % | 11.5 s | ~3–4 m straight |
| `X2` | Longitudinal PRBS (60 ms bit) | 40 % ± 10 % | 9.1 s | ~3 m straight |
| `X3` | Longitudinal chirp 0.2→3 Hz | 40 % ± 10 % | 11.5 s | ~3–4 m straight |
| `X4` | Lateral PRBS (80 ms bit) | 35 % ± 8 % differential | 11.7 s | ~3 m, car weaves |
| `X5` | Lateral double lane change | 35 % ± 8 % differential | 8.0 s | ~2.5 m × 1 m |
| `X6` | Lateral single lane change | 35 % ± 8 % differential | 5.9 s | ~2 m × 1 m |

- Use the USB cable (USART2, 115200 baud) for the dump. The HC-12 also gets the text, but slowly.
- Easiest: run `capture_sysid.m` in MATLAB. It sends the command and saves the CSV.
- Run every test **twice** (`_1`, `_2`). Run 1 trains, run 2 is an extra validation set.
- Same floor, same battery charge for all runs. Charge the battery before.
- If the car drives too fast for the room, lower `LON_BASE` / `LAT_BASE` in `sysid.c` and rerun all tests.
