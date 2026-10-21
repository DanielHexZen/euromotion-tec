# CLAUDE.md — MR2006B vehicle firmware

STM32F446RE (NUCLEO-F446RE) firmware for an autonomous differential-drive robot. HAL-based C, STM32CubeMX + STM32CubeIDE 2.2 on macOS, Git in the terminal.

**Always use the `/ponytail` skill** for work in this repository.

## Sources of truth
- `docs/FIRMWARE_SPEC.md`: normative. Defines what the code does: interfaces, numbers, `REQ-*` IDs. If code disagrees with it, the code is wrong.
- `docs/MR2006B_Vehicle_Technical_Reference.pdf`: explains why (models, physics, rationale). Read it when a requirement is unclear; do not code from it.
- Never invent a value tagged `[MEASURE]`, `[VERIFY]` or `[DECIDE]`. Make it a parameter in `par_table.h` and flag it.

## Hard rules
- C11 (`gnu11`). No C++, RTOS, `malloc`, recursion or VLAs.
- `float` only: every literal has an `f` suffix, use `sinf`/`fabsf`/…, no `double`. `-Wdouble-promotion` stays clean. Never use `-ffast-math`. No `%f` in printf.
- SI units internally (m, s, rad, V, duty in [−1, 1]). Legacy units (mm, %, ms) exist only in `console.c`.
- Nothing blocks after `app_init()`: no `HAL_Delay`, no HAL call with a timeout, no blocking UART. Use `_IT`/`_DMA` variants. ISRs only snapshot and set flags; no float math in ISRs.
- Pure modules (`prof whl drv att gnd lat fwd ekf map nav evd msn proto par`) must not include HAL or `main.h`. They must build on the host.
- `mot_apply()` is the only writer of TIM1 CCR1–4. Write the inactive channel before the active one.
- Fault handlers and `Error_Handler` call `sys_fault_hard_stop()` first.
- Tunables live in `par_table.h` (X-macro). Parameter IDs are never reused.

## CubeMX / Git protocol
- All user code goes in `Core/Src` and `Core/Inc` with module prefixes. `main.c` holds only the 4 USER CODE hooks (spec §16.2). All HAL callbacks go in `app_isr.c`.
- Never add source folders to `.cproject`. Never hand-edit generated code outside USER CODE blocks.
- Commit before every CubeMX regeneration, then run `git diff Core/Src/main.c .cproject Core/Inc/stm32f4xx_hal_conf.h`. Check TIM6 (PSC 89 / ARR 999), TIM2 (PSC 89) and PLLM 4 in the generated code.
- Name the implemented `REQ-*` IDs in commit messages.

## Build & test
- Host unit tests: `make -C test` (`-Werror`, ASan/UBSan). Every pure-module change needs a passing test that names its REQ ID.
- `APP_CALIB=1` builds the calibration firmware (`CALIB_README.md`). Leave `calib.c` unchanged.
- Debug `-Og`, Release `-O2`. The Release build must run on the vehicle by week 13.

## Working order
- Implement phase by phase (spec §15). Do not start a phase before the previous gate passes.
- Phase 0 is a pure restructure: `D300`/`D1000`/`D5000` must end within 2 mm, the same as the pre-spec firmware.
- Commission one layer at a time. Any deviation from the spec goes into spec §18.2 with a rationale, not silently into the code.
