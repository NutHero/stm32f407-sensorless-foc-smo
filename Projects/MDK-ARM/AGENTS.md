# AGENTS.md
注：注释使用中文
Bare-metal STM32F407ZETx sensorless FOC (sliding-mode observer) motor controller.
Keil MDK-ARM / uVision project only — **no git, no make/CMake, no CI, no tests**.

## Build & clean

- Open `BLDC_FOC_SMO.uvprojx` in Keil uVision and build from the IDE. There is no CLI build wired up.
- Compiler is **ARM Compiler 5 (AC5)** (`uAC6=0`), optimization `-O1`. `main.c` branches on `__ARMCC_VERSION`, so AC5/AC6 code paths differ.
- Output goes to `../../Output/` (`.o`, `.axf`, `.hex`, `.map`, `.sct`).
- Project root is two levels up (`../../`). `keilkill.bat` there recursively deletes build artifacts (`*.o`, `*.axf`, `*.map`, etc.) — warning: it deletes `*.dbgconf` too, breaking debug configs.

## Layout

- `../../User/` — `main.c`, `stm32f4xx_it.c`, `stm32f4xx_hal_conf.h`.
- `../../Drivers/FOC/` — control math: `bldc_control.c` (state machine + `MotorHandler`), `smo.c` (observer), `pid.c`, `clarke_park.c`, `svpwm.c`/`spwm.c`, `filter.c`, `sincos16.c`, `trap_planner.c`, `sensor.c`.
- `../../Drivers/BSP/` — `adc.c`, `gpio.c`, `time.c`, `spi.c`, `sram.c`, `led.c`.
- `../../Drivers/App/` — `task_process.c` (scheduler), `decodeVofa+.c` (host commands), `quen.c`, `myiic.c`.
- `../../Drivers/SYSTEM/` — `sys`, `delay`, `usart` (printf retarget).

## Toolchain gotchas

- The `.uvprojx` holds an **explicit `<Files>` list and include paths**; there is no globbing. A new `.c` file is NOT compiled until you add it to the project in uVision.
- Some sources exist but are intentionally not built (e.g. `Drivers/App/iic.c`, `sort.c`). Do not assume file presence means it compiles.
- **All source files are GB2312/GBK encoded** (Chinese comments). UTF-8-based editing/writing will corrupt existing comments. Prefer ASCII-only additions; avoid rewriting lines that contain Chinese text.
- Edit HAL feature selection in `User/stm32f4xx_hal_conf.h`, not the vendored HAL tree.

## Runtime architecture (non-obvious)

- **Current loop**: ADC3 injected-conversion-complete ISR → `HAL_ADCEx_InjectedConvCpltCallback` (`Drivers/BSP/adc.c`) → `MotorHandler()` (`Drivers/FOC/bldc_control.c`). Ts = 50us.
- **System tick**: TIM6 1ms ISR → `HAL_TIM_PeriodElapsedCallback` → `sys_process_run()` (`Drivers/App/task_process.c`), which runs the 10ms process state machine (offset calibration → angle reset → wait/open/close motor).
- **Sensorless**: `smo.c` estimates angle/speed (with PLL) and feeds commutation — there is no physical encoder; `dev_encoder` is the observer-backed encoder abstraction.
- Control math is **Q15 fixed-point** (`pid_q15`, `trap_init_q15`, `sincos16`). Units: current mA, speed RPM, angle degrees, voltage Q15 (-32768..32767).
- External SDRAM via FSMC at `0x68000000` (`sram_init()`); `main.c` places a large buffer there with a compiler-version-specific attribute.

## Debug / telemetry

- `printf` retargets to **USART6 @ 115200** (`fputc` in `Drivers/SYSTEM/usart/uart.c`).
- VOFA+ JustFloat frames go to **USART3 @ 2000000** via `VOFA_SEND` (`Drivers/App/decodeVofa+.c`), which also parses incoming PID-tuning commands.
