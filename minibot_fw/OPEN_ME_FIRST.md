# minibot_fw — STM32F407VET6 firmware

**Status: VERIFIED. This compiles clean with arm-none-eabi-gcc 13.2.1.**
0 errors, 0 warnings in project code. 19,104 B flash (3.7% of 512 KB).

The project is now named **`minibot_fw`** (was `Minibot_v6_rev2_STM`).
That rename is deliberate — see "Why your import failed" below.

---

## Fastest path: flash it NOW, no IDE

The compiled firmware is already in `prebuilt/`. You do not need CubeIDE
to get the robot moving.

```bash
# ST-LINK
st-flash --reset write prebuilt/minibot.bin 0x8000000

# or, with the ST tool
STM32_Programmer_CLI -c port=SWD -w prebuilt/minibot.hex -rst
```

Then bench-test it (robot on blocks, no ROS):
```bash
python3 stm32_bench.py --watch          # expect $F,... frames at ~50 Hz
python3 stm32_bench.py --verify-steps   # commands exactly 1 wheel revolution
```

## Rebuild from source, no IDE

```bash
make            # -> build/minibot.elf .hex .bin
make flash
```
Needs only `gcc-arm-none-eabi`. This is the build I ran and verified.

---

## Importing into STM32CubeIDE

### Why your import failed before

The error was **"already exist in the workspace OR project description
file is corrupted."** It was the FIRST half, not the second.

Proof: the Import dialog *listed the project in the Projects box*. Eclipse
can only do that by successfully parsing `.project`. A corrupt description
file would not have appeared at all.

What actually happened: you launched CubeIDE with the **extracted project
folder itself as the workspace** (the title bar read `Minibot_v6_rev2_STM`
— that is the workspace name). Eclipse cannot import a project that lives
at the workspace root. The names collided.

Renaming the project to `minibot_fw` removes the collision no matter what
your workspace is called.

### Import steps

1. **File -> Switch Workspace -> Other...**
   Pick a folder that is NOT this project, e.g. `~/STM32CubeIDE/workspace`
2. **File -> Import -> General -> Existing Projects into Workspace**
3. Root directory: the folder containing this file (`minibot_fw/`)
4. `minibot_fw` appears checked -> **Finish**
5. Ctrl+B to build

**Golden rule:** workspace folder and project folder must be two different
places. Never open a project folder as a workspace.

### If CubeIDE still fights you

Skip its managed build entirely — the Makefile is right there and it works:

**File -> Import -> C/C++ -> Existing Code as Makefile Project**
- Location: this folder
- Toolchain: **Cross ARM GCC**
- Finish

You get the editor and debugger; `make` does the building.

---

## Hardware this expects

| | |
|---|---|
| MCU | STM32F407VET6 |
| UART to Pi | USART1 — **PA9 = TX, PA10 = RX**, 115200 8N1 |
| Step engine | TIM7 ISR @ 40 kHz (DDS phase accumulator) |
| Shared ENABLE | **PE3** (both DRV8825 nENABLE, active LOW) |
| LEFT STEP / DIR | **PE4 / PE5** |
| RIGHT STEP / DIR | **PE6 / PE7** |

Pi TX -> PA10, Pi RX -> PA9, **and a common GND**. TX/RX cross over.

## Layout

```
minibot_fw/
├── .project .cproject .mxproject .settings/   CubeIDE metadata
├── minibot_fw.ioc            open in CubeMX
├── minibot_fw.launch         ST-LINK debug config
├── Makefile                  verified working build
├── STM32F407VETX_FLASH.ld
├── prebuilt/                 minibot.hex / .bin / .elf  <- FLASH THESE
├── Core/
│   ├── Inc/  main, gpio, tim, usart, stm32f4xx_hal_conf, stm32f4xx_it,
│   │         minibot_config.h, minibot_motion.h, minibot_protocol.h
│   ├── Src/  main.c, gpio.c, tim.c, usart.c, hal_msp, it, system,
│   │         syscalls, sysmem, minibot_motion.c, minibot_protocol.c
│   └── Startup/startup_stm32f407xx.s
└── Drivers/  STM32F4xx_HAL_Driver + CMSIS
```

**Your code lives in `minibot_motion.c`, `minibot_protocol.c`, and
`minibot_config.h`.** Everything else is CubeMX-generated or ST vendor code.
CubeMX regeneration will not touch those three files.

---

## Before you trust the distances it drives

`minibot_config.h` assumes **200 steps/rev x 16 microstep = 3200 steps per
wheel revolution**, and **67.2 mm wheels**. If your DRV8825 jumpers are
actually 1/8, or the motors are 0.9 deg/step, **every distance is silently
2x wrong** and nothing will tell you.

`stm32_bench.py --verify-steps` commands exactly one wheel revolution
(= 211 mm of travel on 67.2 mm wheels). Mark the wheel and the floor.
Run it before you trust anything.

---

## Fix applied 2026-07-13: CubeIDE build error (DAP_config.h not found)

CubeIDE's managed build auto-discovers every `.c` file under `Drivers/`,
unlike the Makefile which only compiles a fixed list. The vendored CMSIS
repo had extra folders that were never part of this project:

- `Drivers/CMSIS/DAP/` — ST's CMSIS-DAP USB debug-probe firmware, needs a
  `DAP_config.h` that was never included -> fatal error in CubeIDE only
- `Drivers/CMSIS/Core/`, `Drivers/CMSIS/Core_A/` — Cortex-M template /
  Cortex-A variants, irrelevant to the F407 (Cortex-M4)
- `Drivers/STM32F4xx_HAL_Driver/Src/Legacy/*.c` (CAN, ETH) — not used by
  this firmware, not in the Makefile's source list either

All of the above are now deleted. `Drivers/` contains exactly the 22 `.c`
files the Makefile always compiled, plus only the headers actually
included. Verified: `find Core Drivers -name "*.c"` now returns exactly
the Makefile's `C_SOURCES` list, and `make` still produces the identical
19,104-byte binary. CubeIDE's build should now match the Makefile build.
