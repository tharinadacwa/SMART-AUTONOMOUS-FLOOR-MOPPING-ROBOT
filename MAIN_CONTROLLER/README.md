# minibot_fw: Main Controller Firmware (STM32F407VET6)

Firmware for the main controller of the **Smart Autonomous Floor Mopping Robot**.
It runs on an STM32F407VET6 and owns four independent subsystems:

| Subsystem | Hardware | Control |
|---|---|---|
| **Drive** | 2 × stepper motor via DRV8825 | 40 kHz DDS pulse engine, jerk-limited S-curve profile, commanded over UART |
| **Mopping** | 4 × brushed DC motor via 2 × L298N | Fixed direction, on at boot, no speed control |
| **Water pump** | 1 × pump via IRLZ44N low-side MOSFET | Free-running timer: 3 s ON / 5 s OFF |
| **Host link** | USART1 → Raspberry Pi 5 | CRC-8 framed ASCII protocol, 50 Hz odometry feedback |

The drive layer is the only closed-loop-ish part: the Pi sends velocity targets, the
STM32 executes them and streams back an exact step count that serves as odometry.
The mop motors and pump are open-loop and run autonomously from power-on.

---

## ⚠️ Read this before you power the robot

**At power-on, with no host connected:**

| | State at boot |
|---|---|
| Stepper drivers | **Disabled** (PE3 nENABLE driven HIGH by `MX_GPIO_Init()` before anything else). Wheels do not move until a `$V` or `$E,1` command arrives. |
| 4 mop DC motors | **Running.** `main()` calls `DCM_Run()` unconditionally. They spin the moment the board is powered. |
| Water pump | **Cycling.** 5 s off, 3 s on, forever, from `Sig_Task()`. |

If you want a silent bench board, comment out `DCM_Run()` in `main()` and/or
`Sig_Init()`/`Sig_Task()`. Do this before the first power-up on a workbench; four
brushed motors starting unannounced is a good way to lose a finger or a probe.

**Common ground is mandatory.** STM32 GND, both L298N GND terminals, the MOSFET
source, and the 12 V battery negative must all be one node. Missing common ground
is the single most frequent cause of "the motors behave randomly".

---

## Hardware

| Item | Value |
|---|---|
| MCU | STM32F407VET6 (Cortex-M4F, 512 KB flash, 128 KB RAM) |
| Clock | HSE 8 MHz → PLL → **168 MHz** (see crystal warning below) |
| Toolchain | `arm-none-eabi-gcc` (13.2.1 verified) or STM32CubeIDE |
| Host link | USART1, 115200 8N1, no flow control |
| Drive | 2 × DRV8825, 1/16 microstep, 200 step/rev motors |
| Mop | 2 × L298N, 4 × brushed DC, 12 V |
| Pump | IRLZ44N low-side switch, 12 V |

### Complete pin map

| Pin | Function | Direction | Notes |
|---|---|---|---|
| **PE3** | `nENABLE`, both DRV8825 | out | **Active LOW.** HIGH = motors dead. Driven HIGH first thing at boot. |
| **PE4** | LEFT STEP | out | `GPIO_SPEED_FREQ_VERY_HIGH`, needed for clean 2 µs edges |
| **PE5** | LEFT DIR | out | |
| **PE6** | RIGHT STEP | out | |
| **PE7** | RIGHT DIR | out | |
| **PA9** | USART1_TX | out | → Raspberry Pi **RX** |
| **PA10** | USART1_RX | in | → Raspberry Pi **TX** |
| **PD0** | L298N #1 IN1 | out | Mop motor 1 |
| **PD1** | L298N #1 IN2 | out | Mop motor 1 |
| **PD2** | L298N #1 IN3 | out | Mop motor 2 |
| **PD3** | L298N #1 IN4 | out | Mop motor 2 |
| **PA2** | L298N #2 IN1 | out | Mop motor 3 |
| **PA3** | L298N #2 IN2 | out | Mop motor 3 |
| **PE14** | L298N #2 IN3 | out | Mop motor 4 |
| **PE15** | L298N #2 IN4 | out | Mop motor 4 |
| **PA12** | Pump gate (via 220 Ω) | out | USB_DM on this package, but USB is unused in this project |

TX/RX **cross over**: Pi TX → PA10, Pi RX → PA9, plus common GND.

Detailed wiring instructions, including diagrams, live in:

- [`WIRING_L298N.md`](WIRING_L298N.md): DC motor drivers, ENA/ENB tie-up, power distribution
- [`WIRING_PA12_PUMP.md`](WIRING_PA12_PUMP.md): MOSFET switch, flyback diode placement, gate pull-down

### ⚠️ Check your crystal before first flash

The firmware assumes an **8 MHz HSE crystal**. Many F407VET6 "black boards" ship
with **25 MHz**. If yours does and you don't change it, the baud rate *and* the
step rate are both wrong by the same 25/8 ratio: you get garbage on the serial
port and wrong motor speeds simultaneously, which is a confusing pair of symptoms
to debug.

Two edits:

| File | Change |
|---|---|
| `Core/Inc/stm32f4xx_hal_conf.h` | `#define HSE_VALUE ((uint32_t)25000000U)` |
| `Core/Src/main.c` → `SystemClock_Config()` | `osc.PLL.PLLM = 25;` |

Leave `PLLN`/`PLLP`/`PLLQ` alone. `PLLM` exists solely to bring the PLL input to
1 MHz (8/8 = 1, 25/25 = 1), so everything downstream still lands on 168 MHz.

### Clock tree

```
HSE 8 MHz ──/PLLM=8──► 1 MHz ──×PLLN=336──► 336 MHz ──/PLLP=2──► SYSCLK 168 MHz
                                                      /PLLQ=7──►  48 MHz (unused)

AHB  /1 → HCLK 168 MHz
APB1 /4 → PCLK1  42 MHz → TIM7 clock 84 MHz   (×2 rule when APB prescaler ≠ 1)
APB2 /2 → PCLK2  84 MHz → USART1
```

TIM7: `PSC = 84-1` (84 MHz → 1 MHz), `ARR = 25-1` (1 MHz → **40 kHz**).
Change the clock config and you *must* change these two numbers, or every step
rate (and therefore all odometry) is silently wrong by the same ratio.

---

## Drive architecture

Three layers, all inside the 40 kHz TIM7 ISR, plus a main-loop task.

```
TIM7 ISR @ 40 kHz ─┬─ Layer 1: DDS pulse engine
                   │    32-bit phase accumulator per motor.
                   │    inc = rate_hz × (2³² / 40000); acc += inc each tick;
                   │    a wrap past 2³² emits one STEP pulse AND bumps the
                   │    step counter in the same breath, which is why the
                   │    reported odometry is a tally, not an estimate.
                   │
                   └─ every 40th tick (1 kHz): Layer 2 + Layer 3
                        a += clamp(a_desired − a, ±jerk·dt)
                        v += a·dt                       ← S-curve, not trapezoid
                        v is SIGNED and passes through zero on a reversal, so
                        DIR only ever flips while the motor is ~stopped.

main loop ── MB_Task():  drain command queue, comms watchdog, 50 Hz feedback
          └─ Sig_Task():  non-blocking pump timer
```

**Why the DDS low-time is guaranteed, not lucky.** STEP is driven LOW at the top
of the ISR and HIGH at the bottom, giving ~23 µs high and ≥25 µs low against the
DRV8825's 1.9 µs minimum. If the accumulator increment is < 0.5 (i.e. step rate
< ISR_HZ/2 = 20 kHz), then immediately after a wrap the accumulator holds a value
below 0.5, so the *next* tick cannot wrap. Two pulses can never land on
consecutive ticks. At the configured ceiling of 1066 sps you are using **5.3 % of
that limit**, an enormous margin.

**Why jerk limiting matters here.** A plain trapezoidal ramp steps acceleration
discontinuously from 0 to a_max, which is a torque step, exactly the impulse
that makes a stepper skip. A coverage robot reverses at the end of every lane,
dozens of times per room, and there are no encoders to catch a skipped step. The
S-curve is what keeps the odometry honest across a full run.

### Interrupt priorities: do not reorder

| Interrupt | Preempt priority |
|---|---|
| TIM7 | **0** (highest) |
| USART1 | 1 |
| SysTick | 15 (lowest) |

The TIM7 ISR is ~2 µs. At 115200 baud a UART byte arrives every 87 µs, so even
worst case the UART is serviced ~2 µs late, 40× inside its deadline. Giving the
UART top priority "so we never miss a byte" trades a problem you don't have for
one you can't detect: a late step pulse is jitter at best and a lost step at
worst, and on an encoderless robot a lost step is a silent lie in the odometry.

---

## Serial protocol

ASCII, so you can drive it from a terminal.

```
$<PAYLOAD>*<CRC8><LF>
```

| Field | Meaning |
|---|---|
| `$` | Start of frame. Resynchronises the parser after any garbage. |
| `PAYLOAD` | Comma-separated fields; first field is the command letter. |
| `*` | End of payload. |
| `CRC8` | Two uppercase hex digits. **CRC-8/ATM**: poly `0x07`, init `0x00`, no reflection, no final XOR, computed over `PAYLOAD` only. |
| `LF` | `\n` |

### Pi → STM32

| Frame | Meaning |
|---|---|
| `$V,<left_sps>,<right_sps>*XX` | Signed steps/second target. Also implicitly enables the drivers and clears E-stop. |
| `$E,<0\|1>*XX` | Enable (1) / disable (0) the stepper drivers. |
| `$S*XX` | **Emergency stop.** Bypasses the ramp entirely and kills pulses immediately. |
| `$R*XX` | Reset step counters to zero. |
| `$P*XX` | Ping → STM32 replies `$K*F6`. |

### STM32 → Pi (50 Hz)

```
$F,<l_steps>,<r_steps>,<l_sps>,<r_sps>,<flags>,<seq>*XX
```

| Field | Type | Meaning |
|---|---|---|
| `l_steps`, `r_steps` | int32, signed, cumulative | **This is your odometry.** |
| `l_sps`, `r_sps` | int32 | Actual current step rate, post-profile |
| `flags` | bitfield | See below |
| `seq` | uint16, wraps | Lets the host detect dropped frames instead of silently interpolating across them |

| Flag | Value | Meaning |
|---|---|---|
| `MB_FLAG_ENABLED` | 1 | Drivers energised |
| `MB_FLAG_ESTOP` | 2 | Emergency stop latched |
| `MB_FLAG_CMD_TIMEOUT` | 4 | No command from host → ramped to stop |
| `MB_FLAG_CRC_ERR` | 8 | At least one bad CRC since boot (sticky) |
| `MB_FLAG_OVERRUN` | 16 | UART overrun seen since boot (sticky) |
| `MB_FLAG_CLAMPED` | 32 | Last command exceeded `MB_MAX_STEP_RATE` |

### Why CRC-8 and not a checksum

The motor wires running alongside this cable switch amps at kHz rates. One
flipped bit in a velocity command is a robot driving into a wall. CRC-8 catches
all single-bit errors, all double-bit errors, and all burst errors up to 8 bits.
A sum or XOR does not.

### Verified test vectors

Regenerated and confirmed against this source tree:

| Payload | CRC | Full frame |
|---|---|---|
| `V,1000,-1000` | `0x47` | `$V,1000,-1000*47\n` |
| `F,12345,-6789,500,-500,33,7` | `0x31` | *(feedback example)* |
| `P` | `0xB7` | `$P*B7\n` |
| `K` | `0xF6` | `$K*F6\n` (ping reply) |
| `S` | `0xBE` | `$S*BE\n` |
| `R` | `0xB9` | `$R*B9\n` |
| `E,1` | `0x83` | `$E,1*83\n` |
| `E,0` | `0x84` | `$E,0*84\n` |

Copy-paste bench test from a Linux host:

```bash
stty -F /dev/ttyUSB0 115200 raw -echo
cat /dev/ttyUSB0 &                          # watch $F frames at ~50 Hz
printf '$P*B7\n'          > /dev/ttyUSB0    # expect $K*F6 back
printf '$V,500,500*XX\n'  > /dev/ttyUSB0    # replace XX with the real CRC
printf '$S*BE\n'          > /dev/ttyUSB0    # emergency stop
```

Reference CRC helper:

```python
def crc8(payload: str) -> str:
    c = 0
    for ch in payload.encode():
        c ^= ch
        for _ in range(8):
            c = ((c << 1) ^ 0x07) & 0xFF if c & 0x80 else (c << 1) & 0xFF
    return f"{c:02X}"

def frame(payload: str) -> bytes:
    return f"${payload}*{crc8(payload)}\n".encode()
```

---

## Configuration

All drive tuning lives in [`Core/Inc/minibot_config.h`](Core/Inc/minibot_config.h).

| Constant | Value | Meaning |
|---|---|---|
| `MB_STEPS_PER_REV` | 3200 | 200 steps/rev × 16 microstep |
| `MB_WHEEL_RADIUS_M` | 0.0336 | 67.2 mm wheel diameter |
| `MB_WHEEL_SEP_M` | 0.400 | Track width |
| `MB_MAX_STEP_RATE` | 1066 sps | ≈ 20 RPM |
| `MB_MIN_STEP_RATE` | 266 sps | ≈ 5 RPM |
| `MB_MAX_ACCEL_SPS2` | 2132 | 0 → max in 0.5 s |
| `MB_MAX_JERK_SPS3` | 14213.3 | 0 → max accel in 0.15 s |
| `MB_ISR_HZ` | 40000 | TIM7 step-engine tick |
| `MB_RAMP_HZ` | 1000 | Profile update rate |
| `MB_FEEDBACK_HZ` | 50 | `$F` frame rate |
| `MB_CMD_TIMEOUT_MS` | 500 | Comms watchdog |
| `MB_IDLE_DISABLE_MS` | 10000 | Drop coils after idle |
| `MB_LEFT_DIR_INVERT` | 1 | Per-side direction flip |
| `MB_RIGHT_DIR_INVERT` | 0 | |

### Derived motion envelope

| Quantity | Value |
|---|---|
| Wheel circumference | 211.1 mm |
| Distance per step | **65.97 µm** |
| Max linear speed | ≈ **70 mm/s** |
| Min linear speed | ≈ 17.5 mm/s |
| Max yaw rate (spin in place) | ≈ 0.35 rad/s (20 °/s) |

### Fixing direction problems

| Symptom | Fix |
|---|---|
| Robot **spins** instead of driving forward | Flip exactly **one** of `MB_LEFT_DIR_INVERT` / `MB_RIGHT_DIR_INVERT` |
| Robot drives **backwards** | Flip **both** |
| A mop motor turns the wrong way | Swap its two `OUTx` wires, or swap `DCM_CW`/`DCM_CCW` for that motor in `DCM_Run()` |

### Pump timing

Edit `Core/Src/minibot_signal.c`:

```c
#define SIG_HIGH_MS   3000u   /* pump ON  time (ms) */
#define SIG_LOW_MS    5000u   /* pump OFF time (ms) */
```

The pump starts in the OFF phase at boot, which is the safe default.

---

## Build and flash

### Option A: Makefile (no IDE required)

```bash
sudo apt install gcc-arm-none-eabi stlink-tools

make          # → build/minibot.elf, .hex, .bin
make flash    # ST-Link
make dfu      # USB DFU: hold BOOT0, tap RESET, then run this
make size
make clean
```

The Makefile compiles a fixed source list and builds at `-O2`.

### Option B: STM32CubeIDE

1. **File → Switch Workspace → Other…** Pick a folder that is **not** this
   project directory. Opening the project folder as the workspace is what causes
   the *"already exist in the workspace"* import error.
2. **File → Import → General → Existing Projects into Workspace**
3. Root directory: this folder. `minibot_fw` appears checked → **Finish**
4. `Ctrl+B` to build, then flash `Debug/minibot_fw.elf`.

CubeIDE's managed build auto-discovers every `.c` under `Drivers/`, unlike the
Makefile's fixed list, so keep `Drivers/` free of unused vendor subtrees (the
CMSIS `DAP/`, `Core/`, `Core_A/` folders and HAL `Src/Legacy/` were removed for
exactly this reason).

If CubeIDE keeps fighting you, use **File → Import → C/C++ → Existing Code as
Makefile Project**, toolchain *Cross ARM GCC*. You get the editor and debugger;
`make` does the building.

### Option C: CubeMX regeneration

`minibot_fw.ioc` matches the peripheral setup in `main.c`. Regenerating
**overwrites** `main.c`, `gpio.c`, `tim.c`, `usart.c`. Everything project-specific
lives either between `USER CODE BEGIN/END` markers or in `minibot_*.c/h`, which
CubeMX never touches. Diff before you commit.

### Build size

Measured from the committed `Debug/minibot_fw.elf` (CubeIDE debug build, `-O0 -g`):

```
FLASH:  42,052 B / 524,288 B   ( 8.0 % )
RAM  :   3,776 B / 131,072 B   ( 2.9 % )
```

The `-O2` Makefile build is roughly half the flash. Either way there is a great
deal of headroom.

---

## Repository layout

```
minibot_fw/
├── Core/
│   ├── Inc/
│   │   ├── main.h                 ← complete pin map lives here
│   │   ├── minibot_config.h       ← all drive tuning constants
│   │   ├── minibot_motion.h       ← step-engine design rationale
│   │   ├── minibot_protocol.h     ← wire format spec
│   │   ├── minibot_dcmotor.h      ← L298N mop-motor module
│   │   ├── minibot_signal.h       ← PA12 pump timer
│   │   ├── gpio.h  tim.h  usart.h  stm32f4xx_hal_conf.h  stm32f4xx_it.h
│   ├── Src/
│   │   ├── main.c                 ← boot order, clock config, HAL callbacks
│   │   ├── minibot_motion.c       ← DDS + S-curve + watchdog + feedback (379 lines)
│   │   ├── minibot_protocol.c     ← CRC-8, frame parser, frame builder
│   │   ├── minibot_dcmotor.c      ← 4 DC motors, 8 IN pins
│   │   ├── minibot_signal.c       ← pump 3 s / 5 s cycle
│   │   ├── gpio.c  tim.c  usart.c  stm32f4xx_hal_msp.c  stm32f4xx_it.c
│   │   └── syscalls.c  sysmem.c  system_stm32f4xx.c
│   └── Startup/startup_stm32f407xx.s
├── Drivers/                       ← ST HAL + CMSIS (vendor code, unmodified)
├── Debug/                         ← CubeIDE build output (see note below)
├── prebuilt/REBUILD_REQUIRED.txt  ← old binaries deliberately removed
├── Makefile                       ← standalone GCC build
├── STM32F407VETX_FLASH.ld
├── minibot_fw.ioc                 ← CubeMX project
├── minibot_fw.launch              ← ST-Link debug config
├── WIRING_L298N.md
└── WIRING_PA12_PUMP.md
```

**Your code is in `minibot_motion.c`, `minibot_protocol.c`, `minibot_dcmotor.c`,
`minibot_signal.c` and `minibot_config.h`.** Everything else is CubeMX-generated
or ST vendor code.

---

## Bring-up checklist

Work through this in order the first time. Robot on blocks, wheels off the ground.

1. **Crystal.** Confirm 8 MHz vs 25 MHz and patch if needed (see above).
2. **Grounds.** STM32 GND, both L298N GND, MOSFET source, battery −. All one node.
3. **Power the board with motor power OFF.** Confirm the stepper drivers stay
   disabled and nothing moves.
4. **Serial link.** `$P*B7` → expect `$K*F6`. Confirm `$F` frames arrive at
   ~50 Hz with `flags` = 0 or 4 (`CMD_TIMEOUT` is expected before you send
   anything).
5. **Microstepping sanity.** Command exactly one wheel revolution: `$V` at a
   known rate for a known duration, or reset counters with `$R` and watch
   `l_steps` reach 3200. Mark the wheel and the floor. **If the DRV8825 jumpers
   are actually 1/8, or the motors are 0.9°/step, every distance is silently 2×
   wrong and nothing in the system will tell you.**
6. **Direction.** Command forward. Fix with the `*_DIR_INVERT` flags, not by
   swapping wires, so the config stays the source of truth.
7. **Watchdog.** Stop sending commands mid-motion. The robot should ramp to a
   stop within ~500 ms and set flag bit 4, while staying energised.
8. **E-stop.** Send `$S*BE` at speed. Motion must cease immediately.
9. **Mop motors and pump** last, with the drivetrain verified.

---

## Safety behaviour

| Mechanism | Behaviour |
|---|---|
| **Boot order** | `MX_GPIO_Init()` runs before any other peripheral and drives nENABLE HIGH, so the steppers are dead before anything can twitch. `DCM_Init()` pre-loads all L298N IN pins LOW *before* switching them to outputs, so no mop motor twitches during bring-up. |
| **Comms watchdog** | No command for 500 ms → ramp to stop, but **stay energised**, keeping holding torque so the robot doesn't roll on a slope. |
| **Idle disable** | No command for 10 s → drop the coils. A stationary stepper at full current is a ~10 W heater doing no work. |
| **Emergency stop** | Bypasses the profile entirely. Losing steps is an acceptable trade for stopping now. |
| **Fault handlers** | HardFault / BusFault / UsageFault kill the steppers **and** call `DCM_StopAll()`, then hang. Firmware that has faulted must not still be driving motors. |
| **NaN guard** | Non-finite velocity → stop. |
| **UART overrun recovery** | An unhandled ORE silently kills RX forever, so `MB_UartErrorISR()` clears every error flag and re-arms reception. |
| **CRC rejection** | Frames with a bad CRC are discarded silently and flagged, never acted on. |

---

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| Garbage on serial **and** wrong motor speeds | 25 MHz crystal, unpatched. Both scale by 25/8. |
| Motors hum but don't turn | Driver current limit too low, or STEP edges too slow down long jumper wires |
| Motor "sometimes just doesn't move" | STEP line slew rate; keep `GPIO_SPEED_FREQ_VERY_HIGH`, shorten the wire |
| Robot spins instead of driving forward | Flip exactly one `*_DIR_INVERT` |
| Distances consistently 2× off | DRV8825 microstep jumpers don't match `MB_STEPS_PER_REV` |
| Flag bit 4 (`CMD_TIMEOUT`) always set | Host isn't sending, or CRC mismatch; check flag bit 8 |
| Flag bit 8 (`CRC_ERR`) sticky | Wiring noise, wrong baud, or a host-side CRC implementation bug |
| Flag bit 16 (`OVERRUN`) sticky | Host is flooding the link, or an ISR is running long |
| Mop motors do nothing | ENA/ENB jumpers missing → enable node not at +5 V |
| Mop motors behave randomly | No common ground |
| Pump twitches at power-up | Missing 10 kΩ gate pull-down; PA12 is high-Z during reset |
| MOSFET runs hot | 3.3 V gate drive raises R_DS(on); add a heatsink or a gate driver |

---

## Known gaps and open items

Documented honestly so the next person doesn't lose an evening:

- **`minibot_config.h` claims to be auto-generated** from `robot.yaml` by
  `tools/generate_config.py`. Neither file is in this tree. Until they are
  restored, the header **is** the source of truth and must be hand-edited, and the
  "do not hand-edit" banner is currently misleading.
- **`stm32_bench.py` is referenced but absent.** Step-verification has to be done
  manually (see the bring-up checklist) or with your own host script.
- **`prebuilt/` contains no binaries.** They were removed deliberately: they
  predate the L298N module and would run the robot with the mop motors dead.
  Build from source.
- **Comment/code mismatch in `main.c`.** Two comments describe the PA12 pump as
  "4 s HIGH / 4 s LOW"; the implementation in `minibot_signal.c` is 3 s / 5 s.
  The code is correct, the comments are stale.
- **`Debug/` build artifacts are committed** (~30 MB of `.o`, `.elf`, `.map`,
  `.list`). Consider adding a `.gitignore` and untracking them:

  ```gitignore
  Debug/
  build/
  *.o
  *.elf
  *.bin
  *.hex
  *.list
  *.map
  *.su
  *.cyclo
  *.d
  ```

  ```bash
  git rm -r --cached MAIN_CONTROLLER/firmware_Main/minibot_fw/Debug
  ```

- **No PWM speed control for the mop motors.** All eight free pins are used as
  L298N IN lines, so ENA/ENB are hard-tied to +5 V. Adding speed control needs two
  more free pins fed from a timer PWM output.
- **No encoders.** Odometry is an exact tally of emitted step pulses, which is
  correct only as long as no step is skipped. That's why the jerk limiting and the
  DIR-through-zero rule matter, and why step verification is on the checklist.
- **The mop motors and pump run open-loop from boot** with no host command and no
  way to stop them over the serial link. If host-controlled mopping is wanted,
  `DCM_Set()` and the pump need protocol commands wired to them.

---

## License

The `Drivers/` tree contains ST and ARM vendor code under their respective
licenses (see `Drivers/CMSIS/LICENSE.txt`,
`Drivers/STM32F4xx_HAL_Driver/LICENSE.md`). Project code in `Core/` is part of the
Smart Autonomous Floor Mopping Robot project.
