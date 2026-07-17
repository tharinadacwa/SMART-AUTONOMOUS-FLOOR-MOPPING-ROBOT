# STM32F407VET6 firmware — build & flash

**Complete and self-contained.** The official ST HAL drivers are included. You do
**not** need STM32CubeMX or CubeIDE — but a `.ioc` is provided if you want them.

I compiled this exact tree with `arm-none-eabi-gcc 13.2.1`:

```
   text    data     bss     dec
  19104     116    3304   22524

  FLASH:  19220 / 524288  (3.7%)
  RAM  :   3420 / 131072  (2.6%)
```

**Zero warnings in project code.** (The three `-Wunused-parameter` warnings are
inside ST's own `stm32f4xx_hal_flash_ex.c` and are theirs, not ours.)

---

## Build

```bash
sudo apt install gcc-arm-none-eabi stlink-tools
cd stm32
make            # -> build/minibot.{elf,hex,bin}
make flash      # ST-Link
make dfu        # USB DFU: hold BOOT0, tap RESET, then run this
make size
```

### CubeIDE instead?
`File → Import → C/C++ → Existing Code as Makefile Project`, point at `stm32/`,
toolchain *Cross ARM GCC*. Or open `minibot.ioc` in CubeMX — the peripheral
config there matches `main.c` exactly.

> If you regenerate from the `.ioc`, CubeMX **overwrites** `main.c`, `gpio.c`,
> `tim.c` and `usart.c`. Everything of ours lives between `USER CODE BEGIN/END`
> markers or in `minibot_*.c/h`, which CubeMX never touches. Diff before you
> commit.

---

## ⚠️ CHECK YOUR CRYSTAL FIRST

The firmware assumes an **8 MHz HSE crystal**. Many F407VET6 "black boards" ship
**25 MHz**.

If yours is 25 MHz and you don't change it, your **baud rate AND your step rate
are both wrong by the same 25/8 ratio**. The symptom is *"garbage on the serial
port AND the motors run at the wrong speed"* — two confusing symptoms at once,
and people lose an evening to it.

Two edits:

| File | Change |
|---|---|
| `Core/Inc/stm32f4xx_hal_conf.h` | `#define HSE_VALUE ((uint32_t)25000000U)` |
| `Core/Src/main.c`, `SystemClock_Config()` | `osc.PLL.PLLM = 25;` |

Leave `PLLN`/`PLLP`/`PLLQ` alone. `PLLM` exists purely to bring the PLL input to
1 MHz (`8/8 = 1`, `25/25 = 1`), so everything downstream still lands on 168 MHz.

---

## Architecture — three layers

```
 TIM7 ISR @ 40 kHz  ─┬─  Layer 1: DDS pulse engine
                     │      phase accumulator; a 32-bit wrap = one STEP pulse.
                     │      Increments the step COUNTER in the same breath, which
                     │      is why odometry is exact and not an estimate.
                     │
                     └─  every 40th tick (1 kHz):
                            Layer 2: jerk-limited S-curve profile
                              a += clamp(a_des - a, ±jerk·dt)
                              v += a·dt
                            Layer 3: signed velocity passes THROUGH zero, so DIR
                              only flips while the motor is essentially stopped.

 main loop  ─────────  MB_Task(): parse commands, comms watchdog, 50 Hz feedback
```

### The DDS low-time proof

The ISR drives `STEP` **low at the top** of the tick and **high at the bottom**.
That gives ~23 µs high / ≥25 µs low. The DRV8825 needs 1.9 µs of each.

The low-time guarantee is *arithmetic*, not luck: if the accumulator increment is
< 0.5 (i.e. step rate < ISR_HZ/2), then immediately after a wrap the accumulator
holds a value < 0.5, so the **next** tick cannot possibly wrap. Two pulses can
never land on consecutive ticks.

`tools/generate_config.py` **refuses to emit a header that violates this.**
At 20 RPM you use **1066 sps = 5.3% of the 20 kHz ceiling**. Enormous margin.

### Why jerk limiting is not a nicety

A plain trapezoidal ramp steps acceleration discontinuously from 0 to a_max —
which is a torque step, exactly the impulse that makes a stepper skip. Your robot
**reverses direction at the end of every coverage lane (~19× per room)**. The
S-curve is what keeps the odometry honest across a whole run.

---

## Serial protocol

```
$<PAYLOAD>*<CRC8_HEX><LF>          CRC-8/ATM, poly 0x07, init 0x00, over PAYLOAD
```

| Pi → STM32 | |
|---|---|
| `$V,<l_sps>,<r_sps>*XX` | signed steps/second target |
| `$E,<0\|1>*XX` | driver enable / disable |
| `$S*XX` | EMERGENCY STOP (bypasses the ramp entirely) |
| `$R*XX` | reset step counters |
| `$P*XX` | ping → replies `$K*XX` |

| STM32 → Pi (50 Hz) | |
|---|---|
| `$F,<l_steps>,<r_steps>,<l_sps>,<r_sps>,<flags>,<seq>*XX` | `l_steps`/`r_steps` are **your odometry** |

Flags: `1`=ENABLED `2`=ESTOP `4`=CMD_TIMEOUT `8`=CRC_ERR `16`=OVERRUN `32`=CLAMPED

**Why CRC and not a checksum:** the motor wires next to this cable switch amps at
20 kHz. One flipped bit in a velocity command is a robot driving into a wall.
CRC-8 catches all single-bit, all double-bit and all burst errors up to 8 bits.
A sum or XOR does not.

**Verified.** Known-answer vectors, pinned across all three implementations:

| payload | CRC |
|---|---|
| `V,1000,-1000` | `0x47` |
| `F,12345,-6789,500,-500,33,7` | `0x31` |

Firmware C, the ROS 2 C++ (`serial_link.hpp`) and the Python bench tool
(`stm32_bench.py`) were tested against each other and **agree bit-for-bit**.
100 % of single-bit flips are rejected; resync after line noise and recovery from
a truncated frame both work.

---

## Safety

| | |
|---|---|
| Boot | `MX_GPIO_Init()` runs **first** and drives nENABLE HIGH — motors dead before anything else can twitch |
| Comms watchdog | no command for 500 ms → ramp to stop, **stay energised** (holding torque, won't roll on a slope) |
| Idle | no command for 10 s → drop the coils (a stationary stepper at full current is a 10 W heater doing no work) |
| Fault handlers | HardFault/BusFault/etc **kill the motors, then hang**. A firmware that has faulted must not still be driving |
| E-stop | bypasses the profile. Losing steps is an acceptable trade for stopping now |
| NaN guard | non-finite velocity → send STOP |

## Interrupt priorities — do not swap these

| | |
|---|---|
| TIM7 | **0** (highest) |
| USART1 | 1 |
| SysTick | 15 (lowest) |

TIM7 must never be delayed: a late step pulse is jitter, which is audible whine
at best and a lost step at worst — and a lost step on an encoderless robot is a
silent lie in your odometry that nothing downstream can detect.

The TIM7 ISR is ~2 µs. USART1 at 115200 gets a byte every 87 µs. Even in the
worst case the UART is serviced ~2 µs late — **40× inside its deadline**. The
temptation is to give the UART top priority "so we never miss a byte"; that
trades a problem you don't have for one you can't detect.

## Tuning

Everything derives from `robot.yaml`. **Never hand-edit `minibot_config.h`.**

```bash
# edit ../robot.yaml, then:
python3 ../tools/generate_config.py
make clean && make
```
