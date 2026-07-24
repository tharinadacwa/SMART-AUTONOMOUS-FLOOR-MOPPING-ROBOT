# Hardware — the definitive record

**Everything in this project derives from this page. If a number here is wrong,
everything downstream is wrong.** Revision 1 proved that the hard way.

## Confirmed by measurement

| | Value | Notes |
|---|---|---|
| **Wheel diameter** | **67.2 mm** (with rubber) | → radius **0.0336 m**. *Measure it LOADED* — rubber compresses ~1 mm, and 1 mm on a 33.6 mm radius is a **3 % odometry error**. |
| **Wheel separation** | **0.400 m** | Centre-to-centre of the contact patches. (Outer ends are 41–42 cm; the kinematics needs the **midpoint** figure.) |
| **Wheel outer span** | **0.410–0.420 m** | → **`robot_radius` = 0.210 m** |
| **Body diameter** | 0.380 m | Chassis disc. **Narrower than the wheels.** |
| **LIDAR height** | **0.250 m** from the ground | → URDF `lidar_z` = 0.250 − 0.0336 = **0.2164 m** |
| **Cleaning brushes** | 2 × 100 mm dia, centres 200 mm apart | → **swept width 0.300 m**, **100 mm gap in the middle** |
| **Robot mass** | ~6 kg | |
| Main controller | STM32F407VET6 | |
| Compute | Raspberry Pi 5 | ROS 2 Jazzy |
| Motor drivers | 2 × DRV8825 @ **1.4 A** | Microstep jumpers **M0=LOW, M1=LOW, M2=HIGH** (1/16) |
| Motors | 2 × NEMA-17 42-34 | **NO ENCODERS.** Open loop. |
| LIDAR | RPLIDAR C1 | USB |
| IMU | MPU6050 | I²C-1, addr 0x68 |

## :rotating_light: THE WHEELS ARE THE WIDEST PART OF THE ROBOT

```
        body: 380 mm  ├────────────────────┤
      wheels: 420 mm ├──────────────────────┤   ← WIDER
```

Nav2 gets **`robot_radius: 0.210`** — half the *wheel* span, not half the body.

Plan against 0.190 and Nav2 will confidently drive the **wheels** into walls it
believes the *body* clears. This is the kind of error that presents as
"the robot keeps scraping along walls" and looks like a controller problem.

## :rotating_light: The 9 cm wall strip — a MECHANICAL limit

```
wall │
     │←── 0.240 m ──→│              robot centre can get no closer
     │               ●              (robot_radius 0.210 + safety 0.030)
     │←9cm→│←──── 0.300 m brush ────→│
     │▓▓▓▓▓│                          the brush stops 9 cm short
     └ NEVER CLEANED
```

**No software fixes this.** Your options:
1. **Accept** a 9 cm uncleaned perimeter
2. Drop `safety_margin` 0.03 → 0.01 → strip becomes **7 cm** (costs wall clearance)
3. **Mount the brushes further outboard** — the real fix

## :warning: The 100 mm gap between your brushes — CONFIRM THIS

```
   left brush          GAP           right brush
  ├─────────┤    ├───────────┤    ├─────────┤
 -0.15    -0.05  -0.05    +0.05  +0.05    +0.15
```

This is **normal and fine IF your suction inlet sits in that gap** (side brushes
sweeping debris inward to a central intake is the standard robot-vacuum design).

**If there is nothing in the middle, you have a 100 mm uncleaned stripe down the
centre of every single lane**, and no path planner can fix it. The coverage
"% swept" readout would then be *overstating* reality.

## :rotating_light: The 20 RPM cap is costing you an hour per run

| RPM | Speed | Step rate | Pulse engine used | 49 m² flat |
|---|---|---|---|---|
| **20** *(your spec)* | 0.070 m/s | 1066 sps | **5.3 %** | **94 min** |
| 40 | 0.141 m/s | 2133 sps | 10.7 % | 47 min |
| 60 | 0.211 m/s | 3200 sps | 16.0 % | **31 min** |
| 100 | 0.352 m/s | 5333 sps | 26.7 % | 19 min |

At 20 RPM the pulse engine is **5 % used** and the motors have **~12× torque
margin** at 6 kg. **The cap is buying you nothing.**

Add the lane-spacing fix (0.19 → 0.25, still 5 cm of overlap on a 0.30 m brush)
and 60 RPM gives you **26 minutes for identical coverage**.

Both are **one line each** in `robot.yaml`. I left them at your stated values.

## GPIO UART wiring — Pi 5 <-> STM32 (no USB adapter)

The STM32 <-> Pi link is a direct 3-wire UART connection, not USB:

```
   Pi 5 GPIO header              STM32F407VET6
   GPIO15 / pin 10 (RXD) <------- PA9  (USART1_TX)
   GPIO14 / pin 8  (TXD) -------> PA10 (USART1_RX)
   GND     / pin 6, 9...  ------- GND
```

TX crosses to RX and RX crosses to TX — wiring TX-to-TX gives silence on
both ends. Common ground is not optional; without it the UART has no shared
voltage reference. Both sides are native 3.3V logic, so no level shifter.

**Pi 5-specific step — this is the part every Pi 4 tutorial gets wrong for
you.** Add to `/boot/firmware/config.txt`:

```
dtoverlay=uart0-pi5
```

Reboot, then it appears as `/dev/ttyAMA0`. Unlike a Pi 4, the Pi 5's console
lives on its dedicated 3-pin debug UART connector, not on GPIO14/15 — so
there is normally no serial-console-login-shell conflict to disable first.

Verify before trusting anything above this layer:

```bash
ros2 run minibot_bringup stm32_bench.py --watch
#   -> "$F,..." feedback frames at ~50 Hz
```

If nothing appears, check in order: TX/RX crossover, common ground, the
`dtoverlay=uart0-pi5` line (and that you rebooted after adding it), then
baud rate (115200) and the HSE crystal (see risk #6 below) — a mismatched
crystal desyncs both the baud rate *and* the step rate simultaneously,
which is a confusing pair of symptoms to debug separately.

**Tradeoff vs. a USB-TTL adapter:** GPIO UART is simpler (2 signal wires,
no extra chip) but shares the Pi's ground plane directly with the STM32
board. Keep that board's ground solidly tied and route these three wires
away from the DRV8825/motor wiring to avoid noise coupling.

## Still UNVERIFIED — these are the remaining risks

| # | Unknown | Why it matters | How to resolve |
|---|---|---|---|
| 1 | **Motor holding torque at 1.4 A** | I assumed 0.40 N·m (typical for the size class). All torque margins depend on it. | Read the label / datasheet on your actual motor |
| 2 | **1.8°/step or 0.9°/step?** | A 2× error in every distance | `stm32_bench.py --verify-steps` |
| 3 | **Microstep jumpers really 1/16?** | A 2× error in every distance | Same test |
| 4 | **Suction inlet in the brush gap?** | 100 mm stripe down every lane if not | Look at the robot |
| 5 | **IMU height** | I guessed 60 mm | Measure it |
| 6 | **HSE crystal — 8 MHz or 25 MHz?** | Wrong baud **and** wrong step rate together | Look at the crystal |
