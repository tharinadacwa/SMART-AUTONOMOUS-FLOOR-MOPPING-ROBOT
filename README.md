# minibot — autonomous coverage-cleaning robot  · **REVISION 2**

**STM32F407VET6 · 2× DRV8825 @1.4A · 2× NEMA-17 (NO ENCODERS) · Raspberry Pi 5 · RPLIDAR C1 · MPU6050**
**67.2 mm wheels · 400 mm track · 420 mm wheel span · pre-mapped · AMCL · ROS 2 Jazzy**

---

## :rotating_light: I GOT THE GEOMETRY WRONG IN REVISION 1. THIS IS THE FIX.

Revision 1 was built on **wheel_radius 0.19 m** and **wheel_separation 0.30 m**.
You measured the real thing:

| | Rev 1 | **Your actual hardware** | Error |
|---|---|---|---|
| wheel radius | 0.1900 m | **0.0336 m** (67.2 mm dia) | **5.65× too big** |
| wheel separation | 0.3000 m | **0.400 m** | **33 % too small** |
| robot radius | 0.1900 m | **0.210 m** | the wheels stick out past the body |

### Why it happened — a reasoning error, not just a bad input

I argued: *"5–20 RPM only makes sense with big wheels, otherwise you'd get 7 cm/s,
which is absurd."*

**Your wheels are small.** So the correct conclusion was never *"the wheels must be
big"* — it was **"the 20 RPM cap is the thing that's wrong."** Given two
contradictory inputs, I distrusted the wrong one, then built 336 files on it.

That lesson is now **encoded in the tooling**: `generate_config.py` warns whenever
the pulse engine is under 15 % utilised *and* torque margin is high — the exact
signature of an artificial speed cap throttling the robot.

Full changelog: **`docs/REVISION_2.md`**. Hardware record: **`docs/HARDWARE.md`**.

---

## Three things you probably haven't hit yet

### 1. The WHEELS are the widest part of the robot

```
  body: 380 mm   ├────────────────────┤
wheels: 420 mm  ├──────────────────────┤   ← WIDER
```

Nav2 now gets **`robot_radius: 0.210`** — half the *wheel* span. Plan against the
body (0.190) and Nav2 will confidently drive the **wheels** into walls it believes
it clears. Presents as *"it keeps scraping along walls"* and looks like a
controller bug.

**Consequence:** doorways narrower than **48 cm** are now unreachable (was 44 cm).

### 2. A 9 cm strip along every wall is NEVER cleaned

```
wall │
     │←──── 0.240 m ────→│     robot centre can get no closer
     │                   ●     (robot_radius 0.210 + safety 0.030)
     │←9cm→│←── 0.300 m brush ──→│
     │▓▓▓▓▓│ NEVER CLEANED
```

**This is mechanical. No software fixes it.** Accept it, drop `safety_margin`
0.03→0.01 (→7 cm), or **mount the brushes further outboard**.

### 3. :warning: The 100 mm gap between your two brushes — CONFIRM THIS

Two 100 mm brushes with centres 200 mm apart leaves a **100 mm gap in the middle**.

That's **fine if your suction inlet sits there** (side brushes sweeping inward is
the standard design). **If nothing is there, you have a 100 mm uncleaned stripe
down the centre of every single lane** — and the "% swept" readout would be
overstating reality. Look at the robot and tell me.

---

## :rotating_light: Your 20 RPM cap costs you 68 minutes per run

| RPM | Speed | Step rate | Pulse engine | 49 m² flat |
|---|---|---|---|---|
| **20** *(your spec — what I shipped)* | 0.070 m/s | 1066 sps | **5.3 %** | **94 min** |
| 60 | 0.211 m/s | 3200 sps | 16.0 % | **31 min** |
| 60 **+ lane 0.19→0.25** | 0.211 m/s | 3200 sps | 16.0 % | **26 min** |

At 20 RPM the pulse engine is **5 % used** and the motors have **~12× torque
margin** at 6 kg. **The cap is buying you nothing.**

Lane spacing 0.19 m against your **0.30 m** brush means **11 cm of overlap** — you
clean **37 % of the floor twice**. 0.25 m still leaves 5 cm of overlap.

**Both are one line each in `robot.yaml`.** I shipped your stated values.

---

## The good news: 6 kg is now easy

Small wheels **multiply force** (`F = torque / radius`):

| | Rev 1 (0.19 m wheels) | **Rev 2 (0.0336 m wheels)** |
|---|---|---|
| force per wheel @ 0.40 N·m | 2.1 N | **11.9 N** |
| torque needed for 6 kg | 0.565 N·m | **0.034 N·m** |
| verdict | **IMPOSSIBLE** (40 % short) | **~12× margin** |

6 kg is comfortable. So is 10 kg. You're now limited by **traction**, not torque.

> Caveat: this assumes **0.40 N·m** holding torque at 1.4 A — a *typical* figure for
> a NEMA-17 42-34. **I do not have your motor's datasheet.** Read the real number
> off the motor label before you trust that margin.

---

## What was verified, and what wasn't

| | |
|---|---|
| ✅ **Firmware compiles** | `arm-none-eabi-gcc 13.2.1`, **0 warnings in project code**, 3.7 % flash |
| ✅ **CRC protocol** | firmware C ↔ ROS C++ ↔ Python bench tool **agree bit-for-bit**; 100 % of single-bit flips rejected |
| ✅ **URDF expands** | 11 links, wheels at ±0.200, lidar at 0.2164, brushes at ±0.100 |
| ✅ **Coverage planner** | run end-to-end; correctly flags unreachable rooms and the 9 cm wall strip |
| ✅ **DDS invariant** | machine-checked — the generator *refuses* to emit a config that breaks it |
| ❌ **`colcon build` never run** | **no ROS 2 in my sandbox.** The C++ plugin and launch files are unverified against real ROS headers |
| ❌ **Nothing has run on hardware** | AMCL/EKF/RPP values are reasoned from docs, not measured |

**Be clear-eyed:** the logic/math/protocol is genuinely tested. The ROS-runtime
parts are well-reasoned but unproven. A `colcon build` on the Pi will find any
issues fast — they'd be compile errors, not flaky-robot-in-the-field bugs.

---

## Two directories

```
stm32/      →  cd stm32 && make flash
ros2_ws/    →  cd tools && ./setup_pi.sh
```

Everything derives from **`robot.yaml`** via **`tools/generate_config.py`**.

## Do this, in order

```bash
# 0. BENCH-TEST THE STM32. No ROS. Robot on blocks. DO NOT SKIP.
python3 stm32_bench.py --watch
python3 stm32_bench.py --verify-steps
#    -> commands ONE wheel revolution = 211 mm of travel on your 67.2 mm wheels.
#       Mark the wheel AND the floor. If it turns twice, your DRV8825 jumpers
#       say 1/8 not 1/16, and every distance is silently 2x wrong.

# 1. Install on the Pi
cd tools && ./setup_pi.sh && sudo reboot

# 2. IMU axis check (an inverted z makes the EKF fuse a heading that turns
#    the WRONG WAY, and the symptom looks nothing like the cause)
ros2 run minibot_imu imu_calibrate.py --axis-check

# 3. Calibrate the two numbers everything rests on
ros2 launch minibot_bringup robot.launch.py
ros2 run minibot_bringup calibrate_odom.py --straight 2.0   # fixes wheel_radius
ros2 run minibot_bringup calibrate_odom.py --rotate 10      # fixes wheel_separation
#    -> results into robot.yaml, then: python3 tools/generate_config.py

# 4. MAP your room (once)
ros2 launch minibot_bringup map.launch.py
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/maps/home

# 5. PLAN offline, on your laptop  <-- the step that saves your afternoon
python3 prepare_map.py --map maps/home.yaml --start 0 0
#    -> READ maps/home_qa.txt.  LOOK AT maps/home_coverage_preview.png.

# 6. CLEAN
ros2 launch minibot_bringup clean.launch.py \
    map:=$HOME/ros2_ws/maps/home.yaml \
    path:=$HOME/ros2_ws/maps/home_coverage_path.yaml
ros2 service call /coverage/start std_srvs/srv/Trigger
```

---

## Docs

| | |
|---|---|
| **`docs/HARDWARE.md`** | **the definitive hardware record + everything still unverified** |
| **`docs/REVISION_2.md`** | **what was wrong in Rev 1 and why** |
| `docs/DERIVED_NUMBERS.md` | generated — the audit trail from `robot.yaml` |
| `docs/ARCHITECTURE.md` | three phases, TF tree, velocity chain |
| `docs/INTERFACES.md` | every topic, service, action, TF frame, UART packet |
| `docs/FILE_MANIFEST.md` | every file: why it exists, who calls it, when it runs |
| `docs/TESTING.md` | bring-up order + troubleshooting table |
| `stm32/README.md` | build, flash, the 25 MHz crystal trap, the protocol |

---

## :rotating_light: SIX THINGS I STILL CANNOT VERIFY — please confirm

| # | Unknown | Why it matters |
|---|---|---|
| 1 | **Motor holding torque at 1.4 A** | I assumed 0.40 N·m. Every torque margin rests on it. **Read the motor label.** |
| 2 | **1.8°/step or 0.9°/step?** | A **2× error** in every distance |
| 3 | **Are the DRV8825 jumpers really 1/16?** (M0=LOW M1=LOW M2=HIGH) | A **2× error** in every distance |
| 4 | **Is your suction inlet in the 100 mm brush gap?** | If not: a 100 mm uncleaned stripe down every lane |
| 5 | **IMU height** — I guessed 60 mm | Minor, but measure it |
| 6 | **HSE crystal — 8 MHz or 25 MHz?** | Wrong baud **AND** wrong step rate *simultaneously* — a genuinely confusing pair of symptoms |

Answer #1–#4 and I'll regenerate. They're all one line each in `robot.yaml`.
