# Revision 2 — what changed, and why Revision 1 was wrong

## The error

Revision 1 was built on **wheel_radius = 0.190 m** and **wheel_separation = 0.300 m**.
Both were wrong.

| | Rev 1 | Measured | Error |
|---|---|---|---|
| wheel radius | 0.1900 m | **0.0336 m** | **5.65× too big** |
| wheel separation | 0.3000 m | **0.400 m** | **33 % too small** |
| robot radius | 0.1900 m | **0.210 m** | wheels stick out past the body |

## Why the "physics check" failed — this was a reasoning error, not just bad input

Revision 1 contained this argument:

> *"The 5–20 RPM band only makes physical sense with large wheels — small wheels
> would give 1.8–7.1 cm/s, which is absurdly slow. Therefore the wheels must be
> large."*

**The wheels are small.** So the correct conclusion was never *"the wheels are
large."* It was:

> **"The 20 RPM cap is the number that's wrong."**

Given a contradiction between two inputs, I picked the wrong one to distrust —
and then built 336 files on top of that choice. The lesson is encoded in the
tooling now: `generate_config.py` **warns** whenever the pulse engine is under
15 % utilised *and* torque margin is high, because that combination means an
artificial speed cap is throttling the robot.

## What every wrong number caused

| Consequence | Rev 1 | Rev 2 |
|---|---|---|
| `v_max` (20 RPM) | 0.3979 m/s | **0.0704 m/s** |
| `omega_max` | 2.6529 rad/s | **0.3519 rad/s** |
| 180° spin | 1.18 s | **8.93 s** |
| distance per step | 0.3731 mm | **0.0660 mm** |
| Nav2 `robot_radius` | 0.190 | **0.210** |
| Nav2 `inflation_radius` | 0.22 | **0.24** |
| min passable gap | 0.44 m | **0.48 m** |
| 49 m² flat | 16 min | **94 min** |

## Nav2 parameters that were ACTIVELY BROKEN

These would not merely have been suboptimal — the robot would not have worked:

| Parameter | Rev 1 | Problem | Rev 2 |
|---|---|---|---|
| `min_theta_velocity_threshold` | 0.30 | `omega_max` is only **0.352** → Nav2 would classify **every rotation the robot can perform** as "stopped". **It would never turn.** | **0.050** |
| `min_x_velocity_threshold` | 0.05 | `v_max` is only **0.070** → 71 % of the speed range written off as "stopped" | **0.010** |
| `rotate_to_heading_angular_vel` | 2.20 | **6.3× over** the physical maximum | **0.2991** |
| `lookahead_dist` | 0.40 m | at 0.070 m/s that is **5.7 seconds** ahead — would smooth straight through the lane-end turns coverage depends on | **0.20 m** |
| `movement_time_allowance` | 30 s | a 180° turn now takes **8.9 s** with zero linear progress | **45 s** |
| `robot_radius` | 0.19 | ignores that the **wheels** are the widest part | **0.210** |

## The good news — torque

Small wheels **multiply force** (`F = torque / radius`):

| | Rev 1 (0.190 m wheels) | Rev 2 (**0.0336 m wheels**) |
|---|---|---|
| force per wheel @ 0.40 N·m | 2.1 N | **11.9 N** |
| torque needed for 6 kg | 0.565 N·m | **0.034 N·m** |
| **verdict** | **IMPOSSIBLE** (40 % short) | **~12× margin** |

**6 kg was impossible on the wheels I assumed. It is comfortable on your real
wheels.** So is 10 kg. The binding constraint is now traction, not torque.

## What did NOT change

**The firmware.** `MB_MAX_STEP_RATE` is still 1066 sps, because step rate depends
only on RPM and microstepping — **not on wheel size**. The wheel radius only
determines how much *distance* those steps produce. Only the informational
`MB_WHEEL_RADIUS_M` / `MB_WHEEL_SEP_M` constants moved.

That is a direct benefit of having put every derived number behind one generator.
