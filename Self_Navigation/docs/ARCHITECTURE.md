# Architecture

## The three phases

```
PHASE 1 — MAP (once)                          PHASE 2 — PLAN (offline, laptop)
────────────────────                          ────────────────────────────────
robot.launch.py                               prepare_map.py
  + map.launch.py  (slam_toolbox)               ├─ clean:  seal wall gaps, despeckle
  + teleop                                      ├─ QA:     reachability, bottlenecks, time
        │                                       ├─ plan:   edge pass + 19 cm boustrophedon
        ▼                                       └─ render: a PNG you LOOK AT
  map_saver_cli                                       │
        │                                             ▼
        └──────► home.pgm / home.yaml ──────►  home_coverage_path.yaml
                                                home_qa.txt
                                                home_coverage_preview.png

PHASE 3 — CLEAN (every run, all on the Pi)
──────────────────────────────────────────
  map_server ──► AMCL ──────────────────────► map → odom  (corrections; JUMPS)
                   ▲
                  /scan
                   │
  coverage_server ─┴─► NavigateThroughPoses ─► bt_navigator
        (plays back the frozen path)                │
                                                    ▼
                                    planner_server (NavFn)
                                    controller_server (RegulatedPurePursuit)
                                                    │
                                          /cmd_vel_smoothed
                                                    │
                                       twist_stamper ──► /nav_vel
                                                    │
                                       twist_mux ───┴── /joy_vel (priority 100)
                                                    │
                                   /diff_drive_controller/cmd_vel
                                                    │
                                        diff_drive_controller
                                                    │
                            minibot_hardware/StepperDiffDrive
                                                    │
                                 $V,<l>,<r>*CRC over UART @ 115200
                                                    │
                                              STM32F407
                                                    │
                                   40 kHz DDS + S-curve profile
                                                    │
                                         2× DRV8825 → 2× NEMA-17

  STM32 ──► $F,<steps>,... @ 50 Hz ──► StepperDiffDrive
                                             │
                            /diff_drive_controller/odom  (enable_odom_tf: FALSE)
                                             │
  MPU6050 ──► /imu/data_raw ──► madgwick ──► /imu/data  (publish_tf: FALSE)
                                             │
                                        ekf_filter_node
                                             │
                            /odom  AND  odom → base_link  ◄── SOLE PUBLISHER
```

## TF tree

```
map                     ← AMCL (phase 3) or slam_toolbox (phase 1)
 └── odom               ← robot_localization EKF.  ONLY THIS NODE.
      └── base_link     ← the centre of rotation = the centre of the disc
           ├── base_footprint
           ├── body
           ├── left_wheel      (continuous, from joint_state_broadcaster)
           ├── right_wheel     (continuous)
           ├── caster_front / caster_rear
           ├── lidar_frame     ← CENTRED at x=0, y=0
           └── imu_link
```

### The single most common way to break this robot

**Two publishers on `odom → base_link`.** The symptom is the robot flickering
between two poses in RViz and the local costmap jittering. Guard it with:

```
diff_drive_controller : enable_odom_tf = false
imu_filter_madgwick   : publish_tf     = false
ekf_filter_node       : publish_tf     = true      ← the only one
```

### Why `world_frame: odom` and not `map` in the EKF

`odom → base_link` must be **smooth and continuous** — Nav2's local costmap
depends on it. AMCL's corrections are **discontinuous**: it jumps. Those jumps
belong on the `map → odom` edge, where nothing downstream integrates them. Set
`world_frame: map` and you get AMCL's jumps injected into the local costmap.

---

## Where odometry comes from with no encoders

The STM32 **generates** every STEP pulse, so it knows exactly how many it emitted
and in which direction. It reports that tally at 50 Hz:

```
position_rad = steps × 2π / steps_per_rev
```

This is **not an estimate**. It is a count of pulses that physically left the pin.
It is exact right up until the motor *skips a step*.

That is the whole design constraint, and it drives three decisions:

1. **The 20 RPM cap and the jerk-limited S-curve** exist to make skipping
   arbitrarily unlikely. At 1066 sps you're at 5.3 % of the pulse engine's
   ceiling and nowhere near the motor's torque limit.
2. **The MPU6050 is fused in** to catch the *heading* error that a skipped step
   would otherwise hide. The gyro measures yaw **rate** directly — it doesn't
   care about wheel slip, wheel radius, or wheel separation.
3. **AMCL, not slam_toolbox localization.** slam_toolbox's own README says its
   localization mode *"needs high quality odometry"*. Open-loop step counts are,
   by definition, not that. AMCL's motion model has explicit noise parameters
   (`alpha1..alpha5`) where you **tell it** how bad your odometry is:

   ```yaml
   alpha1: 0.30   # rotation noise from rotation       ← HIGH: the weak axis
   alpha3: 0.05   # translation noise from translation ← LOW: steps are exact
   ```

   Note that translation really *is* exact here: at 33.6 mm wheels each step is
   **0.066 mm** of travel — a very fine ruler. Rotation is the weak axis because
   it divides by `wheel_separation`, which you measured with a tape.

   That asymmetry *is* the encoding of "no encoders". A scan matcher has no such
   release valve.

---

## Why a circular robot makes coverage tractable

For a disc, **inscribed radius == circumscribed radius**. For *this* robot that radius is **0.210 m** — half the **wheel** span, because the wheels (410–420 mm) are wider than the body (380 mm). The robot can
rotate in place *anywhere it can physically stand*. A rectangular robot cannot —
it needs extra clearance to sweep its corners around.

Consequence: **every point on the coverage path is automatically a valid turning
point.** That's why Nav2 gets `robot_radius: 0.19` (a scalar) instead of a
footprint polygon, and why the boustrophedon lanes can be planned without any
turn-feasibility check.

---

## The velocity chain — every layer must agree

```
Nav2 FollowPath  ≤  velocity_smoother  ≤  diff_drive_controller
    ≤  Pi-side clamp (max_step_rate)  ≤  STM32 clamp (MB_MAX_STEP_RATE)
    ≤  what the motor can do without skipping
```

If any layer asks for more than the one below it can deliver, the excess is
**silently clipped** — and then the planner believes the robot went somewhere it
did not. This single class of bug causes more "my odometry is drifting" reports
than anything else.

**That is why `tools/generate_config.py` exists.** The same physical constants
are needed in five places. Generate them; never hand-maintain five copies.

```bash
python3 tools/generate_config.py          # writes them
python3 tools/generate_config.py --check  # exits 1 if stale — wire into CI
```
