# Testing & troubleshooting

**Do these in order.** Each step depends on the one before. Skipping ahead is how
you spend a day debugging Nav2 for what is a swapped TX/RX pair.

---

## 0. Bench-test the STM32 — NO ROS. Robot on blocks.

```bash
python3 stm32_bench.py --watch
```

**Expect:** `steps L=0 R=0 | sps L=0 R=0 | seq=... | -` at ~50 Hz.

| Symptom | Cause |
|---|---|
| Nothing at all | TX/RX swapped; no common ground; wrong device |
| Garbage bytes | Wrong baud — **or the 25 MHz crystal problem** (see below) |
| `BAD CRC` lines | Noisy cable. Shorten it, ground it, keep it away from motor wires |
| Frames but `CRC_ERR` flag set | Same — the STM32 is rejecting corrupt commands (correctly) |

> ### The 25 MHz crystal trap
> If your board has a 25 MHz crystal (many F407VET6 "black boards" do) and you
> didn't change `HSE_VALUE` + `PLLM`, then your **baud rate and step rate are both
> wrong by the same 25/8 ratio**. The symptom is *garbage on serial AND motors at
> the wrong speed* — two confusing symptoms at once. See `stm32/README.md`.

### Direction check
```bash
python3 stm32_bench.py --left 500 --right 500 --secs 3
```
Both wheels should turn so the robot would go **forward**.

| What happens | Fix |
|---|---|
| Robot would **spin** | flip exactly **ONE** of `MB_LEFT_DIR_INVERT` / `MB_RIGHT_DIR_INVERT` |
| Robot would go **backward** | flip **BOTH** |

Fix it in `robot.yaml` → `firmware.left_dir_invert` / `right_dir_invert`, then
`python3 tools/generate_config.py && cd stm32 && make flash`.

> Fix direction in the **firmware**, not in `ros2_control.xacro`. That way the
> STM32's own step counts are correct even when you drive it from the bench tool.

### Microstep check — this one silently doubles every distance
```bash
python3 stm32_bench.py --verify-steps
```
Commands exactly 3200 steps = **one wheel revolution** at 1/16 microstepping.
One revolution of your 67.2 mm wheel = **211 mm of travel**. Mark the wheel AND
mark the floor — check both.

If the wheel turns **twice**, your DRV8825 jumpers say 1/8, not 1/16. Every
distance the robot ever reports will be off by 2×. Fix the jumpers
(M2=HIGH, M1=LOW, M0=LOW) **or** set `geometry.microstepping: 8` in `robot.yaml`
and regenerate.

---

## 1. IMU

```bash
i2cdetect -y 1                 # MUST show 68
ros2 run minibot_imu imu_calibrate.py
```

**Expect:** `|accel| ≈ 9.81`, gyro noise < 0.05 rad/s, `accel z ≈ +9.81`.

| Symptom | Meaning |
|---|---|
| `accel z` is **negative** | board is upside down → `axis_map: ['x','-y','-z']` |
| `accel z` ≈ 0 | board is on its side/rotated → run `--axis-check` |
| Gyro noise > 0.05 | **the robot moved during calibration** — the bias is garbage and every heading downstream inherits it |

### The axis check that actually matters
```bash
ros2 run minibot_imu imu_calibrate.py --axis-check
```
Rotate the robot **counter-clockwise** (viewed from above). `gyro z` **must go
positive** — that's the ROS convention.

If it's negative and you don't fix it, the EKF will confidently fuse a heading
that turns the **wrong way**. The symptom (a map that tears itself apart during
turns) looks nothing like the cause.

**Fix it in `axis_map`, never in the EKF.** The EKF cannot distinguish a mounting
error from real motion — it will happily fuse your mistake.

---

## 2. TF — exactly ONE publisher on `odom → base_link`

```bash
ros2 launch minibot_bringup robot.launch.py
ros2 run tf2_tools view_frames
ros2 topic echo /tf | grep -A2 base_link      # who is publishing?
```

**If the robot flickers between two poses in RViz, this is why.** Check:

```
diff_drive_controller : enable_odom_tf = false
imu_filter_madgwick   : publish_tf     = false
ekf_filter_node       : publish_tf     = true    ← the only one
```

---

## 3. LIDAR orientation — get this wrong and your map is mirrored

In RViz: fixed frame `odom`, add `LaserScan` on `/scan` and `RobotModel`.

1. Point the robot at a wall. The scan wall must appear **in front of** the robot,
   not behind it. If it's behind → `rpy="0 0 ${pi}"` in `lidar.xacro`.
2. Drive forward slowly. **The walls must stay still in `odom`.** If they smear or
   rotate, your `wheel_radius`/`wheel_separation` are wrong — go to step 4.

A mirrored `lidar_frame` produces a mirrored map, and then Nav2 confidently drives
into walls it believes are behind it.

---

## 4. Odometry calibration — the two numbers everything rests on

**Straight line FIRST.** Rotation error depends on wheel radius, so calibrating
rotation against an uncalibrated radius just moves the error around.

```bash
ros2 run minibot_bringup calibrate_odom.py --straight 2.0
# measure the ACTUAL distance with a tape measure, type it in
```

```bash
ros2 run minibot_bringup calibrate_odom.py --rotate 10
# count the ACTUAL revolutions, type it in
```

Apply the corrections to **`robot.yaml`**, then:
```bash
python3 tools/generate_config.py
cd stm32 && make flash
cd ../ros2_ws && colcon build --symlink-install
```

---

## 5. Map it (phase 1)

```bash
ros2 launch minibot_bringup robot.launch.py     # terminal 1
ros2 launch minibot_bringup map.launch.py       # terminal 2
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -r /cmd_vel:=/joy_vel            # terminal 3
```

**Drive slowly. CLOSE THE LOOP** — return to where you started before saving.
Without a loop closure, slam_toolbox has no chance to correct accumulated drift
and your map ends up subtly skewed. Every coverage lane inherits that skew.

```bash
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/maps/home
```

---

## 6. Plan it — OFFLINE, on your laptop

```bash
python3 prepare_map.py --map maps/home.yaml --start 0 0
```

Then, and this is **not optional**:

1. **Read `maps/home_qa.txt`.** Any unreachable regions? Any corridor narrower
   than 0.44 m?
2. **Open `maps/home_coverage_preview.png` and LOOK AT IT.**

> If the path is wrong on your screen, it will be wrong on your floor. A mistake
> caught here costs a re-run of a script. The same mistake caught on the floor
> costs a robot wedged behind the sofa for 15 minutes.

---

## 7. Clean it

```bash
ros2 launch minibot_bringup clean.launch.py \
    map:=$HOME/ros2_ws/maps/home.yaml \
    path:=$HOME/ros2_ws/maps/home_coverage_path.yaml
```

In RViz: **2D Pose Estimate** at the robot's real pose. Then **check two things
before you start**:

1. **The laser scan lines up with the map walls.**
2. **`/particlecloud` has CONVERGED** — a tight cluster, not a spread-out cloud.
   If it's still spread out, AMCL doesn't know where it is. Do not start the run.

```bash
ros2 service call /coverage/start std_srvs/srv/Trigger
ros2 topic echo /coverage_progress
```

---

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| Robot flickers between two poses in RViz | Two publishers on `odom → base_link` | §2 |
| Map is mirrored | `lidar_frame` rpy | §3 |
| Robot drives 2× too far | Microstep jumpers say 1/8 | `--verify-steps`, §0 |
| Robot spins instead of driving forward | One DIR inverted | flip **one** `MB_*_DIR_INVERT` |
| `CLAMPED` flag in feedback | Nav2 asking for more than the 20 RPM cap | Your limits disagree somewhere. `generate_config.py` and re-paste **all** of them |
| `CMD_TIMEOUT` flag | Pi went quiet >500 ms | Is `controller_manager` alive? Is the cable seated? |
| `CRC_ERR` flag | Corrupt frames | Cable too long / near the motor wires / no common ground |
| Coverage ends at ~60 % | Nav2 aborted chunks | `grep FAILED` the log. Usually AMCL drift or an obstacle the map doesn't know about |
| Nav2 aborts during every lane-end turn | Progress checker sees the in-place spin as a stall | Already fixed: `required_movement_radius: 0.25`, `movement_time_allowance: 30.0` |
| Edge pass won't plan | `inflation_radius` too big | Must be ~0.22, not Nav2's stock 0.70 |
| Uncleaned stripes between lanes | `lane_spacing` ≥ cleaning head width | Reduce `coverage.lane_spacing` in `robot.yaml` |
| Uncleaned stripe down the **centre** of every lane | The 100 mm gap between your two brushes, with no suction inlet in it | **Mechanical.** Check whether your intake actually sits in that gap |
| Uncleaned **9 cm strip along every wall** | The brush stops short of where the robot can reach | **Mechanical.** Mount brushes further outboard, or accept it |
| Robot never turns at all | `min_theta_velocity_threshold` > `omega_max` | Fixed in Rev 2 (0.050 vs omega_max 0.352). If you raise the RPM cap, regenerate |
| Costmap hangs on startup | keepout filter enabled with no mask server | Either launch with `use_keepout:=true mask:=...`, or remove the `filters:` lines from **both** costmaps |
| A room is never cleaned | Doorway < **0.48 m** | `prepare_map.py` told you this in `home_qa.txt`. It physically will not fit |
| Motors get hot when idle | Working as designed for 10 s, then coils drop | `serial.idle_disable_ms` |
| Motors whine / lose steps on turns | Jerk limit too loose, or DIR flipping at speed | Raise `limits.jerk_time_s`; the signed-velocity design already prevents the latter |
