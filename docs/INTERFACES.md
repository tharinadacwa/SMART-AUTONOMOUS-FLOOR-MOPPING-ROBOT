# Interfaces — topics, services, actions, frames, packets

## Topics

| Topic | Type | Published by | Consumed by |
|---|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | `sllidar_ros2` | AMCL, both costmaps, slam_toolbox, collision_monitor |
| `/imu/data_raw` | `sensor_msgs/Imu` | `mpu6050_node` | `imu_filter_madgwick` |
| `/imu/data` | `sensor_msgs/Imu` | `imu_filter_madgwick` | `ekf_filter_node` |
| `/imu/temperature` | `sensor_msgs/Temperature` | `mpu6050_node` | diagnostics |
| `/diff_drive_controller/odom` | `nav_msgs/Odometry` | `diff_drive_controller` | `ekf_filter_node`, `calibrate_odom.py` |
| `/diff_drive_controller/cmd_vel` | `geometry_msgs/TwistStamped` | `twist_mux` | `diff_drive_controller` |
| `/odom` | `nav_msgs/Odometry` | `ekf_filter_node` | Nav2, `velocity_smoother` |
| `/joint_states` | `sensor_msgs/JointState` | `joint_state_broadcaster` | `robot_state_publisher` |
| `/nav_vel` | `TwistStamped` | `twist_stamper` | `twist_mux` (priority 10) |
| `/joy_vel` | `TwistStamped` | teleop | `twist_mux` (**priority 100** — a human always wins) |
| `/cmd_vel_smoothed` | `geometry_msgs/Twist` | `velocity_smoother` | `twist_stamper` |
| `/map` | `nav_msgs/OccupancyGrid` | `map_server` (latched) | AMCL, global costmap, `coverage_server` |
| `/particlecloud` | `geometry_msgs/PoseArray` | AMCL | **you, in RViz** |
| `/coverage_path` | `nav_msgs/Path` (latched) | `coverage_server` | RViz |
| `/coverage_map` | `OccupancyGrid` (latched) | `coverage_server` | RViz — cleaned (light) vs to-do (dark) |
| `/coverage_progress` | `std_msgs/Float32` | `coverage_server` | you |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | IMU, coverage | `diagnostic_aggregator` |
| `/costmap_filter_info` | `nav2_msgs/CostmapFilterInfo` | `costmap_filter_info_server` | both costmaps |
| `/keepout_filter_mask` | `OccupancyGrid` | `keepout_filter_mask_server` | keepout filter |

## Services

| Service | Type | Provided by | What it does |
|---|---|---|---|
| `/coverage/start` | `std_srvs/Trigger` | `coverage_server` | begin (or resume) the run |
| `/coverage/stop` | `std_srvs/Trigger` | `coverage_server` | abort — **resumable**, checkpoint is kept |
| `/coverage/skip` | `std_srvs/Trigger` | `coverage_server` | robot is wedged; skip this chunk and carry on |
| `/coverage/dock` | `std_srvs/Trigger` | `coverage_server` | go home now |
| `/imu/recalibrate` | `std_srvs/Trigger` | `mpu6050_node` | re-measure gyro bias (**hold the robot still**) |
| `/{local,global}_costmap/clear_entirely_*` | `nav2_msgs/ClearEntireCostmap` | Nav2 | used by the BT recovery |

## Actions

| Action | Type | Server | Client |
|---|---|---|---|
| `navigate_through_poses` | `nav2_msgs/NavigateThroughPoses` | `bt_navigator` | `coverage_server` (12 waypoints per goal) |
| `navigate_to_pose` | `nav2_msgs/NavigateToPose` | `bt_navigator` | `coverage_server` (docking), RViz |

## TF frames

| Frame | Parent | Published by |
|---|---|---|
| `map` | — | AMCL (phase 3) / slam_toolbox (phase 1) |
| `odom` | `map` | ↑ same |
| `base_link` | `odom` | **`ekf_filter_node` — SOLE PUBLISHER** |
| `base_footprint` | `base_link` | `robot_state_publisher` (static) |
| `body`, `left_wheel`, `right_wheel`, `caster_*` | `base_link` | `robot_state_publisher` |
| `lidar_frame` | `base_link` | `robot_state_publisher` (static) |
| `imu_link` | `base_link` | `robot_state_publisher` (static) |

## UART packets

Frame: `$<PAYLOAD>*<CRC8_HEX><LF>` — CRC-8/ATM (poly `0x07`, init `0x00`) over
PAYLOAD only. `$` **always** restarts the parser, which is what lets it
resynchronise after line noise instead of wedging.

### Pi → STM32

| Packet | Meaning |
|---|---|
| `$V,<l_sps>,<r_sps>*XX` | signed steps/second target. Clamped to ±1066 on both sides. |
| `$E,1*XX` / `$E,0*XX` | energise / de-energise the drivers |
| `$S*XX` | **EMERGENCY STOP** — bypasses the ramp entirely |
| `$R*XX` | zero the step counters |
| `$P*XX` | ping → `$K*XX` |

### STM32 → Pi, at 50 Hz

```
$F,<l_steps>,<r_steps>,<l_sps>,<r_sps>,<flags>,<seq>*XX
```

| Field | Meaning |
|---|---|
| `l_steps` / `r_steps` | int32, signed, cumulative. **THIS IS YOUR ODOMETRY.** |
| `l_sps` / `r_sps` | the *actual* current step rate, post-profile |
| `flags` | bitfield, below |
| `seq` | uint16, wraps. Lets the Pi **detect dropped frames** rather than silently interpolating across them. |

| Flag | Value | Meaning |
|---|---|---|
| `ENABLED` | 1 | drivers energised |
| `ESTOP` | 2 | emergency stop latched |
| `CMD_TIMEOUT` | 4 | the Pi went quiet for >500 ms; motors stopped |
| `CRC_ERR` | 8 | at least one corrupt frame since boot → **check your cable/grounding** |
| `OVERRUN` | 16 | UART overrun since boot |
| `CLAMPED` | 32 | a command exceeded the RPM cap → **your limits disagree somewhere; run `generate_config.py`** |

### Verified CRC vectors

| Payload | CRC |
|---|---|
| `V,1000,-1000` | `0x47` |
| `F,12345,-6789,500,-500,33,7` | `0x31` |

Firmware C, ROS 2 C++ (`serial_link.hpp`), and the Python bench tool were tested
against each other and **agree bit-for-bit**.
