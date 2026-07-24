# File manifest

For every file: **why it exists · who calls it · what depends on it · when it runs**.

## Root

| File | Why · Who · When |
|---|---|
| `robot.yaml` | **THE SINGLE SOURCE OF TRUTH.** All geometry and limits. Read by `tools/generate_config.py` only. Never read at runtime. |
| `tools/generate_config.py` | Derives every number in the project from `robot.yaml`. Run manually after any geometry change. `--check` mode exits 1 if generated files are stale. |
| `tools/setup_pi.sh` | One-shot Pi 5 install: apt packages, source deps, udev, colcon build. Run once. |

## `stm32/` — firmware

| File | Why · Who calls it · When |
|---|---|
| `minibot.ioc` | CubeMX project. Optional — the tree builds with plain `make`. Provided so you can regenerate peripherals if you prefer the GUI. |
| `Makefile` | `make` / `make flash` / `make dfu` / `make size`. |
| `STM32F407VETX_FLASH.ld` | 512 K flash @ 0x08000000, 128 K RAM @ 0x20000000, 64 K CCM. Caps the heap below the stack so a runaway alloc fails cleanly instead of corrupting step-engine state. |
| `Core/Inc/minibot_config.h` | **GENERATED.** Every tuning constant. Do not hand-edit. |
| `Core/Src/main.c` | `main()`, `SystemClock_Config()` (168 MHz), and the three HAL callbacks that bridge into the motion core. |
| `Core/Src/gpio.c` | `MX_GPIO_Init()`. Called **first** among peripherals — it drives nENABLE HIGH so the motors are dead before anything else can twitch. |
| `Core/Src/tim.c` | `MX_TIM7_Init()`. PSC=83, ARR=24 → 40 kHz. Started by `MB_Init()`. |
| `Core/Src/usart.c` | `MX_USART1_UART_Init()`. PA9/PA10, 115200 8N1, RX interrupt. |
| `Core/Src/stm32f4xx_hal_msp.c` | Peripheral clocks, AF pins, **NVIC priorities** (TIM7=0, USART1=1). Called from inside `HAL_*_Init()`. |
| `Core/Src/stm32f4xx_it.c` | Vector handlers. Fault handlers **kill the motors then hang**. |
| `Core/Src/minibot_motion.c` | The three-layer step engine. `MB_StepISR()` runs at 40 kHz from TIM7; `MB_Task()` runs in the main loop. |
| `Core/Src/minibot_protocol.c` | CRC-8 framing. `MB_ProtoRxByte()` runs inside the USART1 ISR. |
| `Core/Src/syscalls.c` / `sysmem.c` | newlib stubs. Never called in practice; the linker demands the symbols. |
| `Drivers/` | Official ST HAL + CMSIS, trimmed to the modules we compile. |

## `ros2_ws/src/minibot_hardware/` — the ros2_control plugin

| File | Why · Who calls it · When |
|---|---|
| `stepper_diffdrive.{hpp,cpp}` | Replaces `diffdrive_arduino`, whose protocol is built entirely around quadrature encoders and PID — there is no parameter that makes it work here. Loaded by `controller_manager` from the `<ros2_control>` block in the URDF. `read()`/`write()` at 50 Hz. |
| `serial_link.hpp` | Header-only POSIX termios + CRC-8. Deliberately no `serial` package dependency. |
| `test/test_protocol.cpp` | Pins the CRC known-answer vectors against the firmware. If these ever diverge, the STM32 silently rejects every command and you debug Nav2 for a day. `colcon test`. |
| `minibot_hardware.xml` | pluginlib manifest. |

## `ros2_ws/src/minibot_description/`

| File | Why |
|---|---|
| `urdf/base.xacro` | 380 mm circular body. `base_link` = centre of rotation = centre of the disc. |
| `urdf/lidar.xacro` | RPLIDAR C1, **centred at x=0, y=0**. Off-centre means every in-place turn sweeps the scan origin in a circle, and any frame error goes straight into yaw. |
| `urdf/imu.xacro` | MPU6050 frame. |
| `urdf/ros2_control.xacro` | Binds joints → the STM32 plugin. `steps_per_rev` here **must equal** `MB_STEPS_PER_REV` in the firmware. |
| `rviz/navigation.rviz` | Includes `/particlecloud`. **Watch it** — if it stays spread out, AMCL has not converged and you must not start the run. |

## `ros2_ws/src/minibot_imu/`

| File | Why · When |
|---|---|
| `mpu6050_node.py` | I²C driver → `/imu/data_raw`. `orientation_covariance[0] = -1` (ROS for "no orientation here") — Madgwick supplies it downstream so two filters never fight over the same quantity. Publishes `/diagnostics`. 100 Hz. |
| `imu_calibrate.py` | **Standalone, no ROS.** `--axis-check` walks you through determining `axis_map`. Run before you trust the EKF: an inverted z-axis makes the EKF confidently fuse a heading that turns the wrong way. |
| `config/ekf.yaml` | robot_localization. Fuses `vx` (trustworthy — a literal pulse count) and `vyaw` from the gyro (which doesn't care about wheel slip). **Sole publisher of `odom → base_link`.** |
| `config/imu_params.yaml` | Driver + Madgwick params. `gain: 0.1` leans deliberately toward the gyro — the accelerometer is the noisy one on a robot with motors next to the IMU. |

## `ros2_ws/src/minibot_navigation/`

| File | Why |
|---|---|
| `config/nav2_params.yaml` | AMCL (alphas tuned for open-loop steppers) + RegulatedPurePursuit + keepout filter. **All velocity numbers generated.** |
| `config/keepout_params.yaml` | Costmap filter servers. Only possible because the map is frozen. |
| `config/slam_mapping.yaml` | Phase 1 only. `minimum_travel_heading: 0.10` (stock is 0.5) — this robot spins in place, travelling 0 m, and the stock threshold would never trigger a scan-match during a turn. |
| `behavior_trees/navigate_through_poses_coverage.xml` | **No Spin in recovery.** A Spin pushes the robot off the lane it is mid-way through sweeping; Nav2 replans from the new pose and carries on, leaving an uncleaned stripe that nothing reports. |
| `behavior_trees/navigate_to_pose_dock.xml` | Spin **is** allowed here — no lane is being swept. |

## `ros2_ws/src/minibot_coverage/`

| File | Why · When |
|---|---|
| `prepare_map.py` | **OFFLINE, on your laptop.** Clean → QA → plan → render. The most important tool in the project. A mistake caught here costs a re-run of a script; the same mistake on the floor costs a robot wedged behind the sofa. |
| `coverage_server.py` | **Deliberately dumb.** Plays back the frozen path. Checkpoints after every chunk (resumable). Returns to dock. Services: `/coverage/{start,stop,skip,dock}`. |

## `ros2_ws/src/minibot_bringup/`

| File | Why · When |
|---|---|
| `launch/robot.launch.py` | Hardware layer. Always running. |
| `launch/map.launch.py` | Phase 1. Once. |
| `launch/clean.launch.py` | Phase 3. AMCL + Nav2 + keepout + coverage. **On the Pi.** |
| `launch/rviz.launch.py` | Laptop. Viewing + manual override only — **no autonomy**, so a Wi-Fi drop doesn't stop the robot mid-run. |
| `scripts/stm32_bench.py` | **RUN THIS FIRST.** No ROS. If this doesn't work, nothing above it will, and you'll waste a day blaming Nav2 for a swapped TX/RX pair. |
| `scripts/calibrate_odom.py` | Empirically fixes `wheel_radius` then `wheel_separation`. Listens to `/diff_drive_controller/odom`, **not** `/odom` — calibrating against the EKF means calibrating the wheels against the filter that's compensating for them. |
| `config/controllers.yaml` | `enable_odom_tf: false` — the EKF owns that TF. |
| `udev/99-minibot.rules` | Names the LIDAR's USB-serial device (assigned in enumeration order otherwise). The STM32 link is now GPIO UART (`/dev/ttyAMA0`, fixed by hardware, no udev rule needed) rather than a second USB-serial device. |
| `systemd/*.service` | `minibot-clean` is **not** enabled by default and does **not** auto-start the run. A robot that starts driving before anyone confirmed AMCL converged is a robot that drives into a wall. |
