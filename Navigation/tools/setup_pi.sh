#!/usr/bin/env bash
# =============================================================================
#  setup_pi.sh -- one-shot setup on a Raspberry Pi 5 running Ubuntu 24.04 +
#  ROS 2 Jazzy. Read it before you run it: it uses sudo.
#
#      cd tools && chmod +x setup_pi.sh && ./setup_pi.sh
# =============================================================================
set -euo pipefail

WS="${WS:-$HOME/ros2_ws}"
HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"

echo "=== workspace: $WS ==="
mkdir -p "$WS/src" "$WS/maps"

echo
echo "=== ROS 2 Jazzy packages ==="
sudo apt-get update
sudo apt-get install -y \
  ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
  ros-jazzy-diff-drive-controller ros-jazzy-joint-state-broadcaster \
  ros-jazzy-imu-filter-madgwick ros-jazzy-robot-localization \
  ros-jazzy-twist-mux ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
  ros-jazzy-nav2-amcl ros-jazzy-nav2-map-server ros-jazzy-nav2-lifecycle-manager \
  ros-jazzy-nav2-regulated-pure-pursuit-controller \
  ros-jazzy-slam-toolbox ros-jazzy-diagnostic-aggregator \
  ros-jazzy-diagnostic-updater ros-jazzy-xacro \
  ros-jazzy-teleop-twist-keyboard ros-jazzy-teleop-twist-joy ros-jazzy-joy

echo
echo "=== Python deps (the coverage planner needs these) ==="
sudo apt-get install -y \
  python3-numpy python3-scipy python3-opencv python3-yaml \
  python3-matplotlib python3-smbus2 python3-serial i2c-tools

echo
echo "=== source-only deps ==="
cd "$WS/src"
[ -d twist_stamper ] || git clone https://github.com/joshnewans/twist_stamper.git
[ -d sllidar_ros2 ]  || git clone https://github.com/Slamtec/sllidar_ros2.git

echo
echo "=== minibot packages ==="
for pkg in minibot_description minibot_hardware minibot_imu \
           minibot_navigation minibot_coverage minibot_bringup; do
  rm -rf "$WS/src/$pkg"
  cp -r "$ROOT/ros2_ws/src/$pkg" "$WS/src/$pkg"
  echo "  $pkg"
done

echo
echo "=== udev rules (stable /dev/minibot_lidar) ==="
sudo cp "$ROOT/ros2_ws/src/minibot_bringup/udev/99-minibot.rules" /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger

echo
echo "=== groups ==="
sudo usermod -aG i2c,dialout "$USER"

echo
echo "=== enable I2C (Pi 5) ==="
if ! grep -q "^dtparam=i2c_arm=on" /boot/firmware/config.txt 2>/dev/null; then
  echo "dtparam=i2c_arm=on" | sudo tee -a /boot/firmware/config.txt
  echo "  added dtparam=i2c_arm=on -- REBOOT REQUIRED"
fi

echo
echo "=== enable GPIO UART for the STM32 link (PA9/PA10 <-> GPIO14/15) ==="
if ! grep -q "^dtoverlay=uart0-pi5" /boot/firmware/config.txt 2>/dev/null; then
  echo "dtoverlay=uart0-pi5" | sudo tee -a /boot/firmware/config.txt
  echo "  added dtoverlay=uart0-pi5 -- REBOOT REQUIRED"
fi
# On the Pi 5 the console lives on the dedicated 3-pin debug UART connector,
# NOT GPIO14/15, so unlike a Pi 4 there is normally no login-shell-on-serial
# conflict to disable here. Confirm after reboot with: ls -l /dev/ttyAMA0

echo
echo "=== build ==="
cd "$WS"
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install

cat <<'EOF'

=============================================================
 DONE.

 1. REBOOT   (the i2c/dialout group change AND the uart0-pi5 overlay need it)

 2. VERIFY THE HARDWARE, IN THIS ORDER. Do not skip.

      i2cdetect -y 1              -> MUST show 68 (the MPU6050)
      ls -l /dev/ttyAMA0          -> the STM32 GPIO UART link
      ls -l /dev/minibot_lidar    -> the LIDAR (USB)

      # Robot ON BLOCKS, wheels in the air:
      ros2 run minibot_bringup stm32_bench.py --watch
      #   -> "$F,..." frames at ~50 Hz. If not, check in this order:
      #      1) TX/RX crossed?  PA9(TX)->GPIO15(RX), PA10(RX)->GPIO14(TX)
      #      2) common GROUND between STM32 board and Pi?
      #      3) dtoverlay=uart0-pi5 actually in /boot/firmware/config.txt
      #         and you rebooted after adding it?
      #      4) baud rate (115200) and the HSE crystal (8/25 MHz) match?
      #      Fix it HERE. Nothing above this layer can work until it does.

 3. source ~/ros2_ws/install/setup.bash

 If /dev/ttyAMA0 is missing after reboot:
      cat /boot/firmware/config.txt | grep uart0-pi5   # confirm the overlay is there
      dmesg | grep -i tty                              # see what the kernel enumerated
=============================================================
EOF
