cmake_minimum_required(VERSION 3.16)
project(minibot_bringup)
find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/stm32_bench.py
  scripts/calibrate_odom.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config udev systemd DESTINATION share/${PROJECT_NAME})
ament_package()
