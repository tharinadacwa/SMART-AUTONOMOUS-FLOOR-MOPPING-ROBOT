cmake_minimum_required(VERSION 3.16)
project(minibot_imu)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  minibot_imu/mpu6050_node.py
  minibot_imu/imu_calibrate.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
ament_package()
