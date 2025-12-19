cmake_minimum_required(VERSION 3.5)
project(ds4_teleop)

# Find packages and libraries
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(ds4_driver_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Add the Python package
ament_python_install_package(${PROJECT_NAME})

# Install Python scripts
install(PROGRAMS
  ds4_teleop/controller_node.py
  ds4_teleop/imu_controller_node.py
  ds4_teleop/combined_controller_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Set up the build dependencies
ament_package()

