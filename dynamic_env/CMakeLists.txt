cmake_minimum_required(VERSION 3.5)
project(my_dynamic_environment)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(turtlebot3_gazebo REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(xacro REQUIRED)

install(DIRECTORY launch worlds urdf
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
