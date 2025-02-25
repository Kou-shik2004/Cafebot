cmake_minimum_required(VERSION 3.8)
project(cafebot)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(joint_state_publisher REQUIRED)
find_package(joint_state_publisher_gui REQUIRED)
find_package(gazebo_ros REQUIRED)

# Install directories
install(DIRECTORY
  urdf
  launch
  models
  config
  worlds
  DESTINATION share/${PROJECT_NAME}
)

# Mark executable scripts for installation
install(PROGRAMS
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()