cmake_minimum_required(VERSION 3.10)
project(tb4_commander_app)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclpy REQUIRED)

include_directories(include)

add_executable(follow_waypoints src/follow_waypoints.cpp)
ament_target_dependencies(follow_waypoints rclcpp geometry_msgs nav2_msgs nav2_util)

install(TARGETS follow_waypoints
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
scripts/follow_waypoints.py
DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()