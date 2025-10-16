cmake_minimum_required(VERSION 3.10)
project(tb4_map_server)

# Find dependencies
find_package(ament_cmake REQUIRED)

install(DIRECTORY 
  launch 
  maps 
  config
  DESTINATION share/${PROJECT_NAME})

ament_package()