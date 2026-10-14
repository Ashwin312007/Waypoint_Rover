cmake_minimum_required(VERSION 3.8)
project(rover_waypoint_follower)

find_package(ament_cmake REQUIRED)

# Install all resource directories
install(
  DIRECTORY
  launch
  scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
