################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(tb3_1_navigation2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)

################################################################################
# Install
################################################################################
install(
  DIRECTORY launch map param rviz
  DESTINATION share/${PROJECT_NAME}
)

# Install Python relay scripts and mark them executable.
#
# tf_relay.py    — bridges slam_toolbox's root /tf → /{ns}/tf (map→odom only).
#                  The Pi bringup now publishes hardware TFs directly to
#                  /{ns}/tf, so this relay handles slam_toolbox output only.
#
# map_relay.py   — bridges slam_toolbox's root /map → /{ns}/map with
#                  TRANSIENT_LOCAL so the global costmap never misses the map.
#
# tf_static_relay.py is RETIRED. The Pi bringup (robot_state_publisher) now
# publishes directly to /{ns}/tf_static with TRANSIENT_LOCAL durability.
# The script is kept in scripts/ for reference but is NOT installed or launched.
install(
  PROGRAMS
    scripts/map_relay.py
    scripts/tf_relay.py
  DESTINATION lib/${PROJECT_NAME}
)

################################################################################
# Macro for ament package
################################################################################
ament_package()