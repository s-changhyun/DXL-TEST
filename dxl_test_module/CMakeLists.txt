cmake_minimum_required(VERSION 2.8.3)
project(dxl_test_module)

################################################################################
# Packages
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  cmake_modules
  eigen_conversions
  dynamixel_sdk
  robotis_device
  robotis_math
  robotis_controller_msgs
  robotis_framework_common
)

find_package(Eigen REQUIRED)

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Catkin specific configuration
################################################################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dxl_test_module
)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_library(dxl_test_module
  src/test_module.cpp
)

add_dependencies(dxl_test_module ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(dxl_test_module
  yaml-cpp
  ${catkin_LIBRARIES}
)
