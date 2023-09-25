cmake_minimum_required(VERSION 3.0.2)
project(panda_physical)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  moveit_core
  moveit_ros_planning_interface
)

## Declare a catkin package
catkin_package()

## Build
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)

add_executable(move_group_interface_test src/main.cpp src/Manipulate.cpp)
target_link_libraries(move_group_interface_test ${catkin_LIBRARIES} ${Boost_LIBRARIES})

