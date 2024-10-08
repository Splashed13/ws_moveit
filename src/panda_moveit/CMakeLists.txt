cmake_minimum_required(VERSION 3.0.2)
project(panda_moveit)

set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  message_runtime
  gazebo_ros
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_perception
  rviz_visual_tools
  moveit_visual_tools
  tf2_geometry_msgs
  geometric_shapes
  tf2_ros
  tf2_eigen
  actionlib_msgs
  visualization_msgs
  control_msgs
  pluginlib
  tf2
  sensor_msgs
  moveit_commander
  geometry_msgs
  realsense_gazebo_description
  pcl_conversions
  pcl_ros
  actionlib
  franka_gripper
  #realsense2_description
)

find_package(gazebo REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

# # Or other packages containing msgs
generate_messages(
  DEPENDENCIES 
  std_msgs
  geometry_msgs
  moveit_msgs  
  actionlib_msgs 
  trajectory_msgs 
  control_msgs 
  visualization_msgs 
  sensor_msgs 
)

link_directories(${GAZEBO_LIBRARY_DIRS})

# INCLUDE_DIRS include/${PROJECT_NAME}/
#  LIBRARIES ws_moveit
#  CATKIN_DEPENDS franka_ros moveit panda_moveit_config roscpp std_msgs
#  DEPENDS
#    roscpp
#    gazebo_ros


# catkin_package ensures that the libraries and executables created in the package are linked together
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  INCLUDE_DIRS 
  CATKIN_DEPENDS 
    rospy 
    roscpp 
    std_msgs 
    message_runtime 
    gazebo_ros 
    moveit_core 
    moveit_ros_planning 
    moveit_ros_planning_interface 
    moveit_ros_perception 
    rviz_visual_tools 
    moveit_visual_tools 
    tf2_geometry_msgs 
    geometric_shapes 
    tf2_ros 
    tf2_eigen 
    actionlib_msgs 
    visualization_msgs 
    control_msgs 
    pluginlib 
    tf2 
    sensor_msgs 
    moveit_commander
    realsense_gazebo_description
    actionlib
    franka_gripper
    #realsense2_description
    pcl_conversions
    pcl_ros
  DEPENDS
    EIGEN3
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${GAZEBO_INCLUDE_DIRS}
)

# Allows for python scripts to be installed to the bin directory 
catkin_install_python(PROGRAMS scripts/pick_and_place.py
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# Pick and Place node
add_executable(pick_and_place src/main.cpp src/PickAndPlace.cpp src/Utilities.cpp)
target_link_libraries(pick_and_place ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# New point cloud subscriber node
add_executable(pcd_writer src/PCDWriter.cpp)
target_link_libraries(pcd_writer ${catkin_LIBRARIES} ${Boost_LIBRARIES})

