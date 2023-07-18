#!/bin/bash
# Script: run_gazebo.sh

CAMERA=$1

# Set environment variable
export INCLUDE_D435_CAMERA=$CAMERA

# Call roslaunch
roslaunch panda_moveit run_gazebo.launch