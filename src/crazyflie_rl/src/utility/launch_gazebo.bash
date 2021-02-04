#!/bin/bash
killall gzserver gzclient # kills all current gazebo processes
cd ~/catkin_ws
source devel/setup.bash

## Initializes ROS Path variables for:
# GAZEBO_PLUGIN_PATH
# GAZEBO_MODEL_PATH
# LD_LIBRARY_PATH
source src/crazyflie_simulation/src/crazyflie_gazebo/setup_gazebo.bash $(pwd) $(pwd)/devel/lib


roslaunch crazyflie_gazebo crazyflie_sitl.launch world_name:="ceiling" vehicle:="crazyflie_model_Narrow-Short" gui:="false"
# script does not move past this line

# Command | ROS Package (not strictly folder name) | launch file | launch parameters
