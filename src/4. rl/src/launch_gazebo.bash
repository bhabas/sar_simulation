#!/bin/bash
killall gzserver gzclient # kills all current gazebo processes
cd ~/catkin_ws
source devel/setup.bash

## Initializes correct ROS Path variables for:
# GAZEBO_PLUGIN_PATH
# GAZEBO_MODEL_PATH
# LD_LIBRARY_PATH
source src/crazyflie_simulation/src/crazyflie_gazebo/setup_gazebo.bash $(pwd) $(pwd)/devel/lib

roslaunch   crazyflie_gazebo crazyflie_sitl.launch world_name:="ceiling" vehicle:="crazyflie_landing_gears" gui:="true"
# Command | ROS Package (not strictly folder name) | launch file | launch parameters
