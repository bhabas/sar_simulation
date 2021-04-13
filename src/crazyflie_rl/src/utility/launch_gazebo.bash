#!/bin/bash
killall gzserver gzclient # kills all current gazebo processes
source ~/catkin_ws/devel/setup.bash

roslaunch crazyflie_gazebo crazyflie_sitl.launch world_name:="ceiling" model_name:="crazyflie_model_Narrow-Long" gui:="true"
# Script does not move past this line

# Command | ROS Package (not strictly folder name) | launch file | launch parameters
