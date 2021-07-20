#!/bin/bash
killall gzserver gzclient # kills all current gazebo processes
source ~/catkin_ws/devel/setup.bash

# <rosparam command="load" file="$(find crazyflie_rl)/config/Sim_Settings.yaml" />
roslaunch crazyflie_rl params.launch
MODEL_NAME=$(rosparam get /Model_Name)
GUI_FLAG=$(rosparam get /GUI)
roslaunch crazyflie_gazebo crazyflie_sitl.launch world_name:="ceiling" model_name:=$MODEL_NAME gui:=$GUI_FLAG
# Script does not move past this line

# Command | ROS Package (not strictly folder name) | launch file | launch parameters
