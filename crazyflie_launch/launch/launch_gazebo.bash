#!/bin/bash
killall gzserver gzclient # kills all current gazebo processes
source ~/catkin_ws/devel/setup.bash

## UPLOAD ROS PARAMS
roslaunch crazyflie_launch params.launch
MODEL_NAME=$(rosparam get /MODEL_NAME)
WORLD_NAME=$(rosparam get /WORLD_NAME)
GUI_FLAG=$(rosparam get /GUI_FLAG)

## START GAZEBO 
roslaunch crazyflie_launch crazyflie_gazebo.launch world_name:=$WORLD_NAME model_name:=$MODEL_NAME gui:=$GUI_FLAG
