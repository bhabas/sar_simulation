#!/bin/bash
killall gzserver gzclient # kills all current gazebo processes
source ~/catkin_ws/devel/setup.bash

## UPLOAD ROS PARAMS
roslaunch crazyflie_launch params.launch
CONFIG_NAME=$(rosparam get /QUAD_SETTINGS/Config)
SURFACE_NAME=$(rosparam get /SURFACE_NAME)
GROUND_NAME=$(rosparam get /GROUND_NAME)
GUI_FLAG=$(rosparam get /GUI_FLAG)
PAUSE_FLAG=$(rosparam get /PAUSE_FLAG)

## START GAZEBO 
roslaunch crazyflie_launch crazyflie_gazebo.launch \
    surface_name:=$SURFACE_NAME \
    config_name:=$CONFIG_NAME \
    ground_name:=$GROUND_NAME \
    gui_flag:=$GUI_FLAG \
    pause_flag:=$PAUSE_FLAG \
