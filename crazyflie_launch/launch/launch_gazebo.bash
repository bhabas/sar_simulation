#!/bin/bash
killall gzserver gzclient # kills all current gazebo processes
source ~/catkin_ws/devel/setup.bash

## LOAD GAZEBO PARAMS
roslaunch crazyflie_launch params.launch
GROUND_NAME=$(rosparam get /ENV_SETTINGS/Ground_Name)
GUI_FLAG=$(rosparam get /SIM_SETTINGS/GUI_Flag)
PAUSE_FLAG=$(rosparam get /SIM_SETTINGS/Pause_Flag)

## LOAD QUADROTOR PARAMS
CONFIG_NAME=$(rosparam get /QUAD_SETTINGS/CF_Config)

## LOAD PLANE CONFIG PARAMS
Plane_Config=$(rosparam get /PLANE_SETTINGS/Plane_Config)
Plane_Model=$(rosparam get /PLANE_SETTINGS/Plane_Model)


## START GAZEBO 
roslaunch crazyflie_launch crazyflie_gazebo.launch \
    Quad_Model:=$CONFIG_NAME \
    Ground_Model:=$GROUND_NAME \
    Plane_Model:=$Plane_Model \
    gui_flag:=$GUI_FLAG \
    pause_flag:=$PAUSE_FLAG \


