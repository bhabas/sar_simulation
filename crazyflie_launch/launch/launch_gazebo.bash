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
Plane_Pos_X=$(rosparam get /Plane_Config/$Plane_Config/Pos_X)
Plane_Pos_Y=$(rosparam get /Plane_Config/$Plane_Config/Pos_Y)
Plane_Pos_Z=$(rosparam get /Plane_Config/$Plane_Config/Pos_Z)
Plane_Angle=$(rosparam get /Plane_Config/$Plane_Config/Plane_Angle)

## CONVERT PLANE ANGLE TO RADIANS
Plane_Angle_rad=$(echo "(180-$Plane_Angle)*3.14159/180" | bc -l)

## START GAZEBO 
roslaunch crazyflie_launch crazyflie_gazebo.launch \
    config_name:=$CONFIG_NAME \
    ground_name:=$GROUND_NAME \
    gui_flag:=$GUI_FLAG \
    pause_flag:=$PAUSE_FLAG \
    Plane_Model:=$Plane_Model \
    Plane_Pos_x:=$Plane_Pos_X \
    Plane_Pos_y:=$Plane_Pos_Y \
    Plane_Pos_z:=$Plane_Pos_Z \
    Plane_Angle:=$Plane_Angle_rad \

