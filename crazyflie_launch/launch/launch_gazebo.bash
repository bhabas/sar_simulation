#!/bin/bash
killall gzserver gzclient # kills all current gazebo processes
source ~/catkin_ws/devel/setup.bash

## LOAD GAZEBO PARAMS
roslaunch crazyflie_launch params.launch
GROUND_NAME=$(rosparam get /ENV_SETTINGS/Ground_Name)
GUI_FLAG=$(rosparam get /SIM_SETTINGS/GUI_Flag)
PAUSE_FLAG=$(rosparam get /SIM_SETTINGS/Pause_Flag)

## LOAD QUADROTOR PARAMS
CONFIG_NAME=$(rosparam get /QUAD_SETTINGS/Config)

## LOAD PLANE CONFIG PARAMS
SURFACE_NAME=$(rosparam get /ENV_SETTINGS/Surface_Name)

Plane_Config=$(rosparam get /PLANE_SETTINGS/Plane_Config)
Plane_Model=$(rosparam get /PLANE_SETTINGS/Plane_Model)
Plane_Pos_X=$(rosparam get /Plane_Config/$Plane_Config/Pos_X)
Plane_Pos_Y=$(rosparam get /Plane_Config/$Plane_Config/Pos_Y)
Plane_Pos_Z=$(rosparam get /Plane_Config/$Plane_Config/Pos_Z)
Plane_Theta=$(rosparam get /Plane_Config/$Plane_Config/Theta)

## CONVERT PLANE THETA ANGLE TO RADIANS
Plane_Theta_rad=$(echo "(180-$Plane_Theta)*3.14159/180" | bc -l)

## START GAZEBO 
roslaunch crazyflie_launch crazyflie_gazebo.launch \
    surface_name:=$SURFACE_NAME \
    config_name:=$CONFIG_NAME \
    ground_name:=$GROUND_NAME \
    gui_flag:=$GUI_FLAG \
    pause_flag:=$PAUSE_FLAG \
    Plane_Model:=$Plane_Model \
    Plane_Pos_x:=$Plane_Pos_X \
    Plane_Pos_y:=$Plane_Pos_Y \
    Plane_Pos_z:=$Plane_Pos_Z \
    Plane_Theta:=$Plane_Theta_rad \

