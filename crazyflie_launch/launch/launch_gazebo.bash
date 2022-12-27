#!/bin/bash
killall gzserver gzclient # kills all current gazebo processes
source ~/catkin_ws/devel/setup.bash

## UPLOAD ROS PARAMS
roslaunch crazyflie_launch params.launch
CONFIG_NAME=$(rosparam get /QUAD_SETTINGS/Config)
SURFACE_NAME=$(rosparam get /ENV_SETTINGS/Surface_Name)
GROUND_NAME=$(rosparam get /ENV_SETTINGS/Ground_Name)
GUI_FLAG=$(rosparam get /SIM_SETTINGS/GUI_Flag)
PAUSE_FLAG=$(rosparam get /SIM_SETTINGS/Pause_Flag)

Plane_Pos_x=$(rosparam get /ENV_SETTINGS/Plane_Position/X)
Plane_Pos_y=$(rosparam get /ENV_SETTINGS/Plane_Position/Y)
Plane_Pos_z=$(rosparam get /ENV_SETTINGS/Plane_Position/Z)
Plane_Theta=$(rosparam get /ENV_SETTINGS/Plane_Theta)

## CONVERT THETA ANGLE TO RADIANS
Plane_Theta_rad=$(echo "(180-$Plane_Theta)*3.14159/180" | bc -l)

## START GAZEBO 
roslaunch crazyflie_launch crazyflie_gazebo.launch \
    surface_name:=$SURFACE_NAME \
    config_name:=$CONFIG_NAME \
    ground_name:=$GROUND_NAME \
    gui_flag:=$GUI_FLAG \
    pause_flag:=$PAUSE_FLAG \
    Plane_Pos_x:=$Plane_Pos_x \
    Plane_Pos_y:=$Plane_Pos_y \
    Plane_Pos_z:=$Plane_Pos_z \
    Plane_Theta:=$Plane_Theta_rad \

