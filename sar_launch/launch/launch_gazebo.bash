#!/bin/bash
killall gzserver gzclient # kills all current gazebo processes
source ~/catkin_ws/devel/setup.bash

## FIND SIMULATION PATH
SIMULATION_PATH=$(find ~/catkin_ws/src -name 'crazyflie_simulation' -type d | head -n 1)

## LOAD GAZEBO PARAMS
roslaunch sar_launch params.launch
GROUND_NAME=$(rosparam get /ENV_SETTINGS/Ground_Name)
GUI_FLAG=$(rosparam get /SIM_SETTINGS/GUI_Flag)
PAUSE_FLAG=$(rosparam get /SIM_SETTINGS/Pause_Flag)

## LOAD SAR PARAMS
SAR_TYPE=$(rosparam get /SAR_SETTINGS/SAR_Type)
SAR_CONFIG=$(rosparam get /SAR_SETTINGS/SAR_Config)

## LOAD PLANE CONFIG PARAMS
Plane_Config=$(rosparam get /PLANE_SETTINGS/Plane_Config)
Plane_Model=$(rosparam get /PLANE_SETTINGS/Plane_Model)

## CREATE MODEL PATH
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:${SIMULATION_PATH}/sar_gazebo/models/${SAR_TYPE}


## START GAZEBO 
roslaunch sar_launch Gazebo_Sim.launch \
    Gui_Flag:=$GUI_FLAG \
    Pause_Flag:=$PAUSE_FLAG \
    SAR_Type:=$SAR_TYPE \
    SAR_Model:=$SAR_CONFIG \
    Ground_Model:=$GROUND_NAME \
    Plane_Model:=$Plane_Model \


