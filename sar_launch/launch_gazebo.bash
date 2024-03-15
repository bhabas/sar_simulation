#!/bin/bash
killall gzserver gzclient # kills all current gazebo processes
source ~/catkin_ws/devel/setup.bash

## FIND SIMULATION PATH
SIMULATION_PATH=$(find ~/catkin_ws/src -name 'sar_simulation' -type d | head -n 1)

## LOAD GAZEBO PARAMS
GUI_FLAG=$(rosparam get /SIM_SETTINGS/GUI_Flag)
PAUSE_FLAG=$(rosparam get /SIM_SETTINGS/Pause_Flag)

## LOAD SAR PARAMS
SAR_TYPE=$(rosparam get /SAR_SETTINGS/SAR_Type)
SAR_CONFIG=$(rosparam get /SAR_SETTINGS/SAR_Config)
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:${SIMULATION_PATH}/sar_gazebo/models/${SAR_TYPE}
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:${SIMULATION_PATH}/sar_gazebo/models/${SAR_TYPE}/Configs

## LOAD PLANE CONFIG PARAMS
PLANE_TYPE=$(rosparam get /PLANE_SETTINGS/Plane_Type)
PLANE_CONFIG=$(rosparam get /PLANE_SETTINGS/Plane_Config)
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:${SIMULATION_PATH}/sar_gazebo/models/${PLANE_TYPE}
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:${SIMULATION_PATH}/sar_gazebo/models/${PLANE_TYPE}/Configs

## LOAD PLANE CONFIG PARAMS
GROUND_TYPE=$(rosparam get /GROUND_SETTINGS/Ground_Type)
GROUND_CONFIG=$(rosparam get /GROUND_SETTINGS/Ground_Config)
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:${SIMULATION_PATH}/sar_gazebo/models/${GROUND_TYPE}
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:${SIMULATION_PATH}/sar_gazebo/models/${GROUND_TYPE}/Configs


## START GAZEBO 
roslaunch sar_launch Gazebo_Sim.launch \
    Gui_Flag:=$GUI_FLAG \
    Pause_Flag:=$PAUSE_FLAG \
    SAR_Type:=$SAR_TYPE \
    SAR_Config:=$SAR_CONFIG \
    Plane_Type:=$PLANE_TYPE \
    Plane_Config:=$PLANE_CONFIG \
    Ground_Type:=$GROUND_TYPE \
    Ground_Config:=$GROUND_CONFIG \



