#!/bin/bash

## FIND SIMULATION PATH
DEEP_RL_PATH=$(find ~/catkin_ws/src -name 'sar_simulation' -type d | head -n 1)/sar_projects/DeepRL

## 135 DEG TRAINING AND DATA COLLECTION
python $DEEP_RL_PATH/T2_Policy_FineTuning.py \
    --GroupName SOV5_3D_aRot60/135deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_3D_Sim/aRot60/A1/SOV5_A1_135deg_S2D.json \
    --PT_GroupName SOV5_2D_aRot60/135deg \
    --PT_TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot60/A1/SOV5_A1_135deg_S2D.json \
    --t_step_load 150000 \
    --S3_Upload true && \


echo "Enter the t_step_load value for T3_Policy_Data_Collection.py:"
read T_STEP_LOAD


python $DEEP_RL_PATH/T3_Policy_Data_Collection.py \
    --GroupName SOV5_3D_aRot60/135deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_3D_Sim/aRot60/A1/SOV5_A1_135deg_S2D.json \
    --S3_Upload true \
    --t_step_load $T_STEP_LOAD
    