#!/bin/bash

## FIND SIMULATION PATH
DEEP_RL_PATH=$(find ~/catkin_ws/src -name 'sar_simulation' -type d | head -n 1)/sar_projects/DeepRL


python $DEEP_RL_PATH/T1_Policy_Pre-Training.py \
    --GroupName SOV5_2D_aRot90/Multi \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot90/C2/SOV5_C2_Multi_S2D.json \
    --S3_Upload false

# python $DEEP_RL_PATH/T3_Policy_Data_Collection.py \
#     --GroupName SOV5_2D_aRot90/Multi \
#     --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot90/C2/SOV5_C2_Multi_S2D.json \
#     --S3_Upload true \
#     --t_step_load 150000
