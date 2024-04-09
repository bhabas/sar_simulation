#!/bin/bash

## FIND SIMULATION PATH
DEEP_RL_PATH=$(find ~/catkin_ws/src -name 'sar_simulation' -type d | head -n 1)/sar_projects/DeepRL


## 0 DEG TRAINING AND DATA COLLECTION
python $DEEP_RL_PATH/T1_Policy_Pre-Training.py \
    --GroupName IM_2D_aRot60/0deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/IM_2D_Sim/aRot60/D3/IM_D3_0deg_S2D.json \
    --S3_Upload true

python $DEEP_RL_PATH/T3_Policy_Data_Collection.py \
    --GroupName IM_2D_aRot60/0deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/IM_2D_Sim/aRot60/D3/IM_D3_0deg_S2D.json \
    --S3_Upload true \
    --t_step_load 150000



## 45 DEG TRAINING AND DATA COLLECTION
python $DEEP_RL_PATH/T1_Policy_Pre-Training.py \
    --GroupName IM_2D_aRot60/45deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/IM_2D_Sim/aRot60/D3/IM_D3_45deg_S2D.json \
    --S3_Upload true

python $DEEP_RL_PATH/T3_Policy_Data_Collection.py \
    --GroupName IM_2D_aRot60/45deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/IM_2D_Sim/aRot60/D3/IM_D3_45deg_S2D.json \
    --S3_Upload true \
    --t_step_load 150000



## 90 DEG TRAINING AND DATA COLLECTION
python $DEEP_RL_PATH/T1_Policy_Pre-Training.py \
    --GroupName IM_2D_aRot60/90deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/IM_2D_Sim/aRot60/D3/IM_D3_90deg_S2D.json \
    --S3_Upload true

python $DEEP_RL_PATH/T3_Policy_Data_Collection.py \
    --GroupName IM_2D_aRot60/90deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/IM_2D_Sim/aRot60/D3/IM_D3_90deg_S2D.json \
    --S3_Upload true \
    --t_step_load 150000



## 135 DEG TRAINING AND DATA COLLECTION
python $DEEP_RL_PATH/T1_Policy_Pre-Training.py \
    --GroupName IM_2D_aRot60/135deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/IM_2D_Sim/aRot60/D3/IM_D3_135deg_S2D.json \
    --S3_Upload true

python $DEEP_RL_PATH/T3_Policy_Data_Collection.py \
    --GroupName IM_2D_aRot60/135deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/IM_2D_Sim/aRot60/D3/IM_D3_135deg_S2D.json \
    --S3_Upload true \
    --t_step_load 150000
    


## 180 DEG TRAINING AND DATA COLLECTION
python $DEEP_RL_PATH/T1_Policy_Pre-Training.py \
    --GroupName IM_2D_aRot60/180deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/IM_2D_Sim/aRot60/D3/IM_D3_180deg_S2D.json \
    --S3_Upload true

python $DEEP_RL_PATH/T3_Policy_Data_Collection.py \
    --GroupName IM_2D_aRot60/180deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/IM_2D_Sim/aRot60/D3/IM_D3_180deg_S2D.json \
    --S3_Upload true \
    --t_step_load 150000