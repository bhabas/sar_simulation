#!/bin/bash

## FIND SIMULATION PATH
DEEP_RL_PATH=$(find ~/catkin_ws/src -name 'sar_simulation' -type d | head -n 1)/sar_projects/DeepRL


## 0 DEG TRAINING AND DATA COLLECTION
python $DEEP_RL_PATH/T1_Policy_Pre-Training.py \
    --GroupName 2D_0deg_aRot60 \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot60/D2/SOV5_D2_0deg_S2D.json \
    --S3_Upload true

python $DEEP_RL_PATH/T3_Policy_Data_Collection.py \
    --GroupName 2D_0deg_aRot60 \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot60/D2/SOV5_D2_0deg_S2D.json \
    --S3_Upload true \
    --t_step_load 150000



## 45 DEG TRAINING AND DATA COLLECTION
python $DEEP_RL_PATH/T1_Policy_Pre-Training.py \
    --GroupName 2D_45deg_aRot60 \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot60/D2/SOV5_D2_45deg_S2D.json \
    --S3_Upload true

python $DEEP_RL_PATH/T3_Policy_Data_Collection.py \
    --GroupName 2D_45deg_aRot60 \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot60/D2/SOV5_D2_45deg_S2D.json \
    --S3_Upload true \
    --t_step_load 150000



## 90 DEG TRAINING AND DATA COLLECTION
python $DEEP_RL_PATH/T1_Policy_Pre-Training.py \
    --GroupName 2D_90deg_aRot60 \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot60/D2/SOV5_D2_90deg_S2D.json \
    --S3_Upload true

python $DEEP_RL_PATH/T3_Policy_Data_Collection.py \
    --GroupName 2D_90deg_aRot60 \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot60/D2/SOV5_D2_90deg_S2D.json \
    --S3_Upload true \
    --t_step_load 150000



## 135 DEG TRAINING AND DATA COLLECTION
python $DEEP_RL_PATH/T1_Policy_Pre-Training.py \
    --GroupName 2D_135deg_aRot60 \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot60/D2/SOV5_D2_135deg_S2D.json \
    --S3_Upload true

python $DEEP_RL_PATH/T3_Policy_Data_Collection.py \
    --GroupName 2D_135deg_aRot60 \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot60/D2/SOV5_D2_135deg_S2D.json \
    --S3_Upload true \
    --t_step_load 150000
    


## 180 DEG TRAINING AND DATA COLLECTION
python $DEEP_RL_PATH/T1_Policy_Pre-Training.py \
    --GroupName 2D_180deg_aRot60 \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot60/D2/SOV5_D2_180deg_S2D.json \
    --S3_Upload true

python $DEEP_RL_PATH/T3_Policy_Data_Collection.py \
    --GroupName 2D_180deg_aRot60 \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot60/D2/SOV5_D2_180deg_S2D.json \
    --S3_Upload true \
    --t_step_load 150000