#!/bin/bash

## FIND SIMULATION PATH
DEEP_RL_PATH=$(find ~/catkin_ws/src -name 'sar_simulation' -type d | head -n 1)/sar_projects/DeepRL


## 0 DEG TRAINING AND DATA COLLECTION
python $DEEP_RL_PATH/T1_Policy_Pre-Training.py \
    --GroupName SOV5_2D_aRot30/0deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot30/C3/SOV5_C3_0deg_S2D.json \
    --S3_Upload true

python $DEEP_RL_PATH/T3_Policy_Data_Collection.py \
    --GroupName SOV5_2D_aRot30/0deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot30/C3/SOV5_C3_0deg_S2D.json \
    --S3_Upload true \
    --t_step_load 150000



## 45 DEG TRAINING AND DATA COLLECTION
python $DEEP_RL_PATH/T1_Policy_Pre-Training.py \
    --GroupName SOV5_2D_aRot30/45deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot30/C3/SOV5_C3_45deg_S2D.json \
    --S3_Upload true

python $DEEP_RL_PATH/T3_Policy_Data_Collection.py \
    --GroupName SOV5_2D_aRot30/45deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot30/C3/SOV5_C3_45deg_S2D.json \
    --S3_Upload true \
    --t_step_load 150000



## 90 DEG TRAINING AND DATA COLLECTION
python $DEEP_RL_PATH/T1_Policy_Pre-Training.py \
    --GroupName SOV5_2D_aRot30/90deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot30/C3/SOV5_C3_90deg_S2D.json \
    --S3_Upload true

python $DEEP_RL_PATH/T3_Policy_Data_Collection.py \
    --GroupName SOV5_2D_aRot30/90deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot30/C3/SOV5_C3_90deg_S2D.json \
    --S3_Upload true \
    --t_step_load 150000



## 135 DEG TRAINING AND DATA COLLECTION
python $DEEP_RL_PATH/T1_Policy_Pre-Training.py \
    --GroupName SOV5_2D_aRot30/135deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot30/C3/SOV5_C3_135deg_S2D.json \
    --S3_Upload true

python $DEEP_RL_PATH/T3_Policy_Data_Collection.py \
    --GroupName SOV5_2D_aRot30/135deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot30/C3/SOV5_C3_135deg_S2D.json \
    --S3_Upload true \
    --t_step_load 150000
    


## 180 DEG TRAINING AND DATA COLLECTION
python $DEEP_RL_PATH/T1_Policy_Pre-Training.py \
    --GroupName SOV5_2D_aRot30/180deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot30/C3/SOV5_C3_180deg_S2D.json \
    --S3_Upload true

python $DEEP_RL_PATH/T3_Policy_Data_Collection.py \
    --GroupName SOV5_2D_aRot30/180deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot30/C3/SOV5_C3_180deg_S2D.json \
    --S3_Upload true \
    --t_step_load 150000