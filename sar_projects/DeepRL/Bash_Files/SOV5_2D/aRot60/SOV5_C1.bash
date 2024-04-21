#!/bin/bash

## FIND SIMULATION PATH
DEEP_RL_PATH=$(find ~/catkin_ws/src -name 'sar_simulation' -type d | head -n 1)/sar_projects/DeepRL


## 0 DEG TRAINING AND DATA COLLECTION
python $DEEP_RL_PATH/T2_Policy_FineTuning.py \
    --GroupName SOV5_2D_aRot60/0deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot60/C1/SOV5_C1_0deg_S2D.json \
    --PT_GroupName SOV5_2D_aRot90/0deg \
    --PT_TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot90/C1/SOV5_C1_0deg_S2D.json \
    --t_step_load 150000 \
    --S3_Upload true && \

python $DEEP_RL_PATH/T3_Policy_Data_Collection.py \
    --GroupName SOV5_2D_aRot60/0deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot60/C1/SOV5_C1_0deg_S2D.json \
    --S3_Upload true \
    --t_step_load 100000



## 45 DEG TRAINING AND DATA COLLECTION
# python $DEEP_RL_PATH/T2_Policy_FineTuning.py \
#     --GroupName SOV5_2D_aRot60/45deg \
#     --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot60/C1/SOV5_C1_45deg_S2D.json \
#     --PT_GroupName SOV5_2D_aRot90/45deg \
#     --PT_TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot90/C1/SOV5_C1_45deg_S2D.json \
#     --t_step_load 150000 \
#     --S3_Upload true && \

python $DEEP_RL_PATH/T3_Policy_Data_Collection.py \
    --GroupName SOV5_2D_aRot60/45deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot60/C1/SOV5_C1_45deg_S2D.json \
    --S3_Upload true \
    --t_step_load 120000



## 90 DEG TRAINING AND DATA COLLECTION
# python $DEEP_RL_PATH/T2_Policy_FineTuning.py \
#     --GroupName SOV5_2D_aRot60/90deg \
#     --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot60/C1/SOV5_C1_90deg_S2D.json \
#     --PT_GroupName SOV5_2D_aRot90/90deg \
#     --PT_TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot90/C1/SOV5_C1_90deg_S2D.json \
#     --t_step_load 150000 \
#     --S3_Upload true && \

python $DEEP_RL_PATH/T3_Policy_Data_Collection.py \
    --GroupName SOV5_2D_aRot60/90deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot60/C1/SOV5_C1_90deg_S2D.json \
    --S3_Upload true \
    --t_step_load 117000



## 135 DEG TRAINING AND DATA COLLECTION
# python $DEEP_RL_PATH/T2_Policy_FineTuning.py \
#     --GroupName SOV5_2D_aRot60/135deg \
#     --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot60/C1/SOV5_C1_135deg_S2D.json \
#     --PT_GroupName SOV5_2D_aRot90/135deg \
#     --PT_TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot90/C1/SOV5_C1_135deg_S2D.json \
#     --t_step_load 150000 \
#     --S3_Upload true && \

python $DEEP_RL_PATH/T3_Policy_Data_Collection.py \
    --GroupName SOV5_2D_aRot60/135deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot60/C1/SOV5_C1_135deg_S2D.json \
    --S3_Upload true \
    --t_step_load 150000
    


## 180 DEG TRAINING AND DATA COLLECTION
# python $DEEP_RL_PATH/T2_Policy_FineTuning.py \
#     --GroupName SOV5_2D_aRot60/180deg \
#     --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot60/C1/SOV5_C1_180deg_S2D.json \
#     --PT_GroupName SOV5_2D_aRot90/180deg \
#     --PT_TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot90/C1/SOV5_C1_180deg_S2D.json \
#     --t_step_load 150000 \
#     --S3_Upload true && \

python $DEEP_RL_PATH/T3_Policy_Data_Collection.py \
    --GroupName SOV5_2D_aRot60/180deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot60/C1/SOV5_C1_180deg_S2D.json \
    --S3_Upload true \
    --t_step_load 150000