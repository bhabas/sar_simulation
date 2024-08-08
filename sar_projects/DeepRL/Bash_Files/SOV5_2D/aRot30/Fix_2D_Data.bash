#!/bin/bash

## FIND SIMULATION PATH
DEEP_RL_PATH=$(find ~/catkin_ws/src -name 'sar_simulation' -type d | head -n 1)/sar_projects/DeepRL

## 0 deg plots

python $DEEP_RL_PATH/T4.5_Data_Fix.py \
    --GroupName SOV5_2D_aRot30/0deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot30/A1/SOV5_A1_0deg_S2D.json \
    --S3_Upload true

python $DEEP_RL_PATH/T4.5_Data_Fix.py \
    --GroupName SOV5_2D_aRot30/0deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot30/A2/SOV5_A2_0deg_S2D.json \
    --S3_Upload true

python $DEEP_RL_PATH/T4.5_Data_Fix.py \
    --GroupName SOV5_2D_aRot30/0deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot30/B1/SOV5_B1_0deg_S2D.json \
    --S3_Upload true

python $DEEP_RL_PATH/T4.5_Data_Fix.py \
    --GroupName SOV5_2D_aRot30/0deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot30/B2/SOV5_B2_0deg_S2D.json \
    --S3_Upload true

python $DEEP_RL_PATH/T4.5_Data_Fix.py \
    --GroupName SOV5_2D_aRot30/0deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot30/C1/SOV5_C1_0deg_S2D.json \
    --S3_Upload true

python $DEEP_RL_PATH/T4.5_Data_Fix.py \
    --GroupName SOV5_2D_aRot30/0deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot30/C2/SOV5_C2_0deg_S2D.json \
    --S3_Upload true

python $DEEP_RL_PATH/T4.5_Data_Fix.py \
    --GroupName SOV5_2D_aRot30/0deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot30/C3/SOV5_C3_0deg_S2D.json \
    --S3_Upload true

python $DEEP_RL_PATH/T4.5_Data_Fix.py \
    --GroupName SOV5_2D_aRot30/0deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot30/D1/SOV5_D1_0deg_S2D.json \
    --S3_Upload true

python $DEEP_RL_PATH/T4.5_Data_Fix.py \
    --GroupName SOV5_2D_aRot30/0deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot30/D2/SOV5_D2_0deg_S2D.json \
    --S3_Upload true

python $DEEP_RL_PATH/T4.5_Data_Fix.py \
    --GroupName SOV5_2D_aRot30/0deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot30/D3/SOV5_D3_0deg_S2D.json \
    --S3_Upload true




## 45 deg plots

python $DEEP_RL_PATH/T4.5_Data_Fix.py \
    --GroupName SOV5_2D_aRot30/45deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot30/A1/SOV5_A1_45deg_S2D.json \
    --S3_Upload true

python $DEEP_RL_PATH/T4.5_Data_Fix.py \
    --GroupName SOV5_2D_aRot30/45deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot30/A2/SOV5_A2_45deg_S2D.json \
    --S3_Upload true

python $DEEP_RL_PATH/T4.5_Data_Fix.py \
    --GroupName SOV5_2D_aRot30/45deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot30/B1/SOV5_B1_45deg_S2D.json \
    --S3_Upload true

python $DEEP_RL_PATH/T4.5_Data_Fix.py \
    --GroupName SOV5_2D_aRot30/45deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot30/B2/SOV5_B2_45deg_S2D.json \
    --S3_Upload true

python $DEEP_RL_PATH/T4.5_Data_Fix.py \
    --GroupName SOV5_2D_aRot30/45deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot30/C1/SOV5_C1_45deg_S2D.json \
    --S3_Upload true

python $DEEP_RL_PATH/T4.5_Data_Fix.py \
    --GroupName SOV5_2D_aRot30/45deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot30/C2/SOV5_C2_45deg_S2D.json \
    --S3_Upload true

python $DEEP_RL_PATH/T4.5_Data_Fix.py \
    --GroupName SOV5_2D_aRot30/45deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot30/C3/SOV5_C3_45deg_S2D.json \
    --S3_Upload true

python $DEEP_RL_PATH/T4.5_Data_Fix.py \
    --GroupName SOV5_2D_aRot30/45deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot30/D1/SOV5_D1_45deg_S2D.json \
    --S3_Upload true

python $DEEP_RL_PATH/T4.5_Data_Fix.py \
    --GroupName SOV5_2D_aRot30/45deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot30/D2/SOV5_D2_45deg_S2D.json \
    --S3_Upload true

python $DEEP_RL_PATH/T4.5_Data_Fix.py \
    --GroupName SOV5_2D_aRot30/45deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot30/D3/SOV5_D3_45deg_S2D.json \
    --S3_Upload true


## 90 deg plots

python $DEEP_RL_PATH/T4.5_Data_Fix.py \
    --GroupName SOV5_2D_aRot30/90deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot30/A1/SOV5_A1_90deg_S2D.json \
    --S3_Upload true

python $DEEP_RL_PATH/T4.5_Data_Fix.py \
    --GroupName SOV5_2D_aRot30/90deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot30/A2/SOV5_A2_90deg_S2D.json \
    --S3_Upload true

python $DEEP_RL_PATH/T4.5_Data_Fix.py \
    --GroupName SOV5_2D_aRot30/90deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot30/B1/SOV5_B1_90deg_S2D.json \
    --S3_Upload true

python $DEEP_RL_PATH/T4.5_Data_Fix.py \
    --GroupName SOV5_2D_aRot30/90deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot30/B2/SOV5_B2_90deg_S2D.json \
    --S3_Upload true

python $DEEP_RL_PATH/T4.5_Data_Fix.py \
    --GroupName SOV5_2D_aRot30/90deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot30/C1/SOV5_C1_90deg_S2D.json \
    --S3_Upload true

python $DEEP_RL_PATH/T4.5_Data_Fix.py \
    --GroupName SOV5_2D_aRot30/90deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot30/C2/SOV5_C2_90deg_S2D.json \
    --S3_Upload true

python $DEEP_RL_PATH/T4.5_Data_Fix.py \
    --GroupName SOV5_2D_aRot30/90deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot30/C3/SOV5_C3_90deg_S2D.json \
    --S3_Upload true

python $DEEP_RL_PATH/T4.5_Data_Fix.py \
    --GroupName SOV5_2D_aRot30/90deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot30/D1/SOV5_D1_90deg_S2D.json \
    --S3_Upload true

python $DEEP_RL_PATH/T4.5_Data_Fix.py \
    --GroupName SOV5_2D_aRot30/90deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot30/D2/SOV5_D2_90deg_S2D.json \
    --S3_Upload true

python $DEEP_RL_PATH/T4.5_Data_Fix.py \
    --GroupName SOV5_2D_aRot30/90deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot30/D3/SOV5_D3_90deg_S2D.json \
    --S3_Upload true


## 135 deg plots

python $DEEP_RL_PATH/T4.5_Data_Fix.py \
    --GroupName SOV5_2D_aRot30/135deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot30/A1/SOV5_A1_135deg_S2D.json \
    --S3_Upload true

python $DEEP_RL_PATH/T4.5_Data_Fix.py \
    --GroupName SOV5_2D_aRot30/135deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot30/A2/SOV5_A2_135deg_S2D.json \
    --S3_Upload true

python $DEEP_RL_PATH/T4.5_Data_Fix.py \
    --GroupName SOV5_2D_aRot30/135deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot30/B1/SOV5_B1_135deg_S2D.json \
    --S3_Upload true

python $DEEP_RL_PATH/T4.5_Data_Fix.py \
    --GroupName SOV5_2D_aRot30/135deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot30/B2/SOV5_B2_135deg_S2D.json \
    --S3_Upload true

python $DEEP_RL_PATH/T4.5_Data_Fix.py \
    --GroupName SOV5_2D_aRot30/135deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot30/C1/SOV5_C1_135deg_S2D.json \
    --S3_Upload true

python $DEEP_RL_PATH/T4.5_Data_Fix.py \
    --GroupName SOV5_2D_aRot30/135deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot30/C2/SOV5_C2_135deg_S2D.json \
    --S3_Upload true

python $DEEP_RL_PATH/T4.5_Data_Fix.py \
    --GroupName SOV5_2D_aRot30/135deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot30/C3/SOV5_C3_135deg_S2D.json \
    --S3_Upload true

python $DEEP_RL_PATH/T4.5_Data_Fix.py \
    --GroupName SOV5_2D_aRot30/135deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot30/D1/SOV5_D1_135deg_S2D.json \
    --S3_Upload true

python $DEEP_RL_PATH/T4.5_Data_Fix.py \
    --GroupName SOV5_2D_aRot30/135deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot30/D2/SOV5_D2_135deg_S2D.json \
    --S3_Upload true

python $DEEP_RL_PATH/T4.5_Data_Fix.py \
    --GroupName SOV5_2D_aRot30/135deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot30/D3/SOV5_D3_135deg_S2D.json \
    --S3_Upload true


## 180 deg plots

python $DEEP_RL_PATH/T4.5_Data_Fix.py \
    --GroupName SOV5_2D_aRot30/180deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot30/A1/SOV5_A1_180deg_S2D.json \
    --S3_Upload true

python $DEEP_RL_PATH/T4.5_Data_Fix.py \
    --GroupName SOV5_2D_aRot30/180deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot30/A2/SOV5_A2_180deg_S2D.json \
    --S3_Upload true

python $DEEP_RL_PATH/T4.5_Data_Fix.py \
    --GroupName SOV5_2D_aRot30/180deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot30/B1/SOV5_B1_180deg_S2D.json \
    --S3_Upload true

python $DEEP_RL_PATH/T4.5_Data_Fix.py \
    --GroupName SOV5_2D_aRot30/180deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot30/B2/SOV5_B2_180deg_S2D.json \
    --S3_Upload true

python $DEEP_RL_PATH/T4.5_Data_Fix.py \
    --GroupName SOV5_2D_aRot30/180deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot30/C1/SOV5_C1_180deg_S2D.json \
    --S3_Upload true

python $DEEP_RL_PATH/T4.5_Data_Fix.py \
    --GroupName SOV5_2D_aRot30/180deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot30/C2/SOV5_C2_180deg_S2D.json \
    --S3_Upload true

python $DEEP_RL_PATH/T4.5_Data_Fix.py \
    --GroupName SOV5_2D_aRot30/180deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot30/C3/SOV5_C3_180deg_S2D.json \
    --S3_Upload true

python $DEEP_RL_PATH/T4.5_Data_Fix.py \
    --GroupName SOV5_2D_aRot30/180deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot30/D1/SOV5_D1_180deg_S2D.json \
    --S3_Upload true

python $DEEP_RL_PATH/T4.5_Data_Fix.py \
    --GroupName SOV5_2D_aRot30/180deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot30/D2/SOV5_D2_180deg_S2D.json \
    --S3_Upload true

python $DEEP_RL_PATH/T4.5_Data_Fix.py \
    --GroupName SOV5_2D_aRot30/180deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot30/D3/SOV5_D3_180deg_S2D.json \
    --S3_Upload true