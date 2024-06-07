#!/bin/bash

## FIND SIMULATION PATH
DEEP_RL_PATH=$(find ~/catkin_ws/src -name 'sar_simulation' -type d | head -n 1)/sar_projects/DeepRL

## 0 deg plots

python $DEEP_RL_PATH/T4_Policy_Data_Plotting.py \
    --GroupName SOV5_3D_aRot90/0deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_3D_Sim/aRot90/A1/SOV5_A1_0deg_S2D.json \
    --S3_Upload true
    
python $DEEP_RL_PATH/T4_Policy_Data_Plotting.py \
    --GroupName SOV5_3D_aRot90/0deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_3D_Sim/aRot90/B2/SOV5_B2_0deg_S2D.json \
    --S3_Upload true

python $DEEP_RL_PATH/T4_Policy_Data_Plotting.py \
    --GroupName SOV5_3D_aRot90/0deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_3D_Sim/aRot90/C2/SOV5_C2_0deg_S2D.json \
    --S3_Upload true

python $DEEP_RL_PATH/T4_Policy_Data_Plotting.py \
    --GroupName SOV5_3D_aRot90/0deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_3D_Sim/aRot90/D1/SOV5_D1_0deg_S2D.json \
    --S3_Upload true

python $DEEP_RL_PATH/T4_Policy_Data_Plotting.py \
    --GroupName SOV5_3D_aRot90/0deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_3D_Sim/aRot90/D3/SOV5_D3_0deg_S2D.json \
    --S3_Upload true




## 45 deg plots

python $DEEP_RL_PATH/T4_Policy_Data_Plotting.py \
    --GroupName SOV5_3D_aRot90/45deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_3D_Sim/aRot90/A1/SOV5_A1_45deg_S2D.json \
    --S3_Upload true

python $DEEP_RL_PATH/T4_Policy_Data_Plotting.py \
    --GroupName SOV5_3D_aRot90/45deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_3D_Sim/aRot90/B2/SOV5_B2_45deg_S2D.json \
    --S3_Upload true

python $DEEP_RL_PATH/T4_Policy_Data_Plotting.py \
    --GroupName SOV5_3D_aRot90/45deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_3D_Sim/aRot90/C2/SOV5_C2_45deg_S2D.json \
    --S3_Upload true

python $DEEP_RL_PATH/T4_Policy_Data_Plotting.py \
    --GroupName SOV5_3D_aRot90/45deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_3D_Sim/aRot90/D1/SOV5_D1_45deg_S2D.json \
    --S3_Upload true

python $DEEP_RL_PATH/T4_Policy_Data_Plotting.py \
    --GroupName SOV5_3D_aRot90/45deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_3D_Sim/aRot90/D3/SOV5_D3_45deg_S2D.json \
    --S3_Upload true


## 90 deg plots

python $DEEP_RL_PATH/T4_Policy_Data_Plotting.py \
    --GroupName SOV5_3D_aRot90/90deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_3D_Sim/aRot90/A1/SOV5_A1_90deg_S2D.json \
    --S3_Upload true

python $DEEP_RL_PATH/T4_Policy_Data_Plotting.py \
    --GroupName SOV5_3D_aRot90/90deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_3D_Sim/aRot90/B2/SOV5_B2_90deg_S2D.json \
    --S3_Upload true

python $DEEP_RL_PATH/T4_Policy_Data_Plotting.py \
    --GroupName SOV5_3D_aRot90/90deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_3D_Sim/aRot90/C2/SOV5_C2_90deg_S2D.json \
    --S3_Upload true

python $DEEP_RL_PATH/T4_Policy_Data_Plotting.py \
    --GroupName SOV5_3D_aRot90/90deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_3D_Sim/aRot90/D1/SOV5_D1_90deg_S2D.json \
    --S3_Upload true

python $DEEP_RL_PATH/T4_Policy_Data_Plotting.py \
    --GroupName SOV5_3D_aRot90/90deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_3D_Sim/aRot90/D3/SOV5_D3_90deg_S2D.json \
    --S3_Upload true


## 135 deg plots

python $DEEP_RL_PATH/T4_Policy_Data_Plotting.py \
    --GroupName SOV5_3D_aRot90/135deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_3D_Sim/aRot90/A1/SOV5_A1_135deg_S2D.json \
    --S3_Upload true

python $DEEP_RL_PATH/T4_Policy_Data_Plotting.py \
    --GroupName SOV5_3D_aRot90/135deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_3D_Sim/aRot90/B2/SOV5_B2_135deg_S2D.json \
    --S3_Upload true

python $DEEP_RL_PATH/T4_Policy_Data_Plotting.py \
    --GroupName SOV5_3D_aRot90/135deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_3D_Sim/aRot90/C2/SOV5_C2_135deg_S2D.json \
    --S3_Upload true

python $DEEP_RL_PATH/T4_Policy_Data_Plotting.py \
    --GroupName SOV5_3D_aRot90/135deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_3D_Sim/aRot90/D1/SOV5_D1_135deg_S2D.json \
    --S3_Upload true

python $DEEP_RL_PATH/T4_Policy_Data_Plotting.py \
    --GroupName SOV5_3D_aRot90/135deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_3D_Sim/aRot90/D3/SOV5_D3_135deg_S2D.json \
    --S3_Upload true


## 180 deg plots

python $DEEP_RL_PATH/T4_Policy_Data_Plotting.py \
    --GroupName SOV5_3D_aRot90/180deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_3D_Sim/aRot90/A1/SOV5_A1_180deg_S2D.json \
    --S3_Upload true

python $DEEP_RL_PATH/T4_Policy_Data_Plotting.py \
    --GroupName SOV5_3D_aRot90/180deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_3D_Sim/aRot90/B2/SOV5_B2_180deg_S2D.json \
    --S3_Upload true

python $DEEP_RL_PATH/T4_Policy_Data_Plotting.py \
    --GroupName SOV5_3D_aRot90/180deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_3D_Sim/aRot90/C2/SOV5_C2_180deg_S2D.json \
    --S3_Upload true

python $DEEP_RL_PATH/T4_Policy_Data_Plotting.py \
    --GroupName SOV5_3D_aRot90/180deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_3D_Sim/aRot90/D1/SOV5_D1_180deg_S2D.json \
    --S3_Upload true

python $DEEP_RL_PATH/T4_Policy_Data_Plotting.py \
    --GroupName SOV5_3D_aRot90/180deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_3D_Sim/aRot90/D3/SOV5_D3_180deg_S2D.json \
    --S3_Upload true