# python $DEEP_RL_PATH/T2_Policy_FineTuning.py \
#     --GroupName IM_2D_aRot90/135deg \
#     --TrainConfig $DEEP_RL_PATH/Config_Files/IM_2D_Sim/aRot90/A2/IM_A2_135deg_S2D.json \
#     --S3_Upload true \
#     --PT_GroupName SOV5_2D_aRot90/135deg \
#     --PT_TrainConfig $DEEP_RL_PATH/Config_Files/SOV5_2D_Sim/aRot90/A2/SOV5_A2_135deg_S2D.json \
#     --t_step_load 150000


python $DEEP_RL_PATH/T3_Policy_Data_Collection.py \
    --GroupName IM_2D_aRot90/135deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/IM_2D_Sim/aRot90/A2/IM_A2_135deg_S2D.json \
    --S3_Upload true \
    --t_step_load 150000