import numpy as np
import pandas as pd


BASE_PATH = "/home/bhabas/catkin_ws/src/sar_simulation/sar_projects/DeepRL/TB_Logs/SAR_2D_DeepRL/DeepRL_Policy_09:02:31"
df_onboard = pd.read_csv(f"{BASE_PATH}/PolicyPerformance_Data_Onboard.csv")
df_offboard = pd.read_csv(f"{BASE_PATH}/PolicyPerformance_Data_Offboard.csv")

print()