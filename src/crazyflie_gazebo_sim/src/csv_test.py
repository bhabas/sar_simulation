import numpy as np
import pandas as pd

df = pd.read_csv("~/catkin_ws/src/crazyflie_simulation/src/crazyflie_gazebo_sim/src/Test_list.csv")

arr = df.to_numpy()

for vz_d in arr[:,0]:
    for vx_d in arr[:,1]:
        print(f"{vz_d} | {vx_d}")