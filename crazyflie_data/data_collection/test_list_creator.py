import numpy as np

list = []
for V_d in np.arange(1.0,4.0,0.25):    # [m/s]
    for phi in np.arange(90,30,-15):      # [deg]
        for trial_num in np.arange(0,3,1):
            list.append([V_d,phi,trial_num])


print(np.asarray(list))
np.set_printoptions(suppress=True)
np.savetxt("crazyflie_data/data_collection/MasterTestList.csv", np.asarray(list), delimiter=",",fmt="%.1f",header='vz_d,vx_d,trial_num')
        


