import numpy as np

List = []
for V_d in np.arange(1.0,4.25,0.5):    # [m/s]
    for phi in np.arange(90,15,-7.5):      # [deg]
        for trial_num in np.arange(3,8,1):
            List.append([V_d,phi,trial_num])


print(np.asarray(List))
np.set_printoptions(suppress=True)
np.savetxt("crazyflie_data/data_collection/MasterTestList.csv", np.asarray(List), delimiter=",",fmt="%.1f",header='vz_d,vx_d,trial_num')
        


