import numpy as np

test_list = []
for V_d in np.arange(1.0,4.25,0.5):    # [m/s]
    for phi in np.arange(90,15,-7.5):      # [deg]
        for trial_num in np.arange(3,8,1):
            test_list.append([V_d,phi,trial_num])

test_list = np.array(test_list)
test_list = np.flip(test_list,axis=0)


np.set_printoptions(suppress=True)
np.savetxt("crazyflie_data/data_collection/MasterTestList.csv", np.asarray(test_list), delimiter=",",fmt="%.1f",header='vz_d,vx_d,trial_num')
        


