import numpy as np

test_list = []
for V_d in np.arange(0.25,2.0,0.25):    # [m/s]
    for phi in np.concatenate((np.arange(90,40,-10),np.arange(45,15,-2.5))):      # [deg]
        for trial_num in np.arange(10,12,1):
            test_list.append([V_d,phi,trial_num])

test_list = np.array(test_list)
# test_list = np.flip(test_list,axis=0)


np.set_printoptions(suppress=True)
np.savetxt("crazyflie_data/data_collection/MasterTestList.csv", np.asarray(test_list), delimiter=",",fmt="%.2f",header='vz_d,vx_d,trial_num')
        


