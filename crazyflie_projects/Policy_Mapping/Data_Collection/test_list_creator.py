import numpy as np
import os
import rospkg

BASE_PATH = os.path.dirname(rospkg.RosPack().get_path('crazyflie_projects'))

test_list = []
Tau_0_list = [0.20,0.25,0.30] # Initial tau values for mu distribution

for V_d in np.arange(1.0,3.75,0.25):    # [m/s]
    for phi in np.arange(90,15,-5):      # [deg]
        for ii,trial_num in enumerate(np.arange(0,3,1)):
            test_list.append([V_d,phi,Tau_0_list[ii],trial_num])

test_list = np.array(test_list)
np.set_printoptions(suppress=True)
np.savetxt(f"{BASE_PATH}/crazyflie_projects/Policy_Mapping/Data_Collection/MasterTestList.csv", 
            np.asarray(test_list), 
            delimiter=",",
            fmt="%.2f",
            header='V_d,phi,Tau_0,trial_num')
        


