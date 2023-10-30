import numpy as np
import os
import rospkg

BASE_PATH = os.path.dirname(rospkg.RosPack().get_path('crazyflie_projects'))

test_list = []
Tau_0_list = [0.18,0.22,0.24] # Initial tau values for mu distribution

for V_d in np.linspace(1.5,3.5,21):    # [m/s]
    for phi in np.linspace(30,90,17):      # [deg]
        for ii,trial_num in enumerate(np.arange(0,len(Tau_0_list),1)):
            test_list.append([V_d,phi,Tau_0_list[ii],trial_num])

test_list = np.array(test_list)
np.set_printoptions(suppress=True)
np.savetxt(f"{BASE_PATH}/crazyflie_projects/ML3_Policy/Data_Collection/MasterTestList.csv", 
            np.asarray(test_list), 
            delimiter=",",
            fmt="%.2f",
            header='V_d,phi,Tau_0,trial_num')
        


