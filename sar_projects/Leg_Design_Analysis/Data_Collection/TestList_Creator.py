import numpy as np
import os
import rospkg

BASE_PATH = os.path.dirname(rospkg.RosPack().get_path('crazyflie_projects'))

test_list = []

for V_d in np.linspace(1.5,3.5,21):    # [m/s]
    for phi in np.linspace(30,90,17):      # [deg]
            test_list.append([V_d,phi])
test_list = np.array(test_list)
np.set_printoptions(suppress=True)
np.savetxt(f"{BASE_PATH}/crazyflie_projects/DeepRL/Data_Collection/MasterTestList.csv", 
            np.asarray(test_list), 
            delimiter=",",
            fmt="%.2f",
            header='V_d,phi')
        


