import numpy as np

test_list = []
vel_range = np.arange(3.0,0.75,-0.25)
phi_range = np.arange(90,20,-10)
d_ceiling_range = np.arange(0.55,0.05,-0.05)
My_range = np.arange(-3,-10,-1)
trial_num = 0




for vel in vel_range:    # [m/s]
    for phi in phi_range:      # [deg]
        trial_num += 1
        for d_ceiling in d_ceiling_range: # [m]
            for My in My_range:

                test_list.append([vel,phi,d_ceiling,My,trial_num])


test_list = np.array(test_list)
# test_list = np.flip(test_list,axis=0)


np.set_printoptions(suppress=True)
np.savetxt(
    "Projects/Policy_Mapping/PolicyMappingList.csv", 
    np.asarray(test_list), 
    delimiter=",",
    fmt="%.2f",
    header='vel,phi,d_ceiling,My,trial_num')
        


