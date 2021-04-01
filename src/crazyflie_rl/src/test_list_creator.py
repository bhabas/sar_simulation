import numpy as np

list = []
for V_d in np.arange(1.0,5.25,0.25):    # [m/s]
    for phi in np.arange(30,90,10):      # [deg]
        for trial_num in np.arange(0,4,1):
            list.append([V_d,phi,trial_num])


print(np.asarray(list))
np.savetxt("foo.csv", np.asarray(list), delimiter=",")
        


