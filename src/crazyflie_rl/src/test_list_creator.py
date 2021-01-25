import numpy as np

list = []
for vz_d in np.arange(4.0,1.25,-0.25):
    for vx_d in np.arange(0.0,3.0,0.25):
        for trial_num in np.arange(0,6,1):
            list.append([vz_d,vx_d,trial_num])



np.savetxt("foo.csv", np.asarray(list), delimiter=",")
        


