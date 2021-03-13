import numpy as np

list = []
for vz_d in np.arange(1.5,4.25,0.25):
    for vx_d in np.arange(0,2.0,0.25):
        for trial_num in np.arange(7,10,1):
            list.append([vz_d,vx_d,trial_num])


print(np.asarray(list))
np.savetxt("foo.csv", np.asarray(list), delimiter=",")
        


