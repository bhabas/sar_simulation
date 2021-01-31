import numpy as np

list = []
for vz_d in np.arange(3.0,1.25,-10):
    for vx_d in np.arange(1.5,3.0,10):
        for trial_num in np.arange(0,40,1):
            list.append([vz_d,vx_d,trial_num])


print(np.asarray(list))
np.savetxt("foo.csv", np.asarray(list), delimiter=",")
        


