import os,fnmatch
import numpy as np

## CREATE LINK TO DATAPATH MODULE
import sys
sys.path.insert(0,'/home/bhabas/catkin_ws/src/crazyflie_simulation/src/crazyflie_rl/src/utility')
from data_analysis import DataFile


os.system("clear")



dataPath = '/home/bhabas/catkin_ws/src/crazyflie_simulation/local_files/data/Wide-Short_Data_1-27-21/'


list = []
for vz in np.arange(4.0,1.25,-0.25):
    for vx in np.arange(0.0,3.0,0.25):
        
        ## CREATE LIST OF FILES THAT MATCH IC
        fileList = []
        for file_name in os.listdir(dataPath):
            if fnmatch.fnmatch(file_name, f'*Vz_{vz:.2f}--Vx_{vx:.2f}*'):
                fileList.append(file_name)

        ## SORT FILE LIST TO ASCENDING TRIAL ORDER
        fileList = sorted(fileList)

        ## OUTPUT PLOT FOR EACH FILE
        for file_name in fileList:
            filepath = dataPath + file_name
            trial = DataFile(filepath)
            print(file_name)
            trial.plot_rewardData(file_name)


        str = input('Input files to be deleted: ')
        try:
            delList = [int(i) for i in str.split(' ')]
            for trial in delList:
                filepath = f"{dataPath}EM_PEPG--Vz_{vz:.2f}--Vx_{vx:.2f}--trial_{trial}.csv"
            # os.remove(filepath)
        except:
            pass
    





    
    







# vz = 1.75

# vx = 0
# trialNum = 4
# agent = "EM_PEPG"

# filepath = f"{dataPath}/{agent}--Vz_{vz:.2f}--Vx_{vx:.2f}--trial_{trialNum}.csv"



# trial = DataFile(filepath)



# trial.plot_rewardData()



