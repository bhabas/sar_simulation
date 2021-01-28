import os,fnmatch
import send2trash
import numpy as np
os.system("clear")

## CREATE LINK TO DATAPATH MODULE
import sys
sys.path.insert(0,'/home/bhabas/catkin_ws/src/crazyflie_simulation/src/crazyflie_rl/src/utility')
from data_analysis import DataFile


dataPath = '/home/bhabas/catkin_ws/src/crazyflie_simulation/local_files/data/Wide-Short_Data_1-27-21/'

redoList = []
## ITERATE THROUGH ALL COMBINATIONS
for vz in np.arange(4.0,1.25,-0.25):
    for vx in np.arange(0.0,3.0,0.25):
        
        ## CREATE LIST OF FILES THAT MATCH IC
        fileList = []
        for fileName in os.listdir(dataPath):
            if fnmatch.fnmatch(fileName, f'*Vz_{vz:.2f}--Vx_{vx:.2f}*'):
                fileList.append(fileName)

        ## SORT FILE LIST TO ASCENDING TRIAL NUMBER ORDER
        fileList = sorted(fileList)

        ## OUTPUT PLOT FOR EACH FILE
        for fileName in fileList:
            
            trial = DataFile(dataPath,fileName)
            print(fileName)
            try:
                trial.plotSummary()
            except: ## IF EEROR DELETE FILE
                print(f"Delete File: {fileName}")
                # send2trash.send2trash(filepath)
        

        ## INPUT TRIAL NUMBERS TO BE DISCARDED
        str = input('Input files to be deleted: ')
        try:
            delList = [int(i) for i in str.split(' ')]
            for trial in delList:
                ## APPEND IC TO LIST TO BE REDONE
                redoList.append([vz,vx,trial])

                ## RECREATE FILE NAME AND DELETE FILE
                filepath = f"{dataPath}EM_PEPG--Vz_{vz:.2f}--Vx_{vx:.2f}--trial_{trial}.csv"
                # send2trash.send2trash(filepath)
        except:
            pass

## PRINT REDO ARRAY AND SAVE TO CSV FILE
print(np.asarray(redoList))
np.savetxt("runAgain_List.csv", np.asarray(redoList), delimiter=",")
    





    
    







# vz = 1.75

# vx = 0
# trialNum = 4
# agent = "EM_PEPG"

# filepath = f"{dataPath}/{agent}--Vz_{vz:.2f}--Vx_{vx:.2f}--trial_{trialNum}.csv"



# trial = DataFile(filepath)



# trial.plot_rewardData()



