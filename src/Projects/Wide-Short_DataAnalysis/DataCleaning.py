import os,fnmatch
import send2trash
import numpy as np
os.system("clear")

## CREATE LINK TO DATAPATH MODULE
import sys
sys.path.insert(0,'/home/bhabas/catkin_ws/src/crazyflie_simulation/src/crazyflie_rl/src/utility')
from data_analysis import DataFile


dataPath = '/home/bhabas/catkin_ws/src/crazyflie_simulation/src/crazyflie_rl/src/log/Policy_Derivation_Data_Vx-0.0/'


redoList = []
fileList = []

## DEFINE DATA RANGE TO ITERATE OVER
vz = np.arange(3.0,2,0.25)
vx = np.arange(0,1.5,0.25)
trials = np.arange(0,40,1)

vz = 3.0
vx = 0.0

test_arr = np.array(np.meshgrid(vx, trials, vz)).T.reshape(-1,3)

## ITERATE THROUGH ALL COMBINATIONS
for vx,trials,vz in test_arr:
    
    ## CREATE LIST OF FILES THAT MATCH IC Vz,Vx
    
    for fileName in os.listdir(dataPath):
        if fnmatch.fnmatch(fileName, f'*Vz_{vz:.2f}--Vx_{vx:.2f}*'):
            fileList.append(fileName)

## SORT FILE LIST TO ASCENDING TRIAL NUMBER ORDER
fileList = sorted(fileList)

## ITERATE THROUGH FILE LIST
for fileName in fileList:
    filepath = dataPath+fileName
    
    ## CREATE TRIAL OBJECT FOR CURRENT FILE
    trial = DataFile(dataPath,fileName)
    
    try:
        print(trial.rewardAvg_trial())
        trial.plotSummary()
        ## IF BROKEN RUN THEN JUST REMOVE
        # if trial.rewardAvg_trial() <= 30:
        #     raise Exception
        # if trial.k_epMax <= 17:
        #     raise  Exception
        
        # SHOW FILENAME AND SUMMARY
        print(fileName)
        
    except: ## IF ERROR DELETE FILE
        print(f"Deleting File: {fileName}")
        redoList.append([fileName])

        # send2trash.send2trash(filepath)


## INPUT TRIAL NUMBERS TO BE DISCARDED
str = input('Input spaced trial numbers to be deleted: ')
try:
    trialList = [int(i) for i in str.split(' ')]
    for trial in trialList:
        ## APPEND TRIAL TO LIST TO BE REDONE AND SAVE CURRENT ARRRAY TO CSV
        np.savetxt("runAgain_List.csv", np.asarray(redoList), delimiter=",")

        ## RECREATE FILE NAME AND DELETE FILE
        filepath = f"{dataPath}EM_PEPG--Vz_{vz:.2f}--Vx_{vx:.2f}--trial_{trial}.csv"
        # send2trash.send2trash(filepath)
except:
    pass


# ## PRINT REDO ARRAY AND SAVE TO CSV FILE
# print(np.asarray(redoList))
# np.savetxt("runAgain_List.csv", np.asarray(redoList), delimiter=",")
    


