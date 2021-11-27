import os,time,datetime
import pandas as pd
import warnings
import numpy as np
import csv
import rospkg


## CREATE LINK TO DATAPATH MODULE
import sys
# os.system("clear")



## ADD CRAZYFLIE_SIMULATION DIRECTORY TO PYTHONPATH SO ABSOLUTE IMPORTS CAN BE USED
BASE_PATH = os.path.dirname(rospkg.RosPack().get_path('crazyflie_data'))
sys.path.insert(0,BASE_PATH)
# print(sys.path)


from crazyflie_data.data_analysis.Data_Analysis import DataFile

print("Testing")
model_name = input("Model_Name: ")
dataPath = f"/home/bhabas/catkin_ws/src/crazyflie_simulation/crazyflie_data/local_logs/{model_name}/"
df_list = []
num_files = len(os.listdir(dataPath))

run_avg = 20
start_time = time.time()
end_time = time.time()
alpha = 0.15

## ITER OVER ALL FILES IN DIR
for ii,fileName in enumerate(os.listdir(dataPath)): # Iter over all files in dir

    try:
        ## PROGRESS PRINTING (BASIC STUFF STARTING FOR TIME ESTIMATION)
        

        diff = end_time - start_time

        run_avg = alpha*diff + (1-alpha)*(run_avg)
        start_time = time.time()

    except:
        # send2trash.send2trash(dataPath+fileName)
        end_time = time.time()
        print(f"Trashing file {fileName}")
        # pass

