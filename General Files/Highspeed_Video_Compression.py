## REDUCE RAW SIZE OF HIGHSPEED VIDEO TO MANAGABLE SIZE

import os
import sys


# dir = input("Input directory as string: ")
dir = "/home/bhabas/Shared_Folder/ICRA_2023_HighSpeed_Videos/C001H001S0001"
for file in os.listdir(dir):
    if file.endswith(".avi"):
        os.system(f'ffmpeg -i {os.path.join(dir,file)} {os.path.join(dir,file[:-4])}_Compr.mp4')
        print()

