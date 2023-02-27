import os
import time





LogDir = "/home/bhabas/catkin_ws/src/crazyflie_simulation/crazyflie_projects/Optical_Flow_Estimation/local_logs/"
# FolderName = "Check_Pattern_Divergent_Flow"
# FileDir = "D_0.5--V_perp_0.0--V_para_1.0--L_0.02"

# FilePath = os.path.join(LogDir,FolderName,FileDir)

L_list = [0.02,0.05,0.12,0.25,0.5,0.75,1.00,2.00]

for L in L_list:

    LogDir = "/home/bhabas/catkin_ws/src/crazyflie_simulation/crazyflie_projects/Optical_Flow_Estimation/local_logs/"
    FolderName = "Check_Pattern_Divergent_Flow"
    FileDir=f"D_0.5--V_perp_0.5--V_para_0.0--L_{L:.2f}"

    FilePath = os.path.join(LogDir,FolderName,FileDir)


    os.system(f'ffmpeg -i {FilePath}/Data_Overview--SSL_0.mp4 -i {FilePath}/Data_Overview--SSL_1.mp4 -filter_complex hstack {FilePath}/Temp_Top.mp4')
    time.sleep(2.0)
    os.system(f'ffmpeg -i {FilePath}/Data_Overview--SSL_2.mp4 -i {FilePath}/Data_Overview--SSL_3.mp4 -filter_complex hstack {FilePath}/Temp_Bottom.mp4')
    time.sleep(2.0)
    os.system(f'ffmpeg -i {FilePath}/Temp_Top.mp4 -i {FilePath}/Temp_Bottom.mp4 -filter_complex vstack {FilePath}/Data_Overview--SSL_grid.mp4')
    os.remove(f"{FilePath}/Temp_Top.mp4")
    os.remove(f"{FilePath}/Temp_Bottom.mp4")

   
    
