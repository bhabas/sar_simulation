import os
import time


LogDir = "/home/bhabas/catkin_ws/src/crazyflie_simulation/crazyflie_projects/Optical_Flow_Estimation/local_logs/"
FolderName = "Check_Pattern_Translation_Flow"
FileName = "D_0.5--V_perp_0.0--V_para_1.0--L_2.0"

FileDir = os.path.join(LogDir,FolderName,FileName)


os.system(f'ffmpeg -i {FileDir}/Data_Overview--SSL_0.mp4 -i {FileDir}/Data_Overview--SSL_1.mp4 -filter_complex hstack {FileDir}/Temp_Top.mp4')
time.sleep(2.0)
os.system(f'ffmpeg -i {FileDir}/Data_Overview--SSL_2.mp4 -i {FileDir}/Data_Overview--SSL_3.mp4 -filter_complex hstack {FileDir}/Temp_Bottom.mp4')
time.sleep(2.0)
os.system(f'ffmpeg -i {FileDir}/Temp_Top.mp4 -i {FileDir}/Temp_Bottom.mp4 -filter_complex vstack {FileDir}/Data_Overview--SSL_grid.mp4')
os.remove(f"{FileDir}/Temp_Top.mp4")
os.remove(f"{FileDir}/Temp_Bottom.mp4")

   
    
