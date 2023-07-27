import os
import sys
import shutil

dir_path = "/home/bhabas/Shared_Folder/T_RO_2023/"
video_name = "2_Leg_4_Side"

video_path = os.path.join(dir_path,video_name,video_name+".mp4")
## CONVERT VIDEO TO IMAGE FILES
os.system(f'ffmpeg -i {video_path} "{dir_path}/{video_name}_%04d.png"')