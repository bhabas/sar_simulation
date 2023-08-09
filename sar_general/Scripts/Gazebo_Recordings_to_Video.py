import os
import sys
import shutil


tmp_dir = "/tmp/Gazebo_Videos"


# RENAME IMAGE FILES SEQUENTIALLY
for ii,file in enumerate(sorted(os.listdir(tmp_dir))):
    # print(file,ii)
    old_file = os.path.join(tmp_dir,file)
    new_file = os.path.join(tmp_dir,f"Image_{ii:04d}.jpg")
    os.rename(old_file,new_file)

# CONVERT IMAGE FILES BACK INTO VIDEO
# fps = int(input("Input fps as int: "))
fps = 30

# video_file = input("Video FileName: ")
video_file = f"Ep_{6000}.mp4"
video_path = os.path.join(tmp_dir,video_file)

os.system(f'ffmpeg -r {fps} -y -i "{tmp_dir}/Image_%04d.jpg" {video_path}')