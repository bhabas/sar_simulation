import os
import sys
import shutil


dir = "/home/bhabas/Shared_Folder/ICRA_Inverted_Landing_Videos/HighSpeed_Videos"
video_file = "False_Trigger.mp4"


tmp_dir = os.path.join(dir,"tmp")
video_path = os.path.join(dir,video_file)

if not os.path.exists(tmp_dir):
    os.makedirs(tmp_dir)

## CONVERT VIDEO TO IMAGE FILES
os.system(f'ffmpeg -i {video_path} "{tmp_dir}/%04d.png"')


## REMOVE IMAGE FILES WITH FLICKER
n = int(input("Enter initial frame to drop: "))
drop_interval = 5
for ii,file in enumerate(sorted(os.listdir(tmp_dir))):
    
    frame_num = int(file[-8:-4])
    if((frame_num-n)%drop_interval == 0):
        deleted_file = os.path.join(tmp_dir,file)
        os.remove(deleted_file)
    

# RENAME IMAGE FILES SEQUENTIALLY
for ii,file in enumerate(sorted(os.listdir(tmp_dir))):
    # print(file,ii)
    old_file = os.path.join(tmp_dir,file)
    new_file = os.path.join(tmp_dir,f"Image_{ii:04d}.png")
    os.rename(old_file,new_file)

# CONVERT IMAGE FILES BACK INTO VIDEO
# fps = int(input("Input fps as int: "))
fps = 30
os.system(f'ffmpeg -r {fps} -y -i "{tmp_dir}/Image_%04d.png" {video_path}')


## DELETE TMP DIRECTORY AND ALL IMAGES
try:
    shutil.rmtree(tmp_dir)
except OSError as e:
    print ("Error: %s - %s." % (e.filename, e.strerror))