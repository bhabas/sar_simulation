import os
import sys
import shutil


tmp_dir = "/tmp/Gazebo_Recording/NL_2.5_30_500FPS"



# CONVERT IMAGE FILES BACK INTO VIDEO
# fps = int(input("Input fps as int: "))
fps = 30

# video_file = input("Video FileName: ")
video_file = f"Video_Recording.mp4"
video_path = os.path.join(tmp_dir,video_file)

ffmpeg_cmd = f'ffmpeg -r {fps} -y -i "{tmp_dir}/image_%04d.png" -vcodec libx264 -pix_fmt yuv420p {video_path}'
os.system(ffmpeg_cmd)