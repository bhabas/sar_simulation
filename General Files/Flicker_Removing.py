import os
import sys


# dir = input("Input directory as string: ")
dir = "/home/bhabas/Documents/"
video_file = "4_Leg_2_Side.mp4"
tmp_dir = os.path.join(dir,"tmp")


if not os.path.exists(tmp_dir):
    os.makedirs(tmp_dir)

# for ii,file in enumerate(sorted(os.listdir(dir))):
#     print(file,ii)
#     old_file = os.path.join(dir,file)
#     new_file = os.path.join(dir,f"Image_{ii:04d}.png")
#     os.rename(old_file,new_file)
    




# dir = "/home/bhabas/Documents/Images"
# fps = int(input("Input fps as int: "))

# os.system(f'ffmpeg -r {fps} -i "{dir}/%04d.png" Output.mp4')