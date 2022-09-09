import os
import sys


# dir = input("Input directory as string: ")
dir = "/tmp/images/"
dir = "/home/bhabas/Documents/Images"
# fps = int(input("Input fps as int: "))
fps = 30

os.system(f'ffmpeg -r {fps} -i "{dir}/Image_%4d.png" Output.mp4')