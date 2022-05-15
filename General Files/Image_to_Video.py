import os
import sys


# dir = input("Input directory as string: ")
dir = "/tmp/images/"
fps = int(input("Input fps as int: "))

os.system(f'ffmpeg -r {fps} -i "{dir}/Image_%04d.png" Output.mp4')