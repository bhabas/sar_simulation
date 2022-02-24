import os
import sys


# dir = input("Input directory as string: ")
dir = "/tmp/camera_save_tutorial"
fps = int(input("Input fps as int: "))

os.system(f'ffmpeg -r {fps} -i "{dir}/default_camera_link_High-Speed_Camera(1)-%04d.jpg" Output.mp4')