import os
import subprocess

def sort_files(file):
    """
    Custom sort function to handle file names numerically.
    Assumes file names contain numbers that can be used for sorting.
    """
    num_part = ''.join(filter(str.isdigit, file))
    return int(num_part) if num_part.isdigit() else float('inf')

def rename_files(directory, files):
    """
    Rename files to a standardized format like 'image_0000.png'.
    """
    for i, file in enumerate(files):
        new_name = f"image_{i:04d}.png"
        os.rename(os.path.join(directory, file), os.path.join(directory, new_name))

def convert_images_to_video(directory, fps, output_file):
    # Get all PNG files in the directory
    files = [f for f in os.listdir(directory) if f.endswith('.png')]
    files.sort(key=sort_files)

    # Rename files
    rename_files(directory, files)

    # Update file list after renaming
    files = [f for f in os.listdir(directory) if f.endswith('.png')]
    files.sort(key=sort_files)

    # Prepare the FFmpeg command
    ffmpeg_command = [
        'ffmpeg',
        '-r', str(fps),
        '-i', os.path.join(directory, 'image_%04d.png'),
        '-c:v', 'prores_ks',  # Using ProRes codec
        '-profile:v', '2',    # ProRes 422 HQ; use '2' for ProRes 422
        '-pix_fmt', 'yuv422p10le',  # Pixel format for ProRes
        os.path.join(directory,output_file)
    ]

    # Run the FFmpeg command
    subprocess.run(ffmpeg_command)

# Usage
directory = '/tmp/Gazebo_Recording'  # Replace with your directory path
fps = 30  # Replace with your desired frames per second
output_file = 'Targeted_Landing_2.mov'  # Replace with your desired output file name
convert_images_to_video(directory, fps, output_file)
