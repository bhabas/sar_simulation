#!/usr/bin/env python3
"""
extract_frames.py

A script to extract every n-th frame from a video using FFmpeg and save them as images.

Usage:
    python extract_frames.py --input path/to/video.mp4 --output_dir path/to/output_frames --interval 10 --image_format jpg
"""

import os
import subprocess
import argparse
import sys

def check_ffmpeg_installed():
    """Check if FFmpeg is installed and accessible."""
    try:
        subprocess.run(['ffmpeg', '-version'], stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=True)
    except subprocess.CalledProcessError:
        print("FFmpeg is not installed or not found in PATH. Please install FFmpeg and try again.", file=sys.stderr)
        sys.exit(1)
    except FileNotFoundError:
        print("FFmpeg is not installed or not found in PATH. Please install FFmpeg and try again.", file=sys.stderr)
        sys.exit(1)

def parse_arguments():
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(description="Extract every n-th frame from a video using FFmpeg.")
    parser.add_argument('--input', '-i', required=True, help="Path to the input video file.")
    parser.add_argument('--output_dir', '-o', required=True, help="Directory to save the extracted frames.")
    parser.add_argument('--interval', '-n', type=int, required=True, help="Extract every n-th frame.")
    parser.add_argument('--image_format', '-f', default='png', choices=['png', 'jpg', 'jpeg', 'bmp', 'tiff'],
                        help="Image format for the saved frames. Default is png.")
    return parser.parse_args()

def construct_ffmpeg_command(input_video, output_dir, interval, image_format):
    """
    Construct the FFmpeg command to extract every n-th frame.

    FFmpeg Command:
    ffmpeg -i input_video -vf "select=not(mod(n\,interval))" -vsync vfr output_dir/frame_%04d.png
    """
    # Ensure the output directory exists
    os.makedirs(output_dir, exist_ok=True)

    # Define the output filename pattern
    output_pattern = os.path.join(output_dir, f"frame_%04d.{image_format}")

    # Construct the FFmpeg filter
    # 'select=not(mod(n\,interval))' selects every n-th frame
    # '-vsync vfr' ensures variable frame rate to avoid duplication
    vf_filter = f"select=not(mod(n\\,{interval}))"

    command = [
        'ffmpeg',
        '-i', input_video,
        '-vf', vf_filter,
        '-vsync', 'vfr',
        '-q:v', '2',  # Quality parameter for JPEG. Lower is better. Ignored for PNG.
        output_pattern
    ]

    return command

def extract_frames_ffmpeg(input_video, output_dir, interval, image_format):
    """Use FFmpeg to extract every n-th frame from the video."""
    command = construct_ffmpeg_command(input_video, output_dir, interval, image_format)
    print("Executing FFmpeg command:")
    print(' '.join(command))

    try:
        # Execute the FFmpeg command
        subprocess.run(command, check=True)
        print(f"Frames have been successfully extracted to '{output_dir}'.")
    except subprocess.CalledProcessError as e:
        print(f"An error occurred while extracting frames: {e}", file=sys.stderr)
        sys.exit(1)

def main():
    # Check if FFmpeg is installed
    check_ffmpeg_installed()

    # Parse command-line arguments
    args = parse_arguments()

    # Validate input video file
    if not os.path.isfile(args.input):
        print(f"Input video file '{args.input}' does not exist.", file=sys.stderr)
        sys.exit(1)

    # Validate interval
    if args.interval <= 0:
        print("Interval must be a positive integer.", file=sys.stderr)
        sys.exit(1)

    # Extract frames using FFmpeg
    extract_frames_ffmpeg(args.input, args.output_dir, args.interval, args.image_format)

if __name__ == "__main__":
    main()
