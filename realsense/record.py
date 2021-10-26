# Open3D: www.open3d.org
# The MIT License (MIT)
# See license file or visit www.open3d.org for details

# examples/python/reconstruction_system/sensors/realsense_recorder.py

# pyrealsense2 is required.
# Please see instructions in https://github.com/IntelRealSense/librealsense/tree/master/wrappers/python
import pyrealsense2 as rs
import numpy as np
import cv2
import argparse
from os import makedirs
from os.path import exists, join
import shutil
import json
from enum import IntEnum
import inquirer


class Preset(IntEnum):
    Custom = 0
    Default = 1
    Hand = 2
    HighAccuracy = 3
    HighDensity = 4
    MediumDensity = 5


def make_clean_folder(path_folder):
    if not exists(path_folder):
        makedirs(path_folder)
    else:
        user_input = input("%s not empty. Overwrite? (y/n) : " % path_folder)
        if user_input.lower() == 'y':
            shutil.rmtree(path_folder)
            makedirs(path_folder)
        else:
            exit()


def save_intrinsic_as_json(filename, frame, depth_scale, fps):
    intrinsics = frame.profile.as_video_stream_profile().intrinsics
    with open(filename, 'w') as outfile:
        json.dump(
            {
                'width':
                    intrinsics.width,
                'height':
                    intrinsics.height,
                'intrinsic_matrix': [
                    intrinsics.fx, 0, 0, 0, intrinsics.fy, 0, intrinsics.ppx,
                    intrinsics.ppy, 1
                ],
                'depth_scale': 1/depth_scale,
                'fps': fps,
                'camera_model': "pinhole",
                'color_format' : "BGR8",
	            'depth_format' : "Z16"
            },
            outfile,
            indent=4)


if __name__ == "__main__":

    parser = argparse.ArgumentParser(
        description=
        "Realsense Recorder. Please select one of the optional arguments")
    parser.add_argument("--out",
                        default='./collected_dataset/',
                        help="set output folder")

    args = parser.parse_args()

    path_output = args.out
    path_depth = join(args.out, "depth")
    path_color = join(args.out, "color")
    make_clean_folder(path_output)
    make_clean_folder(path_depth)
    make_clean_folder(path_color)
    pipeline = rs.pipeline()
    config = rs.config()
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()

    #Loop through available sensors
    for sensor in device.sensors:
        if sensor.is_color_sensor():
            sensor_type = "color"
        elif sensor.is_depth_sensor():
            sensor_type = "depth"
        else:
            print("Unknown sensor type, skipping.")
            continue

        options = []

        #Loop through stream profiles
        for profile in sensor.get_stream_profiles():
            profile = profile.as_video_stream_profile()
            #Pick streams with fps at least 30 and proper formats
            if profile.fps() >= 30:
                if (sensor_type == "color" and int(profile.format()) == int(rs.format.bgr8)) or (sensor_type == "depth" and int(profile.format() == int(rs.format.z16))):
                    option = dict(width=profile.width(), height=profile.height(), fps=profile.fps(), format=profile.format())
                    options.append(option)
        
        #Prompt selection of available resolutions/fps in the shell
        prompt = [
        inquirer.List('value',
                        message=f"Choose {sensor_type} sensor stream type",
                        choices=options
                    ),
        ]
        result = inquirer.prompt(prompt)
        width = result["value"]["width"]
        height = result["value"]["height"]
        format = result["value"]["format"]
        fps = result["value"]["fps"]
        stream_type = profile.stream_type()
        config.enable_stream(stream_type, width, height, format, fps)

    # Start streaming
    profile = pipeline.start(config)
    depth_sensor = profile.get_device().first_depth_sensor()
    color_sensor = profile.get_device().first_color_sensor()

    #Set sensor options, all available here: https://intelrealsense.github.io/librealsense/python_docs/_generated/pyrealsense2.option.html

    #Set depth sensor options, recommendations: https://dev.intelrealsense.com/docs/d400-series-visual-presets
    depth_sensor.set_option(rs.option.visual_preset, Preset.HighAccuracy)  # Using preset HighAccuracy for recording

    #Set color sensor options
    color_sensor.set_option(rs.option.enable_auto_exposure, 0.0) #Disable auto exposure
    color_sensor.set_option(rs.option.enable_auto_white_balance, 0.0) #Disable auto white balance  
    color_sensor.set_option(rs.option.white_balance, 3700.0) #White balance, adjust as needed   
    color_sensor.set_option(rs.option.gain, 64.0) #Gain, adjust as needed  
    color_sensor.set_option(rs.option.exposure, 953.0) #Exposure, adjust as needed 

    # Getting the depth sensor's depth scale (see rs-align example for explanation)
    depth_scale = depth_sensor.get_depth_scale()

    # We will not display the background of objects more than
    #  clipping_distance_in_meters meters away
    clipping_distance_in_meters = 3  # 3 meter
    clipping_distance = clipping_distance_in_meters / depth_scale

    # Create an align object
    # rs.align allows us to perform alignment of depth frames to others frames
    # The "align_to" is the stream type to which we plan to align depth frames.
    align_to = rs.stream.color
    align = rs.align(align_to)

    # Streaming loop
    frame_count = 0
    try:
        while True:
            # Get frameset of color and depth
            frames = pipeline.wait_for_frames()

            # Align the depth frame to color frame
            aligned_frames = align.process(frames)

            # Get aligned frames
            aligned_depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

            # Validate that both frames are valid
            if not aligned_depth_frame or not color_frame:
                continue

            depth_image = np.asanyarray(aligned_depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            if frame_count == 0:
                save_intrinsic_as_json(
                    join(args.out, "camera_intrinsics.json"),
                    color_frame, depth_scale, fps)
            cv2.imwrite("%s/%06d.png" % \
                    (path_depth, frame_count), depth_image)
            cv2.imwrite("%s/%06d.jpg" % \
                    (path_color, frame_count), color_image)
            print("Saved color + depth image %06d" % frame_count)
            frame_count += 1

            # Remove background - Set pixels further than clipping_distance to grey
            grey_color = 153
            #depth image is 1 channel, color is 3 channels
            depth_image_3d = np.dstack((depth_image, depth_image, depth_image))
            bg_removed = np.where((depth_image_3d > clipping_distance) | \
                    (depth_image_3d <= 0), grey_color, color_image)

            # Render images
            depth_colormap = cv2.applyColorMap(
                cv2.convertScaleAbs(depth_image, alpha=0.09), cv2.COLORMAP_JET)
            images = np.hstack((bg_removed, depth_colormap))
            cv2.namedWindow('Recorder Realsense', cv2.WINDOW_GUI_NORMAL)
            cv2.imshow('Recorder Realsense', images)
            key = cv2.waitKey(1)

            # if 'esc' button pressed, escape loop and exit program
            if key == 27:
                cv2.destroyAllWindows()
                break
    finally:
        pipeline.stop()
