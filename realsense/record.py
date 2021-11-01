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
import time

class PostProcessor():
    def __init__(self):
        self.decimation = rs.decimation_filter()
        self.spatial = rs.spatial_filter()
        self.spatial.set_option(rs.option.filter_magnitude, 5)
        self.spatial.set_option(rs.option.filter_smooth_alpha, 1)
        self.spatial.set_option(rs.option.filter_smooth_delta, 50)
        self.spatial.set_option(rs.option.holes_fill, 3)
        self.hole_filling = rs.hole_filling_filter()
        self.temporal = rs.temporal_filter()
        self.depth_to_disparity = rs.disparity_transform(True)
        self.disparity_to_depth = rs.disparity_transform(False)

    def __call__(self, input_frame):
        frame = self.decimation.process(input_frame)
        frame = self.depth_to_disparity.process(frame)
        frame = self.spatial.process(frame)
        frame = self.temporal.process(frame)
        frame = self.disparity_to_depth.process(frame)
        frame = self.hole_filling.process(frame)
        return frame



class Preset(IntEnum):
    Custom = 0
    Default = 1
    Hand = 2
    HighAccuracy = 3
    HighDensity = 4
    MediumDensity = 5


def make_clean_folders(args, path_output, path_depth, path_color, path_colorized_depth):
    for path_folder in [path_output, path_depth, path_colorized_depth, path_color]:
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

def setup_pipeline_and_config(args, json_config):
    pipeline = rs.pipeline()
    config = rs.config()
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    device.hardware_reset()
    time.sleep(args.reset_sleep)
    advaced_mode = rs.rs400_advanced_mode(device)
    advaced_mode.load_json(str(json_config).replace("'", '\"'))
    return pipeline, config




if __name__ == "__main__":

    parser = argparse.ArgumentParser(
        description=
        "Realsense Recorder. Please select one of the optional arguments")
    parser.add_argument("--out",
                        default='./collected_dataset/',
                        help="set output folder")
    parser.add_argument("--reset-sleep",
                        default=5,
                        help="Time to sleep (seconds) after hardware reset.")
    parser.add_argument("--config",
                        default="./rsconfig.json",
                        help="Time to sleep (seconds) after hardware reset.")

    args = parser.parse_args()
    path_output = args.out
    path_depth = join(args.out, "depth")
    path_colorized_depth = join(args.out, "colorized_depth")
    path_color = join(args.out, "color")
    make_clean_folders(args, path_output, path_depth, path_color, path_colorized_depth)
    json_config = json.load(open(args.config))
    pipeline, config = setup_pipeline_and_config(args, json_config)
    fps = int(json_config['stream-fps'])
    width, height = int(json_config['stream-width']), int(json_config['stream-height'])
    config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
    config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)

    profile = pipeline.start(config)
    depth_sensor = profile.get_device().first_depth_sensor()
    color_sensor = profile.get_device().first_color_sensor()

    #Set depth sensor options, recommendations: https://dev.intelrealsense.com/docs/d400-series-visual-presets
    #depth_sensor.set_option(rs.option.visual_preset, Preset.HighAccuracy)  # Using preset HighAccuracy for recording

    #Set color sensor options
    color_sensor.set_option(rs.option.enable_auto_exposure, 0.0) #Disable auto exposure
    color_sensor.set_option(rs.option.enable_auto_white_balance, 0.0) #Disable auto white balance  
    color_sensor.set_option(rs.option.white_balance, 3700.0) #White balance, adjust as needed   
    color_sensor.set_option(rs.option.gain, 64.0) #Gain, adjust as needed  
    color_sensor.set_option(rs.option.exposure, 953.0) #Exposure, adjust as needed 

    colorizer = rs.colorizer()
    post_processor = PostProcessor()
    
    depth_scale = depth_sensor.get_depth_scale()

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

            aligned_depth_frame = post_processor(aligned_depth_frame)

            colorized_depth_image = np.asanyarray(colorizer.colorize(aligned_depth_frame).get_data())
            colorized_depth_image = cv2.resize(colorized_depth_image, (width, height), interpolation=cv2.INTER_NEAREST_EXACT)
            depth_image = np.asanyarray(aligned_depth_frame.get_data())
            depth_image = cv2.resize(depth_image, (width, height), interpolation=cv2.INTER_NEAREST_EXACT)
            color_image = np.asanyarray(color_frame.get_data())

            if frame_count == 0:
                save_intrinsic_as_json(
                    join(args.out, "camera_intrinsics.json"),
                    color_frame, depth_scale, fps)
            cv2.imwrite("%s/%06d.png" % \
                    (path_depth, frame_count), depth_image)
            cv2.imwrite("%s/%06d.jpg" % \
                    (path_color, frame_count), color_image)
            cv2.imwrite("%s/%06d.jpg" % \
                    (path_colorized_depth, frame_count), colorized_depth_image)
            print("Saved color + depth image %06d" % frame_count)
            frame_count += 1

            cv2.imshow('Color', color_image)
            cv2.imshow("Depth", colorized_depth_image)
            key = cv2.waitKey(1)
            # if 'esc' button pressed, escape loop and exit program
            if key == 27:
                cv2.destroyAllWindows()
                break
    finally:
        pipeline.stop()
