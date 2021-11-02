import pyrealsense2 as rs
import numpy as np
import cv2
import click
from os import makedirs
from os.path import exists, join
import shutil
import json
import time
from straypublic.utils import Preset, PostProcessor, get_rs_pipeline_and_config, set_default_rs_sensor_options


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

@click.command()
@click.option("--out", default='./collected_dataset/', help="set output folder")
@click.option("--reset-sleep", default=5, help="Time to sleep (seconds) after hardware reset.")
@click.option('--json-config', default="../../../configs/rsconfig.json")
def main(**args):
    path_output = args["out"]
    path_depth = join(path_output, "depth")
    path_colorized_depth = join(path_output, "colorized_depth")
    path_color = join(path_output, "color")
    make_clean_folders(args, path_output, path_depth, path_color, path_colorized_depth)
    json_config = json.load(open(args["json_config"]))
    pipeline, config = get_rs_pipeline_and_config(args, json_config)
    fps = int(json_config['stream-fps'])
    width, height = int(json_config['stream-width']), int(json_config['stream-height'])
    config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
    config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)

    profile = pipeline.start(config)
    depth_sensor = profile.get_device().first_depth_sensor()
    color_sensor = profile.get_device().first_color_sensor()

    set_default_rs_sensor_options(color_sensor, depth_sensor)

    colorizer = rs.colorizer()
    post_processor = PostProcessor()
    
    depth_scale = depth_sensor.get_depth_scale()

    align_to = rs.stream.color
    align = rs.align(align_to)

    # Streaming loop
    frame_count = 0
    try:
        while True:
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            aligned_depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

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
                    join(path_output, "camera_intrinsics.json"),
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


if __name__ == "__main__":
    main()