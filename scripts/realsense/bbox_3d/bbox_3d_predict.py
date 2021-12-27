import click
import json
import pyrealsense2 as rs
from straypublic.utils import RealSensePostProcessor, get_rs_pipeline_and_config
import numpy as np
import torch
from straymodel.utils.visualization_utils import render_example
from straymodel import BoundingBox3DDetector
import cv2
import os
from collections import deque

@click.command()
@click.argument('checkpoint', nargs=1)
@click.option("--out", required=True, help="Image save location.")
@click.option("--reset-sleep", default=5, help="Time to sleep (seconds) after hardware reset.")
@click.option('--json-config', default="../../../configs/rsconfig.json")
def main(**args):
    os.makedirs(args["out"], exist_ok=True)
    #Setup RealSense
    json_config = json.load(open(args["json_config"]))
    pipeline, config = get_rs_pipeline_and_config(args, json_config)
    fps = int(json_config['stream-fps'])
    width, height = int(json_config['stream-width']), int(json_config['stream-height'])
    config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
    config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
    profile = pipeline.start(config)
    depth_sensor = profile.get_device().first_depth_sensor()

    #Get parameters from RealSense
    depth_scale = depth_sensor.get_depth_scale()
    color_profile = profile.get_stream(rs.stream.color)
    intrinsics = color_profile.as_video_stream_profile().get_intrinsics()

    detector = BoundingBox3DDetector(args['checkpoint'])



    camera_matrix = np.array([intrinsics.fx, 0, 0, 0, intrinsics.fy, 0, intrinsics.ppx, intrinsics.ppy, 1]).reshape(3, 3).T

    save_idx = 0

    corners_running = deque([], maxlen=5)

    try:
        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())
            color_image = cv2.rotate(color_image, cv2.ROTATE_90_COUNTERCLOCKWISE) / 255.0

            color_tensor = torch.from_numpy(color_image[np.newaxis,:]).type(torch.FloatTensor)
            color_tensor = torch.permute(color_tensor, (0, 3, 1, 2))


            #Get predictions from the detector based on depth
            predictions = detector(color_tensor)

            p_heatmaps, _, p_corners = predictions

            p_heatmap = torch.sigmoid(p_heatmaps).detach().cpu().numpy()[0]
            p_corners = p_corners.detach().cpu().numpy()[0]


            color_image = color_tensor.detach().cpu().numpy()[0]

            corner_image = render_example(color_image, p_heatmap, p_corners, camera_matrix, corners_running)
            cv2.imshow("3D Bounding Box", corner_image)
            cv2.imwrite(os.path.join(args["out"], f"{save_idx:06}.jpg"), corner_image)
            cv2.waitKey(1)

            save_idx += 1

    finally:
        pipeline.stop()

if __name__ == "__main__":
    main()