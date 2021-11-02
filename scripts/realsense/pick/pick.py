from straymodel import *
import click
import cv2
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation
import pyrealsense2 as rs
from straypublic.utils import RealSensePostProcessor, get_rs_pipeline_and_config, set_default_rs_sensor_options
import json

def get_o3d_frame(pose, scale):
    mesh = o3d.geometry.TriangleMesh.create_coordinate_frame().scale(scale, np.zeros(3))
    frame = mesh.transform(pose)
    return frame

def get_o3d_sphere(p, color=np.array([0, 1, 0])):
    sphere = o3d.geometry.TriangleMesh.create_sphere(0.01).paint_uniform_color(color)
    pose = np.eye(4)
    pose[:3,3] = p
    sphere = sphere.transform(pose)
    return sphere

def get_point_W(p, T_WC):
    point_C = np.ones(4)
    point_C[:3] = p
    point_W = T_WC @ point_C
    return point_W[:3]

def draw_box(image, instance, scale=1):
    rect = ((instance["x"], instance["y"]), (instance["width"]*scale, instance["height"]*scale), instance["rotation"])
    cv_box = cv2.boxPoints(rect)
    cv_box = np.int0(cv_box)
    cv2.drawContours(image, [cv_box], 0, (0,255,0), 2)

def visualize_3d(T_WC, spheres):
    base_frame = get_o3d_frame(np.eye(4), 0.2)
    camera_frame = get_o3d_frame(T_WC, 0.1)
    #NOTE: below blocks, close the window to continue the loop
    o3d.visualization.draw_geometries([base_frame, camera_frame] + spheres)

def visualize_images(color_image, colorized_depth_image):
    cv2.imshow("Detections", color_image)
    cv2.imshow("Depth", colorized_depth_image)
    cv2.waitKey(1)

def load_hand_eye(hand_eye_json):
    with open(hand_eye_json, 'rt') as f:
        data = json.load(f)
    t = data['translation']
    q = data['rotation']
    T_HC = np.eye(4)
    R = Rotation.from_quat([q['i'], q['j'], q['k'], q['w']])
    T_HC[:3, :3] = R.as_matrix()
    T_HC[:3, 3] = [t['x'], t['y'], t['z']]
    return T_HC

def to_transformation_matrix(x, y, z, rx, ry, rz):
    T = np.eye(4)
    R = Rotation.from_euler('xyz', [rx, ry, rz])
    T[:3, :3] = R.as_matrix()
    T[:3, 3] = [x, y, z]
    return T

@click.command()
@click.argument('model', nargs=1)
@click.option("--reset-sleep", default=5, help="Time to sleep (seconds) after hardware reset.")
@click.option('--json-config', default="../../../configs/rsconfig.json")
@click.option('--visualize', type=click.Choice(['images', '3d']), default="images")
@click.option('--z', default=0.66)
@click.option('--minimum-confidence', default=0.9)
@click.option('--depth-box-scale', default=0.5)
@click.option('--hand-eye', type=str)
def main(**args):
    #Setup RealSense
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

    #Get parameters from RealSense
    depth_scale = depth_sensor.get_depth_scale()
    color_profile = profile.get_stream(rs.stream.color)
    color_intrinsics = color_profile.as_video_stream_profile().get_intrinsics()
    fx, fy = color_intrinsics.fx, color_intrinsics.fy

    #Creates a detector from the provided model
    detector = OrientedBoundingBoxDetector(args["model"], width, height, fx, fy, depth_scale, args["minimum_confidence"], args["depth_box_scale"])

    #Setup frame post processing and visualization
    colorizer = rs.colorizer()
    post_processor = RealSensePostProcessor()

    # Load transformation from hand to camera.
    T_HC = load_hand_eye(args['hand_eye'])

    # Transformation from the base of the robot to the hand frame.
    # Changes when the robot moves. Use to_transformation_matrix() to transform euler angles to matrices.
    T_WH = np.eye(4)

    T_WC = T_WH @ T_HC

    align_to = rs.stream.color
    align = rs.align(align_to)

    try:
        while True:
            #Get frameset
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            aligned_depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            if not aligned_depth_frame or not color_frame:
                continue
            
            #Process color and depth images
            aligned_depth_frame = post_processor(aligned_depth_frame)
            colorized_depth_image = np.asanyarray(colorizer.colorize(aligned_depth_frame).get_data())
            colorized_depth_image = cv2.resize(colorized_depth_image, (width, height), interpolation=cv2.INTER_NEAREST_EXACT)
            depth_image = np.asanyarray(aligned_depth_frame.get_data())
            depth_image = cv2.resize(depth_image, (width, height), interpolation=cv2.INTER_NEAREST_EXACT)
            color_image = np.asanyarray(color_frame.get_data())

            #Get predictions from the detector based on depth
            predictions_from_depth = detector(color_image, depth_image)

            #Get predictions from the detector based on known pick height z
            predictions_from_height = detector(color_image, depth_image, args["z"])
            
            spheres = []

            #Iterate detections
            for pred_depth, pred_height in zip(predictions_from_depth, predictions_from_height):
                #Draw detections on color and depth image
                draw_box(color_image, pred_depth)
                draw_box(colorized_depth_image, pred_depth, args["depth_box_scale"])

                #Convert depth based 3d point to robot coordinates for downstream use
                point_3d_depth_W = get_point_W(pred_depth["point_3d"], T_WC)
                #Convert height based 3d point to robot coordinates for downstream use
                point_3d_height_W = get_point_W(pred_height["point_3d"], T_WC)


                #Add points into 3d visualization
                if args["visualize"] == "3d":
                    spheres.append(get_o3d_sphere(point_3d_depth_W, np.array([0, 1, 0]))) #Green spheres

                    spheres.append(get_o3d_sphere(point_3d_height_W, np.array([0, 0, 1]))) #Blue spheres
                

            if args["visualize"] == "3d":
                #NOTE: below blocks the loop, close the window to continue
                visualize_3d(T_WC, spheres)
            elif args["visualize"] == "images":
                visualize_images(color_image, colorized_depth_image)

    finally:
        pipeline.stop()




if __name__ == "__main__":
    main()

