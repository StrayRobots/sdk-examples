import argparse
import numpy as np
import json
import shutil
from PIL import Image
from robolink import *
from matplotlib import pyplot as plt

FAR_LENGTH = 2**32
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
FIELD_OF_VIEW = 45.0 # in degrees

def write_trajectory(flags, trajectory):
    scene_path = os.path.join(flags.out, 'scene')
    os.makedirs(scene_path, exist_ok=True)
    T_IW = np.eye(4) # np.linalg.inv(trajectory[0])
    with open(os.path.join(scene_path, 'trajectory.log'), 'wt') as f:
        for i, T_WC in enumerate(trajectory):
            T = T_IW @ T_WC
            f.write(f"{i:06}\n")
            np.savetxt(f, T, delimiter=' ')

def write_camera_parameters(flags):
    cx = IMAGE_WIDTH * 0.5
    cy = IMAGE_HEIGHT * 0.5
    f = IMAGE_HEIGHT / np.tan(np.deg2rad(FIELD_OF_VIEW) * 0.5) * 0.5
    intrinsics = [f, 0., 0., 0., f, 0., cx, cy, 1.0]
    with open(os.path.join(flags.out, 'camera_intrinsics.json'), 'wt') as f:
        f.write(json.dumps({
            'depth_format': 'Z16',
            'depth_scale': 1000.0,
            'height': IMAGE_HEIGHT,
            'width': IMAGE_WIDTH,
            'camera_model': 'pinhole',
            'intrinsic_matrix': intrinsics
        }, indent=2))

def main(flags):
    out_color = os.path.join(flags.out, 'color')
    out_depth = os.path.join(flags.out, 'depth')
    if os.path.exists(out_color):
        shutil.rmtree(out_color)

    if os.path.exists(out_depth):
        shutil.rmtree(out_depth)

    os.makedirs(out_color)
    os.makedirs(out_depth)

    link = Robolink()
    robot = link.Item('Arm')
    scan_targets = [t for t in link.Item('Arm Base').Childs() if 'Scan' in t.Name()]
    scan_targets.sort(key=lambda t: t.Name())

    camera_ref = link.Item('OAK-D')
    robot.setPoseTool(camera_ref)
    color_camera = link.Item('Color Camera')
    depth_camera = link.Item('Depth Camera')
    link.Cam2D_SetParams(f"FOV={FIELD_OF_VIEW} SNAPSHOT={IMAGE_WIDTH}x{IMAGE_HEIGHT}", color_camera)
    link.Cam2D_SetParams(f"FOV={FIELD_OF_VIEW} SNAPSHOT={IMAGE_WIDTH}x{IMAGE_HEIGHT} FAR_LENGTH={int(FAR_LENGTH)} DEPTH", depth_camera)
    trajectory = []

    for i, target in enumerate(scan_targets):
        print(f"Capturing image {i}", end='\r')
        robot.MoveL(target)
        color_image_path = os.path.join(os.getcwd(), out_color, f"{i:06}.jpg")
        depth_image_path_tmp = '/tmp/robodk_depth_image.grey32'
        depth_image_path_out = os.path.join(os.getcwd(), out_depth, f"{i:06}.png")

        color_captured = link.Cam2D_Snapshot(color_image_path, color_camera)
        depth_captured = link.Cam2D_Snapshot(depth_image_path_tmp, depth_camera)
        if color_captured != 1 or depth_captured != 1:
            print("failed to capture frame")
            exit(1)

        depth_image = np.fromfile(depth_image_path_tmp, dtype='>u4')
        w, h = depth_image[:2]
        depth_image = np.flipud(np.reshape(depth_image[2:], (h, w)))
        # Seems the unit is 2 millimeters
        depth_image = np.round(depth_image * 0.5).astype(np.uint16)
        Image.fromarray(depth_image).save(depth_image_path_out)
        T_WC = np.array(robot.Pose().Rows())
        T_WC[:3, 3] /= 1000.0
        trajectory.append(T_WC)

    write_camera_parameters(flags)
    write_trajectory(flags, trajectory)

def read_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('out')
    return parser.parse_args()

if __name__ == "__main__":
    main(read_args())


