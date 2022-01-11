import argparse
import numpy as np
import json
import shutil
import robodk
import time
from pynput import keyboard
from scipy.spatial.transform import Rotation, Slerp
from PIL import Image
from robolink import *
from matplotlib import pyplot as plt
from simulation import Simulation
import threading

FAR_LENGTH = 1 << 32
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
FIELD_OF_VIEW = 50.0 # in degrees

def write_trajectory(scene_dir, trajectory):
    scene_path = os.path.join(scene_dir, 'scene')
    os.makedirs(scene_path, exist_ok=True)
    T_IW = np.eye(4) # np.linalg.inv(trajectory[0])
    with open(os.path.join(scene_path, 'trajectory.log'), 'wt') as f:
        for i, T_WC in enumerate(trajectory):
            T = T_IW @ T_WC
            f.write(f"{i:06}\n")
            np.savetxt(f, T, delimiter=' ')

def write_camera_parameters(scene_dir):
    cx = IMAGE_WIDTH * 0.5
    cy = IMAGE_HEIGHT * 0.5
    f = IMAGE_HEIGHT / np.tan(np.deg2rad(FIELD_OF_VIEW) * 0.5) * 0.5
    intrinsics = [f, 0., 0., 0., f, 0., cx, cy, 1.0]
    with open(os.path.join(scene_dir, 'camera_intrinsics.json'), 'wt') as f:
        f.write(json.dumps({
            'depth_format': 'Z16',
            'depth_scale': 1000.0,
            'height': IMAGE_HEIGHT,
            'width': IMAGE_WIDTH,
            'camera_model': 'pinhole',
            'intrinsic_matrix': intrinsics
        }, indent=2))

def interpolate_targets(prev_target, next_target, n=10):
    T_WP = np.array(prev_target.Pose().Rows())
    T_WN = np.array(next_target.Pose().Rows())
    R_WP = Rotation.from_matrix(T_WP[:3, :3])
    R_WN = Rotation.from_matrix(T_WN[:3, :3])
    xs = np.sort(np.random.uniform(0, 1, n))
    translations = (1.0 - xs[:, None]) * T_WP[:3, 3] + xs[:, None] * T_WN[:3, 3]
    rotations = Rotation.from_quat([R_WP.as_quat(), R_WN.as_quat()])
    transforms = []
    slerp = Slerp([0.0, 1.0], rotations)
    for i, x in enumerate(xs):
        T = np.eye(4)
        T[:3, :3] = slerp(x).as_matrix()
        T[:3, 3] = translations[i]
        transforms.append(T)
    return transforms

def scan(link, scene_dir):
    out_color = os.path.join(scene_dir, 'color')
    out_depth = os.path.join(scene_dir, 'depth')
    if os.path.exists(out_color):
        shutil.rmtree(out_color)

    if os.path.exists(out_depth):
        shutil.rmtree(out_depth)

    os.makedirs(out_color)
    os.makedirs(out_depth)

    robot = link.Item('Arm')
    scan_targets = [t for t in link.Item('ArmFrame').Childs() if 'Scan' in t.Name()]
    scan_targets.sort(key=lambda t: t.Name())

    camera_ref = link.Item('OAK-D')
    robot.setPoseTool(camera_ref)
    color_camera = link.Item('Color Camera')
    depth_camera = link.Item('Depth Camera')
    link.Cam2D_SetParams(f"FOV={FIELD_OF_VIEW} SIZE={IMAGE_WIDTH}x{IMAGE_HEIGHT}", color_camera)
    link.Cam2D_SetParams(f"FOV={FIELD_OF_VIEW} SIZE={IMAGE_WIDTH}x{IMAGE_HEIGHT} FAR_LENGTH={int(FAR_LENGTH)} DEPTH", depth_camera)
    trajectory = []

    i = 0
    for prev_target, next_target in zip(scan_targets, scan_targets[1:] + [scan_targets[0]]):
        targets = interpolate_targets(prev_target, next_target)
        for target in targets:
            print(f"Capturing image {i}" + " " * 10, end='\r')
            robot.MoveL(robodk.Mat(target.tolist()))
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
            depth_image = np.floor(depth_image * 0.5).astype(np.uint16)
            Image.fromarray(depth_image).save(depth_image_path_out)
            T_WC = np.array(robot.Pose().Rows())
            T_WC[:3, 3] /= 1000.0
            trajectory.append(T_WC)
            i += 1

    write_camera_parameters(scene_dir)
    write_trajectory(scene_dir, trajectory)
    print("Done scanning" + " " * 20)

class Runner:
    def __init__(self, flags):
        self.flags = flags
        self.output_dir = flags.out
        self.scanning = False
        self.link = robolink.Robolink()
        self.simulation = Simulation()
        self.simulation.reset_box()
        self.scans = self._count_scans() + 1

    def _count_scans(self):
        if not os.path.exists(self.output_dir):
            return -1
        scans = os.listdir(self.output_dir)
        scans.sort()
        return int(os.path.basename(scans[-1]))

    def _scan(self):
        if self.scanning:
            return
        print("")
        self.scanning = True
        self.simulation.pause(True)
        scan_dir = os.path.join(self.output_dir, f"{self.scans:04}")
        time.sleep(0.1)
        scan(self.link, scan_dir)
        print("Scanned", scan_dir)
        self.scans += 1
        self.simulation.pause(False)
        self.scanning = False

    def run(self):
        try:
            with keyboard.GlobalHotKeys({
                's': self._scan,
                }) as listener:
                listener.join()
        except KeyboardInterrupt:
            self.simulation.close()

def read_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('out')
    return parser.parse_args()

if __name__ == "__main__":
    Runner(read_args()).run()


