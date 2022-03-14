import numpy as np
import robodk
import time
import argparse
import robolink
import tempfile
import os
import shutil
from detect import Detector
from robolink import Robolink
from simulation import Simulation
from scipy.spatial.transform import Rotation
from constants import *

BOX_WIDTH = 220.0
BOX_LENGTH = 300.0
BOX_HEIGHT = BOX_WIDTH
T_TP = np.eye(4)
T_TP[:3, :3] = (Rotation.from_rotvec([0.0, 0.0, -np.pi/2.0]) * Rotation.from_rotvec([np.pi, 0.0, 0])).as_matrix()

def to_mat(pose):
    return np.array(pose.Rows())

def to_pose(matrix):
    return robodk.Mat(matrix.tolist())

class ImageCache:
    def __init__(self):
        filenames = [f"image_{i}" for i in range(10)]
        self.tempdir = tempfile.mkdtemp()
        self.image_paths = [os.path.join(self.tempdir, f) for f in filenames]

    def next(self):
        path = self.image_paths.pop()
        self.image_paths = [path] + self.image_paths
        return path

    def close(self):
        shutil.rmtree(self.tempdir)

class Runner:
    def __init__(self, flags):
        self.simulation = Simulation()
        self.link = Robolink()
        self.robot = self.link.Item('Arm')
        self.prepick_pose = np.array(self.link.Item('PrePickPose').Pose().Rows())
        self.pick_pose = self.link.Item('PickPose')
        self.T_AW = np.linalg.inv(to_mat(self.link.Item('ArmFrame').Pose()))
        self.detector = Detector(flags.model)
        self.image_cache = ImageCache()
        self.T_WC = to_mat(self.link.Item('WaitPose').Pose())
        self._setup_camera()

    def delete_frame(self, frame_name):
        try:
            frame = self.link.Item(frame_name)
            self.link.Delete(frame)
        except ValueError:
            pass

    def _estimate_pick_pose(self, T_WP):
        T_WP[0, 3] += 0.5 * BOX_LENGTH
        T_WP[1, 3] -= 0.5 * BOX_WIDTH
        T_WP[2, 3] += BOX_HEIGHT + 5.0
        return self.T_AW @ T_WP @ np.linalg.inv(T_TP)

    def _setup_camera(self):
        self.color_camera = self.link.Item('Color Camera')
        self.depth_camera = self.link.Item('Depth Camera')
        self.color_camera.setParam('Open', '1')
        self.depth_camera.setParam('Open', '1')
        self.link.Cam2D_SetParams(f"FOV=50 SIZE=640x480", self.color_camera)
        self.link.Cam2D_SetParams(f"FOV=50 SIZE=640x480 DEPTH FAR_LENGTH={int(FAR_LENGTH)}", self.depth_camera)

    def _compute_pick_time_and_pose(self, timestamp, detection_m):
        detection = detection_m * 1000.0
        T_WP = self.prepick_pose.copy()

        # Convert detection to world frame.
        p_C = np.concatenate([detection, np.ones(1)])[:, None]
        p_W = self.T_WC @ p_C

        # Box moving in the y direction. Figure out when it will be beneath the pre pick pose.
        target_y = self.prepick_pose[1, 3]
        box_velocity = np.array([0.0, -BELT_VELOCITY * 1000.0, 0.0])
        distance = p_W[1, 0] - target_y

        # Calculate when it will be at the pre-pick pose.
        dt = np.abs(distance / box_velocity[1])
        p_W[:3, 0] += distance * np.array([0.0, -1.0, 0.0])

        T_WP[:3, 3] = p_W[:3, 0]

        return timestamp + dt, T_WP

    def wait_until(self, t):
        while True:
            current_time = self.link.SimulationTime()
            if current_time >= t:
                break
            time.sleep(0.01)

    def capture_image(self):
        if self.detector.full():
            return
        filepath = self.image_cache.next()
        color_file = filepath + '.jpg'
        depth_file = filepath + '.grey32'
        sim_time = self.link.SimulationTime()
        captured_color = self.link.Cam2D_Snapshot(color_file, self.color_camera)
        captured_depth = self.link.Cam2D_Snapshot(depth_file, self.depth_camera)
        if captured_color == 1 and captured_depth == 1:
            self.detector.push((sim_time, color_file, depth_file))

    def run(self):
        robot = self.robot
        conveyor = self.link.Item('ConveyorFrame')
        arm_frame = self.link.Item('ArmFrame')
        gripper = self.link.Item('Gripper')
        robot.setPoseFrame(arm_frame)
        robot.setPoseTool(gripper)
        self.simulation.reset_box()
        pick_pose = self.link.Item('PickPose')
        joints = robot.Joints()
        while True:
            robot.MoveL(self.link.Item('WaitPose'))
            self.capture_image()
            detector_result = self.detector.get()
            if detector_result is None:
                time.sleep(0.1)
                continue

            pick_time, T_AP = self._compute_pick_time_and_pose(*detector_result)
            target_pose = to_pose(T_AP)
            pick_pose.setPose(target_pose)

            robot.MoveL(self.link.Item('PrePickPose'), blocking=False)

            try:
                # Wait until the pick time.
                self.wait_until(pick_time)
                probed = self._probe(robot, pick_pose)
                robot.MoveL(probed)
                try:
                    self.simulation.write_lock.acquire()
                    box = self.link.Item('Box')
                    gripper.AttachClosest(list_objects=[box])
                finally:
                    self.simulation.write_lock.release()
                robot.MoveL(self.link.Item('PreDropPose'))
                robot.MoveL(self.link.Item('DropPose'))
                # Clear any detections that might be waiting.
                self.detector.clear()
                try:
                    self.simulation.write_lock.acquire()
                    gripper.DetachAll()
                finally:
                    self.simulation.write_lock.release()
            except robolink.TargetReachError:
                pass

    def _probe(self, robot, pose):
        target_pose = None
        poses = self._pose_split(robot.Pose(), pose.Pose(), 25.0)
        for pose in poses:
            robot.SearchL(pose)
            status = robot.setParam("Driver", "Status")
            target_pose = robot.Pose()

            if len(status) > 0:
                if "1" in status:
                    # There is contact.
                    break
                else:
                    target_pose = None
                    continue
            else:
                if False and robodk.pose_is_similar(robot.Pose(), pose, 0.1):
                    target_pose = None
                    continue
                else:
                    # Found target pose
                    break
        return target_pose

    def _pose_split(self, pose1, pose2, delta_mm):
        """
        Split the move between 2 poses given delta_mm increments
        returns a list of pose sub-pose
        """
        pose_delta = robodk.invH(pose1) * pose2
        distance = robodk.norm(pose_delta.Pos())
        if distance <= delta_mm:
            return [pose2]

        pose_list = []

        x, y, z, w, p, r = robodk.Pose_2_UR(pose_delta)

        steps = max(1, int(distance / delta_mm))

        xd = x / steps
        yd = y / steps
        zd = z / steps
        wd = w / steps
        pd = p / steps
        rd = r / steps
        for i in range(steps - 1):
            factor = i + 1
            pose_list.append(pose1 * robodk.UR_2_Pose([xd * factor, yd * factor, zd * factor, wd * factor, pd * factor, rd * factor]))

        return pose_list

    def close(self):
        self.image_cache.close()

def read_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', required=True)
    return parser.parse_args()

if __name__ == "__main__":
    flags = read_args()
    try:
        runner = Runner(flags)
        runner.run()
    finally:
        runner.close()

