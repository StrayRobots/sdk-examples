import numpy as np
import robodk
import time
import robolink
from robolink import Robolink
from simulation import Simulation
from scipy.spatial.transform import Rotation

BOX_WIDTH = 220.0
BOX_LENGTH = 300.0
BOX_HEIGHT = BOX_WIDTH
T_TP = np.eye(4)
T_TP[:3, :3] = (Rotation.from_rotvec([0.0, 0.0, -np.pi/2.0]) * Rotation.from_rotvec([np.pi, 0.0, 0])).as_matrix()

def to_mat(pose):
    return np.array(pose.Rows())

def to_pose(matrix):
    return robodk.Mat(matrix.tolist())

class Runner:
    def __init__(self):
        self.simulation = Simulation()
        self.link = Robolink()
        self.robot = self.link.Item('Arm')
        self.prepick_pose = np.array(self.link.Item('PrePickPose').Pose().Rows())
        self.T_AW = np.linalg.inv(to_mat(self.link.Item('ArmFrame').Pose()))

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

    def _to_arm_frame(self, T_WP):
        return self.T_AW @ T_WP

    def _compute_pick_time_and_pose(self):
        box = self.link.Item('Box')
        box_pose = np.array(box.Pose().Rows())
        t0 = self.link.SimulationTime()

        if box_pose[0, 3] < 0.0:
            # In the exit lane, skip.
            return None, None

        T_AP = self._estimate_pick_pose(box_pose)

        # Box moving in the y direction. Figure out when it will be beneath the pre pick pose.
        target_y = self.prepick_pose[1, 3]
        x0 = T_AP[:3, 3]

        if x0[1] < target_y:
            # Box is past the pick point. Skip.
            return None, None

        box_velocity = np.array([0.0, -200.0, 0.0])
        distance = target_y - x0[1]
        dt = (distance / box_velocity[1])

        pick_pose = self.prepick_pose.copy()
        # The pick pose is the same as the prepick, but set to the boxes actual height.
        pick_pose[2, 3] = T_AP[2, 3]
        return t0 + dt, pick_pose

    def wait_until(self, t):
        while True:
            current_time = self.link.SimulationTime()
            if current_time >= t:
                break
            time.sleep(0.01)

    def run(self):
        robot = self.robot
        conveyor = self.link.Item('ConveyorFrame')
        arm_frame = self.link.Item('ArmFrame')
        gripper = self.link.Item('Gripper')
        robot.setPoseFrame(arm_frame)
        robot.setPoseTool(gripper)
        box = self.link.Item('Box')
        self.simulation.reset_box()
        pick_pose = self.link.Item('PickPose')
        robot.MoveL(self.link.Item('WaitPose'))
        time.sleep(1.0)
        while True:

            pick_time, T_AP = self._compute_pick_time_and_pose()
            if T_AP is None:
                continue

            pick_pose.setPose(to_pose(T_AP))

            robot.MoveL(self.link.Item('PrePickPose'), blocking=False)

            try:
                # Wait until the pick time.
                self.wait_until(pick_time - 0.5)
                robot.MoveL(pick_pose.Pose())
                try:
                    self.simulation.write_lock.acquire()
                    gripper.AttachClosest(list_objects=[box])
                finally:
                    self.simulation.write_lock.release()
                robot.MoveL(self.link.Item('PreDropPose'))
                robot.MoveL(self.link.Item('DropPose'))
                try:
                    self.simulation.write_lock.acquire()
                    gripper.DetachAll()
                finally:
                    self.simulation.write_lock.release()
                robot.MoveL(self.link.Item('WaitPose'))
            except robolink.TargetReachError:
                pass




if __name__ == "__main__":
    Runner().run()
