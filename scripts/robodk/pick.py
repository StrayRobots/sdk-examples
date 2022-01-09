import numpy as np
import robodk
import time
import robolink
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
        self.simulation = Simulation(paused=True)
        self.link = self.simulation.link
        self.robot = self.link.Item('Arm')

    def delete_frame(self, frame_name):
        try:
            frame = self.link.Item(frame_name)
            self.link.Delete(frame)
        except ValueError:
            pass

    def run(self):
        box = self.link.Item('Box')
        robot = self.robot
        conveyor = self.link.Item('ConveyorFrame')
        self.simulation.reset_box()
        while True:
            robot.MoveL(self.link.Item('WaitPose'), blocking=False)
            box_pose = np.array(box.Pose().Rows())
            T_WP = box_pose.copy()

            if T_WP[0, 3] < 0.0:
                # In the exit lane, skip.
                continue

            self.simulation.pause(True)

            T_WP[0, 3] += 0.5 * BOX_LENGTH
            T_WP[1, 3] -= 0.5 * BOX_WIDTH
            T_WP[2, 3] += BOX_HEIGHT + 5.0

            gripper = self.link.Item('Gripper')
            arm_frame = self.link.Item('ArmFrame')
            robot.setPoseFrame(arm_frame)
            robot.setPoseTool(gripper)
            self.delete_frame("PickPose")

            T_WA = to_mat(arm_frame.Pose())
            T_AP = np.linalg.inv(T_WA) @ T_WP
            T_AT = T_AP @ np.linalg.inv(T_TP)

            pick_pose = self.link.AddTarget('PickPose', arm_frame, robot)
            # Ensure the frame is perfectly straight. Seems to cause unreashable if the is
            # some precision error.
            T_AT[:3, :3] = T_TP[:3, :3]
            pick_pose.setPose(to_pose(T_AT))
            try:
                robot.MoveL(pick_pose.Pose())
                gripper.AttachClosest(list_objects=[box])
                robot.MoveL(self.link.Item('PreDropPose'))
                robot.MoveL(self.link.Item('DropPose'))
                gripper.DetachAll()
            except robolink.TargetReachError:
                pass


            self.simulation.pause(False)
            time.sleep(0.5)



if __name__ == "__main__":
    Runner().run()
