import numpy as np
import robodk
import time
from scipy.spatial.transform import Rotation, Slerp
from PIL import Image
from robolink import *
from matplotlib import pyplot as plt
import threading


class Simulation:
    CONVEYOR_BELT_END = 100.0
    def __init__(self, paused=False):
        self.sleep_for = 0.05
        self.box_velocity = np.array([0.0, -0.2, 0.0])
        self.paused = paused
        self.done = False
        self.link = Robolink()
        self.link.Render(True)
        self.box = self.link.Item('Box')
        self.background_thread = threading.Thread(target=self._simulation_loop, daemon=True)
        self.background_thread.start()
        self.previous_sim_time = None
        if not paused:
            self._reset_box()

    def _simulation_loop(self):
        self.link.setSimulationSpeed(1.0)
        self.previous_sim_time = self.link.SimulationTime()
        while not self.done:
            if self.paused:
                time.sleep(0.01)
                continue
            self._step_simulation()

    def reset_box(self):
        gripper = self.link.Item('Gripper')
        gripper.DetachAll()
        box = self.link.Item('Box')
        box.setParent(self.link.Item('picking_setup'))
        box_pose = np.array(box.Pose().Rows())
        box_pose[:3, :3] = Rotation.from_rotvec([0.0, 0.0, -np.pi / 2.0]).as_matrix()
        box_pose[0, 3] = 200.0
        box_pose[1, 3] = 1800.0
        box_pose[2, 3] = 0.0
        box.setPose(robodk.Mat(box_pose.tolist()))

    def _step_simulation(self):
        current_time = self.link.SimulationTime()
        diff = current_time - self.previous_sim_time
        current_pose = np.array(self.box.Pose().Rows())
        if current_pose[1, 3] < self.CONVEYOR_BELT_END:
            self.reset_box()
            return

        if current_pose[2, 3] < 100.0:
            # On conveyor belt. Let's move it.
            current_pose[:3, 3] += diff * self.box_velocity * 1000.0 # Pose is in millimeters.
        else:
            print(current_pose[:3, 3])
        self.box.setPose(robodk.Mat(current_pose.tolist()))
        self.previous_sim_time = current_time
        time.sleep(self.sleep_for)

    def pause(self, value):
        self.paused = value
        if not self.paused:
            self.previous_sim_time = self.link.SimulationTime()

    def close(self):
        self.done = True

