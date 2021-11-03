from enum import IntEnum
import pyrealsense2 as rs
import time

class RealSensePostProcessor():
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

def get_rs_pipeline_and_config(args, json_config):
    pipeline = rs.pipeline()
    config = rs.config()
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    device.hardware_reset()
    time.sleep(args["reset_sleep"])
    advaced_mode = rs.rs400_advanced_mode(device)
    advaced_mode.load_json(str(json_config).replace("'", '\"'))
    return pipeline, config
