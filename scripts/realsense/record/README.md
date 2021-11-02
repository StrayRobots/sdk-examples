# RealSense Recorder
Records color and depth frames from RealSense D400 cameras
![Screenshot from 2021-11-01 15-56-25](https://user-images.githubusercontent.com/4254623/139684115-c63562a8-87bd-4a98-99df-7a768e91282d.png)


## Usage

1. Clone this repository with `git clone git@github.com:StrayRobots/StrayPublic.git` and navigate into this directory with `cd StrayPublic/realsense`
2. Install requirements with `pip install -r requirements.txt` into your current python environment
3. Connect your RealSense sensor to the computer via USB
4. Start image capture with `python record.py --out <OUPUT_FOLDER, default=./collected_dataset> --json-config <PATH_TO_CONFIG, default=./rsconfig.json> --reset-sleep <RESET_SLEEP, default=5>`
6. Images are saved into `<OUPUT_FOLDER>/color`, `<OUPUT_FOLDER>/depth`, and `<OUPUT_FOLDER>/colorized_depth` along with a `<OUPUT_FOLDER>/camera_intrinsics.json` file
7. To stop data collection press `esc` or `CTRL+C` in your terminal

## Settings and configuration
* A working preset configuration for the depth stream is provided in the `rsconfig.json` file and it is used by default
* To use an alternative config for the depth stream, first save a preset using the `realsense-viewer` app by clicking the save button next to the preset selector
  - ![139684923-5028eed9-6e7a-4ebd-84a4-f0c8b6ef30f1](https://user-images.githubusercontent.com/4254623/139693688-b87106df-d06b-48d7-8208-f645a9e40e79.png)
  - Specify the path to the saved json file with the `--config` flag

* The `--reset-sleep` flag is used to specify how long to sleep after the device settings have been reset to default settings. Increase the number if the script seems to crash

* If color/depth stream settings need to be changed inside the script, examples are provided here:
  - Depth stream: https://github.com/StrayRobots/StrayPublic/blob/42044bfc7d425b22701f5b5da0a5edc1f3be8e6f/realsense/record.py#L136
  - Color stream: https://github.com/StrayRobots/StrayPublic/blob/42044bfc7d425b22701f5b5da0a5edc1f3be8e6f/realsense/record.py#L139
  - All available options are documented here: https://intelrealsense.github.io/librealsense/python_docs/_generated/pyrealsense2.option.html
