# RealSense Recorder
Records color and depth frames from RealSense D400 cameras
![Screenshot from 2021-11-01 15-56-25](https://user-images.githubusercontent.com/4254623/139684115-c63562a8-87bd-4a98-99df-7a768e91282d.png)


## Usage

1. Install dependencies from https://github.com/StrayRobots/StrayPublic
2. Connect your RealSense sensor to the computer via USB
3. Start image capture with `python record.py --out <OUPUT_FOLDER> --json-config <PATH_TO_CONFIG> --reset-sleep <RESET_SLEEP>`
  - `--out` is the path to the directory where the dataset is saved
  - `--json-config` is the path to the saved realsense settings, defaults to the file in `StrayPublic/configs/rsconfig.json`
  - `--reset-sleep` is the time to sleep after resetting the camera settings, defaults to 5 seconds
4. Images are saved into `<OUPUT_FOLDER>/color`, `<OUPUT_FOLDER>/depth`, and `<OUPUT_FOLDER>/colorized_depth` along with a `<OUPUT_FOLDER>/camera_intrinsics.json` file
5. To stop data collection press `esc` or `CTRL+C` in your terminal

## Settings and configuration
* A working preset configuration for the depth stream is provided in the `rsconfig.json` file and it is used by default
* To use an alternative config for the depth stream, first save a preset using the `realsense-viewer` app by clicking the save button next to the preset selector
  - ![139684923-5028eed9-6e7a-4ebd-84a4-f0c8b6ef30f1](https://user-images.githubusercontent.com/4254623/139693688-b87106df-d06b-48d7-8208-f645a9e40e79.png)
  - Specify the path to the saved json file with the `--json-config` flag

* The `--reset-sleep` flag is used to specify how long to sleep after the device settings have been reset to default settings. Increase the number if the script seems to crash

* If color/depth stream settings need to be changed inside the script, examples are provided here:
  - Depth stream: https://github.com/StrayRobots/StrayPublic/blob/42044bfc7d425b22701f5b5da0a5edc1f3be8e6f/realsense/record.py#L136
  - Color stream: https://github.com/StrayRobots/StrayPublic/blob/42044bfc7d425b22701f5b5da0a5edc1f3be8e6f/realsense/record.py#L139
  - All available options are documented here: https://intelrealsense.github.io/librealsense/python_docs/_generated/pyrealsense2.option.html


## Recommended RGB settings
* The following settings are the main levers to adjust the quality of RGB images depending on lighting conditions etc. Try turning the automatic settings on/off and adjust the sliders until a desirable result is achieved. 
* <img width="307" alt="Screen Shot 2021-11-09 at 10 46 16 AM" src="https://user-images.githubusercontent.com/4254623/140891784-0f8e7134-0e29-4b8c-8736-ca7736859755.png">
* <img width="282" alt="Screen Shot 2021-11-09 at 10 46 38 AM" src="https://user-images.githubusercontent.com/4254623/140891833-ef8e4449-8ffc-4ef7-ad4b-8076798936b3.png">
* <img width="290" alt="Screen Shot 2021-11-09 at 10 46 57 AM" src="https://user-images.githubusercontent.com/4254623/140891880-d3a7ae2c-e95b-422c-abcf-13a20faa0419.png">
* <img width="277" alt="Screen Shot 2021-11-09 at 10 47 22 AM" src="https://user-images.githubusercontent.com/4254623/140891930-5292f543-d116-473d-955c-38e084755e22.png">
* <img width="277" alt="Screen Shot 2021-11-09 at 10 47 44 AM" src="https://user-images.githubusercontent.com/4254623/140891979-4b91741b-5383-4a09-b040-641037da7930.png">
* <img width="291" alt="Screen Shot 2021-11-09 at 10 47 58 AM" src="https://user-images.githubusercontent.com/4254623/140892018-10d26784-49bf-470b-974d-af2d30f591f4.png">

