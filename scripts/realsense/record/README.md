# RealSense Recorder
Records color and depth frames from RealSense D400 cameras
![Screenshot from 2021-11-01 15-56-25](https://user-images.githubusercontent.com/4254623/139684115-c63562a8-87bd-4a98-99df-7a768e91282d.png)


## Usage

1. Clone this repository with `git clone git@github.com:StrayRobots/StrayPublic.git` and navigate into this directory with `cd StrayPublic/realsense`
2. Install requirements with `pip install -r requirements.txt` into your current python environment
3. Connect your RealSense sensor to the computer via USB
4. Start image capture with `python record.py --out <OUPUT_FOLDER>` where the `--out <OUPUT_FOLDER>` is optional and used to specify where the collected images are saved (defaults to `./collected_dataset/` if not specified)
5. Images are saved into `<OUPUT_FOLDER>/color` and `<OUPUT_FOLDER>/depth` along with a `<OUPUT_FOLDER>/camera_intrinsics.json` file
6. To stop data collection press `esc` or `CTRL+C` in your terminal

## Settings
* To use an alternative config for the depth stream (specified with `--config`), first save a preset using the `realsense-viewer` app by clikcking the save button next to the preset selector.
  - ![Screenshot from 2021-11-01 16-07-27](https://user-images.githubusercontent.com/4254623/139684923-5028eed9-6e7a-4ebd-84a4-f0c8b6ef30f1.png)

* If color/depth stream settings need to be changed inside the script, examples are provided in the script
  - Depth: https://github.com/StrayRobots/StrayPublic/blob/8416b1d6ce239a0ab717a7f8cf93a37b2153dafe/realsense/record.py#L131
  - Color: https://github.com/StrayRobots/StrayPublic/blob/8416b1d6ce239a0ab717a7f8cf93a37b2153dafe/realsense/record.py#L134
  - All available options are documented here: https://intelrealsense.github.io/librealsense/python_docs/_generated/pyrealsense2.option.html
