# RealSense Recorder
Records color and depth frames from RealSense

![Screenshot from 2021-07-27 15-30-42](https://user-images.githubusercontent.com/4254623/127154159-45bf4ad6-6a24-473c-bf49-3b20f844993b.png)

## Usage

1. Clone this repository with `git clone git@github.com:StrayRobots/StrayPublic.git` and navigate into this directory with `cd StrayPublic/realsense`
2. Install requirements with `pip install -r requirements.txt` into your current python environment
3. Connect your RealSense sensor to the computer via USB
4. Start image capture with `python record.py --out <OUPUT_FOLDER>` where the `--out <OUPUT_FOLDER>` is optional and used to specify where the collected images are saved (defaults to `./collected_dataset/` if not specified)
5. Images are saved into `<OUPUT_FOLDER>/color` and `<OUPUT_FOLDER>/depth` along with a `<OUPUT_FOLDER>/camera_intrinsics.json` file
6. To stop data collection press `CTRL+C` in your terminal

## Settings
* The script will prompt the user to select an available resolution for both color and depth streams![Screenshot from 2021-10-26 15-16-00](https://user-images.githubusercontent.com/4254623/138876849-2c28d08b-e517-430b-99fa-02e83e271972.png)
![Screenshot from 2021-10-26 15-15-40](https://user-images.githubusercontent.com/4254623/138876857-3a0e0de4-b5b9-4f9c-b3fb-b146c91a7be8.png)
* For the color stream, choosing the highest resolution with the highest fps is recommended
* Depth stream: D435, use 848x480 resolution @30fps. D415, use 1280x720 resolution @30fps. Further recommendations in the [documentation](https://dev.intelrealsense.com/docs/d400-series-visual-presets).
* If color/depth stream settings need to be changed, examples are provided in the script
  - Depth: https://github.com/StrayRobots/StrayPublic/blob/8416b1d6ce239a0ab717a7f8cf93a37b2153dafe/realsense/record.py#L131
  - Color: https://github.com/StrayRobots/StrayPublic/blob/8416b1d6ce239a0ab717a7f8cf93a37b2153dafe/realsense/record.py#L134
  - All available options are documented here: https://intelrealsense.github.io/librealsense/python_docs/_generated/pyrealsense2.option.html
