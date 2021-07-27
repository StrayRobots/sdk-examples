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
