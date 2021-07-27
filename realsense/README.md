# RealSense Recorder
Records color and depth frames from RealSense

## Usage

1. `cd` into this directory
2. Install requirements with `pip install -r requirements.txt` into your current python environment
3. Connect your RealSense sensor to the computer via USB
4. Start image capture with `python record.py --out <OUPUT_FOLDER>` where the `--out <OUPUT_FOLDER>` is optional and used to specify where the collected images are saved (defaults to `./collected_dataset/` if not specified)
5. Images are saved into `<OUPUT_FOLDER>/color` and `<OUPUT_FOLDER>/depth` along with a `camera_intrinsics.json` file
6. To stop data collection press `CTRL+C` in your terminal
