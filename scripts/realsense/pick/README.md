# Picking Example using the Intel RealSense
Predicts 3D pick points of objects
![Screenshot from 2021-11-02 12-46-03](https://user-images.githubusercontent.com/4254623/139871411-22094581-5f67-4417-81dd-c9d3a2ad4477.png)



## Usage

1. Follow the basic installation steps here: https://github.com/StrayRobots/StrayPublic
2. Install specific requirements for the picking example into your current python environment by running `pip install -r requirements.txt` in this directory
3. Connect your RealSense sensor to the computer via USB
4. Start the prediction pipeline with `python pick.py <MODEL_DIR> --json-config <PATH_TO_CONFIG> --reset-sleep <RESET_SLEEP> --visualize <VISUALIZATION_TYPE> --z <PICK_HEIGHT> --minimum-confidence <MINIMUM_CONFIDENCE> --depth-box-scale <DEPTH_BOX_SCALE> --hand-eye <HAND_EYE>`
    - `<MODEL_DIR>` is the path to the provided prediction model directory
    - `--json-config` is the path to the saved realsense settings, defaults to the file in `StrayPublic/configs/rsconfig.json`
    - `--reset-sleep` is the time to sleep after resetting the camera settings, defaults to 5 seconds
    - `--visualize` specifies the visualization type, can be either `3d` or `images`
    - `--z` specifies the known pick height of the object from the camera, can be used as an alternative to depth predictions
    - `--minimum-confidence` filters detections with a lower confidence than the specified minimum
    - `--depth-box-scale` determines how big the bounding box should be compared to the original that is used for depth estimation


## Notes
* Pick point predictions can be made based on both depth estimates (default) and based on a known pick height (specified with `--z`)
* Before using the depth predictions, the bounding boxes on the depth images should be verified that they are fully inside predicted objects (using `--visualize images`), if the boxes include parts outside of the object, you should consider using a smaller `--depth-box-scale`
* Before using the 3d points in a downstream task, verify that the predictions are approximately on an even place with `--visualize 3d`. The visualization blocks the execution, you need to close the window to get the next prediction
