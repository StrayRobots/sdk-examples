
# Box Picking

This example shows how to build datasets for picking and placing cardboard boxes within a [RoboDK](https://robodk.com/) simulation environment.

The purpose of this simulation is to showcase how computer vision can be used in dynamic pick and place applications. In this case, our robot is tasked to move boxes rolling on one conveyor belt to the other.

The robot waits for boxes to come down the conveyor belt, where it detects the top corners of the box. We then compute a picking location at the center top of the cardboard box and command the robot to pick it up with its suction cup gripper. The robot then moves the box over to the other conveyor belt and starts over.


## Installing dependencies

We recommend using [Anaconda](https://docs.anaconda.com/anaconda/) for package management. To create a new environment and install the dependencies, run:
```
conda create -n robodk Python=3.8 && conda activate robodk
pip install -r requirements.txt
```

## Running the picking simulation with a trained model

First open the simulation in the RoboDK UI by opening RoboDK, then select File > Open... and open the `picking_setup.rdk` file.

Start the simulation and picking script by running `python pick.py --model model.pts`.

You should now see the simulation running with the robot picking boxes and moving them over.

## Collecting a dataset and training an object detector

As is common these days, the object detection algorithm we use is learning based. To train this algorithm, we need to collect example data from the robots workspace and annotate it to teach our robot to recognize the objects we want to pick.

To collect data, we provide the script `scan.py` which runs the simulation and can be triggerred to stop the production line and scan the current state of the line. It is run with while the picking simulation is open in the RoboDK UI:
```
python scan.py --out scans
```

To stop the line and scan the conveyor belt, press the `s` key.

The scans are saved in the path given to the `--out` parameter. For each performed scan, a subdirectory will be created within that directory which will contain the captured color and depth images, along with the camera poses.

After scanning, we need to process the scans to compute the 3D representation from the captured camera and depth images. This is done using the `stray studio integrate` command. Run it with:
```
stray studio integrate scans/ --skip-mapping
```

As in this case, our camera is mounted on our calibrated robot, we use `--skip-mapping` parameter to tell the system that we know the camera poses and that these do not have to be inferred.

We then annotate each of the scans by opening the up in the Stray Studio user interface. To open a scan, run:
```
stray studio open scans/0001
```


