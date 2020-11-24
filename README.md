# motion_capture
Objective: Replace Optitrack Motion Capture System with RGB camera

The code is modified based on the MATLAB exmaple code motion-based tracking algorithm.
Adjusments were made to better serve our goal of replacing Optitrack motion capture system.
The code is able to track the object and predict generally next movements with its corsponding
[x, y] position on the screen. Then with the conversion from image frame to world frame, the
exact location of the object will be able to obatined.

## Getting Started
Follow the steps below in order to run the algorithm.

### Prerequisites
You will need a camera that can connect to your computer.

### Run the code
1. Calibrate camera
- In order to correctly predict object information, a calibration is required
- Follow the instructions here: https://www.mathworks.com/help/vision/ug/single-camera-calibrator-app.html
2. Run the extrinsic_calibration.m
- This file is to calibrate the location of camera in algorithm's sense
- You should see a 3D image of estimated camera position in the physical world
3. Run the motion_capture.m
- You should see two videos with processed box
- [x, y] coordinates are provided during the processing
