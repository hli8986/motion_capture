# motion_capture
Replace Optitrack Motion Capture System with RGB camera

The code is modified based on the MATLAB exmaple code motion-based tracking algorithm.
Few adjusment were made to fit in the requirements of the hardware.
The code is able to track the object and predict generally next movements with its corsponding
[x, y] position on the screen. Then with the conversion from image frame to world frame, the
exact location of the object will be able to obatined.
