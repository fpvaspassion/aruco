# aruco
Odroid code for Aruco marker detection and communication with PX4

To start application - use following command sudo python aruco_pose_estimation_v4.py --device /dev/ttyUSB0 --baudrate 57600
Where device - serial device communicating with PX4 - USB for PC, ttyS... - for Odroid
 
For debugging needs can be used with option --verbose - it will show messages and camera frame
