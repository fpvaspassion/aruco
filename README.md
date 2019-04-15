# aruco
Odroid code for Aruco marker detection and communication with PX4 using MAVLink

To start application on PC  :
  sudo python aruco_pose_estimation_v4.py --device /dev/ttyUSB0 --baudrate 57600 --show --verbose
       Where 
             --device    - serial device communicating with PX4 - USB for PC, ttyS... 
             --baudrate  - communication speed
             --show      - display video frame ( stop app from this vindow with 'q' key)
             --verbose   - display debug messages in terminal

To start on Odroid :
  sudo python aruco_pose_estimation_v4.py --device /dev/ttyS1 --baudrate 57600

#test MAVlink
Test program to check MAVlink exchange with PX4

Typical use:
  sudo python test_mavlink.py --baudrate 57600 --device /dev/ttyUSB0
             --baudrate  - communication speed
             --device    - serial device communicating with PX4 - USB for PC, ttyS... 

