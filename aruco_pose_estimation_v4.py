import numpy as np
import cv2
import cv2.aruco as aruco
import sys, time, math, os

from optparse import OptionParser
from pymavlink import mavutil
from datetime import datetime
from datetime import timedelta

sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), '../mavlink'))

### Define Tag
id_to_find  = 77
dictionnary_to_use = aruco.DICT_4X4_100
marker_size  = 10 ### [cm]
calib_path  = "./camera_01/"
calibration_file = 'cameraMatrix.txt'
distortion_file = 'cameraDistortion.txt'
camera_frame_width = 640
camera_frame_heigh = 480
yaw_mark = 5 ### inital value greater then possible
yaw_mark_detected = False 

### Variables
start_time = datetime.now()
first_loop = True
yaw_camera = 0

### 180 deg rotation matrix around the x axis
R_flip  = np.zeros((3,3), dtype=np.float32)
R_flip[0,0] = 1.0
R_flip[1,1] =-1.0
R_flip[2,2] =-1.0
### camera attitude matrix definition
attitude = np.zeros((3), dtype=np.float32)
attitude[0] = 0
attitude[1] = 0
attitude[2] = 0
### copter attitude from MAVLink
attitude_mav = np.zeros((6), dtype=np.float32)
pos_camera_cm = np.zeros((3), dtype=np.float32)

def marker_position_to_angle(x, y, z):
    angle_x = math.atan2(x,z)
    angle_y = math.atan2(y,z)
    return (angle_x, angle_y)
    
def camera_to_uav(x_cam, y_cam):
    x_uav =-y_cam
    y_uav = x_cam
    return(x_uav, y_uav)
    
def uav_to_ne(x_uav, y_uav, yaw_rad):
    c       = math.cos(yaw_rad)
    s       = math.sin(yaw_rad)
    north   = x_uav*c - y_uav*s
    east    = x_uav*s + y_uav*c 
    return(north, east)

def delta_millis():
    dt = datetime.now() - start_time
    ms = (dt.days * 24 * 60 * 60 + dt.seconds) * 1000 + dt.microseconds / 1000.0
    return ms

###------------------------------------------------------------------------------
###------- ROTATIONS https://www.learnopencv.com/rotation-matrix-to-euler-angles/
###------------------------------------------------------------------------------
### Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6

### Calculates rotation matrix to euler angles
### The result is the same as MATLAB except the order
### of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R):
    assert (isRotationMatrix(R))
    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6
    if not singular:
         x = math.atan2(R[2, 1], R[2, 2])
         y = math.atan2(-R[2, 0], sy)
         z = math.atan2(R[1, 0], R[0, 0])
    else:
         x = math.atan2(-R[1, 2], R[1, 1])
         y = math.atan2(-R[2, 0], sy)
         z = 0
    return np.array([x, y, z])

### Handle incomig Attitude message from PX4
def handle_attitude(msg):
         global yaw_mark_detected, yaw_mark, yaw_cam, opts
         attitude_mav = (msg.roll, msg.pitch, msg.yaw, msg.rollspeed, 
				msg.pitchspeed, msg.yawspeed)
         ### Show reseived information
         if opts.showmessages:
              print ("MSG type= ATTITUDE")
	      print "Roll\tPit\tYaw\tRSpd\tPSpd\tYSpd"
	      print "%0.4f\t%0.4f\t%0.4f\t%0.4f\t%0.4f\t%0.4f\t" % attitude_mav
         ### if first loop - clculate yaw for mark
         if yaw_mark_detected and yaw_mark == 5:
              yaw_mark = attitude_mav[2] + yaw_cam
              yaw_mark_detected = True
              if opts.showmessages:
                   print ("Copter yaw        = %4.2f" % attitude_mav[2])
                   print ("Cam yaw           = %4.2f" % yaw_cam)              
                   print ("Mark yaw detected = %4.2f" % yaw_mark)
             
### Get the camera calibration path
camera_matrix   = np.loadtxt(calib_path + calibration_file, delimiter=',')
camera_distortion   = np.loadtxt(calib_path + distortion_file, delimiter=',')

### Define the aruco dictionary
aruco_dict  = aruco.getPredefinedDictionary(dictionnary_to_use)
parameters  = aruco.DetectorParameters_create()

### Capture the videocamera (this may also be a video or a picture)
cap = cv2.VideoCapture(0)
### Set the camera size as the one it was calibrated with
cap.set(cv2.CAP_PROP_FRAME_WIDTH, camera_frame_width)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, camera_frame_heigh)

### Font for the text in the image
font = cv2.FONT_HERSHEY_PLAIN

### read command line options
parser = OptionParser("readdata.py [options]")
parser.add_option("--baudrate", dest="baudrate", type='int',
				  help="master port baud rate", default=115200)
parser.add_option("--device", dest="device", default=None, help="serial device")
parser.add_option("--rate", dest="rate", default=4, type='int', help="requested stream rate")
parser.add_option("--source-system", dest='SOURCE_SYSTEM', type='int',
				  default=255, help='MAVLink source system for this GCS')
parser.add_option("-v", "--verbose",
                  action="store_true", dest="showmessages", default=False,
                  help="Print status messages to stdout")

(opts, args) = parser.parse_args()

if opts.device is None:
    print("You must specify a serial device")
    sys.exit(1)

### create a mavlink serial instance
master = mavutil.mavlink_connection(opts.device, baud=opts.baudrate)

### wait for the heartbeat msg to find the system ID
if opts.showmessages:
    print "Waiting for PX4 heartbeat..."
master.wait_heartbeat()

### Alert of start
if opts.showmessages:
    print "Starting main loop"
### Get time of start
start_time = datetime.now()

### Fill previous camera attitude by zeros
prev_attitude = attitude
    
### Main loop
while True:
    # Process only mavlink message with attitude
    msg = master.recv_match(blocking=False)
    if msg:
         # handle the message based on its type
         msg_type = msg.get_type()
         # print ("MSG type=" + msg_type)
         if msg_type == "ATTITUDE":
              handle_attitude(msg)
    	
    ### Print timestamp
    delta_time= delta_millis()    
    if opts.showmessages:
         print ("Time delta=%d" % delta_time)
    start_time = datetime.now()
    
    ### Read the camera frame
    ret, frame = cap.read()

    ### Convert in gray scale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) #-- remember, OpenCV stores color images in Blue, Green, Red

    ### Find all the aruco markers in the image
    corners, ids, rejected = aruco.detectMarkers(image=gray, dictionary=aruco_dict, parameters=parameters,
                              cameraMatrix=camera_matrix, distCoeff=camera_distortion)
    ### Check for marker ID
    if ids is not None and ids[0] == id_to_find:
         ### alow to calculate real mark yaw
         yaw_mark_detected = True
         ### Parse markers
         ret = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)	
         ### Unpack the output, get only the first
         rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]
         ### Obtain the rotation matrix tag->camera
         R_ct = np.matrix(cv2.Rodrigues(rvec)[0])
         R_tc = R_ct.T
         ### Calc Position, Attitude and velocity respect to the marker
         if not first_loop :
              ### Keep previous values for velocity calculation
              prev_pos_camera_cm = pos_camera_cm
	      prev_attitude = attitude
         ### Position and attitude of the camera respect to the marker
         pos_camera_cm = -R_tc*np.matrix(tvec).T
         pitch_cam, yaw_cam, roll_cam = rotationMatrixToEulerAngles(R_flip*R_tc)
	 if not first_loop :
              if yaw_mark_detected:
                   ### Calc copter yaw angle
                   yaw_copter=attitude_mav[2]+yaw_mark
		   ### Convert to NED and meters
		   p_x_cm, p_y_cm = uav_to_ne(pos_camera_cm[0], pos_camera_cm[1], yaw_copter)
                   p_z_cm = pos_camera_cm[2]
                   ### Send data to FC position is NED
		   master.mav.vision_position_estimate_send(delta_time, p_x_cm * 0.01, p_y_cm * 0.01, p_z_cm * 0.01, roll_cam, pitch_cam, yaw_copter)	
         else :
              first_loop = False
    ### Display the frame
    if opts.showmessages:
         cv2.imshow('frame', frame)
         ### use 'q' to quit
         key = cv2.waitKey(1) & 0xFF
         if key == ord('q'):
              cap.release()
              cv2.destroyAllWindows()
              break