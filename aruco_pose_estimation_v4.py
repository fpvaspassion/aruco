#   Manual offboard with Aruco marker
#   Steps:
#         1. Wait for PX4 hearbeat
#         2. Look for mark
#         3. When mark visible - start to send Vision_position_estimate
#         4. When LPOS is received from PX4 - start to send offboard correction
#
#       process_mavlink() -processing incoming messages
#       process_camera() - process images from camera and send Vision_poistion_estimate message
#       do_correction() - send LPOS correction to PX4

import numpy as np
import cv2
import cv2.aruco as aruco
import sys, time, math, os

from optparse import OptionParser
from pymavlink import mavutil
from datetime import datetime
from datetime import timedelta

sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), '../mavlink'))

class Target(object):
    def __init__(self, x, y, z, yaw):
         self.x = x
         self.y = y
         self.z = z
         self.yaw = yaw

### State machine definitions
State_waiting_px       = 1
State_waiting_mark     = 2
State_waiting_lpos     = 3
State_correcting       = 5 

### Defines and params
id_to_find = 77
dictionnary_to_use = aruco.DICT_4X4_100
marker_size  = 20 ### [cm]
calib_path  = "./camera_02/"
calibration_file = 'cameraMatrix.txt'
distortion_file = 'cameraDistortion.txt'
camera_frame_width = 640
camera_frame_heigh = 480
yaw_mark = 0 ### inital value greater then possible 5 , changed to 0 to debug
yaw_mark_detected = False 
observation_start = 0
small_correcting = False # if False - send final target

### Variables
start_boot = datetime.now()
first_loop = True
yaw_camera = 0
marker_visible = False
lpos_received_time = 0
app_should_stop = False
copter_mode = 0
# Landing precision parameters
x_precision = 0.1
y_precision = 0.1
z_precision = 0.25
yaw_precision = 0.2

### define landing target
prelanding_target = Target(-1.5,0.1,-0.1,0.01)

###CONSTATNS
MIN_OBSERVATION_PERIOD = 200   #millis
LPOS_OBSERVATION_MILLIS = 8000 #millis
COPTER_SYS_ID = 1
MAX_OBSERVATION_LOST_MILLIS = 200

#define position targew for offbord mode - for first time use it only for Y-axis
LPOS_TYPE_MASK = 0x4 and 0x38 and 0x1C0 and 0x400 and 0x800 and 0x3000

### Setup state variables
curr_state = prev_state = State_waiting_px

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
    north   = -1 * (x_uav*c - y_uav*s)
    east    = x_uav*s + y_uav*c 
    return(north, east)

def delta_millis(start_time):
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

### Handle incomig MAVLink messages from PX4
def handle_attitude(msg):
         global attitude_mav, yaw_mark_detected, yaw_mark, yaw_cam, opts

         attitude_mav = (msg.roll, msg.pitch, msg.yaw, msg.rollspeed, 
				msg.pitchspeed, msg.yawspeed)
         ### Show reseived information
         if opts.showmessages and False:
              print ("MSG type= ATTITUDE")
              print "Roll\tPit\tYaw\tRSpd\tPSpd\tYSpd"
              print "%0.4f\t%0.4f\t%0.4f\t%0.4f\t%0.4f\t%0.4f\t" % attitude_mav
 #         ### if first loop - clculate yaw for mark
 #        if yaw_mark_detected and yaw_mark == 5:
 #             yaw_mark = attitude_mav[2] - yaw_cam
 #             yaw_mark_detected = True
 #             if opts.showmessages:
 #                  print ("Copter yaw        = %4.2f" % attitude_mav[2])
 #                  print ("Cam yaw           = %4.2f" % yaw_cam)              
 #                  print ("Mark yaw detected = %4.2f" % yaw_mark)

def handle_hud(msg):
         global altitude_amsl, altitude_amsl_updated
         hud_data = (msg.airspeed, msg.groundspeed, msg.heading, 
				msg.throttle, msg.alt, msg.climb)
         if opts.showmessages and False:
              print ("MSG type= VFR_HUD")
              print "Aspd\tGspd\tHead\tThro\tAlt\tClimb"
              print "%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f" % hud_data
		
def handle_heartbeat(msg):
	global copter_mode, is_armed
        copter_mode = mavutil.mode_string_v10(msg)
	is_armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
	#is_enabled = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED
        if opts.showmessages:
              print("MAV mode=%s, Armed=%d" % (copter_mode, is_armed))
	  
def handle_lpos(msg):
         global altitude_amsl, altitude_amsl_updated, lpos_received_time, lpos_data
         lpos_data = (msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz)
         # We check if is position LPOS or only Altitude 
         if msg.x<>0 and msg.y<>0:
              lpos_received_time = datetime.now() 
              if opts.showmessages and False:
                   print ("MSG type= LPOS")
                   print "X\tY\tZ\tVx\tVy\tVz"
                   print "%0.4f\t%0.4f\t%0.4f\t%0.4f\t%0.4f\t%0.4f" % lpos_data
         else:
              if opts.showmessages:
                   print ("LPOS - NO POSITION!")

def process_mavlink():
    global master, msg, msg_type
    have_data = True
    # read messages
    while have_data:
         msg = master.recv_match(blocking=False)
         if msg:
              # handle the message based on its type
              msg_type = msg.get_type()
              #print ("MSG type=" + msg_type)
              if msg_type == "BAD_DATA":
                   if mavutil.all_printable(msg.data):
                        sys.stdout.write(msg.data)
                        sys.stdout.flush()
              elif msg_type == "ATTITUDE":
                   handle_attitude(msg)
              elif msg_type == "HEARTBEAT":
                   handle_heartbeat(msg)    	
              elif msg_type == "VFR_HUD":
                   handle_hud(msg)    	
              elif msg_type == "LOCAL_POSITION_NED":
                   handle_lpos(msg)
         else:
              have_data = False  

def copter_change_mode(new_base_mode, new_custom_mode = 0):
    master.mav.set_mode_send(COPTER_SYS_ID, int(new_base_mode), int(new_custom_mode))

def process_camera():
    global cap, master, first_loop, opts, yaw_copter, marker_visible, app_should_stop, start_time, pos_camera_cm, observation_start, observation_lost
    ### Reset visibility flag
    marker_visible = False

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
	 marker_visible = True
         if observation_start == 0:
              observation_start = datetime.now()
         observation_lost = datetime.now()
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
         ### Show information
         #if opts.showmessages:
         #     print ("Att pitch=%4.2f yaw=%4.2f roll=%4.2f" % (pitch_cam, yaw_cam, roll_cam))
	 if not first_loop :
              if True:  #yaw_mark_detected and altitude_amsl_updated:
                   ### Calc copter yaw angle
                   yaw_copter= yaw_mark + yaw_cam
		   ### Convert to NED and meters and include copter baro alt
		   p_x_cm, p_y_cm = uav_to_ne(pos_camera_cm[2], pos_camera_cm[0], yaw_copter)
                   p_z_m = -0.01 * pos_camera_cm[1]
                   ### Send data to FC position is NED
		   master.mav.vision_position_estimate_send(delta_time, p_x_cm * 0.01, p_y_cm * 0.01, p_z_m, roll_cam, pitch_cam, yaw_copter)
                   if opts.showmessages and False:
                        print("Mark X=%4.2f Y=%4.2f Z=%4.2f Sent X=%4.2f Y=%4.2f Z=%4.2f Yaw_mark=%4.2f Yaw_mav=%4.2F Yaw_copter=%4.2f" % (pos_camera_cm[0], pos_camera_cm[2], pos_camera_cm[1], p_x_cm, p_y_cm, p_z_m, yaw_mark, attitude_mav[2], yaw_copter))
         else:
              first_loop = False

    ### Reset observation start if mark not found
    if not marker_visible:
         observation_start = 0
         observation_lost = datetime.now()
		 
    ### Display the frame
    if opts.showvideo:	
         cv2.imshow('frame', frame)
         ### use 'q' to quit
         key = cv2.waitKey(1) & 0xFF
         if key == ord('q'):
              cap.release()
              cv2.destroyAllWindows()
              app_should_stop = True

def correct_lpos():
    global lpos_data, attitude_mav

    if opts.showmessages:
         print("Correcting LPOS")
    
    ### Calculate deltas
    delta_x = prelanding_target.x - lpos_data[0]
    delta_y = prelanding_target.y - lpos_data[1]
    delta_z = prelanding_target.z - lpos_data[2]
    delta_yaw = prelanding_target.yaw - attitude_mav[2]

    ###Calculate corrected position if flag small correction is checked-  we approach to target slowly
    if small_correcting:
         if delta_x > x_precision:
              new_x_m = lpos_data[0] + delta_x * 0.1
         else:
              new_x_m = lpos_data[0] + delta_x
         if delta_y > y_precision:
              new_y_m = lpos_data[1] + delta_y * 0.1
         else:
              new_y_m = lpos_data[1] + delta_y
         if delta_z > z_precision:
              new_z_m = lpos_data[2] + delta_z * 0.1
         else:
              new_z_m = lpos_data[2] + delta_z
         if delta_yaw > yaw_precision:
              new_yaw = attitude_mav[2] + delta_yaw * 0.1
         else:
              new_yaw = attitude_mav[2] + delta_yaw
    else:
         new_x_m = lpos_data[0] + delta_x
         new_y_m = lpos_data[1] + delta_y
         new_z_m = lpos_data[2] + delta_z
         new_yaw = attitude_mav[2] + delta_yaw

    ### Prepare message components
    nm_time_boot_ms = delta_millis(start_boot)
    nm_sys_id = COPTER_SYS_ID
    nm_coordinate_frame = mavutil.mavlink.MAV_FRAME_LOCAL_NED
    nm_type_mask = LPOS_TYPE_MASK
    nm_x = new_x_m
    nm_y = new_y_m
    nm_z = new_z_m
    #nm_yaw = new_yaw
    nm_yaw = -1 * math.atan2(lpos_data[1], abs(lpos_data[0]))

    if opts.showmessages:
         print("Target =X=%4.2f Y=%4.2f Z=%4.2f Yaw=%4.2f" % (nm_x, nm_y, nm_z, nm_yaw) )

    ### Send message with following full format
    #master.mav.set_position_target_local_ned_send(nm_time_boot_ms, nm_sys_id, nm_sys_comp, nm_coordinate_frame, nm_type_mask, nm_x, nm_y, nm_z, nm_vx, nm_vy, nm_vz, nm_afx, nm_afy, nm_afz, nm_yaw, nm_yawrate)
    master.mav.set_position_target_local_ned_send(nm_time_boot_ms, nm_sys_id, 0, nm_coordinate_frame, nm_type_mask, nm_x, nm_y, nm_z, 0, 0, 0, 0, 0, 0, nm_yaw, 0)

def check_for_landing():
     global lpos_data, attitude_mav, landing_target
     x_ok = abs(prelanding_target.x - lpos_data[0]) < x_precision
     y_ok = abs(prelanding_target.y - lpos_data[1]) < y_precision
     z_ok = abs(prelanding_target.z - lpos_data[2]) < z_precision
     yaw_ok = abs(prelanding_target.yaw - attitude_mav[2]) < yaw_precision
     if ( x_ok and y_ok and z_ok and yaw_ok ):
         if opts.showmessages:
              print ("OK for landing")
         setState(State_landing)
	
def setState(new_state):
    global curr_state, prev_state
    if curr_state == State_waiting_mark:
         ### do something		 
         if opts.showmessages:
              print("Call of setState p1")
    elif curr_state == State_waiting_lpos:
         #if new_state == State_waiting_offboard:
         #     copter_change_mode(mavutil.mavlink.MAV_MODE_GUIDED_DISARMED)
         ### do something
         if opts.showmessages:
              print("Call of setState waiting LPOS")
    elif curr_state == State_correcting:
         ### do something
         if opts.showmessages:
              print("Call of setState correcting")
    #Set state for next iterratoins
    curr_state = new_state
    print("State changed, NEW STATE=", curr_state)

def waiting_mark():
    global curr_state, cap, master, first_loop, opts, yaw_copter, msg, msg_type, marker_visible
    ### process communication with PX4
    process_mavlink()
    ### process video
    process_camera()
    ### If observation is stable - switch to next state
    if marker_visible and delta_millis(observation_start) > MIN_OBSERVATION_PERIOD :
         setState(State_waiting_lpos)
	
def waiting_lpos():
    global curr_state, cap, master, first_loop, opts, yaw_copter, msg, msg_type, marker_visible
    ### process communication with PX4
    process_mavlink()
    ### process video
    process_camera()
    ### Check for switch to next state
    if lpos_received_time:
         if  delta_millis(lpos_received_time) < LPOS_OBSERVATION_MILLIS:
    	      setState(State_correcting)
		 
def do_correcting():
    global curr_state, cap, master, first_loop, opts, yaw_copter, msg, msg_type, marker_visible, lpos, attitude_mav, landing_target, observation_lost
    ### process communication with PX4
    process_mavlink()
    ### process video
    process_camera()
    ### calc correction
    correct_lpos()
    ### Reset mode if no mark
    if  delta_millis(lpos_received_time) > LPOS_OBSERVATION_MILLIS:
         setState(State_waiting_lpos)
         copter_change_mode(mavutil.mavlink.MAV_MODE_MANUAL_ARMED, 0)  
    ### If observation is stable - switch to next state
    if delta_millis(observation_lost) > MAX_OBSERVATION_LOST_MILLIS :
         setState(State_waiting_lpos)


#x_cc = -1.37
#y_cc = 0.44
#new_yaw = -1 * math.atan2(y_cc, abs(x_cc))
#print("Yaw = %f" % new_yaw)
#while True:
#    abc = 324



### Get the camera calibration path
camera_matrix = np.loadtxt(calib_path + calibration_file, delimiter=',')
camera_distortion = np.loadtxt(calib_path + distortion_file, delimiter=',')

### Define the aruco dictionary
aruco_dict = aruco.getPredefinedDictionary(dictionnary_to_use)
parameters = aruco.DetectorParameters_create()

### Capture the videocamera (this may also be a video or a picture)
cap = cv2.VideoCapture(0)
### Set the camera size as the one it was calibrated with
cap.set(cv2.CAP_PROP_FRAME_WIDTH, camera_frame_width)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, camera_frame_heigh)

### Font for the text in the image
#font = cv2.FONT_HERSHEY_PLAIN

### read command line options
parser = OptionParser("readdata.py [options]")
parser.add_option("--baudrate", dest="baudrate", type='int', help="master port baud rate", default=115200)
parser.add_option("--device", dest="device", default=None, help="serial device")
parser.add_option("--rate", dest="rate", default=4, type='int', help="requested stream rate")
parser.add_option("--source-system", dest='SOURCE_SYSTEM', type='int', default=255, help='MAVLink source system for this GCS')
parser.add_option("-v", "--verbose", action="store_true", dest="showmessages", default=False, help="Print status messages to stdout")
parser.add_option("-s", "--show", action="store_true", dest="showvideo", default=False, help="Show video from camera to frame")
parser.add_option("--distance", dest="distance", default=None, help="landing distance from aruco mark")

(opts, args) = parser.parse_args()

if opts.device is None:
    print("You must specify a serial device")
    sys.exit(1)

if opts.showmessages:
    print ("Starting with display messages")

### create a mavlink serial instance
master = mavutil.mavlink_connection(opts.device, baud=opts.baudrate)

### wait for the heartbeat msg to find the system ID
if opts.showmessages:
    print "Waiting for PX4 heartbeat..."
master.wait_heartbeat()

# Switch to next state
setState(State_waiting_mark)

### Alert of start
if opts.showmessages:
    print "Starting main loop"
###
start_time = datetime.now()

### Fill previous camera attitude by zeros
prev_attitude = attitude

### Main loop
while True:
    ### Print timestamp
    delta_time= delta_millis(start_time)    
    if opts.showmessages:
         print ("T delta=%d  Mode=%s" % (delta_time, curr_state))
    ###
    start_time = datetime.now()

    if curr_state == State_waiting_mark:
         waiting_mark()
    elif curr_state == State_waiting_lpos:
	 waiting_lpos()
    elif curr_state == State_correcting:
	 do_correcting()
    ##Check for exit from application flag
    if app_should_stop == True:
         break
