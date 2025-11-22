import sys
from wpimath.geometry import Pose2d,Rotation2d,Translation2d
from trackercal import CalibrateOptions
import tracker_sample
import triad_openvr
import math
import argparse
import time
from tracker_coordinate_transform import *
from tracker_sample import *
from vr_trackers_table import *
from trackers import Trackers
from numpy import array

heading_offset = 0
interval = 1/250 # 250 Hz time interval for sampling tracker data

#inst.setServer("localhost")
#inst.setServer("10.4.88.2")
#inst.setServer("127.0.0.1")

parser = argparse.ArgumentParser(prog='trackercal', description='A command line application for tracking and calculating events.')

default = CalibrateOptions()
# Add optional arguments
parser.add_argument('-v', '--verbose', action='store_true', help='Run the command in verbose mode.', default = default.verbose)
parser.add_argument('-s', '--samples', action='store', help='Number of samples to collect when creating the circle.', default = default.samples)
parser.add_argument('-d', '--di stance', action='store', help='Distance between samples to collect, in centimeters', default = default.distance)
parser.add_argument('-r', '--rate', action='store', help='Sampling rate of tracker.', default = default.rate)
parser.add_argument('-x', '--xOffset', action='store', help='offset the x coordinate transform by x amount', default = default.xOffset)
parser.add_argument('-y', '--yOffset', action='store', help='offset the y coordinate transform by y amount', default = default.yOffset)
parser.add_argument('-a', '--address', action='store', help='address of the robot to connect to', default = '127.0.0.1')
#10.4.88.2, drive computer's IP address, 488_2024Comp, local Machine ip address :127.0.0.1
parser.add_argument('-f', '--file', action='store', help='coordinate transform constants file to read', default = "")
parser.add_argument('-j', '--adjustToRobot', action = 'store_true', help='adjust tracker to robot position')
parser.add_argument('-o', '--offlineTest', action = 'store_true', help='test with the trackers offline (no trackers required)', default = default.offlineTest)
parser.add_argument('-z', '--adjustToZero', action = 'store_true', help='set current location as (0,0)', default = False)
parser.add_argument('-b', '--bluefield', action = 'store_true', help='calibrate blue field ', default = default.bluefield)
parser.add_argument('-e', '--redfield', action = 'store_true', help='calibrate red field ', default = default.redfield)
  
# Parse the arguments
args = parser.parse_args()

inst = VRTrackersTable(args.address)
table = inst.trackers_table

posePub_tracker_1 = table.getStructTopic("Tracker_1", Pose2d).publish()
posePub_tracker_2 = table.getStructTopic("Tracker_2", Pose2d).publish()
posePub_tracker_3 = table.getStructTopic("Tracker_3", Pose2d).publish()

R = [[1, 0], [0, 1]]    # default to zero rotation
s = 1.0                 # default to no scaling
t = [0.0, 0.0]          # default to no translation

# Get the calibration information from a previous calibration step that was saved to a file
# Default file name is "transform.txt"
if args.file != "":
    with open(args.file, 'r') as file:
        (R,s,t) = eval(file.read())

if interval:
    
    trackers = Trackers(configfile_path="./config.json")
    trackers.check_for_trackers(args.offlineTest)
  
    # Calibrate and save calibration information to the file "transform.txt"
    # Tracker 1 should be used for calibration
    
    calibrate_options_args = {key: value for key, value in vars(args).items() if key in CalibrateOptions.__init__.__code__.co_varnames}
    calibrate_options = CalibrateOptions(**calibrate_options_args)
        
    if args.bluefield:
        R,s,t = trackers.calibrate_blue(calibrate_options)
    elif args.redfield:
        R,s,t = trackers.calibrate_red(calibrate_options)

    else:
        if args.file == "":
            R,s,t = trackers.get_tracker_1_calibration(calibrate_options)

    # Save the transform calibration information to a file
    with open("transform.txt", "w") as file:
        file.write(str((R,s,t)))
   
    # initialize the translation offset to zero
    tx = 0
    ty = 0
    
    # Synchronize the pose position between the robot and the tracker
    if args.adjustToZero:
        # Set to zero for now TODO: make this variable
        # start line/upper wall blue side
        #xFRC_init = 7.56
        #yFRC_init = 8.05

         # april tag 18
        xFRC_init = 3.6576	
        yFRC_init = 4.0259
        heading_init = 0.0 # degrees
        
        cx = xFRC_init
        cy = yFRC_init 
        
        xFRC, yFRC, headingVR = trackers.get_tracker_1_pos(interval, R, s, t, True, args.offlineTest)
        
        # Calculate the offset to sync the tracker with the robot
        tx, ty = cx - xFRC, cy - yFRC
        if args.verbose:
            print("current robot position (cx,cy): ",cx, cy)
            print ("current tracker position (x,y): ",xFRC, yFRC)
            print ("offset to sync (tx,ty): ",tx, ty)
              
        # Calculate the rotation offset to sync the tracker with the robot
        heading_offset = headingVR - heading_init

        print("Hit Enter to continue")
        input()

    # Synchronize the pose position between the robot and the tracker
    if args.adjustToRobot:
        # Get the current robot pose, as self-reported via AdvantageKit, so that the tracker can be synchronized
        robotPoseSubX = inst.robot_pose_sub_x.subscribe(999.0)
        robotPoseSub = inst.robot_pose_table.getStructTopic("RobotPose", Pose2d).subscribe(Pose2d(999, 999, 999))
        time.sleep(5) # Wait for the robot pose to be published. TODO: experiment with this value
        robotPose = robotPoseSub.get()
        cx = robotPose.X() 
        cy = robotPose.Y() 
        
        xFRC, yFRC, headingVR = trackers.get_tracker_1_pos(interval, R, s, t, True, args.offlineTest)
        
        # Calculate the offset to sync the tracker with the robot
        tx, ty = cx - xFRC, cy - yFRC
        if args.verbose:
            print("current robot position (cx,cy): ",cx, cy)
            print ("current tracker position (x,y): ",xFRC, yFRC)
            print ("offset to sync (tx,ty): ",tx, ty)
              
        # Calculate the rotation offset to sync the tracker with the robot
        heading_offset = headingVR - robotPose.rotation().degrees()

        print("Hit Enter to continue")
        input()
        

    while True:
       # Update the tracker poses
       posePubs = [posePub_tracker_1, posePub_tracker_2, posePub_tracker_3]
       for i, pose in trackers.get_all_tracker_wpi_poses(interval, R, s, t, heading_offset, args.verbose, args.offlineTest):
           posePubs[i].set(pose)
