import sys
import ntcore
from wpimath.geometry import Pose2d,Rotation2d,Translation2d
from trackercal import calibrate, CalibrateOptions, calibrate_blue
import tracker_sample
import triad_openvr
import math
import argparse
import time
import numpy as np
from icecream import ic
from tracker_coordinate_transform import *
from tracker_sample import *
from numpy import array

def update_wpiPose(tracker,interval, R, s, t, tx, ty, verbose=False, offlineTest=False):
    # Get the current tracker position in FRC coordinates, and the heading in VR coordinates
    xFRC, yFRC, headingVR = get_current_tracker_position(tracker, interval, R, s, t, verbose=False, offlineTest=args.offlineTest)

    # Calculate the new angle value for the tracker pose using the tracker offset to match the initial robot heading
    pose_heading = Rotation2d.fromDegrees(heading_offset - headingVR)
    poseXY = Translation2d(xFRC + tx, yFRC + ty)
    wpiPose = Pose2d(poseXY, pose_heading)
    return wpiPose

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
  
# Parse the arguments
args = parser.parse_args()


inst = ntcore.NetworkTableInstance.getDefault()
inst.setServer(args.address)
inst.startClient4("VR_trackers")

table = inst.getTable("Trackers")

posePub_tracker_1 = table.getStructTopic("Tracker_1", Pose2d).publish()
posePub_tracker_2 = table.getStructTopic("Tracker_2", Pose2d).publish()
posePub_tracker_3 = table.getStructTopic("Tracker_3", Pose2d).publish()

R = [[1, 0], [0, 1]]    # default to zero rotation
s = 1.0                 # default to no scaling
t = [0.0, 0.0]          # default to no translation


heading_offset = 0
interval = 1/250 # 250 Hz time interval for sampling tracker data

# Get the calibration information from a previous calibration step that was saved to a file
# Default file name is "transform.txt"
if args.file != "":
    with open(args.file, 'r') as file:
        (R,s,t) = eval(file.read())

if interval:
    
    v = triad_openvr.triad_openvr()
    tracker_1, tracker_2, tracker_3 = check_for_trackers(v,args.offlineTest)
    
  
    # Calibrate and save calibration information to the file "transform.txt"
    # Tracker 1 should be used for calibration
    if args.bluefield:
    #    if (tracker_1==None) or (tracker_2==None) or (tracker_3==None):
    #     print("Error: unable to get all trackers")
    #     print("Make sure all trackers are turned on for calibration")
    #     print("Make sure all tracker USB dongles are plugged in to your PC")
    #     if not args.offlineTest:
    #         exit(1)
            
        calibrate_options_args = {key: value for key, value in vars(args).items() if key in CalibrateOptions.__init__.__code__.co_varnames}
        calibrate_options = CalibrateOptions(**calibrate_options_args)
        R,s,t = calibrate_blue(tracker_1, tracker_2, tracker_3, calibrate_options)

        # Save the transform calibration information to a file
        with open("transform.txt", "w") as file:
            file.write(str((R,s,t)))
    
    else:
        if args.file == "":
            if not "tracker_1" in v.devices:
                print("Error: unable to get tracker 1")
                print("Make sure Tracker 1 is turned on for calibration")
                print("Make sure the tracker 1 USB dongle is plugged in to your PC")
                if not args.offlineTest:
                    exit(1)
            calibrate_options_args = {key: value for key, value in vars(args).items() if key in CalibrateOptions.__init__.__code__.co_varnames}
            calibrate_options = CalibrateOptions(**calibrate_options_args)
            R,s,t = calibrate(tracker_1, calibrate_options)

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
        
        xFRC, yFRC, headingVR = get_current_tracker_position(tracker_1, interval, R, s, t, verbose=True,offlineTest=args.offlineTest)
        
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
        robotPoseTable = inst.getTable("/AdvantageKit/RealOutputs/PoseSubsystem")
        robotPoseSubX = robotPoseTable.getFloatTopic("/AdvantageKit/RealOutputs/PoseSubsystem/RobotPose/translation/x").subscribe(999.0)
        robotPoseSub = robotPoseTable.getStructTopic("RobotPose", Pose2d).subscribe(Pose2d(999, 999, 999))
        time.sleep(5) # Wait for the robot pose to be published. TODO: experiment with this value
        robotPose = robotPoseSub.get()
        cx = robotPose.X() 
        cy = robotPose.Y() 
        
        xFRC, yFRC, headingVR = get_current_tracker_position(tracker_1, interval, R, s, t, verbose=True,offlineTest=args.offlineTest)
        
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
        wpiPose_1 = update_wpiPose(tracker_1,interval, R, s, t, tx, ty, verbose=args.verbose, offlineTest=args.offlineTest)
        wpiPose_2 = update_wpiPose(tracker_2,interval, R, s, t, tx, ty, verbose=args.verbose, offlineTest=args.offlineTest)
        wpiPose_3 = update_wpiPose(tracker_3,interval, R, s, t, tx, ty, verbose=args.verbose, offlineTest=args.offlineTest)
        
        posePub_tracker_1.set(wpiPose_1)
        posePub_tracker_2.set(wpiPose_2)
        posePub_tracker_3.set(wpiPose_3)




    
   