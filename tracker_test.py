import sys
import ntcore
from wpimath.geometry import Pose2d,Rotation2d,Translation2d
from trackercal import calibrate, CalibrateOptions
import tracker_sample
import triad_openvr
import math
import argparse
import time
import numpy as np
from icecream import ic
from tracker_coordinate_transform import *


#inst.setServer("localhost")
#inst.setServer("10.4.88.2")
#inst.setServer("127.0.0.1")

parser = argparse.ArgumentParser(prog='trackercal', description='A command line application for tracking and calculating events.')

default = CalibrateOptions()
# Add optional arguments
parser.add_argument('-v', '--verbose', action='store_true', help='Run the command in verbose mode.', default = default.verbose)
parser.add_argument('-s', '--samples', action='store', help='Number of samples to collect when creating the circle.', default = default.samples)
parser.add_argument('-d', '--distance', action='store', help='Distance between samples to collect, in centimeters', default = default.distance)
parser.add_argument('-r', '--rate', action='store', help='Sampling rate of tracker.', default = default.rate)
parser.add_argument('-x', '--xOffset', action='store', help='offset the x coordinate transform by x amount', default = default.xOffset)
parser.add_argument('-y', '--yOffset', action='store', help='offset the y coordinate transform by y amount', default = default.yOffset)
parser.add_argument('-a', '--address', action='store', help='address of the robot to connect to', default = '127.0.0.1')
parser.add_argument('-f', '--file', action='store', help='calibration file to read', default = "")
parser.add_argument('-j', '--adjustToRobot', action = 'store_true', help='adjust tracker to robot position')
# Parse the arguments
args = parser.parse_args()


inst = ntcore.NetworkTableInstance.getDefault()
inst.setServer(args.address)
inst.startClient4("VR_trackers")

table = inst.getTable("Tracker_1")

posePub = table.getStructTopic("Tracker_1", Pose2d).publish()

trans = (0, 0)
scale = (1, 1)
rotation = 0
interval = 1/250

# Get the calibration information from a previous calibration step that was saved to a file
# Default file name is "transform.txt"
if args.file != "":
    with open(args.file, 'r') as file:
        trans, scale, rotation = eval(file.read())
doCalibrate = False  

if interval:
    
    v = triad_openvr.triad_openvr()
    if not "tracker_1" in v.devices:
        print("Note: unable to get tracker_1")
        tracker_1_found = False
    else:
        print("Success: tracker_1 found")
        tracker_1_found = True
        
    if not "tracker_2" in v.devices:
        print("Note: unable to get tracker_2")
        tracker_2_found = False
    else:
        print("Success: tracker_2 found")
        tracker_2_found = True
        
    if (not tracker_1_found) and (not tracker_2_found):
        print("Error: no trackers found")
        exit(1)
    print("Hit Enter to continue")
    input()

    tracker_1= v.devices["tracker_1"]

    # Calibrate and save calibration information to the file "transform.txt"
    # Tracker 1 should be used for calibration
    if args.file == "":
        trans, scale, rotation = calibrate(tracker_1, CalibrateOptions(args))
        with open("transform.txt", "w") as file:
            file.write(str((trans,scale,rotation)))
    rx = 0
    ry = 0
    tx = 0
    ty = 0
    
    # Synchronize the pose position between the robot and the tracker
    if args.adjustToRobot:
        # Get the current robot pose so that the tracker can be synchronized
        robotPoseTable = inst.getTable("/AdvantageKit/RealOutputs/PoseSubsystem")
        robotPoseSubX = robotPoseTable.getFloatTopic("/AdvantageKit/RealOutputs/PoseSubsystem/RobotPose/translation/x").subscribe(999.0)
        robotPoseSub = robotPoseTable.getStructTopic("RobotPose", Pose2d).subscribe(Pose2d(999, 999, 999))
        time.sleep(5)
        robotPose = robotPoseSub.get()
        cx = robotPose.X() 
        cy = robotPose.Y() 
        print(cx, cy)
        x, y, z, roll, pitch, yaw = tracker_sample.collect_sample(tracker_1, interval= interval, verbose=True)
        cx = robotPose.X() 
        cy = robotPose.Y() 
        print(cx, cy)
        tx, ty = cx - x, cy - y
        print (tx, ty)
        print (x, y)
        
        
        rotation = pitch - robotPose.rotation().degrees()

        print("Hit Enter to continue")
        input()
        #trans = tx, ty


    while True:
        # Pitch corresponds to rotation around flat bottom of tracker.
        # rotation from -180 to +180  degrees
        x, y, z, roll, pitch, yaw = tracker_sample.collect_sample(tracker_1, interval= interval, verbose=True)
        #point = (x, z)  

        #transformed_point = transform_point(point, trans, scale, 0)

        angle = Rotation2d.fromDegrees(-(pitch - rotation)) 
        translation = Translation2d(x + tx, -(z + ty))
        wpiPose = Pose2d(translation,angle)
        posePub.set(wpiPose)
        print (wpiPose)
        