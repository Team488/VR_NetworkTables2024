import sys
import ntcore
from wpimath.geometry import Pose2d,Rotation2d,Translation2d
from trackercal import calibrate, CalibrateOptions
import tracker_sample
import triad_openvr
import math


import numpy as np
from icecream import ic
from tracker_coordinate_transform import *
 

inst = ntcore.NetworkTableInstance.getDefault()
table = inst.getTable("pose")

posePub = table.getStructTopic("pose", Pose2d).publish()

inst.setServer("localhost")
#inst.setServer("10.4.88.2")
#inst.setServer("127.0.0.1")
inst.startClient4("example client")

if len(sys.argv) == 1:
    interval = 1/250
elif len(sys.argv) == 2:
    interval = 1/float(sys.argv[1])
else:
    print("Invalid number of arguments")
    interval = False
    
if interval:
    v = triad_openvr.triad_openvr()
    if not "tracker_1" in v.devices:
        print("Error: unable to get tracker")
        exit(1)
 
    tracker= v.devices["tracker_1"]

    # tracker = tracker_sample.get_tracker()
    # if not tracker:
    #     print("Error: unable to get tracker")
    #     exit(1)

    trans, scale, rotation = calibrate(tracker, CalibrateOptions(verbose = True, rate = 1/interval, xOffset = -5, yOffset = 5))
    while True:
        # Pitch corresponds to rotation around flat bottom of tracker.
        # rotation from -180 to +180  degrees
        x, y, z, roll, pitch, yaw = tracker_sample.collect_sample(tracker, interval= interval, verbose=True)
        point = (x, z)  

        transformed_point = transform_point(point, trans, scale, 0)

        angle = Rotation2d.fromDegrees(-(pitch - rotation)) 
        translation = Translation2d(-transformed_point[0],transformed_point[1])
        wpiPose = Pose2d(translation,angle)
        posePub.set(wpiPose)
        print (wpiPose)
        