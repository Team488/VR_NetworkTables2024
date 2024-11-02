import triad_openvr
import time
import sys
import ntcore
from wpimath.geometry import Pose2d,Rotation2d,Translation2d


import numpy as np
from icecream import ic
from tracker_coordinate_transform import *

v = triad_openvr.triad_openvr()
v.print_discovered_objects()
inst = ntcore.NetworkTableInstance.getDefault()
table = inst.getTable("pose")
# xPub = table.getDoubleTopic("x").publish()
# yPub = table.getDoubleTopic("y").publish()
# zPub = table.getDoubleTopic("z").publish()
# rollPub = table.getDoubleTopic("roll").publish()
# pitchPub = table.getDoubleTopic("pitch").publish()
# yawPub = table.getDoubleTopic("yaw").publish()

posePub = table.getStructTopic("pose", Pose2d).publish()

#inst.setServer("localhost")
inst.setServer("127.0.0.1")
inst.startClient4("example client")





def autoCalibrate():
    if not "tracker_1" in v.devices:
        print("tracker_1 not connected")
        exit(1)
    input()    

def calibrate():
    src_points = [(1.073488, -2.271037),(-0.387351, -1.991891),(1.363704, -0.797259)]
    dst_points = [(0, 0),(1.5, 0),(0, 1.5)]

    # Compute the transformation matrix
    transformation_matrix = ic(compute_transformation_matrix(src_points, dst_points))

    return transformation_matrix


if len(sys.argv) == 1:
    interval = 1/250
elif len(sys.argv) == 2:
    interval = 1/float(sys.argv[1])
else:
    print("Invalid number of arguments")
    interval = False
    
if interval:
    # transformation_matrix = calibrate()
    points = []
    oldPoint = (999,999)
    while(len(points) < 150):
        start = time.time()
        txt = ""
        pose = v.devices["tracker_1"].get_pose_euler()
        if pose:
            x, y, z, roll, pitch, yaw = pose
            # Pitch corresponds to rotation around flat bottom of tracker.
            # rotation from -180 to +180  degrees
            point = (x,z)
            if abs(point[1] - oldPoint[1]) > 0.05 or abs(point[0] - oldPoint[0]) > 0.05:
                points.append(point)
                oldPoint = point
            print(points)

          

            # transformed_point = ic(transform_point(point, transformation_matrix))

            # angle = Rotation2d.fromDegrees(pitch)
            # translation = Translation2d(transformed_point[0],transformed_point[1])
            # wpiPose = Pose2d(translation,angle)
            # # TODO: Add transformation matrix to map coordinates to FRC field
            # # TODO: Add Pose2D angle offset to map tracker rotation angle to robot's FRC field angle
            # posePub.set(wpiPose)
           # print (wpiPose)
        else:
            print("dropped")
        sleep_time = interval-(time.time()-start)
        if sleep_time>0:
            time.sleep(sleep_time)