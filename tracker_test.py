import triad_openvr
import time
import sys
import ntcore
from wpimath.geometry import Pose2d

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


if len(sys.argv) == 1:
    interval = 1/250
elif len(sys.argv) == 2:
    interval = 1/float(sys.argv[1])
else:
    print("Invalid number of arguments")
    interval = False
    
if interval:
    while(True):
        start = time.time()
        txt = ""
        pose = v.devices["tracker_1"].get_pose_euler()
        if pose:
            x, y, z, roll, pitch, yaw = pose
            # Pitch corresponds to rotation around flat bottom of tracker.
            # rotation from -180 to +180  degrees
            wpiPose = Pose2d(angle=pitch, x = x, y = z)
            posePub.set(wpiPose)
            print (wpiPose)
        else:
            print("dropped")
        sleep_time = interval-(time.time()-start)
        if sleep_time>0:
            time.sleep(sleep_time)