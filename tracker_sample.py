import time
import math

start = time.time()
t_zero = time.time()

def __sleep(interval):       
    sleep_time = interval-(time.time()-start)
    if sleep_time>0:
        time.sleep(sleep_time)


def continuous_pose_response(t, radius, x_offset=0, y_offset=0, heading_offset=0, frequency=0.5):
    # Calculate both the position and the orientation of the tracker assuming that the tracker is traveling in a circle
    #  at radius r. Include support for an initial x,y offset for the center of the circle,
    #  and an initial angle offset for the orientation of the tracker.
    # For each time step, keep the orientation pointing at the center of the circle.
    # The heading offset is in degrees.
    # Return the x, y values in meters and the heading in degrees.
    period = 1 / frequency  # seconds
    angle = heading_offset + 2 * math.pi * frequency * t
    x = x_offset + radius * math.cos(angle)
    y = y_offset + radius * math.sin(angle)
    heading = (180 / math.pi * math.atan2(y - y_offset, x - x_offset))-heading_offset # degrees

    return x,y, heading


# get and euler pose from the tracker or from an offline function, depending on the mode set by the offlineTest flag
def get_offline_pose():
    # return a continuous pose response at a frequency of 0.5 Hz, using t_zero as the time reference. 
    # Calculate both the position and the orientation of the tracker assuming that the tracker is traveling in a circle
    # at radius r. Include support for an initial x,y offset for the center of the circle, 
    # and an initial angle offset for the orientation of the tracker.
    # For each time step, keep the orientation pointing at the center of the circle.
    global t_zero

    t = time.time() - t_zero
    frequency = 0.5
    r = 2.0
    x_offset = 2.5
    y_offset = -5.0
    heading_offset = 90.0
    x,y, heading = continuous_pose_response(t, r, x_offset, y_offset, heading_offset, frequency)

    # Debug using contant values if true
    constant = False
    if constant:
        x = 2.5
        y = -5.0
        heading = 0

    # Mapping from the tracker pose to the robot pose
    # tracker pose order is (x, y, z, roll, pitch, yaw)
    # x maps to -x
    # z maps to y
    # pitch maps to heading
    pose = (-x, 0, y, 0, heading, 0)
    
    return pose

def get_pose(tracker, offlineTest=False):
    # x,y,z location and the appropriate Euler angles (in degrees)
    if offlineTest:
        pose = get_offline_pose()
    else:
        pose = tracker.get_pose_euler()
    return pose
    
# Collect a pose from the tracker, but not more often than the specified time interval
def collect_sample(tracker, interval, verbose=False, offlineTest=False):
    global start, t_zero
    
    # Ensure we don't overcollect samples if user is calling this repeatedly.
    __sleep(interval)
    if verbose: 
        print("collecting sample")

    pose = get_pose(tracker, offlineTest)
    start = time.time()
    while not pose:
        if verbose:
            print("missed sample. collecting again.")
        pose = get_pose(tracker, offlineTest)
        __sleep(interval)
        start = time.time()

    if verbose:
        print("collected sample: ", str(pose))

    return pose 

# Collect the horizontal coordinates of the tracker. 
# The x and z axes are the horizontal coordinates when the tracker is 
# oriented with the mounting screw hole facing down.
def collect_position(tracker, interval, verbose = False, offlineTest=False):
      x, y, z, roll, pitch, yaw = collect_sample(tracker, interval, verbose,offlineTest)
      return (x, z)

def check_for_trackers(v, offlineTest):
    trackers = ["tracker_1", "tracker_2", "tracker_3"]
    found_trackers = {}

    for tracker in trackers:
        if tracker in v.devices:
            print(f"Success: {tracker} found")
            found_trackers[tracker] = v.devices[tracker]
        else:
            print(f"Note: unable to get {tracker}")
            found_trackers[tracker] = None

    if all(tracker is None for tracker in found_trackers.values()):
        print("Error: no trackers found")
        if not offlineTest:
            exit(1)
    

    return found_trackers["tracker_1"], found_trackers["tracker_2"], found_trackers["tracker_3"]



