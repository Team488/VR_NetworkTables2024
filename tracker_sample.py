import time


start = time.time()
t_zero = time.time()

def __sleep(interval):       
    sleep_time = interval-(time.time()-start)
    if sleep_time>0:
        time.sleep(sleep_time)


# get and euler pose from the tracker or from an offline function, depending on the mode set by the offlineTest flag
def get_offline_pose():
    return (0, 0, 0, 0, 0, 0)

def get_pose(tracker, offlineTest=False):
    global t_zero
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
      x, y, z, roll, pitch, yaw = collect_sample(tracker, interval, verbose)
      return (x, z)