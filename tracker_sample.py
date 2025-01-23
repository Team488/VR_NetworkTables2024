import time


start = time.time()

def __sleep(interval):       
    sleep_time = interval-(time.time()-start)
    if sleep_time>0:
        time.sleep(sleep_time)


# Collect a pose from the tracker, but not more often than the specified time interval
def collect_sample(tracker, interval, verbose=False):
    global start
    
    # Ensure we don't overcollect samples if user is calling this repeatedly.
    __sleep(interval)
    if verbose: 
        print("collecting sample")

    pose = tracker.get_pose_euler()
    start = time.time()
    while not pose:
        if verbose:
            print("missed sample. collecting again.")
        pose = tracker.get_pose_euler()
        __sleep(interval)
        start = time.time()

    if verbose:
        print("collected sample: ", str(pose))

    return pose 

# Collect the horizontal coordinates of the tracker. 
# The x and z axes are the horizontal coordinates when the tracker is 
# oriented with the mounting screw hole facing down.
def collect_position(tracker, interval, verbose = False):
      x, y, z, roll, pitch, yaw = collect_sample(tracker, interval, verbose)
      return (x, z)