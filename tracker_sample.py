import time


start = time.time()

def __sleep(interval):       
    sleep_time = interval-(time.time()-start)
    if sleep_time>0:
        time.sleep(sleep_time)

# def get_tracker():
#     v = triad_openvr.triad_openvr()
#     if not "tracker_1" in v.devices:
#         return None
#     return v.devices["tracker_1"]

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
def collect_position(tracker, interval, verbose = False):
      x, y, z, roll, pitch, yaw = collect_sample(tracker, interval, verbose)
      return (x, z)