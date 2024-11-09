import time

start = time.time()
interval = 1/250
def __sleep(interval):        
    sleep_time = interval-(time.time()-start)
    if sleep_time>0:
        time.sleep(sleep_time)

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
        __sleep()
        start = time.time()

    if verbose:
        print("collected sample: " + pose)

    return pose 

    