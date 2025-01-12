import math
import tracker_sample
import triad_openvr

import circle_fit as circle
from icecream import ic

from tracker_coordinate_transform import *

import argparse
import numpy as np

    

class CalibrateOptions:
    def __init__(self, verbose = False, samples = 100, distance = 5, rate = 250, infinite = False, xOffset = 0, yOffset = 0) -> None:
        self.verbose = verbose
        self.samples = samples
        self.distance = distance
        self.rate = rate
        self.infinite = infinite
        self.xOffset = xOffset
        self.yOffset = yOffset


def collect_circle(tracker, number, sample_distance, interval, verbose):
    samples = []
    run_forever = number < 0

    x, z= tracker_sample.collect_position(tracker, interval=interval, verbose=verbose)
    prev_position = (x, z)
    while run_forever or len(samples) < number:
        x, z = tracker_sample.collect_position(tracker, interval= interval, verbose = verbose)

        px, pz = prev_position
        if not run_forever:
            distance = math.sqrt((px - x)**2 + (pz - z)**2)
    
            if verbose:
                print("distance between current and previous point: ", distance)
    
            if distance > sample_distance:
                if verbose:
                    print("adding sample")

                samples.append((x, z))
            
                prev_position = x, z
    return samples
def calculate_angle(p1, p2):
    # Calculate the angle in radians between the two points
    x1, y1 = p1
    x2, y2 = p2
    angle1 = math.atan2(y1, x1)
    angle2 = math.atan2(y2, x2)
    
    # Calculate the difference in angles
    angle = angle2 - angle1
    return angle

# Calibrate the tracker with user input. 
def calibrate(tracker, args):
    interval = 1/args.rate

    if not args.infinite:
        print("Set tracker to 0, r position. Press enter to continue.")
        input()
        x, y, z, roll, pitch, yaw = tracker_sample.collect_sample(tracker, interval, args.verbose)
        fixingPoint = (x,z)
        fixingAngle = pitch
        print("Sweep tracker arm in a circle")


    circle_samples = collect_circle(tracker, args.samples if not args.infinite else -1, args.distance / 100, interval, args.verbose)

    if args.verbose:
        print("circle samples: " , str(circle_samples))
        with open("circle_samples.txt", "w") as file:
            file.write(str(circle_samples))
        

    xc, yc, r, sigma = circle.standardLSQ(circle_samples)
    if args.verbose:
        print("Calculated circle with error: ", sigma, " xc: ", xc, " yc: ", yc, " r: ", r)
        circle.plot_data_circle(circle_samples, 0, 0, r)

    src_points = np.array([(xc, yc-r),(xc - r, yc),(xc, yc), (xc + r, yc), fixingPoint])
    dst_points = np.array([(0, -r), (-r, 0), (0,0), (r, 0), (0, r)])

    #transformation_matrix = ic(compute_transformation_matrix(src_points, dst_points))

    #transformation_matrix = ic(find_affine_transform(src_points, dst_points))

    translation = (0 + args.xOffset - xc, 0 + args.yOffset - yc)
    scale = (1,1)
    #transformed_points = [apply_transform([x,y],transformation_matrix) for x,y in circle_samples]
    if args.verbose:
        transformed_points = [transform_point((x,y), translation, scale, 0) for x,y in circle_samples]
        circle.plot_data_circle(transformed_points, 0, 0, r)

    return (translation, scale, fixingAngle)
    
if __name__ == "__main__":
    parser = argparse.ArgumentParser(prog='trackercal', description='A command line application for tracking and calculating events.')

    default = CalibrateOptions()
    # Add optional arguments
    parser.add_argument('-v', '--verbose', action='store_true', help='Run the command in verbose mode.', default = default.verbose)
    parser.add_argument('-s', '--samples', action='store', help='Number of samples to collect when creating the circle.', default = default.samples)
    parser.add_argument('-d', '--distance', action='store', help='Distance between samples to collect, in centimeters', default = default.distance)
    parser.add_argument('-r', '--rate', action='store', help='Sampling rate of tracker.', default = default.rate)
    parser.add_argument('-i', '--infinite', action='store_true', help='Run forever and continuously output pose data instead of calibrating.', default = default.infinite)
    parser.add_argument('-x', '--xOffset', action='store', help='offset the x coordinate transform by x amount', default = default.xOffset)
    parser.add_argument('-y', '--yOffset', action='store', help='offset the y coordinate transform by y amount', default = default.yOffset)
    
    # Parse the arguments
    args = parser.parse_args()
    
    v = triad_openvr.triad_openvr()
    if not "tracker_1" in v.devices:
        print("Error: unable to get tracker")
        exit(1)
 
    tracker= v.devices["tracker_1"]

    print(calibrate(tracker, args))