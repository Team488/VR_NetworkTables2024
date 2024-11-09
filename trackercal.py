import triad_openvr
import math
import tracker_sample

import circle_fit as circle
from icecream import ic

from tracker_coordinate_transform import *

import argparse
    


def collect_circle(number, sample_distance, interval, verbose):
    samples = []
    run_forever = number < 0

    x, y, z, roll, pitch, yaw = tracker_sample.collect_sample(tracker, interval=interval, verbose=verbose_mode)
    prev_position = (x, z)
    while run_forever or len(samples) < number:
        x, y, z, roll, pitch, yaw = tracker_sample.collect_sample(tracker, interval= 1/args.rate, verbose = verbose_mode)

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

parser = argparse.ArgumentParser(prog='trackercal', description='A command line application for tracking and calculating events.')

# Add optional arguments
parser.add_argument('-v', '--verbose', action='store_true', help='Run the command in verbose mode.')
parser.add_argument('-h', '--help', action='help', help='Show this help message and exit.')
parser.add_argument('-s', '--samples', action='store', help='Number of samples to collect when creating the circle.', default = 150)
parser.add_argument('-d', '--distance', action='store', help='Distance between samples to collect, in centimeters', default = 5)
parser.add_argument('-r', '--rate', action='store', help='Sampling rate of tracker.', default=250)
parser.add_argument('-i', '--infinite', action='store_true', help='Run forever and continuously output pose data instead of calibrating.')
# Parse the arguments
args = parser.parse_args()



if args.verbose or args.debug:
    verbose_mode = True

v = triad_openvr.triad_openvr()
if not "tracker_1" in v.devices:
    print("Error: unable to connect to tracker.")
    exit(1)
tracker = v.devices["tracker_1"]

if not args.infinite:
    print("Set tracker to far left position. Press enter to continue.")
    r0Point = tracker_sample.collect_sample(tracker, args.interval, args.verbose)
    input()
    print("Sweep tracker arm in a circle")


circle_samples = collect_circle(args.samples if not args.infinite else -1, args.distance / 100, args.interval, args.verbose)

if args.verbose:
    print("circle samples: " + circle_samples)
    

xc, yc, r, sigma = circle.standardLSQ(circle_samples)
if args.verbose:
    print("Calculated circle with error: ", sigma, " xc: ", xc, " yc: ", yc, " r: ", r)
    circle.plot_data_circle(circle_samples, xc, yc, r)

src_points = [(xc, yc), (xc + r, yc), (xc, yc + r)]
dst_points = [(0,0), (r, 0) (0, r)]

transformation_matrix = ic(compute_transformation_matrix(src_points, dst_points))

print(transformation_matrix)