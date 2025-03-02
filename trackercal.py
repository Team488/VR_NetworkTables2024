import math
import tracker_sample
import triad_openvr

import circle_fit as circle
from icecream import ic

from tracker_coordinate_transform import *

import argparse
import numpy as np
import matplotlib.pyplot as plt

    

class CalibrateOptions:
    def __init__(self, verbose = False, samples = 100, distance = 5, rate = 250, 
                 infinite = False, xOffset = 0, yOffset = 0, offlineTest=False, bluefield=False, redfield=False) -> None:
        self.verbose = verbose
        self.samples = samples
        self.distance = distance
        self.rate = rate
        self.infinite = infinite
        self.xOffset = xOffset
        self.yOffset = yOffset
        self.offlineTest = offlineTest
        self.bluefield = bluefield 
        self.redfield = redfield


def negate_xvalues(samples):
    negated_samples = [(-point[0],point[1]) for point in samples]
    return negated_samples

def extract_xvalues(samples):
        xvalues = [point[0] for point in samples]
        return xvalues

def extract_yvalues(samples):
        yvalues = [point[1] for point in samples]
        return yvalues

def plot_samples(samples, title, axislabel):
    # Plot
    plt.plot(samples, marker='o')
    plt.title(title)
    plt.xlabel('Sample Index')
    plt.ylabel(axislabel)
    plt.grid(True)
    plt.show()
    return 

def plot_samples_2(samples1, samples2, title, axislabel):
    # Plot
    plt.plot(samples1, marker='o')
    plt.plot(samples2, marker='x')
    plt.title(title)
    plt.xlabel('Sample Index')
    plt.ylabel(axislabel)
    plt.grid(True)
    plt.show()
    return

def plot_circles_from_samples(samples1_x, samples1_y, samples2_x, samples2_y, 
                         title, xaxislabel, yaxislabel):
    fig, ax = plt.subplots()
    # Set aspect to be square
    ax.set_aspect('equal') 
    ax.plot(samples1_x, samples1_y, marker='o')
    ax.plot(samples2_x, samples2_y, marker='x')
    plt.title(title)
    plt.xlabel(xaxislabel)
    plt.ylabel(yaxislabel)
    # Set aspect to be square
    plt.grid(True)
    plt.show()

def calculate_rotation_angle(point1, point2, center=(0, 0)):
    """
    Calculate the angle between two points on a circle with respect to the center.
    Result is in radians.
    """
    # Convert points to vectors from the center
    vector1 = np.subtract(point1, center)
    vector2 = np.subtract(point2, center)

    # Calculate the angle with atan2
    angle1 = np.arctan2(vector1[1], vector1[0])
    angle2 = np.arctan2(vector2[1], vector2[0])

    # Calculate the difference
    angle = angle2 - angle1

    # Normalize the result to be between -pi and pi
    angle = (angle + np.pi) % (2 * np.pi) - np.pi

    return angle

def get_angle_values(samples,xc, yc, initial_angle=np.pi/2):
    angle_values = [calculate_rotation_angle(samples[0],point, (xc,yc)) for point in samples]
    initial_angle = np.pi/2
    angle_values = [(x + initial_angle) for x in angle_values]
    return angle_values

def calculate_FRC_samples(samples, xc, yc, r, xcFRC=0,ycFRC=0,initial_angle=np.pi/2):
    angle_values = get_angle_values(samples, xc, yc, initial_angle)
    #FRC_samples = [(r*np.cos(angle) + xc, r*np.sin(angle) + yc) for angle in angle_values]
    FRC_samples = [(r*np.cos(angle) + xcFRC, r*np.sin(angle) + ycFRC) for angle in angle_values]
    return FRC_samples

def samples_subset(samples, sample_interval, index_range):
    subset = [point for i, point in enumerate(samples) if (i + 1) % sample_interval == 0][:index_range]
    return subset

def find_transformation_params(points1, points2):
    """
    Calculate the rotation matrix, scale factor, and translation vector
    from points in system 1 to system 2.
    """
    # Convert points to numpy arrays
    points1 = np.array(points1)
    points2 = np.array(points2)

    # Find the centroids of the points
    centroid1 = np.mean(points1, axis=0)
    centroid2 = np.mean(points2, axis=0)

    # Center the points around the origin
    centered_points1 = points1 - centroid1
    centered_points2 = points2 - centroid2

    # Compute the covariance matrix
    H = np.dot(centered_points1.T, centered_points2)

    # Compute the singular value decomposition
    U, S, Vt = np.linalg.svd(H)
    R = np.dot(Vt.T, U.T)

    # Ensure that the rotation matrix is a proper rotation
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = np.dot(Vt.T, U.T)

    # Compute the scale factor
    var1 = np.var(centered_points1)
    var2 = np.var(centered_points2)
    s = np.sqrt(var2 / var1)

    # Compute the translation vector
    t = centroid2 - s * np.dot(R, centroid1)

    return R, s, t
    # Sample data
    # points1 = [(x1, y1), (x2, y2), ...]  # Replace with your points in system 1
    # points2 = [(x1', y1'), (x2', y2'), ...]  # Replace with your corresponding points in system 2

    # R, s, t = find_transformation_params(points1, points2)
    # print(f"Rotation matrix: {R}")
    # print(f"Scale factor: {s}")
    # print(f"Translation vector: {t}")





def collect_circle(tracker, number, sample_distance, interval, verbose, offlineTest=False):
    samples = []
    run_forever = number < 0

    x, z= tracker_sample.collect_position(tracker, interval=interval, verbose=verbose, offlineTest=offlineTest)
    prev_position = (x, z)
    while run_forever or len(samples) < number:
        x, z = tracker_sample.collect_position(tracker, interval= interval, verbose = verbose, offlineTest=offlineTest)

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
        x, y, z, roll, pitch, yaw = tracker_sample.collect_sample(tracker, interval, args.verbose,args.offlineTest)
        # negate x value to match the coordinate system
        fixingPoint = (-x,z)
        fixingAngle = pitch
        print("Sweep tracker arm in a circle")


    circle_samples = collect_circle(tracker, args.samples if not args.infinite else -1, args.distance / 100, interval, args.verbose, args.offlineTest)

    if args.verbose:
        # Save circle samples to file for debugging and testing
        print("circle samples: " , str(circle_samples))
        with open("circle_samples.txt", "w") as file:
            file.write(str(circle_samples))

    # negate x values from circle_samples
    circle_samples = negate_xvalues(circle_samples)
        
    # fit circle parameters to circle_samples
    xc, yc, r, sigma = circle.standardLSQ(circle_samples)
    if args.verbose:
        # sanity check for the circle fit
        print("Calculated circle with error: ", sigma, " xc: ", xc, " yc: ", yc, " r: ", r)
        circle.plot_data_circle(circle_samples, 0, 0, r)

        # measure rotation angle values for each element in circle_samples, relative to the first element 
        # this is just a sanity check to make sure the circle_samples are correct
        # the angle values should start at pi/2 and increase by 2pi/n for each sample
        angle_values = get_angle_values(circle_samples, xc, yc,initial_angle=np.pi/2)
        plot_samples(angle_values,"Angle Values vs. Sample Index", "Angle Value")

    # Calculate the (x,y) points for the calibration circle in the FRC coordinates, using the known starting point (0,r),
    # known center point (0,0), the radius r, and the angle values from VR sampled data points.
    # clean up TODO: use the specified center point of the FRC circle, not (0,0)
    FRC_circle_samples = calculate_FRC_samples(circle_samples, xc, yc, r, xcFRC=0, ycFRC=0, initial_angle=np.pi/2)

    # Calculate the transformation parameters between the circle samples and the FRC circle samples
    # R is the rotation matrix, s is the scale factor, and t is the translation vector
    R, s, t = find_transformation_params(circle_samples, FRC_circle_samples)
    if args.verbose:
        print(f"Rotation matrix: {R}")
        print(f"Scale factor: {s}")
        print(f"Translation vector: {t}")

    #  cleanup TODO: 
        # use transform_coordinates to generate FRC_circle_samples_verify
        # generate verification plots: overlay circles, x values, y values for both circle_samples and FRC_circle_samples_verify
        
        # cleanup TODO: use xOffset and yOffset to adjust the translation vector
        # translation = (0 + args.xOffset - xc, 0 + args.yOffset - yc)

    if args.verbose:
        transformed_points = transform_samples(circle_samples, R, s, t)
        circle.plot_data_circle(transformed_points, 0, 0, r)

    return R, s, t

# Calibrate the field using 3 known field points
# tracker_1, tracker_2, tracker_3 are the trackers at the known field points
# Tracker_1: AT#18 (3.6576, 4.0259)
# Tracker_2: AT#20 (4.90474, 4.745482), 
# Tracker_3: AT#22 (4.90474, 3.306318)
def calibrate_blue(tracker_1, tracker_2, tracker_3, args):
    interval = 1/args.rate
    if not args.infinite:
        apriltag_samples = []
        print("Localize trackers at known field points on blue alliance side.")
        print ("Place Tracker_1 at  AprilTag #18 (3.6576, 4.0259)")
        print ("Place Tracker_2 at  AprilTag #20 (4.90474, 4.745482)")
        print ("Place Tracker_3 at  AprilTag #22 (4.90474, 3.306318)")
        print("Press enter to continue.")
        input()
        for tracker in [tracker_1, tracker_2, tracker_3]:
            x, y, z, roll, pitch, yaw = tracker_sample.collect_sample(tracker, interval, args.verbose,args.offlineTest)
            apriltag_samples.append((-x, z))
            print(f"Tracker {tracker} at position: ({-x},{z})")
        
    if args.verbose:
        # Save circle samples to file for debugging and testing
        print("AprilTag samples: " , str(apriltag_samples))
        with open("AprilTag_samples_blue.txt", "w") as file:
            file.write(str(apriltag_samples))
    # April Tag reference points for the blue alliance side, in meters for AT#18, AT#20, AT#22
    apriltag_references = [(3.6576, 4.0259), (4.90474, 4.745482), (4.90474, 3.306318)]
         
    
    # Calculate the transformation parameters between the circle samples and the FRC circle samples
    # R is the rotation matrix, s is the scale factor, and t is the translation vector
    R, s, t = find_transformation_params(apriltag_samples, apriltag_references)
    if args.verbose:
        print(f"Rotation matrix: {R}")
        print(f"Scale factor: {s}")
        print(f"Translation vector: {t}")

    if args.verbose:
        transformed_points = transform_samples(apriltag_samples, R, s, t)
        # difference between transformed points and apriltag_references should be small
        # calculate the error
        error = np.linalg.norm(np.array(transformed_points) - np.array(apriltag_references))
        print(f"Calibration Error: {error} ; any value less than 0.02 is good.")
        #circle.plot_data_circle(transformed_points, 0, 0, 1.44)

    return R, s, t

# Calibrate the field using 3 known field points
# tracker_1, tracker_2, tracker_3 are the trackers at the known field points
# Tracker_1: AT#7 (13.890498, 4.0259)
# Tracker_2: AT#9 (12.643358, 4.745482), 
# Tracker_3: AT#11 (12.643358, 3.306318)
def calibrate_red(tracker_1, tracker_2, tracker_3, args):
    interval = 1/args.rate
    if not args.infinite:
        apriltag_samples = []
        print("Localize trackers at known field points on blue alliance side.")
        print ("Place Tracker_1 at  AprilTag #7  (13.890498, 4.0259)")
        print ("Place Tracker_2 at  AprilTag #9  (12.643358, 4.745482)")
        print ("Place Tracker_3 at  AprilTag #11 (12.643358, 3.306318)")
        print("Press enter to continue.")
        input()
        for tracker in [tracker_1, tracker_2, tracker_3]:
            x, y, z, roll, pitch, yaw = tracker_sample.collect_sample(tracker, interval, args.verbose,args.offlineTest)
            apriltag_samples.append((-x, z))
            print(f"Tracker {tracker} at position: ({-x},{z})")
        
    if args.verbose:
        # Save circle samples to file for debugging and testing
        print("AprilTag samples: " , str(apriltag_samples))
        with open("AprilTag_samples_red.txt", "w") as file:
            file.write(str(apriltag_samples))
    # April Tag reference points for the blue alliance side, in meters for AT#18, AT#20, AT#22
    apriltag_references = [(13.890498, 4.0259), (12.643358, 4.745482), (12.643358, 3.306318)]
         
    
    # Calculate the transformation parameters between the circle samples and the FRC circle samples
    # R is the rotation matrix, s is the scale factor, and t is the translation vector
    R, s, t = find_transformation_params(apriltag_samples, apriltag_references)
    if args.verbose:
        print(f"Rotation matrix: {R}")
        print(f"Scale factor: {s}")
        print(f"Translation vector: {t}")

    if args.verbose:
        transformed_points = transform_samples(apriltag_samples, R, s, t)
        # difference between transformed points and apriltag_references should be small
        # calculate the error
        error = np.linalg.norm(np.array(transformed_points) - np.array(apriltag_references))
        print(f"Error: {error}")
        #circle.plot_data_circle(transformed_points, 0, 0, 1.44)

    return R, s, t
    
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
    parser.add_argument('-o', '--offlineTest', action = 'store_true', help='test with the trackers offline (no trackers required)', default = default.offlineTest)
    parser.add_argument('-b', '--bluefield', action = 'store_true', help='calibrate blue field ', default = default.bluefield)
    parser.add_argument('-e', '--redfield', action = 'store_true', help='calibrate red field ', default = default.redfield)
    
    # Parse the arguments
    args = parser.parse_args()
    
    v = triad_openvr.triad_openvr()
    tracker_1, tracker_2, tracker_3 = check_for_trackers(v,args.offlineTest)

    if args.bluefield:
        print(calibrate_blue(tracker_1, tracker_2, tracker_3, args))
    elif args.redfield:
        print(calibrate_red(tracker_1, tracker_2, tracker_3, args))
    else:
        print(calibrate(tracker_1, args))