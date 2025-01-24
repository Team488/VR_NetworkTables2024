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
    def __init__(self, verbose = False, samples = 100, distance = 5, rate = 250, infinite = False, xOffset = 0, yOffset = 0) -> None:
        self.verbose = verbose
        self.samples = samples
        self.distance = distance
        self.rate = rate
        self.infinite = infinite
        self.xOffset = xOffset
        self.yOffset = yOffset


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

def calculate_FRC_samples(samples, xc, yc, r, initial_angle=np.pi/2):
    angle_values = get_angle_values(samples, xc, yc, initial_angle)
    FRC_samples = [(r*np.cos(angle) + xc, r*np.sin(angle) + yc) for angle in angle_values]
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

def transform_coordinates(point, R, s, t):
    """
    Transform coordinates from system 1 to system 2 using the calculated
    rotation matrix, scale factor, and translation vector.
    """
    point = np.array(point)
    transformed_point = s * np.dot(R, point) + t
    return transformed_point

    # Arbitrary coordinate in system 1
    # arbitrary_point = (x, y)  # Replace with your point

    # transformed_point = transform_coordinates(arbitrary_point, R, s, t)
    # print(f"Transformed coordinates: {transformed_point}")

def transform_samples(samples, R, s, t):
    transformed_samples = [transform_coordinates(sample, R, s, t) for sample in samples]
    return transformed_samples



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
        # cleanup TODO: negate x value
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

    #  cleanup TODO: 
        # negate x values from circle_samples
        # negated_circle_samples = [(-point[0],point[1]) for point in circle_samples]
        # circle_samples = negated_circle_samples
        # calculate rotation angle of circle_samples
        # calculate FRC_circle_samples using circle xc, yc, and radius, along with rotation angle of circle_samples
        # use circle_samples and FRC_circle samples to calculate translation, scale, and rotation matrices
        # use transform_coordinates to generate FRC_circle_samples_verify
        # generate verification plots: overlay circles, x values, y values for both circle_samples and FRC_circle_samples_verify

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

    # cleanup TODO: return translation, scale, and rotation matrices
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
        print("Error: unable to get tracker 1")
        print("Make sure Tracker 1 is turned on for calibration")
        print("Make sure the tracker 1 USB dongle is plugged in to your PC")
        exit(1)
 
    tracker= v.devices["tracker_1"]

    print(calibrate(tracker, args))