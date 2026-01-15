import tracker_coordinate_transform
import trackercal
import triad_openvr
import numpy as np

class Trackers:

    def __init__(self, configfile_path=None):
        self.v = triad_openvr.triad_openvr(configfile_path)
        self.trackers = ["tracker_1", "tracker_2", "tracker_3"]
        self.found_trackers = {}

    def check_for_trackers(self, offlineTest):

        for tracker in self.trackers:
            if tracker in self.v.devices:
                print(f"Success: {tracker} found")
                self.found_trackers[tracker] = self.v.devices[tracker]
            else:
                print(f"Note: unable to get {tracker}")
                self.found_trackers[tracker] = None

        if all(tracker is None for tracker in self.found_trackers.values()):
            print("Error: no trackers found")
            if not offlineTest:
                exit(1)

    def get_tracker_1_pos(self, interval, R, s, t, verbose, offlineTest):
        return self.found_trackers["tracker_1"].get_current_tracker_position(interval, R, s, t, verbose, offlineTest)

    def get_tracker_1_calibration(self, calibrate_options):
        if "tracker_1" not in self.found_trackers:
            print("Error: unable to get tracker 1")
            print("Make sure Tracker 1 is turned on for calibration")
            print("Make sure the tracker 1 USB dongle is plugged in to your PC")
            if not calibrate_options.offlineTest:
                exit(1)
                
        return self.found_trackers["tracker_1"].calibrate(calibrate_options)
    


    def get_all_tracker_wpi_poses(self, interval, R, s, t, tx, ty, heading_offset, verbose, offlineTest):
        return [(i, self.found_trackers[tracker].update_wpiPose(interval, R, s, t,tx, ty, heading_offset, verbose, offlineTest)) for i,tracker in enumerate(self.trackers) if self.found_trackers[tracker] != None]

    # Calibrate the field using 3 known field points
    # tracker_1, tracker_2, tracker_3 are the trackers at the known field points
    # Tracker_1: AT#18 (3.6576, 4.0259)
    # Tracker_2: AT#20 (4.90474, 4.745482), 
    # Tracker_3: AT#22 (4.90474, 3.306318)
    def calibrate_blue(self, args):
        interval = 1/args.rate
        if not args.infinite:
            apriltag_samples = []
            print("Localize trackers at known field points on blue alliance side.")
            print ("Place Tracker_1 at  AprilTag #18 (3.6576, 4.0259)")
            print ("Place Tracker_2 at  AprilTag #20 (4.90474, 4.745482)")
            print ("Place Tracker_3 at  AprilTag #22 (4.90474, 3.306318)")
            print("Press enter to continue.")
            input()
            for tracker in self.found_trackers.values():
                x, y, z, roll, pitch, yaw = tracker.collect_sample(interval, args.verbose,args.offlineTest)
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
        R, s, t = trackercal.find_transformation_params(apriltag_samples, apriltag_references)
        if args.verbose:
            print(f"Rotation matrix: {R}")
            print(f"Scale factor: {s}")
            print(f"Translation vector: {t}")

        if args.verbose:
            transformed_points = tracker_coordinate_transform.transform_samples(apriltag_samples, R, s, t)
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
    def calibrate_red(self, args):
        interval = 1/args.rate
        if not args.infinite:
            apriltag_samples = []
            print("Localize trackers at known field points on blue alliance side.")
            print ("Place Tracker_1 at  AprilTag #7  (13.890498, 4.0259)")
            print ("Place Tracker_2 at  AprilTag #9  (12.643358, 4.745482)")
            print ("Place Tracker_3 at  AprilTag #11 (12.643358, 3.306318)")
            print("Press enter to continue.")
            input()
            for tracker in self.found_trackers.values():
                x, y, z, roll, pitch, yaw = tracker.collect_sample(interval, args.verbose,args.offlineTest)
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
        R, s, t = trackercal.find_transformation_params(apriltag_samples, apriltag_references)
        if args.verbose:
            print(f"Rotation matrix: {R}")
            print(f"Scale factor: {s}")
            print(f"Translation vector: {t}")

        if args.verbose:
            transformed_points = tracker_coordinate_transform.transform_samples(apriltag_samples, R, s, t)
            # difference between transformed points and apriltag_references should be small
            # calculate the error
            error = np.linalg.norm(np.array(transformed_points) - np.array(apriltag_references))
            print(f"Error: {error}")
            #circle.plot_data_circle(transformed_points, 0, 0, 1.44)

        return R, s, t
    
