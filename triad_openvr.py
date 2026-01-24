import time
import sys
import openvr
import math
import json
import tracker_sample
import trackercal
import tracker_coordinate_transform
import circle_fit as circle
import numpy as np
from wpimath.geometry import Pose2d,Rotation2d,Translation2d


from functools import lru_cache

# Function to print out text but instead of starting a new line it will overwrite the existing line
def update_text(txt):
    sys.stdout.write('\r'+txt)
    sys.stdout.flush()

#Convert the standard 3x4 position/rotation matrix to a x,y,z location and the appropriate Euler angles (in degrees)
def convert_to_euler(pose_mat):
    yaw = 180 / math.pi * math.atan2(pose_mat[1][0], pose_mat[0][0])
    pitch = 180 / math.pi * math.atan2(pose_mat[2][0], pose_mat[0][0])
    roll = 180 / math.pi * math.atan2(pose_mat[2][1], pose_mat[2][2])
    x = pose_mat[0][3]
    y = pose_mat[1][3]
    z = pose_mat[2][3]
    return [x,y,z,yaw,pitch,roll]

#Convert the standard 3x4 position/rotation matrix to a x,y,z location and the appropriate Quaternion
def convert_to_quaternion(pose_mat):
    # Per issue #2, adding a abs() so that sqrt only results in real numbers
    r_w = math.sqrt(abs(1+pose_mat[0][0]+pose_mat[1][1]+pose_mat[2][2]))/2
    r_x = (pose_mat[2][1]-pose_mat[1][2])/(4*r_w)
    r_y = (pose_mat[0][2]-pose_mat[2][0])/(4*r_w)
    r_z = (pose_mat[1][0]-pose_mat[0][1])/(4*r_w)

    x = pose_mat[0][3]
    y = pose_mat[1][3]
    z = pose_mat[2][3]
    return [x,y,z,r_w,r_x,r_y,r_z]

#Define a class to make it easy to append pose matricies and convert to both Euler and Quaternion for plotting
class pose_sample_buffer():
    def __init__(self):
        self.i = 0
        self.index = []
        self.time = []
        self.x = []
        self.y = []
        self.z = []
        self.yaw = []
        self.pitch = []
        self.roll = []
        self.r_w = []
        self.r_x = []
        self.r_y = []
        self.r_z = []

    def append(self,pose_mat,t):
        self.time.append(t)
        self.x.append(pose_mat[0][3])
        self.y.append(pose_mat[1][3])
        self.z.append(pose_mat[2][3])
        self.yaw.append(180 / math.pi * math.atan(pose_mat[1][0] /pose_mat[0][0]))
        self.pitch.append(180 / math.pi * math.atan(-1 * pose_mat[2][0] / math.sqrt(pow(pose_mat[2][1], 2) + math.pow(pose_mat[2][2], 2))))
        self.roll.append(180 / math.pi * math.atan(pose_mat[2][1] /pose_mat[2][2]))
        r_w = math.sqrt(abs(1+pose_mat[0][0]+pose_mat[1][1]+pose_mat[2][2]))/2
        self.r_w.append(r_w)
        self.r_x.append((pose_mat[2][1]-pose_mat[1][2])/(4*r_w))
        self.r_y.append((pose_mat[0][2]-pose_mat[2][0])/(4*r_w))
        self.r_z.append((pose_mat[1][0]-pose_mat[0][1])/(4*r_w))

def get_pose(vr_obj):
    return vr_obj.getDeviceToAbsoluteTrackingPose(openvr.TrackingUniverseStanding, 0, openvr.k_unMaxTrackedDeviceCount)


class vr_tracked_device():
    def __init__(self,vr_obj,index,device_class):
        self.device_class = device_class
        self.index = index
        self.vr = vr_obj

    @lru_cache(maxsize=None)
    def get_serial(self):
        return self.vr.getStringTrackedDeviceProperty(self.index, openvr.Prop_SerialNumber_String)

    def get_model(self):
        return self.vr.getStringTrackedDeviceProperty(self.index, openvr.Prop_ModelNumber_String)

    def get_battery_percent(self):
        return self.vr.getFloatTrackedDeviceProperty(self.index, openvr.Prop_DeviceBatteryPercentage_Float)

    def is_charging(self):
        return self.vr.getBoolTrackedDeviceProperty(self.index, openvr.Prop_DeviceIsCharging_Bool)


    def sample(self,num_samples,sample_rate):
        interval = 1/sample_rate
        rtn = pose_sample_buffer()
        sample_start = time.time()
        for i in range(num_samples):
            start = time.time()
            pose = get_pose(self.vr)
            rtn.append(pose[self.index].mDeviceToAbsoluteTracking,time.time()-sample_start)
            sleep_time = interval- (time.time()-start)
            if sleep_time>0:
                time.sleep(sleep_time)
        return rtn

    def get_pose_euler(self, pose=None):
        if pose == None:
            pose = get_pose(self.vr)
        if pose[self.index].bPoseIsValid:
            return convert_to_euler(pose[self.index].mDeviceToAbsoluteTracking)
        else:
            return None

    def get_pose_matrix(self, pose=None):
        if pose == None:
            pose = get_pose(self.vr)
        if pose[self.index].bPoseIsValid:
            return pose[self.index].mDeviceToAbsoluteTracking
        else:
            return None

    def get_velocity(self, pose=None):
        if pose == None:
            pose = get_pose(self.vr)
        if pose[self.index].bPoseIsValid:
            return pose[self.index].vVelocity
        else:
            return None

    def get_angular_velocity(self, pose=None):
        if pose == None:
            pose = get_pose(self.vr)
        if pose[self.index].bPoseIsValid:
            return pose[self.index].vAngularVelocity
        else:
            return None

    def get_pose_quaternion(self, pose=None):
        if pose == None:
            pose = get_pose(self.vr)
        if pose[self.index].bPoseIsValid:
            return convert_to_quaternion(pose[self.index].mDeviceToAbsoluteTracking)
        else:
            return None

    def get_pose(self, offlineTest=False):
    # x,y,z location and the appropriate Euler angles (in degrees)
        if offlineTest:
            return tracker_sample.get_offline_pose()
        else:
            return self.get_pose_euler()

        # Collect a pose from the tracker, but not more often than the specified time interval
    def collect_sample(self, interval, verbose=False, offlineTest=False):
        # Ensure we don't overcollect samples if user is calling this repeatedly.
        if verbose:
            print("collecting sample")

        pose = None

        while not pose:
            pose = self.get_pose(offlineTest)
            time.sleep(interval)
            if verbose:
                print("missed sample. collecting again.")

        if verbose:
            print("collected sample: ", str(pose))

        return pose

    # Calibrate the tracker with user input.
    def calibrate(self, args):
        interval = 1/args.rate

        if not args.infinite:
            print("Set tracker to 0, r position. Press enter to continue.")
            input()
            x, y, z, roll, pitch, yaw = self.collect_sample(interval, args.verbose,args.offlineTest)
            # negate x value to match the coordinate system
            fixingPoint = (-x,z)
            fixingAngle = pitch
            print("Sweep tracker arm in a circle")


        circle_samples = self.collect_circle(args.samples if not args.infinite else -1, args.distance / 100, interval, args.verbose, args.offlineTest)

        if args.verbose:
            # Save circle samples to file for debugging and testing
            print("circle samples: " , str(circle_samples))
            with open("circle_samples.txt", "w") as file:
                file.write(str(circle_samples))

        # negate x values from circle_samples
        circle_samples = trackercal.negate_xvalues(circle_samples)

        # fit circle parameters to circle_samples
        xc, yc, r, sigma = circle.standardLSQ(circle_samples)
        if args.verbose:
            # sanity check for the circle fit
            print("Calculated circle with error: ", sigma, " xc: ", xc, " yc: ", yc, " r: ", r)
            circle.plot_data_circle(circle_samples, 0, 0, r)

            # measure rotation angle values for each element in circle_samples, relative to the first element
            # this is just a sanity check to make sure the circle_samples are correct
            # the angle values should start at pi/2 and increase by 2pi/n for each sample
            angle_values = trackercal.get_angle_values(circle_samples, xc, yc,initial_angle=np.pi/2)
            trackercal.plot_samples(angle_values,"Angle Values vs. Sample Index", "Angle Value")

        # Calculate the (x,y) points for the calibration circle in the FRC coordinates, using the known starting point (0,r),
        # known center point (0,0), the radius r, and the angle values from VR sampled data points.
        # clean up TODO: use the specified center point of the FRC circle, not (0,0)
        FRC_circle_samples = trackercal.calculate_FRC_samples(circle_samples, xc, yc, r, xcFRC=0, ycFRC=0, initial_angle=np.pi/2)

        # Calculate the transformation parameters between the circle samples and the FRC circle samples
        # R is the rotation matrix, s is the scale factor, and t is the translation vector
        R, s, t = trackercal.find_transformation_params(circle_samples, FRC_circle_samples)
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
            transformed_points = tracker_coordinate_transform.transform_samples(circle_samples, R, s, t)
            circle.plot_data_circle(transformed_points, 0, 0, r)

        return R, s, t

    def collect_circle(self, number, sample_distance, interval, verbose, offlineTest=False):
        samples = []
        run_forever = number < 0

        x, z= self.collect_position(interval=interval, verbose=verbose, offlineTest=offlineTest)
        prev_position = (x, z)
        while run_forever or len(samples) < number:
            x, z = self.collect_position(interval= interval, verbose = verbose, offlineTest=offlineTest)

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

    # Collect the horizontal coordinates of the tracker.
    # The x and z axes are the horizontal coordinates when the tracker is
    # oriented with the mounting screw hole facing down.
    def collect_position(self, interval, verbose = False, offlineTest=False):
        x, y, z, roll, pitch, yaw = self.collect_sample( interval, verbose,offlineTest)
        return (x, z)

    def update_wpiPose(self, interval, R, s, t, tx, ty, heading_offset, verbose=False, offlineTest=False):
    # Get the current tracker position in FRC coordinates, and the heading in VR coordinates
        xFRC, yFRC, headingVR = self.get_current_tracker_position(interval, R, s, t, verbose=False, offlineTest=offlineTest)

        # Calculate the new angle value for the tracker pose using the tracker offset to match the initial robot heading
        pose_heading = Rotation2d.fromDegrees(heading_offset - headingVR)
        poseXY = Translation2d(xFRC + tx, yFRC + ty)
        wpiPose = Pose2d(poseXY, pose_heading)
        return wpiPose

    def get_current_tracker_position(self, interval, R, s, t, verbose=False, offlineTest=False):
        xFRC, yFRC, pitchVR = 0.0, 0.0, 0.0
        # The pitch value from the tracker is the heading in the robot coordinate system
        # The x and z values from the tracker are used to calculate the x and y values in the robot coordinate system
        if not (self == None):
            xVR, yVR, zVR, rollVR, pitchVR, yawVR = self.collect_sample(interval=interval, verbose=verbose, offlineTest=offlineTest)
            # Negate tracker x value before using for consistency with the calibration/transformation functions
            x = -xVR
            #x = xVR
            y = zVR # The VR z-axis corresponds to the y-axis in the robot coordinate system
            # Transform the tracker position to the robot coordinate system
            pointVR =(x,y)
            # Transform the tracker position to the robot coordinate system using the calibration parameters R, s, and t
            # R is the rotation matrix, s is the scale factor, and t is the translation vector
            xFRC, yFRC = tracker_coordinate_transform.transform_coordinates(pointVR,R,s,t)

        # Return the transformed x and y values in the robot coordinate system, and the pitch value in the VR coordinate system
        return xFRC, yFRC, pitchVR

    def controller_state_to_dict(self, pControllerState):
        # This function is graciously borrowed from https://gist.github.com/awesomebytes/75daab3adb62b331f21ecf3a03b3ab46
        # docs: https://github.com/ValveSoftware/openvr/wiki/IVRSystem::GetControllerState
        d = {}
        d['unPacketNum'] = pControllerState.unPacketNum
        # on trigger .y is always 0.0 says the docs
        d['trigger'] = pControllerState.rAxis[1].x
        # 0.0 on trigger is fully released
        # -1.0 to 1.0 on joystick and trackpads
        d['trackpad_x'] = pControllerState.rAxis[0].x
        d['trackpad_y'] = pControllerState.rAxis[0].y
        # These are published and always 0.0
        # for i in range(2, 5):
        #     d['unknowns_' + str(i) + '_x'] = pControllerState.rAxis[i].x
        #     d['unknowns_' + str(i) + '_y'] = pControllerState.rAxis[i].y
        d['ulButtonPressed'] = pControllerState.ulButtonPressed
        d['ulButtonTouched'] = pControllerState.ulButtonTouched
        # To make easier to understand what is going on
        # Second bit marks menu button
        d['menu_button'] = bool(pControllerState.ulButtonPressed >> 1 & 1)
        # 32 bit marks trackpad
        d['trackpad_pressed'] = bool(pControllerState.ulButtonPressed >> 32 & 1)
        d['trackpad_touched'] = bool(pControllerState.ulButtonTouched >> 32 & 1)
        # third bit marks grip button
        d['grip_button'] = bool(pControllerState.ulButtonPressed >> 2 & 1)
        # System button can't be read, if you press it
        # the controllers stop reporting
        return d

    def get_controller_inputs(self):
        result, state = self.vr.getControllerState(self.index)
        return self.controller_state_to_dict(state)

    def trigger_haptic_pulse(self, duration_micros=1000, axis_id=0):
        """
        Causes devices with haptic feedback to vibrate for a short time.
        """
        self.vr.triggerHapticPulse(self.index ,axis_id, duration_micros)

class vr_tracking_reference(vr_tracked_device):
    def get_mode(self):
        return self.vr.getStringTrackedDeviceProperty(self.index,openvr.Prop_ModeLabel_String).decode('utf-8').upper()
    def sample(self,num_samples,sample_rate):
        print("Warning: Tracking References do not move, sample isn't much use...")

class triad_openvr():
    def __init__(self, configfile_path=None):
        # Initialize OpenVR in the
        self.vr = openvr.init(openvr.VRApplication_Other)
        self.vrsystem = openvr.VRSystem()

        # Initializing object to hold indexes for various tracked objects
        self.object_names = {"Tracking Reference":[],"HMD":[],"Controller":[],"Tracker":[]}
        self.devices = {}
        self.device_index_map = {}
        poses = self.vr.getDeviceToAbsoluteTrackingPose(openvr.TrackingUniverseStanding, 0,
                                                               openvr.k_unMaxTrackedDeviceCount)

        # Loading config file
        if configfile_path:
            try:
                with open(configfile_path, 'r') as json_data:
                    config = json.load(json_data)
            except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
                print('config.json not found.')
                exit(1)

            # Iterate through the pose list to find the active devices and determine their type
            for i in range(openvr.k_unMaxTrackedDeviceCount):
                if poses[i].bDeviceIsConnected:
                    device_serial = self.vr.getStringTrackedDeviceProperty(i, openvr.Prop_SerialNumber_String)
                    #device_serial = self.vr.getStringTrackedDeviceProperty(i,openvr.Prop_SerialNumber_String).decode('utf-8')
                    for device in config['devices']:
                        if device_serial == device['serial']:
                            device_name = device['name']
                            self.object_names[device['type']].append(device_name)
                            self.devices[device_name] = vr_tracked_device(self.vr,i,device['type'])
        else:
            # Iterate through the pose list to find the active devices and determine their type
            for i in range(openvr.k_unMaxTrackedDeviceCount):
                if poses[i].bDeviceIsConnected:
                    self.add_tracked_device(i)

    def __del__(self):
        openvr.shutdown()

    def get_pose(self):
        return get_pose(self.vr)

    def poll_vr_events(self):
        """
        Used to poll VR events and find any new tracked devices or ones that are no longer tracked.
        """
        event = openvr.VREvent_t()
        while self.vrsystem.pollNextEvent(event):
            if event.eventType == openvr.VREvent_TrackedDeviceActivated:
                self.add_tracked_device(event.trackedDeviceIndex)
            elif event.eventType == openvr.VREvent_TrackedDeviceDeactivated:
                #If we were already tracking this device, quit tracking it.
                if event.trackedDeviceIndex in self.device_index_map:
                    self.remove_tracked_device(event.trackedDeviceIndex)

    def add_tracked_device(self, tracked_device_index):
        i = tracked_device_index
        device_class = self.vr.getTrackedDeviceClass(i)
        if (device_class == openvr.TrackedDeviceClass_Controller):
            device_name = "controller_"+str(len(self.object_names["Controller"])+1)
            self.object_names["Controller"].append(device_name)
            self.devices[device_name] = vr_tracked_device(self.vr,i,"Controller")
            self.device_index_map[i] = device_name
        elif (device_class == openvr.TrackedDeviceClass_HMD):
            device_name = "hmd_"+str(len(self.object_names["HMD"])+1)
            self.object_names["HMD"].append(device_name)
            self.devices[device_name] = vr_tracked_device(self.vr,i,"HMD")
            self.device_index_map[i] = device_name
        elif (device_class == openvr.TrackedDeviceClass_GenericTracker):
            device_name = "tracker_"+str(len(self.object_names["Tracker"])+1)
            self.object_names["Tracker"].append(device_name)
            self.devices[device_name] = vr_tracked_device(self.vr,i,"Tracker")
            self.device_index_map[i] = device_name
        elif (device_class == openvr.TrackedDeviceClass_TrackingReference):
            device_name = "tracking_reference_"+str(len(self.object_names["Tracking Reference"])+1)
            self.object_names["Tracking Reference"].append(device_name)
            self.devices[device_name] = vr_tracking_reference(self.vr,i,"Tracking Reference")
            self.device_index_map[i] = device_name

    def remove_tracked_device(self, tracked_device_index):
        if tracked_device_index in self.device_index_map:
            device_name = self.device_index_map[tracked_device_index]
            self.object_names[self.devices[device_name].device_class].remove(device_name)
            del self.device_index_map[tracked_device_index]
            del self.devices[device_name]
        else:
            raise Exception("Tracked device index {} not valid. Not removing.".format(tracked_device_index))

    def rename_device(self,old_device_name,new_device_name):
        self.devices[new_device_name] = self.devices.pop(old_device_name)
        for i in range(len(self.object_names[self.devices[new_device_name].device_class])):
            if self.object_names[self.devices[new_device_name].device_class][i] == old_device_name:
                self.object_names[self.devices[new_device_name].device_class][i] = new_device_name

    def print_discovered_objects(self):
        for device_type in self.object_names:
            plural = device_type
            if len(self.object_names[device_type])!=1:
                plural+="s"
            print("Found "+str(len(self.object_names[device_type]))+" "+plural)
            for device in self.object_names[device_type]:
                if device_type == "Tracking Reference":
                    print("  "+device+" ("+self.devices[device].get_serial()+
                          ", Mode "+self.devices[device].get_model()+
                          ", "+self.devices[device].get_model()+
                          ")")
                else:
                    print("  "+device+" ("+self.devices[device].get_serial()+
                          ", "+self.devices[device].get_model()+")")
