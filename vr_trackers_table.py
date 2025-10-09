import ntcore

class VRTrackersTable:

    def __init__(self, address):
        self.inst = ntcore.NetworkTableInstance.getDefault()
        self.inst.setServer(address)
        self.inst.startClient4("VR_trackers")
        self.trackers_table = self.inst.getTable("Trackers")

    def trackers_table(self):
        return self.trackers_table
    
    def robot_pose_table(self):
        return self.inst.getTable("/AdvantageKit/RealOutputs/PoseSubsystem")
    
    def robot_pose_sub_x(self):
        return self.robot_pose_table().getFloatTopic("/AdvantageKit/RealOutputs/PoseSubsystem/RobotPose/translation/x")