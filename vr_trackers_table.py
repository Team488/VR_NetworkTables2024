import ntcore

class VRTrackersTable:

    def __init__(self, address):
        self.inst = ntcore.NetworkTableInstance.getDefault()
        self.inst.setServer(address)
        self.inst.startClient4("VR_trackers")
        self.trackers_table = self.inst.getTable("Trackers")
        self.robot_pose_table = self.inst.getTable("/AdvantageKit/RealOutputs/PoseSubsystem")
        self.pose_sub_x = self.robot_pose_table.getFloatTopic("/AdvantageKit/RealOutputs/PoseSubsystem/RobotPose/translation/x")
    