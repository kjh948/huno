import pybullet as p
import pybullet_data
import time

class HunoSim:
    def __init__(self, urdf_path):
        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)

        self.robot = p.loadURDF(
            urdf_path,
            [0, 0, 0.3],
            useFixedBase=False
        )

        self.joints = {}
        for i in range(p.getNumJoints(self.robot)):
            info = p.getJointInfo(self.robot, i)
            self.joints[info[1].decode()] = i

    def set_leg_angles(self, side, hip, knee, ankle):
        p.setJointMotorControl2(
            self.robot,
            self.joints[f"{side}_hip_pitch_joint"],
            p.POSITION_CONTROL,
            hip
        )
        p.setJointMotorControl2(
            self.robot,
            self.joints[f"{side}_knee_pitch_joint"],
            p.POSITION_CONTROL,
            knee
        )
        p.setJointMotorControl2(
            self.robot,
            self.joints[f"{side}_ankle_pitch_joint"],
            p.POSITION_CONTROL,
            ankle
        )
