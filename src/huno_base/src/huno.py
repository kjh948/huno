#!/usr/bin/python
# -*- coding: utf-8 -*-


import rospy
import time
from std_msgs.msg import Float64
from wck import servo
import numpy
import json
import os
from config import DEFAULT_SERIAL_PORT

# File paths
SCRIPT_DIR = os.path.dirname(__file__)
JOINTS_FILE = os.path.join(SCRIPT_DIR, 'joints.json')
ZERO_FILE = os.path.join(SCRIPT_DIR, 'zero.json')

def load_joints():
    """Load joint mappings from joints.json"""
    default_joints = {
        "j_ankle1_l":3,
        "j_ankle1_r":8,
        "j_ankle2_l":4,
        "j_ankle2_r":9,
        "j_tibia_l":2,
        "j_tibia_r":7,
        "j_thigh2_l":1,
        "j_thigh2_r":6,
        "j_pelvis_l":0,
        "j_pelvis_r":5,
        "j_shoulder_l":10,
        "j_shoulder_r":13,
        "j_high_arm_l":11,
        "j_high_arm_r":14,
        "j_low_arm_l":12,
        "j_low_arm_r":15,
    }
    
    if os.path.exists(JOINTS_FILE):
        try:
            with open(JOINTS_FILE, 'r') as f:
                return json.load(f)
        except Exception as e:
            rospy.logwarn(f"Error loading joints.json, using default: {e}")
            return default_joints
    return default_joints

def load_zero(joints_map, num_joints=16):
    """Load zero position values from zero.json"""
    default_zero = [122,208,158,67,103,123,33,91,180,143,70,38,124,171,210,130]
    
    if os.path.exists(ZERO_FILE):
        try:
            with open(ZERO_FILE, 'r') as f:
                zero_dict = json.load(f)
            # Convert dict back to list
            zero_list = default_zero.copy()
            for name, value in zero_dict.items():
                if name in joints_map:
                    zero_list[joints_map[name]] = value
            return zero_list
        except Exception as e:
            rospy.logwarn(f"Error loading zero.json, using default: {e}")
            return default_zero
    else:
        rospy.logwarn("zero.json not found. Using default zero positions.")
        return default_zero

class Huno:
    def __init__(self, uart=DEFAULT_SERIAL_PORT, auto_init=True):
        self.uart = uart
        self.wck = servo(port=uart, baud=115200)
        self.pi = numpy.pi
        self.ns = 'huno'
        
        # Load joint mappings from JSON
        self.joints = load_joints()
        rospy.loginfo(f"Loaded {len(self.joints)} joint mappings")
        
        # Load zero positions from JSON
        self.zero_offset = load_zero(self.joints)
        rospy.loginfo(f"Loaded zero positions: {self.zero_offset}")
        
        self.cur_joints = [0] * 16
        self._sub_joints = {}

        rospy.sleep(1)
        
        # Initialize motors to zero positions
        if auto_init:
            self.init_to_zero()

    def init_to_zero(self):
        """Initialize all motors to their calibrated zero positions"""
        rospy.loginfo("Initializing motors to zero positions...")
        self.wck.posGroup(len(self.zero_offset) - 1, 4, self.zero_offset)
        rospy.loginfo("Motors initialized to zero positions")

    def read_cur_pos(self):
        for name in self.joints:
            idx = self.joints[name]
            self.cur_joints[idx] = self.wck.readPos(idx)
            rospy.loginfo(f"Reading joint: {name} with {idx} -> {self.cur_joints[idx]}")
    
    def set_pos(self, joint_id, angle):
        """Set position for a single joint
        Args:
            joint_id: Motor ID
            angle: Angle in radians
        """
        # Convert radians to servo value (0-254)
        angle_val = int(angle / self.pi * 127 + 127)
        angle_val = max(0, min(254, angle_val))  # Clamp to valid range
        self.wck.pos(joint_id, 4, angle_val)

    def set_joint_by_name(self, joint_name, angle):
        """Set position for a joint by name
        Args:
            joint_name: Name of the joint (e.g., 'j_ankle1_l')
            angle: Angle in radians
        """
        if joint_name in self.joints:
            joint_id = self.joints[joint_name]
            self.set_pos(joint_id, angle)
        else:
            rospy.logwarn(f"Unknown joint name: {joint_name}")

    def run(self):
        while not rospy.is_shutdown():
            self.read_cur_pos()
            rospy.sleep(0.1)

    def close(self):
        """Close the serial connection"""
        self.wck.close()


if __name__=="__main__":
    rospy.init_node("servo_manager")
    
    rospy.loginfo("Instantiating Huno robot")
    hn = Huno()
    
    try:
        hn.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        hn.close()
