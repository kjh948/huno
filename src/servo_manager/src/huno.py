#!/usr/bin/python
# -*- coding: utf-8 -*-


import rospy
import time
from std_msgs.msg import Float64
from wck import servo
import numpy

class Huno:
    def __init__(self, uart='/dev/ttyUSB0'):
        self.uart = uart
        self.wck = servo(port=uart, baud=115200)
        self.pi = numpy.pi
        self.ns = 'huno'
        self.offset = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        self.cur_joints = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]

        self.joints = {"j_ankle1_l":0,\
                    "j_ankle1_r":1,\
                    "j_ankle2_l":2,\
                    "j_ankle2_r":3,\
                    "j_high_arm_l":4,\
                    "j_high_arm_r":5,\
                    "j_low_arm_l":6,\
                    "j_low_arm_r":7,\
                    "j_pelvis_l":8,\
                    "j_pelvis_r":9,\
                    "j_shoulder_l":10,\
                    "j_shoulder_r":11,\
                    "j_thigh2_l":12,\
                    "j_thigh2_r":13,\
                    "j_tibia_l":14,\
                    "j_tibia_r":15,\
                    "j_wrist_l":16,\
                    "j_wrist_r":17,\
                    }
        self._sub_joints={}
        # self._pub_joints = {}
        # for j in self.joints:
        #     p=rospy.Publisher(self.ns+j+"_position_controller/command",Float64)
        #     self._pub_joints[j]=p

        rospy.sleep(1)

    def read_cur_pos(self):
        for name in self.joints:
            idx = self.joints[name]
            self.cur_joints[idx] = self.wck.readPos(idx)
            rospy.loginfo(f"Reading joint: {name} with {idx} -> {self.cur_joints[idx]}")
    
    def set_pos(self, joint, angle):
        #need to check scaling issue
        angle_val = angle/self.pi*255
        self.wck.pos(id, angle_val)

    def run(self):
        while(1):
            self.read_cur_pos()
            # self.set_pos(self.cur_joints)
            # rospy.sleep(0.05)



if __name__=="__main__":
    rospy.init_node("servo_manager")
    
    rospy.loginfo("Instantiating wck servo")
    hn = Huno()
    hn.run()
