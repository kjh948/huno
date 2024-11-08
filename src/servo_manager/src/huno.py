#!/usr/bin/env python


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

        self._sub_joints={}
        for j in self.joints:
            p=rospy.Publisher(self.ns+j+"_position_controller/command",Float64)
            self._pub_joints[j]=p
        self.cur_joints={}

        rospy.sleep(1)

    def read_cur_pos(self):
        for j in self.joints:
            id = 0
            pos = self.wck.readPos(id)
    
    def set_pos(self, joint, angle):
        #need to check scaling issue
        angle_val = angle/self.pi*255
        self.wck.pos(id, angle_val)


if __name__=="__main__":
    rospy.init_node("servo_manager")
    
    rospy.loginfo("Instantiating wck servo")

'''
/darwin/j_ankle1_l_position_controller/command
/darwin/j_ankle1_r_position_controller/command
/darwin/j_ankle2_l_position_controller/command
/darwin/j_ankle2_r_position_controller/command
/darwin/j_high_arm_l_position_controller/command
/darwin/j_high_arm_r_position_controller/command
/darwin/j_low_arm_l_position_controller/command
/darwin/j_low_arm_r_position_controller/command
/darwin/j_pan_position_controller/command
/darwin/j_pelvis_l_position_controller/command
/darwin/j_pelvis_r_position_controller/command
/darwin/j_shoulder_l_position_controller/command
/darwin/j_shoulder_r_position_controller/command
/darwin/j_thigh2_l_position_controller/command
/darwin/j_thigh2_r_position_controller/command
/darwin/j_thigh2_l_position_controller/command
/darwin/j_thigh2_r_position_controller/command
/darwin/j_tibia_l_position_controller/command
/darwin/j_tibia_r_position_controller/command
/darwin/j_tilt_position_controller/command
/darwin/j_wrist_l_position_controller/command
/darwin/j_wrist_r_position_controller/command
import rospy
from std_msgs.msg import String

def callback(data, source):
    rospy.loginfo(f"I heard {data.data} from {source}")

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("chatter_1", String, lambda data: callback(data, 1))
    rospy.Subscriber("chatter_2", String, lambda data: callback(data, 2))
    rospy.spin()

if __name__ == '__main__':
    listener()

'''