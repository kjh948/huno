from wck import servo 
import time

joints = {          
                    "j_ankle1_l":3,\
                    "j_ankle1_r":8,\
                    "j_ankle2_l":4,\
                    "j_ankle2_r":9,\
                    "j_tibia_l":2,\
                    "j_tibia_r":7,\
                    "j_thigh2_l":1,\
                    "j_thigh2_r":6,\
                    "j_pelvis_l":0,\
                    "j_pelvis_r":5,\

                    "j_shoulder_l":10,\
                    "j_shoulder_r":13,\
                    "j_high_arm_l":11,\
                    "j_high_arm_r":14,\

                    "j_low_arm_l":12,\
                    "j_low_arm_r":15,\

                    # "j_wrist_l":16,\
                    # "j_wrist_r":17,\
                    
                    }
offset = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
cur_joints = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]

a = servo("/dev/ttyUSB0",115200)

delta = 3

for name in joints:
    idx = joints[name]
    cur_joints[idx] = a.readPos(idx)
    print(f"Reading joint: {name} with {idx} -> {cur_joints[idx]}")
    a.pos(idx, 4,cur_joints[idx]+delta)
    # time.sleep(1)
    a.pos(idx, 4,cur_joints[idx])
    # time.sleep(1)
    a.pos(idx, 4,cur_joints[idx]-delta)
    # time.sleep(1)
    a.pos(idx, 4,cur_joints[idx])
    
a.close()