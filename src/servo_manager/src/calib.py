from wck import servo 
import time
import getkey

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
zero_offset =     [122,208,158,67,103,123,33,91,180,143,70,38,124,171,210,130]
cur_joints = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]


a = servo("/dev/ttyUSB0",115200)

def set_zero_pos():
    print("set zero pos")
    for name in joints:
        idx = joints[name]
        a.pos(idx, 4,zero_offset[idx])
        print(f"Reading joint: {name} with {idx} -> {a.readPos(idx)}")

def print_joint():
    for name in joints:
        idx = joints[name]
        cur_joints[idx] = a.readPos(idx)
        print(f"Reading joint: {name} with {idx} -> {cur_joints[idx]}")
    
    print("[")
    for c in range(len(cur_joints)):
        print(f"{cur_joints[c]}, ")
    print("]\n")

def keycon(id):
    print("press up/down arrow to change offset")
    while 1:
        print(f"current pos={cur_joints[id]}\n")
    
        k = getkey.getkey()
        if(k == "\x1b[A"):#up
            cur_joints[id] = cur_joints[id]+1
            a.pos(id, 4,cur_joints[id])
        elif k=="\x1b[B":#down
            cur_joints[id] = cur_joints[id]-1
            a.pos(id, 4,cur_joints[id])
        elif k=="q":
            break

set_zero_pos()

print_joint()

for name in joints:
    idx = joints[name]
    keycon(idx)

print_joint()

a.close()