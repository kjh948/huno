from wck import servo 
import time
import getkey
import json
import os
from config import DEFAULT_SERIAL_PORT, load_joints, JOINTS_FILE, ZERO_FILE

def save_joints(joints_data):
    try:
        with open(JOINTS_FILE, 'w') as f:
            json.dump(joints_data, f, indent=4)
        print(f"Saved joints to {JOINTS_FILE}")
    except Exception as e:
        print(f"Error saving joints: {e}")

def save_zero(zero_data, joints_map):
    """Save zero position values to zero.json"""
    # Convert list to dict with joint names for readability
    zero_dict = {}
    for name, idx in joints_map.items():
        zero_dict[name] = zero_data[idx]
    try:
        with open(ZERO_FILE, 'w') as f:
            json.dump(zero_dict, f, indent=4)
        print(f"Saved zero positions to {ZERO_FILE}")
    except Exception as e:
        print(f"Error saving zero positions: {e}")

def load_zero(joints_map, num_joints=16):
    """Load zero position values from zero.json"""
    default_zero = [122,208,158,67,103,123,33,91,180,143,70,38,124,171,210,130]
    
    if os.path.exists(ZERO_FILE):
        print(f"Loading zero positions from {ZERO_FILE}")
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
            print(f"Error loading zero positions, using default: {e}")
            return default_zero
    else:
        print("Zero file not found. Using default values.")
        return default_zero

def load_joints():
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
        
        # "j_wrist_l":16,
        # "j_wrist_r":17,
    }

    if os.path.exists(JOINTS_FILE):
        print(f"Loading joints from {JOINTS_FILE}")
        try:
            with open(JOINTS_FILE, 'r') as f:
                return json.load(f)
        except Exception as e:
            print(f"Error loading joints, using default: {e}")
            return default_joints
    else:
        print("Joints file not found. Creating new one.")
        save_joints(default_joints)
        return default_joints

joints_ = load_joints()
joints = dict(sorted(joints_.items(), key=lambda x: x[1]))

# Load zero offset from file or use default
zero_offset = load_zero(joints_)
ready_offset = zero_offset.copy()
cur_joints = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]


a = servo(DEFAULT_SERIAL_PORT, 115200)

def set_zero_pos():
    print("set zero pos")
    # a.posGroup(zero_offset)

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

def keycon(id, joint_name):
    print(f"\n=== Calibrating {joint_name} (ID: {id}) ===")
    print("Controls:")
    print("  ↑ (Up Arrow)   : Increase position")
    print("  ↓ (Down Arrow) : Decrease position")
    print("  s              : Save current zero positions")
    print("  q              : Next joint")
    print("  x              : Exit calibration")
    
    while 1:
        print(f"current pos={cur_joints[id]}")
    
        k = getkey.getkey()
        if(k == "\x1b[A"):#up
            cur_joints[id] = cur_joints[id]+1
            a.pos(id, 4, cur_joints[id])
        elif k=="\x1b[B":#down
            cur_joints[id] = cur_joints[id]-1
            a.pos(id, 4, cur_joints[id])
        elif k=="s":
            save_zero(cur_joints, joints_)
            print("Zero positions saved!")
        elif k=="q":
            break
        elif k=="x":
            return False  # Signal to exit calibration
    return True  # Continue to next joint

if __name__ == "__main__":
    print("\n=== HUNO Joint Calibration Tool ===")
    print("This tool helps you calibrate the zero position for each joint.")
    print("The calibrated values will be saved to zero.json\n")
    
    set_zero_pos()
    print_joint()
    
    for name in joints:
        idx = joints[name]
        if not keycon(idx, name):
            print("Calibration aborted by user.")
            break
    
    print("\n=== Final Joint Positions ===")
    print_joint()
    
    # Save final zero positions
    save_zero(cur_joints, joints_)
    
    a.close()
    print("Calibration complete!")