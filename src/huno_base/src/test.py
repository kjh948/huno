from wck import servo
from config import DEFAULT_SERIAL_PORT
import os
import json

SCRIPT_DIR = os.path.dirname(__file__)
ZERO_FILE = os.path.join(SCRIPT_DIR, 'zero.json')
JOINTS_FILE = os.path.join(SCRIPT_DIR, 'joints_leg.json')

def load_zero(joints_map, num_joints=16):
    """Load zero position values from zero.json"""
    default_zero = [122, 208, 158, 67, 103, 123, 33, 91, 180, 143, 70, 38, 124, 171, 210, 130]
    
    if os.path.exists(ZERO_FILE):
        try:
            with open(ZERO_FILE, 'r') as f:
                zero_dict = json.load(f)
            zero_list = default_zero.copy()
            for name, value in zero_dict.items():
                if name in joints_map:
                    zero_list[joints_map[name]] = value
            return zero_list
        except Exception as e:
            print(f"Error loading zero.json, using default: {e}")
            return default_zero
    return default_zero
def load_joints():
    """Load joint mappings from joints.json"""
    default_joints = {
        "j_ankle1_l": 3, "j_ankle1_r": 8,
        "j_ankle2_l": 4, "j_ankle2_r": 9,
        "j_tibia_l": 2, "j_tibia_r": 7,
        "j_thigh2_l": 1, "j_thigh2_r": 6,
        "j_pelvis_l": 0, "j_pelvis_r": 5,
        "j_shoulder_l": 10, "j_shoulder_r": 13,
        "j_high_arm_l": 11, "j_high_arm_r": 14,
        "j_low_arm_l": 12, "j_low_arm_r": 15,
    }
    
    if os.path.exists(JOINTS_FILE):
        try:
            with open(JOINTS_FILE, 'r') as f:
                return json.load(f)
        except Exception as e:
            print(f"Error loading joints.json, using default: {e}")
            return default_joints
    return default_joints

a = servo(DEFAULT_SERIAL_PORT,115200)
joints = load_joints()    
zero_pos = load_zero(joints)
print(joints["j_ankle1_l"])
# a.pos(joints["j_ankle1_l"],4, zero_pos[joints["j_ankle1_l"]] + 10)

positions = [120, 130, 140, 150, 160, 170, 180, 190, 200, 210]
a.posGroup(10, 1, positions)  # lastId=10, torque

a.posGroup(3,4,[120, 130, 62])
input("group")
a.pos(3,4, 62)
input("single")
a.close()
