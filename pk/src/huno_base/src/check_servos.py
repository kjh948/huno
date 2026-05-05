from wck import servo 
import time
from config import DEFAULT_SERIAL_PORT, load_joints

# Load joints from JSON file
joints = load_joints()

offset = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
cur_joints = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]

a = servo(DEFAULT_SERIAL_PORT, 115200)

delta = 10

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