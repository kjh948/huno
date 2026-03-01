#!/usr/bin/python
# -*- coding: utf-8 -*-

import sys
import os
import time
import math
import json
import threading
import numpy as np

# Add huno_base/src to path to import wck and config
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
HUNO_BASE_SRC = os.path.abspath(os.path.join(SCRIPT_DIR, '..', 'huno_base', 'src'))
sys.path.append(HUNO_BASE_SRC)

try:
    from wck import servo
    from config import DEFAULT_SERIAL_PORT
    from uvc_core import UVCCore
except ImportError as e:
    print(f"Error: Could not import required modules: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)
except AttributeError as e:
    print(f"Error: Module found but attribute missing: {e}")
    sys.exit(1)

# File paths for configuration
JOINTS_FILE = os.path.join(HUNO_BASE_SRC, 'joints.json')
ZERO_FILE = os.path.join(HUNO_BASE_SRC, 'zero.json')

def load_json_file(filepath, default=None):
    if os.path.exists(filepath):
        try:
            with open(filepath, 'r') as f:
                return json.load(f)
        except Exception as e:
            print(f"Error loading {filepath}: {e}")
    return default

class UVCRealRobot:
    def __init__(self, port=DEFAULT_SERIAL_PORT, baud=115200):
        print(f"Connecting to Huno on {port}...")
        try:
            self.wck = servo(port=port, baud=baud)
        except Exception as e:
            print(f"Connection failed: {e}")
            sys.exit(1)

        # Initialize UVC Engine (Scale: 0.001 for mm to m)
        self.core = UVCCore(scale=0.001)
        
        # Huno specific UVC parameters (from uvc.py configs)
        self.core.LEG = 180.0 * self.core.scale
        self.core.autoH = 170.0 * self.core.scale
        self.core.autoHs = 180.0 * self.core.scale
        self.core.swf = 12.0 * self.core.scale
        
        # Mapping and Zeroing
        self.joints_map = load_json_file(JOINTS_FILE, {})
        self.zero_map = load_json_file(ZERO_FILE, {})
        
        # Cache zero positions (ID 0-15)
        self.zeros = [127] * 16
        for name, joint_id in self.joints_map.items():
            if name in self.zero_map:
                self.zeros[joint_id] = self.zero_map[name]

        # Radian to WCK unit scaling
        # Typically wCK is ~1.4 degrees per unit. 1 rad = 57.3 deg.
        # 57.3 / 1.4 ~= 40.0. Let's use 45.0 for refinement.
        self.rad_scale = 20.0 

        # Direction signs for Huno (from uvc.py config)
        self.signs = {'hip_p': -1, 'hip_r': 1, 'knee': -1, 'ankle_p': -1, 'ankle_r': 1}
        
        self.is_running = False
        self.should_exit = False
        self.targets = list(self.zeros)

    def map_uvc_to_servos(self):
        """Map UVCCore radian outputs to Servo units"""
        # Right Leg: Index 0, Left Leg: Index 1
        mapping = [
            # Right side
            ('r_hip_pitch', self.core.K0W[0],  self.signs['hip_p'],   'j_thigh2_r'),
            ('r_hip_roll',  self.core.K1W[0],  self.signs['hip_r'],   'j_pelvis_r'),
            ('r_knee',      self.core.HW[0],   self.signs['knee'],    'j_tibia_r'),
            ('r_ankle_p',   self.core.A0W[0],  self.signs['ankle_p'], 'j_ankle1_r'),
            ('r_ankle_r',   self.core.A1W[0],  self.signs['ankle_r'], 'j_ankle2_r'),
            # Left side
            ('l_hip_pitch', self.core.K0W[1],  self.signs['hip_p'],   'j_thigh2_l'),
            ('l_hip_roll',  self.core.K1W[1],  self.signs['hip_r'],   'j_pelvis_l'),
            ('l_knee',      self.core.HW[1],   self.signs['knee'],    'j_tibia_l'),
            ('l_ankle_p',   self.core.A0W[1],  self.signs['ankle_p'], 'j_ankle1_l'),
            ('l_ankle_r',   self.core.A1W[1],  self.signs['ankle_r'], 'j_ankle2_l')
        ]

        for _, rad, sign, name in mapping:
            if name in self.joints_map:
                jid = self.joints_map[name]
                offset = int(rad * self.rad_scale * sign)
                val = self.zeros[jid] + offset
                self.targets[jid] = max(1, min(254, val))

    def run_loop(self):
        print("UVC Robot Loop Started.")
        dt = 0.01 # 100Hz
        
        while not self.should_exit:
            # Update UVC Engine
            if self.is_running:
                self.core.walkF = 0x01
            else:
                self.core.walkF = 0x00
                # If was walking, we might need a reset logic here
                # For now, UVCCore handles idle if walkF is 0
            
            self.core.step()
            self.map_uvc_to_servos()
            
            # Send to hardware
            self.wck.posGroup(15, 4, self.targets)
            
            time.sleep(dt)

    def interactive_menu(self):
        print("\n=== UVC Huno Real Robot Controller ===")
        print("Commands: start, stop, q (quit)")
        
        while not self.should_exit:
            try:
                line = input("UVC> ").strip().lower()
            except EOFError:
                break
                
            if line == 'q':
                self.should_exit = True
                self.is_running = False
            elif line == 'start':
                self.is_running = True
                print(">>> UVC Walking STARTED")
            elif line == 'stop':
                self.is_running = False
                print(">>> UVC Walking STOPPED")
            elif line == 'step':
                # Force one step cycle if needed
                pass

if __name__ == "__main__":
    robot = UVCRealRobot()
    
    # Run loop in thread
    t = threading.Thread(target=robot.run_loop)
    t.daemon = True
    t.start()
    
    try:
        robot.interactive_menu()
    except KeyboardInterrupt:
        robot.should_exit = True
    
    print("Closing...")
    robot.wck.posGroup(15, 4, robot.zeros) # Back to zero
    time.sleep(0.5)
    robot.wck.close()
