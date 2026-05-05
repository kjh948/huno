#!/usr/bin/python
# -*- coding: utf-8 -*-

"""
Motion Generator for HUNO Robot
Allows creating motion files by manually moving the robot (passive mode)
and recording keyframes.
"""

import sys
import os
import time
import json
import logging
import copy

# Import HUNO modules
# Add current directory to path if needed
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from wck import servo
from config import DEFAULT_SERIAL_PORT

# File paths
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MOTIONS_DIR = os.path.join(SCRIPT_DIR, 'motions')
JOINTS_FILE = os.path.join(SCRIPT_DIR, 'joints.json')
ZERO_FILE = os.path.join(SCRIPT_DIR, 'zero.json')

# Constants
PLEN2_VALUE_SCALE = 0.1  # Must match motion_controller.py
NUM_JOINTS = 10

# PLEN2 device name to HUNO joint name mapping (from motion_controller.py)
PLEN2_TO_HUNO_MAP = {
    "left_shoulder_pitch": "j_shoulder_l",
    "left_shoulder_roll": "j_high_arm_l",
    "left_elbow_roll": "j_low_arm_l",
    "right_shoulder_pitch": "j_shoulder_r",
    "right_shoulder_roll": "j_high_arm_r",
    "right_elbow_roll": "j_low_arm_r",
    "left_thigh_roll": "j_pelvis_l",
    "left_thigh_pitch": "j_thigh2_l",
    "left_knee_pitch": "j_tibia_l",
    "left_foot_pitch": "j_ankle1_l",
    "left_foot_roll": "j_ankle2_l",
    "right_thigh_roll": "j_pelvis_r",
    "right_thigh_pitch": "j_thigh2_r",
    "right_knee_pitch": "j_tibia_r",
    "right_foot_pitch": "j_ankle1_r",
    "right_foot_roll": "j_ankle2_r",
}

def load_json_file(filepath, default=None):
    if os.path.exists(filepath):
        try:
            with open(filepath, 'r') as f:
                return json.load(f)
        except Exception as e:
            print(f"Error loading {filepath}: {e}")
    return default

def get_joint_signs():
    # Sign correction for specific joints (must match motion_controller.py)
    signs = [1] * NUM_JOINTS
    signs[4] = -1   # j_ankle2_l
    signs[9] = -1   # j_ankle2_r
    # Note: motion_controller.py also marks 1, 6, 7 as 1 explicitly, which is default.
    return signs

class MotionGenerator:
    def __init__(self, port=DEFAULT_SERIAL_PORT, baud=115200):
        self.wck = servo(port=port, baud=baud)
        self.joints_map = load_json_file(JOINTS_FILE, {})
        self.zero_map = load_json_file(ZERO_FILE, {})
        
        # Invert joints map: ID -> Name
        self.id_to_name = {v: k for k, v in self.joints_map.items()}
        
        # Invert PLEN2 map: HUNO Name -> PLEN2 Device
        self.huno_to_plen2 = {v: k for k, v in PLEN2_TO_HUNO_MAP.items()}
        
        self.zero_offsets = [127] * NUM_JOINTS
        # Populate zero offsets based on zero.json and joints.json
        # Default zero is roughly 127 if not specified, but let's use the list from motion_controller if needed
        # motion_controller.py has a hardcoded default_zero list. structure matching ID index.
        # Let's try to build it from the dict.
        
        for name, joint_id in self.joints_map.items():
            if name in self.zero_map:
                self.zero_offsets[joint_id] = self.zero_map[name]
        
        self.joint_signs = get_joint_signs()
        
        # Frames data
        self.frames = []

    def set_passive_mode(self):
        print("Setting motors to passive mode (torque off)...")
        for name, joint_id in self.joints_map.items():
            if joint_id >= NUM_JOINTS:
                continue

            # Passivate each servo
            # wck.passivate(id) sends command to turn torque off
            try:
                self.wck.passivate(joint_id)
                # Small delay to ensure command is processed
                time.sleep(0.01)
            except Exception as e:
                print(f"Failed to passivate joint {name} (ID {joint_id}): {e}")
        print("Motors are now passive.")

    def goto_zero(self, torque=4):
        """Move all joints to their calibrated zero positions"""
        print("Moving to zero position...")
        # Ensure we only send NUM_JOINTS positions
        target_positions = self.zero_offsets[:NUM_JOINTS]
        
        # Use synchronized move
        self.wck.posGroup(NUM_JOINTS - 1, torque, target_positions)
        time.sleep(0.5)
        print("At zero position. Stiffness is ON.")

    def read_current_pose(self):
        pose = {} # Map PLEN2 device -> value
        
        for joint_id in range(NUM_JOINTS):
            # Read from servo
            raw_pos = self.wck.readPos(joint_id)
            
            # If read failed, it might return -1 or similar. wck.py returns -1 on failure
            if raw_pos < 0:
                print(f"Warning: Could not read position for joint ID {joint_id}")
                raw_pos = self.zero_offsets[joint_id]
                
            # Convert to PLEN2 value
            # logic: plen2_value = ((position - zero_offset) * sign) / value_scale
            # But we need to handle the sign logic carefully:
            # position = zero_offset + sign * (plen2 * scale)
            # => position - zero_offset = sign * plen2 * scale
            # => (position - zero_offset) / sign = plen2 * scale
            # => plen2 = ((position - zero_offset) / sign) / scale
            # Since sign is 1 or -1, / sign is same as * sign.
            
            zero = self.zero_offsets[joint_id]
            sign = self.joint_signs[joint_id]
            
            plen2_val = ((raw_pos - zero) * sign) / PLEN2_VALUE_SCALE
            
            # Find device name
            if joint_id in self.id_to_name:
                huno_name = self.id_to_name[joint_id]
                if huno_name in self.huno_to_plen2:
                    plen2_name = self.huno_to_plen2[huno_name]
                    pose[plen2_name] = int(plen2_val)
                    
        return pose

    def create_frame(self, pose, transition_time):
        outputs = []
        for device, value in pose.items():
            outputs.append({
                "device": device,
                "value": value
            })
        
        return {
            "@index": len(self.frames),
            "transition_time_ms": transition_time,
            "outputs": outputs
        }

    def save_motion(self, name, slot=20):
        if not self.frames:
            print("No frames to save.")
            return

        filename = f"{name}.json"
        filepath = os.path.join(MOTIONS_DIR, filename)
        
        # Ensure directory exists
        if not os.path.exists(MOTIONS_DIR):
            os.makedirs(MOTIONS_DIR)
            
        motion_data = {
            "name": name,
            "slot": slot,
            "@frame_length": len(self.frames),
            "@metadata": {
                "author": "MotionGenerator",
                "target": "PLEN2",
                "required-firmware": "1.4.1~"
            },
            "codes": [],
            "frames": self.frames
        }
        
        with open(filepath, 'w') as f:
            json.dump(motion_data, f, indent=4)
        
        print(f"Motion saved to {filepath}")

    def run(self):
        print("=== Motion Generator ===")
        print("1. Motors will be set to passive mode.")
        print("2. Move robot to desired pose.")
        print("3. Press ENTER to capture keyframe.")
        print("4. Type 'q' to finish and save.")
        print("5. Type '0' to go to zero position.")
        print("6. Type 'r' to re-passivate (after going to zero).")
        
        self.set_passive_mode()
        
        # Verify passive mode (optional, just waiting for user)
        # Maybe we iterate and print current positions continuously until Enter?
        # But 'input()' blocks.
        
        frame_count = 0
        
        while True:
            cmd = input(f"\n[Frame {frame_count}] Press Enter to capture, 'q' to finish, '0' to zero, 'r' to re-passivate: ").strip().lower()
            
            if cmd == 'q':
                break
            
            if cmd == 'r':
                self.set_passive_mode()
                continue
            
            if cmd == '0':
                self.goto_zero()
                continue
            
            # Capture frame
            pose = self.read_current_pose()
            
            # Ask for duration
            try:
                duration_str = input("Transition time (ms) [default 500]: ").strip()
                duration = int(duration_str) if duration_str else 500
            except ValueError:
                duration = 500
                print("Invalid number, using 500ms")
                
            frame = self.create_frame(pose, duration)
            self.frames.append(frame)
            frame_count += 1
            print(f"Frame {frame_count-1} captured with duration {duration}ms.")

        if self.frames:
            name = input("Enter motion name: ").strip()
            if not name:
                name = "new_motion"
            
            try:
                slot_str = input("Enter slot number [default 20]: ").strip()
                slot = int(slot_str) if slot_str else 20
            except:
                slot = 20
            
            self.save_motion(name, slot)
        else:
            print("No frames captured. Exiting.")
            
        self.wck.close()

if __name__ == "__main__":
    generator = MotionGenerator()
    try:
        generator.run()
    except KeyboardInterrupt:
        print("\nInterrupted.")
        generator.wck.close()
