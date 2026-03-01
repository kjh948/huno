#!/usr/bin/python
# -*- coding: utf-8 -*-

import sys
import os
import time
import math
import json
import threading

# huno_base/src is now current directory
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
HUNO_BASE_SRC = SCRIPT_DIR
# sys.path.append(HUNO_BASE_SRC) # Not strictly needed if running from same dir, but good for safety

try:
    from wck import servo
    from config import DEFAULT_SERIAL_PORT
except ImportError:
    print("Error: Could not import wck or config from huno_base/src")
    print(f"Path searched: {HUNO_BASE_SRC}")
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

class ParallelKinematicsWalk:
    def __init__(self, port=DEFAULT_SERIAL_PORT, baud=115200):
        print(f"Connecting to servos on {port}...")
        try:
            self.wck = servo(port=port, baud=baud)
        except Exception as e:
            print(f"Failed to connect: {e}")
            sys.exit(1)

        self.joints_map = load_json_file(JOINTS_FILE, {})
        self.zero_map = load_json_file(ZERO_FILE, {})
        
        # Invert joints map: ID -> Name
        self.id_to_name = {v: k for k, v in self.joints_map.items()}
        
        # Parameters (Default from walk_open_loop_simple.py)
        self.f = 8.0            # Frequency
        self.robot_height = 0.5 # Knee bend offset (z)
        self.shift_y = 0.2      # Lateral shift (y)
        self.step_height = 0.4  # Step lift (z amplitude)
        self.step_length = 0.2  # Step length (x amplitude)
        self.arm_swing = 1.0    # Arm swing magnitude
        self.direction = -1 #forward: -1, backward: 1
        self.side_step = 0.0    # Sideways step amplitude (y)
        
        # Scaling factor from algorithm units to servo units
        self.scale = 30.0
        
        # Joint signs (to handle mounting directions)
        ## backward
        self.signs = {name: 1.0 for name in self.joints_map.keys()}
        self.signs["j_ankle1_r"] = -1.0
        self.signs["j_tibia_l"] = -1.0
        self.signs["j_thigh2_l"] = -1.0

        self.is_walking = False
        self.should_exit = False
        
        # Cache zero positions
        self.zeros = [127] * 16
        for name, joint_id in self.joints_map.items():
            if name in self.zero_map:
                self.zeros[joint_id] = self.zero_map[name]

        # Current target positions
        self.targets = list(self.zeros)

        # Background thread for walk loop
        self.walk_thread = threading.Thread(target=self.run_walk_loop)
        self.walk_thread.daemon = True
        self.walk_thread.start()

    def set_joint_val(self, name, val_offset):
        if name not in self.joints_map:
            return
        joint_id = self.joints_map[name]
        
        # Apply scaling and individual joint sign
        offset = int(val_offset * self.scale * self.signs[name])
        target = self.zeros[joint_id] + offset
        
        # Clamp to 1-254
        target = max(1, min(254, target))
        self.targets[joint_id] = target

    def interpolate_pose(self, start_pose, end_pose, duration_ms, torque=4):
        """Smoothly interpolate between two poses over a given duration"""
        step_time_ms = 20
        steps = max(1, int(duration_ms / step_time_ms))
        actual_delay = duration_ms / 1000.0 / steps
        
        for step in range(1, steps + 1):
            interpolated = []
            ratio = step / float(steps)
            for i in range(16):
                val = start_pose[i] + (end_pose[i] - start_pose[i]) * ratio
                interpolated.append(max(1, min(254, int(val))))
            self.wck.posGroup(15, torque, interpolated)
            time.sleep(actual_delay)
        
        # Ensure target is reached
        self.wck.posGroup(15, torque, end_pose)
    '''### 관절 각도 계산 원리:
    - **무릎(`KneePitch`)**: 높이 `z`를 직접 반영
    - **엉덩이(`HipPitch`)**: 무릎 각도의 절반을 보정 + 전후 위치 `x`
    - **발목(`AnklePitch`)**: 무릎 각도의 절반을 반대로 보정 + 전후 위치 `x`
    - **엉덩이/발목 롤(`Roll`)**: 좌우 위치 `y`를 반대 부호로 설정하여 발 평행 유지'''
    def left_leg(self, x, y, z):
        # Parallel Kinematics Algorithm
        self.set_joint_val("j_tibia_l",  z)          # Knee
        self.set_joint_val("j_thigh2_l", -z/2.0 + x) # Hip
        self.set_joint_val("j_ankle1_l", -z/2.0 - x) # Ankle Pitch
        
        self.set_joint_val("j_pelvis_l",  y)         # Hip Roll
        self.set_joint_val("j_ankle2_l", -y)         # Ankle Roll

    def right_leg(self, x, y, z):
        self.set_joint_val("j_tibia_r",  z)
        self.set_joint_val("j_thigh2_r", -z/2.0 + x)
        self.set_joint_val("j_ankle1_r", -z/2.0 - x)
        
        self.set_joint_val("j_pelvis_r",  y)
        self.set_joint_val("j_ankle2_r", -y)

    def update_arms(self, xLeft, xRight):
        self.set_joint_val("j_shoulder_l", self.arm_swing * xRight)
        self.set_joint_val("j_shoulder_r", self.arm_swing * xLeft)

    def run_walk_loop(self):
        print("Walk loop started.")
        was_walking = False
        while not self.should_exit:
            if self.is_walking:
                # Modulate frequency
                t = (time.time()) * self.direction * self.f
                
                # 1. Lateral sway for balance
                y_sway = math.sin(t) * self.shift_y
                
                # 2. Individual leg lateral movement (side-step)
                y_left_step  = math.cos(t) * self.side_step
                y_right_step = math.cos(t + math.pi) * self.side_step

                # 3. Combine sway and step
                yLeft  = y_sway + y_left_step
                yRight = y_sway + y_right_step
                
                # vertical lift (knee bend)
                zLeft  = (math.sin(t)           + 1.0) / 2.0 * self.step_height + self.robot_height
                zRight = (math.sin(t + math.pi) + 1.0) / 2.0 * self.step_height + self.robot_height

                # forward/backward swing
                xLeft  = math.cos(t)           * self.step_length
                xRight = math.cos(t + math.pi) * self.step_length
                
                self.left_leg(xLeft, yLeft, zLeft)
                self.right_leg(xRight, yRight, zRight)
                self.update_arms(xLeft, xRight)
                
                # Synchronized move (all 16 possible joints)
                self.wck.posGroup(15, 4, self.targets)
                was_walking = True
                sleep_time = 0.02
            else:
                if was_walking:
                    # Smoothly return to zero position
                    print("\nReturning to zero...")
                    self.interpolate_pose(self.targets, self.zeros, 500)
                    # Update targets back to zeros
                    self.targets = list(self.zeros)
                    was_walking = False
                
                # Stand at zero
                self.wck.posGroup(15, 4, self.zeros)
                sleep_time = 0.1
                
            time.sleep(sleep_time)

    def start(self):
        self.is_walking = True
        print("\n>>> Walking STARTED")

    def stop(self):
        self.is_walking = False
        print("\n>>> Walking STOPPED")

    def print_params(self):
        print("\n--- Current Parameters ---")
        print(f"  (f)  Frequency:    {self.f:<8.2f} (h)  Robot Height: {self.robot_height:.2f}")
        print(f"  (y)  Shift Y:      {self.shift_y:<8.2f} (sh) Step Height:  {self.step_height:.2f}")
        print(f"  (sl) Step Length:   {self.step_length:<8.2f} (ss) Side Step:    {self.side_step:.2f}")
        print(f"  (as) Arm Swing:    {self.arm_swing:<8.2f} (d)  Direction:    {self.direction:.0f}")
        print(f"  (sc) Servo Scale: {self.scale:<8.2f}")
        print("  Status: " + ("WALKING" if self.is_walking else "STOPPED"))

    def interactive_menu(self):
        self.print_params()
        print("\nCommands: start, stop, q (quit), params (show)")
        print("To change value: <param_code> <value> (e.g., 'f 10')")
        print("To toggle joint sign: sign <joint_name> (e.g., 'sign j_tibia_l')")
        
        while not self.should_exit:
            try:
                line = input("PK> ").strip().lower()
            except EOFError:
                break
                
            if not line: continue
            
            parts = line.split()
            cmd = parts[0]
            
            if cmd == 'q':
                self.should_exit = True
                self.stop()
            elif cmd == 'start':
                self.start()
            elif cmd == 'stop':
                self.stop()
            elif cmd == 'params':
                self.print_params()
            elif cmd == 'sign' and len(parts) > 1:
                name = parts[1]
                if name in self.signs:
                    self.signs[name] *= -1
                    print(f"Sign of {name} now {self.signs[name]}")
                else:
                    print(f"Unknown joint: {name}")
            elif len(parts) > 1:
                try:
                    val = float(parts[1])
                    if cmd == 'f': self.f = val
                    elif cmd == 'h': self.robot_height = val
                    elif cmd == 'y': self.shift_y = val
                    elif cmd == 'sh': self.step_height = val
                    elif cmd == 'sl': self.step_length = val
                    elif cmd == 'ss': self.side_step = val
                    elif cmd == 'as': self.arm_swing = val
                    elif cmd == 'sc': self.scale = val
                    elif cmd == 'd': self.direction = val
                    else: print(f"Unknown parameter: {cmd}")
                except ValueError:
                    print("Invalid value format")
            else:
                print("Unknown command. Type 'params' to see shortcuts.")


if __name__ == "__main__":
    pk = ParallelKinematicsWalk()
    
    try:
        pk.interactive_menu()
    except KeyboardInterrupt:
        print("\nInterrupt received, returning to zero pose...")
        pk.is_walking = False
        # Wait for the walk thread to finish interpolation
        time.sleep(1.2)
        pk.should_exit = True
    
    print("Exiting...")
    pk.wck.close()
