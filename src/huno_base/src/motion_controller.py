#!/usr/bin/python
# -*- coding: utf-8 -*-

"""
Motion Controller Module for HUNO Robot
This module provides a motion controller using the WCK servo API.
Supports PLEN2 motion file format (.json).
"""

import json
import os
import time
from wck import servo
from config import DEFAULT_SERIAL_PORT

# File paths
SCRIPT_DIR = os.path.dirname(__file__)
JOINTS_FILE = os.path.join(SCRIPT_DIR, 'joints_leg.json')
ZERO_FILE = os.path.join(SCRIPT_DIR, 'zero.json')
MOTIONS_DIR = os.path.join(SCRIPT_DIR, 'motions')

# PLEN2 device name to HUNO joint name mapping
# PLEN2 has 18 joints, HUNO has 16 joints
PLEN2_TO_HUNO_MAP = {
    # Left arm
    "left_shoulder_pitch": "j_shoulder_l",      # Shoulder pitch
    "left_shoulder_roll": "j_high_arm_l",       # Shoulder roll / High arm
    "left_elbow_roll": "j_low_arm_l",           # Elbow roll / Low arm
    
    # Right arm
    "right_shoulder_pitch": "j_shoulder_r",     # Shoulder pitch
    "right_shoulder_roll": "j_high_arm_r",      # Shoulder roll / High arm
    "right_elbow_roll": "j_low_arm_r",          # Elbow roll / Low arm
    
    # Left leg
    "left_thigh_yaw": "j_pelvis_l",             # Pelvis / Hip yaw
    "left_thigh_roll": "j_thigh2_l",            # Thigh roll
    "left_thigh_pitch": "j_tibia_l",            # Thigh pitch -> Tibia (knee area)
    "left_knee_pitch": "j_ankle1_l",            # Knee pitch -> Ankle1
    "left_foot_pitch": "j_ankle2_l",            # Foot pitch -> Ankle2
    "left_foot_roll": None,                     # Not available on HUNO (16 joints)
    
    # Right leg
    "right_thigh_yaw": "j_pelvis_r",            # Pelvis / Hip yaw
    "right_thigh_roll": "j_thigh2_r",           # Thigh roll
    "right_thigh_pitch": "j_tibia_r",           # Thigh pitch -> Tibia
    "right_knee_pitch": "j_ankle1_r",           # Knee pitch -> Ankle1
    "right_foot_pitch": "j_ankle2_r",           # Foot pitch -> Ankle2
    "right_foot_roll": None,                    # Not available on HUNO (16 joints)
}

# Value scaling factor: PLEN2 uses different unit scale
# PLEN2 value range is roughly -900 to 900, servo range is 0-254
# Zero offset is at 127, so we need to scale and offset
PLEN2_VALUE_SCALE = 10.  # Adjust this based on actual robot behavior


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


class MotionController:
    """
    Motion Controller for HUNO Robot
    
    Supports PLEN2 motion file format with the following structure:
    {
        "name": "motion_name",
        "slot": 0,
        "@frame_length": N,
        "frames": [
            {
                "@index": 0,
                "transition_time_ms": 100,
                "outputs": [
                    {"device": "left_shoulder_pitch", "value": 0},
                    ...
                ]
            },
            ...
        ]
    }
    """
    
    def __init__(self, uart=DEFAULT_SERIAL_PORT, baud=115200, auto_init=True):
        """
        Initialize the motion controller
        
        Args:
            uart: Serial port path
            baud: Baud rate for serial communication
            auto_init: If True, initialize motors to zero positions
        """
        self.wck = servo(port=uart, baud=baud)
        self.joints = load_joints()
        self.zero_offset = load_zero(self.joints)
        self.num_joints = 16
        self.cur_pose = self.zero_offset.copy()
        self.value_scale = PLEN2_VALUE_SCALE
        
        # Create reverse mapping: HUNO joint name -> motor ID
        self.joint_to_id = self.joints
        
        # Ensure motions directory exists
        if not os.path.exists(MOTIONS_DIR):
            os.makedirs(MOTIONS_DIR)
        
        if auto_init:
            self.goto_zero()
    
    def goto_zero(self, torque=4):
        """Move all joints to their calibrated zero positions"""
        print("Moving to zero position...")
        self.wck.posGroup(self.num_joints - 1, torque, self.zero_offset)
        self.cur_pose = self.zero_offset.copy()
        time.sleep(0.5)
        print("At zero position")
    
    def set_pose(self, pose, torque=1):
        """
        Set all joints to the specified pose
        
        Args:
            pose: List of 16 joint positions (0-254)
            torque: Torque level (0-7)
        """
        if len(pose) != self.num_joints:
            raise ValueError(f"Pose must have {self.num_joints} values")
        
        self.wck.posGroup(self.num_joints - 1, torque, pose)

        print(pose)

        self.cur_pose = pose.copy()
    
    def set_joint(self, joint_id, position, torque=4):
        """
        Set a single joint position
        
        Args:
            joint_id: Motor ID (0-15)
            position: Target position (0-254)
            torque: Torque level (0-7)
        """
        position = max(0, min(254, int(position)))
        self.wck.pos(joint_id, torque, position)
        self.cur_pose[joint_id] = position
    
    def plen2_value_to_servo(self, plen2_value, joint_id):
        """
        Convert PLEN2 motion value to servo position
        
        PLEN2 values are relative offsets from zero position.
        We apply the offset to the calibrated zero position.
        
        Args:
            plen2_value: Value from PLEN2 motion file
            joint_id: Motor ID to get the zero offset
            
        Returns:
            Servo position (0-254)
        """
        # Apply scaling and add to zero offset
        offset = int(plen2_value * self.value_scale)
        position = self.zero_offset[joint_id] + offset
        
        # Clamp to valid range
        return max(0, min(254, position))
    
    def parse_frame_to_pose(self, frame):
        """
        Parse a PLEN2 motion frame to servo pose
        
        Args:
            frame: Frame dict with 'outputs' list
            
        Returns:
            Tuple of (pose_list, transition_time_ms)
        """
        pose = self.zero_offset.copy()
        transition_time_ms = frame.get('transition_time_ms', 100)
        
        for output in frame.get('outputs', []):
            device = output.get('device')
            value = output.get('value', 0)
            
            # Map PLEN2 device to HUNO joint
            huno_joint = PLEN2_TO_HUNO_MAP.get(device)
            if huno_joint is None:
                # This device is not available on HUNO, skip
                continue
            
            # Get joint ID
            if huno_joint in self.joint_to_id:
                joint_id = self.joint_to_id[huno_joint]
                pose[joint_id] = self.plen2_value_to_servo(value, joint_id)
        
        return pose, transition_time_ms
    
    def interpolate_pose(self, start_pose, end_pose, duration_ms, torque=4):
        """
        Smoothly interpolate between two poses over a given duration
        
        Args:
            start_pose: Starting pose (list of 16 values)
            end_pose: Target pose (list of 16 values)
            duration_ms: Duration in milliseconds
            torque: Torque level
        """
        # Calculate steps based on duration (~50Hz update rate = 20ms per step)
        step_time_ms = 20
        steps = max(1, int(duration_ms / step_time_ms))
        actual_delay = duration_ms / 1000.0 / steps
        
        for step in range(1, steps + 1):
            interpolated = []
            for i in range(self.num_joints):
                value = int(start_pose[i] + (end_pose[i] - start_pose[i]) * step / steps)
                interpolated.append(value)
            self.set_pose(interpolated, torque)
            time.sleep(actual_delay)
    
    def load_motion(self, filename):
        """
        Load a motion from a JSON file
        
        Args:
            filename: Name of the file (with or without .json extension)
            
        Returns:
            Motion data dictionary or None if not found
        """
        if not filename.endswith('.json'):
            filename = f"{filename}.json"
        
        filepath = os.path.join(MOTIONS_DIR, filename)
        
        if os.path.exists(filepath):
            with open(filepath, 'r') as f:
                motion = json.load(f)
            print(f"Motion loaded: {motion.get('name', filename)} ({len(motion.get('frames', []))} frames)")
            return motion
        else:
            print(f"Motion file not found: {filepath}")
            return None
    
    def play_motion(self, motion, speed=1.0, torque=4, loop=False):
        """
        Play a PLEN2 format motion
        
        Args:
            motion: Motion data dict or filename string
            speed: Speed multiplier (1.0 = normal, 2.0 = double speed, 0.5 = half speed)
            torque: Torque level (0-7)
            loop: If True, loop the motion indefinitely
        """
        # Load motion if string is provided
        if isinstance(motion, str):
            motion = self.load_motion(motion)
            if motion is None:
                return
        
        motion_name = motion.get('name', 'unnamed')
        frames = motion.get('frames', [])
        
        if not frames:
            print("No frames in motion")
            return
        
        print(f"Playing motion: {motion_name} (speed: {speed}x)")
        
        try:
            while True:
                prev_pose = self.cur_pose.copy()
                
                for i, frame in enumerate(frames):
                    target_pose, transition_time_ms = self.parse_frame_to_pose(frame)
                    
                    # Apply speed multiplier
                    adjusted_time_ms = int(transition_time_ms / speed)
                    
                    # Interpolate to target pose
                    self.interpolate_pose(prev_pose, target_pose, adjusted_time_ms, torque)
                    prev_pose = target_pose.copy()
                    
                    print(f"  Frame {i + 1}/{len(frames)} complete (time: {adjusted_time_ms}ms)")
                
                if not loop:
                    break
                    
                print("  Looping...")
                
        except KeyboardInterrupt:
            print("\nMotion interrupted by user")
        
        print("Motion complete")
    
    def play_motion_file(self, filename, speed=1.0, torque=4, loop=False):
        """
        Play a motion from a file
        
        Args:
            filename: Motion filename (with or without .json)
            speed: Speed multiplier
            torque: Torque level
            loop: Whether to loop
        """
        motion = self.load_motion(filename)
        if motion:
            self.play_motion(motion, speed, torque, loop)
    
    def list_motions(self):
        """List all available motion files"""
        motions = []
        if os.path.exists(MOTIONS_DIR):
            for f in os.listdir(MOTIONS_DIR):
                if f.endswith('.json'):
                    motions.append(f[:-5])  # Remove .json extension
        return sorted(motions)
    
    def get_motion_info(self, filename):
        """
        Get information about a motion file
        
        Args:
            filename: Motion filename
            
        Returns:
            Dict with motion info
        """
        motion = self.load_motion(filename)
        if motion:
            total_time_ms = sum(f.get('transition_time_ms', 100) for f in motion.get('frames', []))
            return {
                'name': motion.get('name', 'unnamed'),
                'slot': motion.get('slot', -1),
                'frame_count': len(motion.get('frames', [])),
                'total_duration_ms': total_time_ms,
                'total_duration_sec': total_time_ms / 1000.0,
                'metadata': motion.get('@metadata', {})
            }
        return None
    
    def record_pose(self):
        """Read current pose from all servos"""
        pose = []
        for i in range(self.num_joints):
            pos = self.wck.readPos(i)
            pose.append(pos if pos >= 0 else self.zero_offset[i])
        self.cur_pose = pose.copy()
        return pose
    
    def set_value_scale(self, scale):
        """
        Set the PLEN2 value scaling factor
        
        Args:
            scale: Scaling factor (default is 0.1)
        """
        self.value_scale = scale
        print(f"Value scale set to {scale}")
    
    def close(self):
        """Close the serial connection"""
        self.wck.close()
        print("Connection closed")


def interactive_mode():
    """Run in interactive mode for testing"""
    print("=== HUNO Motion Controller ===\n")
    
    try:
        mc = MotionController(auto_init=True)
        
        while True:
            print("\n--- Menu ---")
            print("1. List motions")
            print("2. Play motion")
            print("3. Play motion (loop)")
            print("4. Motion info")
            print("5. Go to zero")
            print("6. Set speed scale")
            print("7. Set value scale")
            print("0. Exit")
            
            choice = input("\nSelect: ").strip()
            
            if choice == '1':
                motions = mc.list_motions()
                print("\nAvailable motions:")
                for m in motions:
                    print(f"  - {m}")
            
            elif choice == '2':
                name = input("Motion name: ").strip()
                speed = float(input("Speed (1.0 = normal): ").strip() or "1.0")
                mc.play_motion_file(name, speed=speed)
            
            elif choice == '3':
                name = input("Motion name: ").strip()
                speed = float(input("Speed (1.0 = normal): ").strip() or "1.0")
                print("Press Ctrl+C to stop")
                mc.play_motion_file(name, speed=speed, loop=True)
            
            elif choice == '4':
                name = input("Motion name: ").strip()
                info = mc.get_motion_info(name)
                if info:
                    print(f"\nMotion: {info['name']}")
                    print(f"  Slot: {info['slot']}")
                    print(f"  Frames: {info['frame_count']}")
                    print(f"  Duration: {info['total_duration_sec']:.2f}s")
            
            elif choice == '5':
                mc.goto_zero()
            
            elif choice == '6':
                speed = float(input("Speed multiplier: ").strip())
                print(f"Speed set to {speed}x")
            
            elif choice == '7':
                scale = float(input("Value scale (default 0.1): ").strip())
                mc.set_value_scale(scale)
            
            elif choice == '0':
                break
        
        mc.close()
        
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    interactive_mode()
