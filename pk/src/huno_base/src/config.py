#!/usr/bin/python
# -*- coding: utf-8 -*-

"""
Common configuration and utilities for HUNO robot
"""

import os
import platform
import json
import glob

# Detect OS and set default serial port
def get_default_serial_port():
    """Get the default serial port based on the operating system
    
    On macOS, automatically finds /dev/tty.wch* devices since the
    number suffix changes on each connection.
    """
    system = platform.system()
    
    if system == 'Darwin':  # macOS
        # Find WCH USB serial devices (number changes each connection)
        wch_devices = glob.glob('/dev/tty.usb*')
        if wch_devices:
            # Return the first found device
            device = sorted(wch_devices)[0]
            print(f"Found serial device: {device}")
            return device
        else:
            # Fallback if no device found
            print("Warning: No WCH USB serial device found in /dev/tty.wch*")
            return '/dev/tty.wchusbserial144210'
    elif system == 'Linux':
        return '/dev/ttyUSB0'
    elif system == 'Windows':
        return 'COM3'
    else:
        return '/dev/ttyUSB0'

# Default serial port
DEFAULT_SERIAL_PORT = get_default_serial_port()
DEFAULT_BAUD_RATE = 115200

# File paths
SCRIPT_DIR = os.path.dirname(__file__)
JOINTS_FILE = os.path.join(SCRIPT_DIR, 'joints_leg.json')
ZERO_FILE = os.path.join(SCRIPT_DIR, 'zero.json')
MOTIONS_DIR = os.path.join(SCRIPT_DIR, 'motions')

# Default joint mappings
DEFAULT_JOINTS = {
    "j_ankle1_l": 3,
    "j_ankle1_r": 8,
    "j_ankle2_l": 4,
    "j_ankle2_r": 9,
    "j_tibia_l": 2,
    "j_tibia_r": 7,
    "j_thigh2_l": 1,
    "j_thigh2_r": 6,
    "j_pelvis_l": 0,
    "j_pelvis_r": 5,
    # "j_shoulder_l": 10,
    # "j_shoulder_r": 13,
    # "j_high_arm_l": 11,
    # "j_high_arm_r": 14,
    # "j_low_arm_l": 12,
    # "j_low_arm_r": 15,
}

# Default zero positions
DEFAULT_ZERO = [122, 208, 158, 67, 103, 123, 33, 91, 180, 143, 70, 38, 124, 171, 210, 130]


def load_joints():
    """Load joint mappings from joints.json"""
    if os.path.exists(JOINTS_FILE):
        try:
            with open(JOINTS_FILE, 'r') as f:
                return json.load(f)
        except Exception as e:
            print(f"Error loading joints.json, using default: {e}")
            return DEFAULT_JOINTS.copy()
    return DEFAULT_JOINTS.copy()


def load_zero(joints_map=None, num_joints=16):
    """Load zero position values from zero.json"""
    if joints_map is None:
        joints_map = load_joints()
    
    if os.path.exists(ZERO_FILE):
        try:
            with open(ZERO_FILE, 'r') as f:
                zero_dict = json.load(f)
            zero_list = DEFAULT_ZERO.copy()
            for name, value in zero_dict.items():
                if name in joints_map:
                    zero_list[joints_map[name]] = value
            return zero_list
        except Exception as e:
            print(f"Error loading zero.json, using default: {e}")
            return DEFAULT_ZERO.copy()
    return DEFAULT_ZERO.copy()


def save_joints(joints_data):
    """Save joint mappings to joints.json"""
    try:
        with open(JOINTS_FILE, 'w') as f:
            json.dump(joints_data, f, indent=4)
        print(f"Saved joints to {JOINTS_FILE}")
    except Exception as e:
        print(f"Error saving joints: {e}")


def save_zero(zero_data, joints_map=None):
    """Save zero position values to zero.json"""
    if joints_map is None:
        joints_map = load_joints()
    
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
