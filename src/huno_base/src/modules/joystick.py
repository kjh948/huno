#!/usr/bin/python
# -*- coding: utf-8 -*-

import os
import threading
import struct
import platform
import time

# Attempt to import ‘inputs’ for cross-platform support (Mac, Windows, Linux)
# Note: 'inputs' usually requires 'PyObjC' on macOS.
try:
    import inputs
    HAS_INPUTS = True
except ImportError:
    HAS_INPUTS = False

class JoystickInput:
    """
    A cross-platform, headless-compatible joystick driver for Huno.
    Works on Debian (using joydev /dev/input/jsX) and macOS (using 'inputs' library).
    Designed to be used without a GUI (X11/Display).
    """
    def __init__(self, device_index=0, deadzone=0.1):
        self.device_index = device_index
        self.deadzone = deadzone
        self.axes = {}    # code -> float (-1.0 to 1.0)
        self.buttons = {} # code -> bool
        self.is_running = False
        self._thread = None
        self.backend = None
        
        self.system = platform.system()
        
    def start(self):
        """Starts the background listening thread."""
        if self.is_running:
            return
            
        self.is_running = True
        
        if self.system == "Linux":
            # For Debian/Linux, direct /dev/input/jsX is the most stable headless way.
            # It requires no external libraries and works reliably over SSH.
            js_path = f"/dev/input/js{self.device_index}"
            if os.path.exists(js_path):
                self.backend = "joydev"
                self._thread = threading.Thread(target=self._run_joydev, args=(js_path,))
            elif HAS_INPUTS:
                self.backend = "inputs"
                self._thread = threading.Thread(target=self._run_inputs)
        else:
            # On macOS, use the 'inputs' library (which uses IOKit).
            if HAS_INPUTS:
                self.backend = "inputs"
                self._thread = threading.Thread(target=self._run_inputs)
            else:
                print("Warning: 'inputs' library not found. Please run: pip install inputs")
                self.backend = "none"
        
        if self._thread:
            self._thread.daemon = True
            self._thread.start()
            print(f"[Joystick] Started using backend: {self.backend}")
        else:
            self.is_running = False
            print("[Joystick] Failed to start: No compatible backend or device found.")

    def _run_joydev(self, path):
        """Linux-specific raw /dev/input/jsX reader (Zero-dependency)."""
        # Event format: time (u32), value (i16), type (u8), index (u8)
        EVENT_FORMAT = 'IhBB'
        EVENT_SIZE = struct.calcsize(EVENT_FORMAT)
        
        try:
            with open(path, 'rb') as f:
                while self.is_running:
                    packet = f.read(EVENT_SIZE)
                    if not packet:
                        break
                    
                    _, value, ev_type, index = struct.unpack(EVENT_FORMAT, packet)
                    
                    # ev_type: 0x01 = Button, 0x02 = Axis
                    # Bit 0x80 is set during initialization for current state
                    is_init = ev_type & 0x80
                    actual_type = ev_type & ~0x80
                    
                    if actual_type == 0x01: # Button
                        code = f"BTN_{index}"
                        self.buttons[code] = bool(value)
                    elif actual_type == 0x02: # Axis
                        code = f"ABS_{index}"
                        # Normalize -32767..32767 to -1.0..1.0
                        norm_val = value / 32767.0
                        if abs(norm_val) < self.deadzone:
                            norm_val = 0.0
                        self.axes[code] = norm_val
        except Exception as e:
            if self.is_running:
                print(f"[Joystick] Joydev error: {e}")
            self.is_running = False

    def _run_inputs(self):
        """Cross-platform 'inputs' library reader."""
        try:
            while self.is_running:
                # get_gamepad() blocks until events are available
                events = inputs.get_gamepad()
                for event in events:
                    if event.ev_type == 'Absolute':
                        # 'inputs' normalization is tricky as it varies by device.
                        # We apply a heuristic or specific mapping.
                        val = event.state
                        # Common ranges: 0..255 (unsigned) or -32k..32k (signed)
                        if "HAT" in event.code: # D-Pad
                            norm_val = float(val)
                        elif abs(val) > 256: # 16-bit
                            norm_val = val / 32767.0
                        else: # 8-bit
                            norm_val = (val - 128) / 127.0
                        
                        if abs(norm_val) < self.deadzone:
                            norm_val = 0.0
                        self.axes[event.code] = norm_val
                        
                    elif event.ev_type == 'Key':
                        self.buttons[event.code] = bool(event.state)
        except Exception as e:
            if self.is_running:
                print(f"[Joystick] Inputs error: {e}")
            self.is_running = False

    def get_axis(self, alias_or_code, default=0.0):
        """
        Get axis value normalized to [-1.0, 1.0].
        Supports common aliases (lx, ly, rx, ry).
        """
        # Common mappings for DualShock, Xbox, and Generic controllers
        mapping = {
            "lx": ["ABS_X", "ABS_0", "axis_0", "ABS_HAT0X"],
            "ly": ["ABS_Y", "ABS_1", "axis_1", "ABS_HAT0Y"],
            "rx": ["ABS_RX", "ABS_3", "ABS_2", "axis_3", "axis_2", "ABS_2", "ABS_Z"],
            "ry": ["ABS_RY", "ABS_4", "ABS_5", "axis_4", "axis_5", "ABS_RZ"],
        }
        
        search_codes = mapping.get(alias_or_code.lower(), [alias_or_code])
        for code in search_codes:
            if code in self.axes:
                # Invert Y axes by convention for control (Up is typically 1.0)
                # Note: Most joystick APIs return 1.0 for Down.
                val = self.axes[code]
                if "LY" in code.upper() or "RY" in code.upper() or "_1" in code or "_4" in code or "_5" in code:
                    return -val
                return val
        return default

    def get_button(self, alias_or_code, default=False):
        """Get button state (True if pressed)."""
        mapping = {
            "a": ["BTN_SOUTH", "BTN_0", "BTN_A", "BTN_304"],
            "b": ["BTN_EAST", "BTN_1", "BTN_B", "BTN_305"],
            "x": ["BTN_WEST", "BTN_2", "BTN_X", "BTN_307"],
            "y": ["BTN_NORTH", "BTN_3", "BTN_Y", "BTN_308"],
            "start": ["BTN_START", "BTN_9", "BTN_315"],
            "select": ["BTN_SELECT", "BTN_8", "BTN_314"],
            "l1": ["BTN_TL", "BTN_4", "BTN_310"],
            "r1": ["BTN_TR", "BTN_5", "BTN_311"],
        }
        search_codes = mapping.get(alias_or_code.lower(), [alias_or_code])
        for code in search_codes:
            if code in self.buttons:
                return self.buttons[code]
        return default

    def stop(self):
        self.is_running = False

if __name__ == "__main__":
    # Test script
    joy = JoystickInput()
    joy.start()
    
    print("Joystick test started. Press Ctrl+C to stop.")
    try:
        while joy.is_running:
            lx = joy.get_axis("lx")
            ly = joy.get_axis("ly")
            btn_a = joy.get_button("a")
            btn_start = joy.get_button("start")
            
            # Use \r to update the same line
            print(f"\rLX: {lx:6.2f} | LY: {ly:6.2f} | A: {btn_a} | START: {btn_start} | Backend: {joy.backend}", end="")
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        joy.stop()
