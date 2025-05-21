#!/usr/bin/env python3

import evdev
from evdev import ecodes
import time
import os
import math

# --- Configuration ---
DEVICE_NAME = "Logitech G29 Driving Force Racing Wheel"  # Adjust for your device (G920, etc.)

# Key mappings from the table
KEY_MAPPINGS = {
    # Axes
    ecodes.ABS_X: {"name": "wheel", "meaning": "方向盘", "event_name": "ABS_X", "event_code": 0},
    ecodes.ABS_Z: {"name": "Accelerater", "meaning": "油门", "event_name": "ABS_Z", "event_code": 2},
    ecodes.ABS_RZ: {"name": "Brake", "meaning": "刹车", "event_name": "ABS_RZ", "event_code": 5},
    
    # Buttons
    292: {"name": "Right_pick", "meaning": "右转向", "event_name": "BTN_TOP2", "event_code": 292},
    293: {"name": "Left_pick", "meaning": "左转向", "event_name": "BTN_PINKIE", "event_code": 293},
    711: {"name": "ENTER", "meaning": "喇叭/蜂鸣器", "event_name": "BTN_TRIGGER_HAPPY8", "event_code": 711},
    294: {"name": "R2", "meaning": "TO_MANNUR转人工", "event_name": "BTN_BASE", "event_code": 294},
    298: {"name": "R3", "meaning": "TO_AUTO转自动", "event_name": "BTN_BASE5", "event_code": 298},
    302: {"name": "Gear_Forward", "meaning": "档位前推~后退", "event_name": "-", "event_code": 302},
    303: {"name": "Gear_Push_Back", "meaning": "档位后推~前进", "event_name": "BTN_DEAD", "event_code": 303},
}

# Map event codes to our KEY_MAPPINGS by event code
EVENT_CODE_TO_KEY = {
    ecodes.EV_ABS: {
        0: KEY_MAPPINGS[ecodes.ABS_X],
        2: KEY_MAPPINGS[ecodes.ABS_Z],
        5: KEY_MAPPINGS[ecodes.ABS_RZ],
    },
    ecodes.EV_KEY: {
        292: KEY_MAPPINGS[292],
        293: KEY_MAPPINGS[293],
        711: KEY_MAPPINGS[711],
        294: KEY_MAPPINGS[294],
        298: KEY_MAPPINGS[298],
        302: KEY_MAPPINGS[302],
        303: KEY_MAPPINGS[303],
    }
}

# --- Mapping Functions ---
def map_wheel(value):
    """
    Map wheel value from evdev range (0-65535) to steering angle (-25 to +25 degrees)
    Wheel center is around 33000
    Physical wheel range is 0-900 degrees (450 left, 450 right from center)
    """
    # Define wheel center and calibration parameters
    wheel_center = 33000
    evdev_range = 65535
    physical_range_degrees = 900
    target_range_degrees = 50  # -25 to +25
    
    # Calculate normalized position (-1 to 1) relative to center
    normalized_pos = (value - wheel_center) / (evdev_range / 2)
    # Limit to range of -1 to 1
    normalized_pos = max(-1, min(1, normalized_pos))
    
    # Map to target range (-25 to +25 degrees)
    steering_angle = normalized_pos * (target_range_degrees / 2)
    
    return steering_angle

def map_brake(value):
    """
    Non-linear mapping for brake
    Input range: 255 (no brake) to 0 (full brake)
    Output range: 0.0 (no brake) to 1.0 (full brake)
    Uses exponential curve for more sensitivity as brake is pressed harder
    """
    # Invert value so 0 is no brake, 255 is full brake
    inverted = 255 - value
    
    # Normalize to 0-1 range
    normalized = inverted / 255.0
    
    # Apply non-linear mapping (exponential curve: x^2)
    # This makes the brake more responsive when pressed harder
    mapped_value = normalized ** 2
    
    return mapped_value

def map_accelerator(value):
    """
    Non-linear mapping for accelerator
    Input range: 255 (no gas) to 0 (full gas)
    Output range: 0.0 (no gas) to 1.0 (full gas)
    Uses exponential curve for more sensitivity as pedal is pressed harder
    """
    # Invert value so 0 is no gas, 255 is full gas
    inverted = 255 - value
    
    # Normalize to 0-1 range
    normalized = inverted / 255.0
    
    # Apply non-linear mapping (exponential curve: x^2)
    # This makes the accelerator more responsive when pressed harder
    mapped_value = normalized ** 2
    
    return mapped_value

def find_device_path(device_name):
    """Find input device path containing the specified name"""
    devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
    for device in devices:
        if device_name.lower() in device.name.lower():
            print(f"Found device: {device.name} ({device.path})")
            return device.path
    return None

def main():
    if os.geteuid() != 0:
        print("Warning: This script may need to be run with root privileges (sudo) to access certain devices.")

    dev_path = find_device_path(DEVICE_NAME)

    if not dev_path:
        print(f"Error: Device '{DEVICE_NAME}' not found. Please check the connection or modify the device name.")
        print("Available devices:")
        for path in evdev.list_devices():
            try:
                dev = evdev.InputDevice(path)
                print(f"  {path}: {dev.name}")
            except Exception:
                print(f"  {path}: (Unable to get name)")
        return

    try:
        device = evdev.InputDevice(dev_path)
    except PermissionError:
        print(f"Error: Permission denied when trying to open device {dev_path}. Try running with 'sudo'.")
        return
    except Exception as e:
        print(f"Error opening device: {e}")
        return

    print(f"\nListening to device: {device.name}")
    print(f"Physical path: {device.phys}")
    
    # Try to disable force feedback (auto-centering)
    try:
        if ecodes.FF_AUTOCENTER in device.capabilities().get(ecodes.EV_FF, []):
            print("Attempting to set auto-center force to 0...")
            device.set_autocenter(0)
            print("Auto-center force feedback has been set to 0.")
        else:
            print("Device does not support direct auto-center adjustment.")
    except Exception as e:
        print(f"Error disabling force feedback: {e}")
        print("Continuing to read input values...")

    print("\n--- Reading input values (Press Ctrl+C to exit) ---")
    print("Monitoring these controls:")
    for event_type, events in EVENT_CODE_TO_KEY.items():
        for code, info in events.items():
            print(f"  {info['name']} ({info['meaning']}): event_type={event_type}, event_code={code}")
    
    print("\n--- Events will appear below ---")
    
    # Store current axis values
    current_values = {}

    try:
        for event in device.read_loop():
            # Only process events we're interested in
            if event.type in EVENT_CODE_TO_KEY and event.code in EVENT_CODE_TO_KEY[event.type]:
                control = EVENT_CODE_TO_KEY[event.type][event.code]
                
                # For axes (ABS events)
                if event.type == ecodes.EV_ABS:
                    current_values[control['name']] = event.value
                    
                    # Apply appropriate mapping based on control
                    if control['name'] == "wheel":
                        mapped_value = map_wheel(event.value)
                        print(f"{control['name']} ({control['meaning']}): Raw={event.value}, Mapped={mapped_value:.2f}° (-25 to +25)")
                    
                    elif control['name'] == "Brake":
                        mapped_value = map_brake(event.value)
                        print(f"{control['name']} ({control['meaning']}): Raw={event.value}, Mapped={mapped_value:.3f} (0.0-1.0)")
                    
                    elif control['name'] == "Accelerater":
                        mapped_value = map_accelerator(event.value)
                        print(f"{control['name']} ({control['meaning']}): Raw={event.value}, Mapped={mapped_value:.3f} (0.0-1.0)")
                    
                    else:
                        print(f"{control['name']} ({control['meaning']}): {event.value}")
                
                # For buttons (KEY events)
                elif event.type == ecodes.EV_KEY:
                    state = "Pressed" if event.value == 1 else "Released" if event.value == 0 else f"Repeat ({event.value})"
                    print(f"{control['name']} ({control['meaning']}): {state}")

    except KeyboardInterrupt:
        print("\nScript interrupted by user.")
    except Exception as e:
        print(f"\nError reading events: {e}")
    finally:
        print("Closing device.")
        if 'device' in locals():
            device.close()

if __name__ == "__main__":
    main()