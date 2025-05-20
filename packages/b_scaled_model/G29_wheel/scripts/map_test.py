#!/usr/bin/env python3

import evdev
from evdev import ecodes
import time
import os

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