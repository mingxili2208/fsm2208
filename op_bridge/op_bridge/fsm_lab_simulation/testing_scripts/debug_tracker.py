import sys
sys.path.append("/home/cityu-fsm-lab-carla/WorkplaceCarla/op_carla/op_bridge/op_bridge/fsm_lab_simulation")
import time
import numpy as np
from vive_tracker import ViveTrackerModule


TRACKER_NAME = "tracker_1"


vtm = ViveTrackerModule()
vtm.print_discovered_objects()
tracker = vtm.devices[TRACKER_NAME]
while True:
    try:
        cam_coord = tracker.get_pose_euler()
       # print(cam_coord)
        print(f"\rCamera coordinate: [x={cam_coord[0]:.4f}, y={cam_coord[1]:.4f}, z={cam_coord[2]:.4f}, roll={cam_coord[3]:.4f}, yaw={cam_coord[4]:.4f}, pitch={cam_coord[5]:.4f}]", end='')
    except Exception:
        continue
    finally:
        time.sleep(0.1)
