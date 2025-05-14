#!/usr/bin/env python
import sys
sys.path.append("/home/cityu-fsm-lab-carla/WorkplaceCarla/op_carla/op_bridge/op_bridge/fsm_lab_simulation")
import time
import numpy as np
from vive_tracker import ViveTrackerModule

from scipy.spatial.transform import Rotation as R

# Transformation matrix from Vive Tracker to Sandbox coordinate system
T_pos=  [
            [-0.54573909, 0.,   0.83795516,     3.77076165],
            [ 0.,         1.,   0.        ,     0.        ],
            [-0.83795516, 0.,  -0.54573909,    -3.79733569],
            [ 0.,         0.,   0.        ,     1.        ]
        ]
# Rotation matrix from Vive Tracker to Sandbox coordinate system
R_euler=[
            [ 0.56649565,   -0.82406473 ,   0.0        ],
            [ 0.82406473,    0.56649565 ,   0.0        ],
            [ 0.0        ,    0.0         ,   1.        ]
        ]


def transform_to_sandbox(cam_coord,T_pos, R_euler):
    """
    Transform Vive Tracker camera coordinates to sandbox world coordinates.
    """
    #roll={cam_coord[3]:.4f}, yaw={}, pitch={}
    orientation = [0, np.radians(round(cam_coord[4], 4)), 0]
    position=[round(cam_coord[0], 4), 0, round(cam_coord[2], 4)]
    ##################
    transformed_position, transformed_orientation=transform_point_pose(position, orientation, T_pos, R_euler)
    yaw=transformed_orientation[1]

    return transformed_position, yaw


def transform_point_pose(position, orientation, T_pos, R_euler):
    """
    Transforms the position and orientation of a point using the given position transformation matrix (T_pos) and orientation transformation matrix (R_euler).

    Args:
        position: The original position of the point, a 3D vector [x, y, z].
        orientation: The original orientation of the point, Euler angles [roll, pitch, yaw] in radians.
        T_pos: The position transformation matrix, a 4x4 matrix.
        R_euler: The orientation transformation matrix, a 3x3 matrix.

    Returns:
        transformed_position: The transformed position, a 3D vector [x', y', z'].
        transformed_orientation: The transformed orientation, Euler angles [roll', pitch', yaw'] in radians.
    """

    # Convert position to homogeneous coordinates
    position_homogeneous = np.append(position, 1)

    # Apply position transformation
    transformed_position_homogeneous = T_pos @ position_homogeneous
    transformed_position = transformed_position_homogeneous[:3]

    # Convert Euler angles to rotation matrix
    R_point = R.from_euler('yxz', [orientation[2], orientation[0], orientation[1]]).as_matrix()

    # Apply orientation transformation
    R_transformed = R_euler @ R_point

    # Convert rotation matrix back to Euler angles
    transformed_orientation = R.from_matrix(R_transformed).as_euler('yxz')

    # Adjust Euler angle order
    transformed_orientation = [transformed_orientation[0], transformed_orientation[2], transformed_orientation[1]]
    #print(transformed_orientation)

    return transformed_position, transformed_orientation

TRACKER_NAME = "tracker_1"


vtm = ViveTrackerModule()
vtm.print_discovered_objects()
tracker = vtm.devices[TRACKER_NAME]
while True:
    try:
        cam_coord = tracker.get_pose_euler()
       # print(cam_coord)
        #print(f"\rCamera coordinate: [x={cam_coord[0]:.4f}, y={cam_coord[1]:.4f}, z={cam_coord[2]:.4f}, roll={cam_coord[3]:.4f}, yaw={cam_coord[4]:.4f}, pitch={cam_coord[5]:.4f}]", end='')
        transformed_position, transformed_yaw=transform_to_sandbox(cam_coord,T_pos,R_euler)
        print(f"\r Transformed position in Sandbox_coordinate: [x={transformed_position[0]:.4f}, y={transformed_position[1]:.4f}, z={transformed_position[2]:.4f}, yaw={np.rad2deg(transformed_yaw):.4f}]", end='')
    except Exception:
        continue
    finally:
        time.sleep(0.1)