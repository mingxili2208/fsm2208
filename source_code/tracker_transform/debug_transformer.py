#!/usr/bin/env python
import sys
sys.path.append("/home/cityu-fsm-lab-carla/Workspace/Carla/op_carla/op_bridge/op_bridge/fsm_lab_simulation")
import time
import numpy as np
from vive_tracker import ViveTrackerModule

from scipy.spatial.transform import Rotation as R

T_pos=  [
            [-0.54573909, 0.,   0.83795516,     3.77076165],
            [ 0.,         1.,   0.        ,     0.        ],
            [-0.83795516, 0.,  -0.54573909,    -3.79733569],
            [ 0.,         0.,   0.        ,     1.        ]
        ]
R_euler=[
            [ 0.56649565,   -0.82406473 ,   0.0        ], 
            [ 0.82406473,    0.56649565 ,   0.0        ],
            [ 0.0        ,    0.0         ,   1.        ]
        ]


def transform_to_sandbox(cam_coord,T_pos, R_euler):
    """
    转换 Vive Tracker 相机坐标系到sandbox世界坐标系。
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
    使用给定的位置变换矩阵 (T_pos) 和方向变换矩阵 (R_euler) 对点的方位和方向进行转换。

    参数:
        position: 点的原始位置，3D 向量 [x, y, z]。
        orientation: 点的原始方向，欧拉角 [roll, pitch, yaw]，单位为弧度。
        T_pos: 位置变换矩阵，4x4 矩阵。
        R_euler: 方向变换矩阵，3x3 矩阵。

    返回值:
        transformed_position: 转换后的位置，3D 向量 [x', y', z']。
        transformed_orientation: 转换后的方向，欧拉角 [roll', pitch', yaw']，单位为弧度。
    """

    # 将位置转换为齐次坐标
    position_homogeneous = np.append(position, 1)

    # 应用位置变换
    transformed_position_homogeneous = T_pos @ position_homogeneous
    transformed_position = transformed_position_homogeneous[:3]

    # 将欧拉角转换为旋转矩阵
    R_point = R.from_euler('yxz', [orientation[2], orientation[0], orientation[1]]).as_matrix()

    # 应用方向变换
    R_transformed = R_euler @ R_point

    # 将旋转矩阵转换回欧拉角
    transformed_orientation = R.from_matrix(R_transformed).as_euler('yxz')

    # 调整欧拉角顺序
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
