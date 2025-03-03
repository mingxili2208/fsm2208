#!/usr/bin/env python

import sys
sys.path.append("/home/cityu-fsm-lab-carla/Workspace/Carla/op_carla/op_bridge/op_bridge/fsm_lab_simulation")

import logging

import numpy as np
from vive_tracker import ViveTrackerModule

from scipy.spatial.transform import Rotation as R


#from std_msgs.msg import String

logging.basicConfig(level=logging.INFO)

class VR2SandBoxTransformer:

    def __init__(self, tracker_name="tracker_1"):
        self.T_pos=     [
            [-0.51372172 , 0.    ,      0.85795687 , 3.8000329 ],
            [ 0.         , 1.    ,      0.         , 0.        ],
            [-0.85795687 , 0.    ,     -0.51372172 ,-3.76950874],
            [ 0.         , 0.    ,      0.         , 1.        ]
            # [-0.51372172 , 0.      ,    0.85795687,  3.8000329 ],
            # [ 0.         , 1.      ,    0.        ,  0.        ],
            # [-0.85795687 , 0.      ,   -0.51372172, -3.66550874],
            # [ 0.         , 0.      ,    0.        ,  1.        ]

            # [-5.21673073e-01,  8.53145477e-01,  2.79930459e-06,  3.57745265e+00],
            # [-8.53145477e-01, -5.21673073e-01,  1.71032557e-06, -3.68823916e+00],
            # [ 2.91947835e-06, -1.49598326e-06,  1.00000000e+00,  2.61764485e-06],
            # [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]
            ]
        self.R_euler=   [
                [ 0.56649565,   -0.82406473 ,   0.0        ], 
                [ 0.82406473,    0.56649565 ,   0.0        ],
                [ 0.0        ,    0.0         ,   1.        ]
            ]
        self.TRACKER_NAME = tracker_name
        self.vtm = ViveTrackerModule()
        self.vtm.print_discovered_objects()
        logging.info(f' inited vtm {self.vtm.print_discovered_objects()}')
        self.tracker = self.vtm.devices[self.TRACKER_NAME]

    def transform_point_pose(self,position, orientation, T_pos, R_euler):
        """
        使用给定的位置变换矩阵 (T_pos) 和方向变换矩阵 (R_euler) 对点的方位和方向进行转换。

        参数:
            position: 点的原始位置, 3D 向量 [x, y, z]。
            orientation: 点的原始方向, 欧拉角 [roll, pitch, yaw]，单位为弧度。
            T_pos: 位置变换矩阵, 4x4 矩阵。
            R_euler: 方向变换矩阵, 3x3 矩阵。

        返回值:
            transformed_position: 转换后的位置, 3D 向量 [x', y', z']。
            transformed_orientation: 转换后的方向, 欧拉角 [roll', pitch', yaw']，单位为弧度。
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

    def transform_to_sandbox(self,cam_coord,T_pos, R_euler):
        """
        转换 Vive Tracker 相机坐标系到sandbox世界坐标系。
        """
        #roll={cam_coord[3]:.4f}, yaw={}, pitch={}
        orientation = [0, np.radians(round(cam_coord[4], 4)), 0]
        position=[round(cam_coord[0], 4), 0, round(cam_coord[2], 4)]
        ##################
        transformed_position, transformed_orientation=self.transform_point_pose(position, orientation, T_pos, R_euler)
        yaw=transformed_orientation[1]

        return transformed_position, yaw
    def get_transformed_coor(self):
        
        try:
            cam_coord = self.tracker.get_pose_euler()
            transformed_position, transformed_yaw=self.transform_to_sandbox(cam_coord,self.T_pos,self.R_euler)
            #print(f"\r Transformed position in Sandbox_coordinate: [x={transformed_position[0]:.4f}, y={transformed_position[1]:.4f}, z={transformed_position[2]:.4f}, yaw={np.rad2deg(transformed_yaw):.4f}]", end='')
            return transformed_position, transformed_yaw      
        except Exception:
            logging.fatal('!!!!!!!!!!!! get_transformed_coor error!')



#if __name__ == '__main__':
#    vst = VR2SandBoxTransformer()
#    vst.get_transformed_coor()
