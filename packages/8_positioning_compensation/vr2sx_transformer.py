#!/usr/bin/env python

import sys
sys.path.append("/home/cityu-fsm-lab-carla/WorkplaceCarla/op_carla/op_bridge/op_bridge/fsm_lab_simulation")

import logging

import numpy as np
from vive_tracker import ViveTrackerModule

from scipy.spatial.transform import Rotation as R


#from std_msgs.msg import String

logging.basicConfig(level=logging.INFO)

class VR2SandBoxTransformer:

    def __init__(self, tracker_name="tracker_1"):
        self.T_pos=     [
           
            # [-6.554661194367805699e-01, 0.000000000000000000e+00, 7.552245800227164185e-01, 3.726792348159205570e+00],
            # [0.000000000000000000e+00, 1.000000000000000000e+00, 0.000000000000000000e+00, 0.000000000000000000e+00],
            # [-7.552245800227164185e-01, 0.000000000000000000e+00, -6.554661194367807919e-01, -3.837253237797098038e+00],
            # [0.000000000000000000e+00, 0.000000000000000000e+00, 0.000000000000000000e+00, 1.000000000000000000e+00]

            [-6.548910364834847897e-01, 0.000000000000000000e+00, 7.557233159917633447e-01, 3.753792794812579103e+00],
            [0.000000000000000000e+00, 1.000000000000000000e+00, 0.000000000000000000e+00, 0.000000000000000000e+00],
            [-7.557233159917634557e-01, 0.000000000000000000e+00, -6.548910364834846787e-01, -3.852285986257896866e+00],
            [0.000000000000000000e+00, 0.000000000000000000e+00, 0.000000000000000000e+00, 1.000000000000000000e+00]
            
            # [-0.6550144012034657, 0.0, 0.7556163935596327, 3.726827671333296], 
            # [0.0, 1.0, 0.0, 0.0], 
            # [-0.7556163935596327, 0.0, -0.6550144012034653, -3.8329851333493403], 
            # [0.0, 0.0, 0.0, 1.0]
            # [-0.6535850710606158, 0.0, 0.7568530603008026, 3.756945786254745],
            # [0.0, 0.9999999999999999, 0.0, 0.0], 
            # [-0.7568530603008026, 2.220446049250313e-16, -0.6535850710606157, -3.8502132456858096], 
            # [0.0, 0.0, 0.0, 1.0]
            # [-0.6535661915604236, 0.0, 0.7568693633971483, 3.6571979511882446], 
            # [0.0, 1.0, 0.0, 0.0], 
            # [-0.756869363397148, 0.0, -0.6535661915604234, -3.9499460310875163], 
            # [0.0, 0.0, 0.0, 1.0]
            # [-0.6682163489209808, 0.0, 0.7439670093725351, 3.7948129048237456],
            # [0.0, 1.0, 0.0, 0.0],
            # [-0.743967009372535, 0.0, -0.6682163489209807, -3.80980849442406],
            # [0.0, 0.0, 0.0, 1.0]
            # [-0.66299882 , 0.       ,   0.74862044,  3.80866506],
            # [ 0.         , 1.       ,   0.        ,  0.        ],
            # [-0.74862044 , 0.       ,  -0.66299882, -3.78739377],
            # [ 0.         , 0.       ,   0.        ,  1.        ]
            # [-0.43949858901201694, 0.0, 0.8982432801064789, 2.6316783346376282],
            # [0.0, -1.0, 0.0, 0.0],
            # [0.898243280106479, 0.0, 0.43949858901201716, -0.8776881963578144], 
            # [0.0, 0.0, 0.0, 1.0]

            # [-4.394985890120169381e-01, 0.000000000000000000e+00, 8.982432801064789141e-01, 2.631678334637628236e+00],
            # [0.000000000000000000e+00 ,-1.000000000000000000e+00, 0.000000000000000000e+00, 0.000000000000000000e+00],
            # [8.982432801064790251e-01 ,0.000000000000000000e+00, 4.394985890120171601e-01, -8.776881963578143653e-01],
            # [0.000000000000000000e+00 ,0.000000000000000000e+00, 0.000000000000000000e+00, 1.000000000000000000e+00]

            # [-0.88510906 , 0.         , 0.46538367  , 3.59162614],
            # [ 0.         , 1.         , 0.          , 0.        ],
            # [-0.46538367 , 0.         ,-0.88510906  ,-4.27316779],
            # [ 0.         , 0.         , 0.          ,1.        ]

            # [-0.83432182 , 0.       ,   0.55127771 , 3.65460789],
            # [ 0.         , 1.       ,   0.         , 0.        ],
            # [-0.55127771 , 0.       ,  -0.83432182 , -4.25914489],
            # [ 0.         , 0.       ,   0.         , 1.        ]
            # [-0.51372172 , 0.    ,      0.85795687 , 3.8000329 ],
            # [ 0.         , 1.    ,      0.         , 0.        ],
            # [-0.85795687 , 0.    ,     -0.51372172 ,-3.76950874],
            # [ 0.         , 0.    ,      0.         , 1.        ]
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

            [ 0.65000227, -0.75993227,  0.        ],
            [ 0.75993227,  0.65000227,  0.        ],
            [ 0.        ,  0.        ,  1.        ]
            # [0.67575897, -0.73712266,  0.        ],
            # [0.73712266, 0.67575897 ,  0.        ],
            # [ 0.       ,  0.        ,  1.        ]
            # [8.904258052275580981e-01, 4.550619452426736267e-01, -7.778905887659460713e-03],
            # [4.550608030268039617e-01, -8.904580788822690218e-01, -2.018737699662562744e-03],
            # [-7.845440297074374181e-03, -1.742339018143674147e-03, -9.999677061391992750e-01]

            # [ 0.88663003 ,-0.46247939 , 0.        ],
            # [ 0.46247939 , 0.88663003 , 0.        ],
            # [ 0.         , 0.         , 1.        ]
            # [-0.53850775,  0.84262056,  0.        ],
            # [-0.84262056, -0.53850775,  0.        ],
            # [ 0.        ,  0.         , 1.        ],
            # [0.83545404 , -0.54956032 , 0.        ],
            # [0.54956032 ,0.83545404 , 0.        ],
            # [ 0.         ,0.          , 1.        ]
            # [-0.83545404 , 0.54956032 , 0.        ],
            # [-0.54956032 ,-0.83545404 , 0.        ],
            # [ 0.         ,0.          , 1.        ]    
                # [ 0.56649565,   -0.82406473 ,   0.0        ], 
                # [ 0.82406473,    0.56649565 ,   0.0        ],
                # [ 0.0        ,    0.0         ,   1.        ]
            ]
        self.TRACKER_NAME = tracker_name
        self.vtm = ViveTrackerModule()
        self.vtm.print_discovered_objects()
        logging.info(f' inited vtm {self.vtm.print_discovered_objects()}')
        self.tracker = self.vtm.devices[self.TRACKER_NAME]
        self.previous_position = None
        self.previous_yaw = None
        self.current_position = None
        self.current_yaw = None

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
