#!/usr/bin/env python

import sys
sys.path.append("/home/cityu-fsm-lab-carla/Workspace/Carla/op_carla/op_bridge/op_bridge/fsm_lab_simulation")

import logging
import numpy as np
from scipy.spatial.transform import Rotation as R
import pysurvive 
import time 

logging.basicConfig(level=logging.INFO)

class VR2SandBoxTransformer:

    def __init__(self, tracker_name="tracker_1"):
        self.use_quate=False
        self.T_pos = [
            [-4.394985890120169381e-01, 0.000000000000000000e+00 , 8.982432801064789141e-01, 2.631678334637628236e+00],
            [ 0.000000000000000000e+00, -1.000000000000000000e+00, 0.000000000000000000e+00, 0.000000000000000000e+00],
            [ 8.982432801064790251e-01, 0.000000000000000000e+00 , 4.394985890120171601e-01, -8.776881963578143653e-01],
            [ 0.000000000000000000e+00, 0.000000000000000000e+00 , 0.000000000000000000e+00, 1.000000000000000000e+00]
        ]
        self.R_euler = [
            [-0.44719669,  0.89443564 , 0.        ],
            [-0.89443564, -0.44719669 , 0.        ],
            [ 0.        ,  0.         , 1.        ]
            # [-0.88663003 , 0.46247939 , 0.        ],
            # [-0.46247939 ,-0.88663003 , 0.        ],
            # [ 0.         , 0.         , 1.        ]
        ]

        self.TRACKER_NAME = tracker_name
        try:
            self.actx = pysurvive.SimpleContext(sys.argv)  # 初始化 pysurvive 追踪环境
            time.sleep(5)
            logging.info("pysurvive context initialized.")
            self.updated=self.actx.NextUpdated()
            while "LH" in str(self.updated.Name()):   
                poseObj = self.updated.Pose()
                poseData = poseObj[0]
                poseTimestamp = poseObj[1]
                print("%s: T: %f P: % 9f,% 9f,% 9f R: % 9f,% 9f,% 9f,% 9f"%(str(self.updated.Name(), 'utf-8'), poseTimestamp, poseData.Pos[0], poseData.Pos[1], poseData.Pos[2], poseData.Rot[0], poseData.Rot[1], poseData.Rot[2], poseData.Rot[3]))
                self.updated = self.actx.NextUpdated()
            time.sleep(5)
        except Exception as e:
            logging.error(f"Error initializing tracker: {e}")
            sys.exit(1)
    def has_quaternion(self):
        return self.use_quate

    def transform_point_pose(self, position, orientation, T_pos, R_euler):
        """
        使用给定的位置变换矩阵 (T_pos) 和方向变换矩阵 (R_euler) 对点的方位和方向进行转换。
        """
        position_homogeneous = np.append(position, 1)
        transformed_position_homogeneous = T_pos @ position_homogeneous
        transformed_position = transformed_position_homogeneous[:3]

        R_point = R.from_euler('yxz', [orientation[2], orientation[0], orientation[1]]).as_matrix()
        R_transformed = R_euler @ R_point
        transformed_orientation = R.from_matrix(R_transformed).as_euler('yxz')
        transformed_orientation = [transformed_orientation[0], transformed_orientation[2], transformed_orientation[1]]

        return transformed_position, transformed_orientation

    def transform_to_sandbox(self, cam_coord, T_pos, R_euler):
        """
        转换 Vive Tracker 相机坐标系到 sandbox 世界坐标系。
        """
        orientation = [0, round(cam_coord[4], 4), 0]
        #x,y,z
        #libsurvive can use x,y,z coor; 
        #but the topic subscribe set transformed_position[0],transformed_position[2] as x, y
        #so we set cam_coord[1] as transformed_position[2]
        position = [round(cam_coord[0], 4), 0, round(cam_coord[1], 4)]
        transformed_position, transformed_orientation = self.transform_point_pose(position, orientation, T_pos, R_euler)
        yaw = transformed_orientation[1]

        return transformed_position, yaw

    def get_transformed_coor(self):
        """
        使用 pysurvive 获取最新的 Tracker 位姿，并转换到 sandbox 坐标系。
        """
        try:
            if self.actx.Running():
                updated = self.actx.NextUpdated()
                if updated:
                    while "LH" in str(updated.Name()):  
                        print("\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! \n this is  LH_pose  ")
                        poseObj = updated.Pose()
                        poseData = poseObj[0]
                        poseTimestamp = poseObj[1]
                        print("%s: T: %f P: % 9f,% 9f,% 9f R: % 9f,% 9f,% 9f,% 9f"%(str(updated.Name(), 'utf-8'), poseTimestamp, poseData.Pos[0], poseData.Pos[1], poseData.Pos[2], poseData.Rot[0], poseData.Rot[1], poseData.Rot[2], poseData.Rot[3]))
                        print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n")
                        updated = self.actx.NextUpdated()
                # if updated:
                #     poseObj = updated.Pose()
                #     poseData = poseObj[0]
                #     poseTimestamp = poseObj[1]
                #     print("%s: T: %f P: % 9f,% 9f,% 9f R: % 9f,% 9f,% 9f,% 9f"%(str(updated.Name(), 'utf-8'), poseTimestamp, poseData.Pos[0], poseData.Pos[1], poseData.Pos[2], poseData.Rot[0], poseData.Rot[1], poseData.Rot[2], poseData.Rot[3]))
                
                    pose_obj = updated.Pose()
                    pose_data = pose_obj[0]
                    timestamp = pose_obj[1]

                    position = pose_data.Pos  # [x, y, z]
                    rotation = pose_data.Rot  # 四元数 (w, x, y, z)

                    # 转换为欧拉角 (yxz)

                    #xyzw
                    r = R.from_quat([rotation[0],rotation[1], rotation[2], rotation[3]])
                    
                    #(pitch, yaw, roll)
                    euler_angles = r.from_quat(rotation).as_euler('yzx', degrees=False)
                    #euler_angles = r.as_euler("yzx")  # [yaw, pitch, roll]
                    #euler_angles = r.as_euler("yxz")

                    #euler_angles_degrees = np.degrees(euler_angles)
                    # 重新格式化返回值，使其匹配 get_pose_euler() 的格式
                    #x, y, z = pose_data.Pos[0],pose_data.Pos[1],pose_data.Pos[2]
                    #yaw, pitch, roll = euler_angles  # 假设 as_euler("yxz") 返回的是 (yaw, pitch, roll)
                    pitch, yaw,roll = euler_angles
                    # just make sure yaw as the 4th from 0
                    transformed_position, transformed_yaw = self.transform_to_sandbox(
                    [position[0], position[1], position[2], roll, yaw, pitch],
                    self.T_pos, self.R_euler)
                    return transformed_position, transformed_yaw 
            else:
                logging.warning("No updated pose data available.")
                return None, None

        except Exception as e:
            logging.fatal(f"!!!!!!!!!!!! get_transformed_coor error: {e}", exc_info=True)
            return None, None

#if __name__ == '__main__':
#    vst = VR2SandBoxTransformer()
#    vst.get_transformed_coor()