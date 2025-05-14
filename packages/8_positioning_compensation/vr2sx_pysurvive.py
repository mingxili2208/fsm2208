#!/usr/bin/env python

import sys
sys.path.append("/home/cityu-fsm-lab-carla/WorkplaceCarla/op_carla/op_bridge/op_bridge/fsm_lab_simulation")

import logging
import numpy as np
from scipy.spatial.transform import Rotation as R
import pysurvive 
import time 

logging.basicConfig(level=logging.INFO)

class VR2SandBoxTransformer:

    def __init__(self, tracker_name="tracker_1"):
        self.use_quate = False
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
        ]

        self.TRACKER_NAME = tracker_name
        try:
            self.actx = pysurvive.SimpleContext(sys.argv)  # 初始化 pysurvive 追踪环境
            time.sleep(5)
            logging.info("pysurvive context initialized.")
            self.updated = self.actx.NextUpdated()
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

    def transform_point_pose_quaternion(self, position, quaternion, T_pos, R_euler):
        """
        使用给定的位置变换矩阵 (T_pos) 和方向变换矩阵 (R_euler) 对点的方位和四元数进行转换。
        """
        position_homogeneous = np.append(position, 1)
        transformed_position_homogeneous = T_pos @ position_homogeneous
        transformed_position = transformed_position_homogeneous[:3]

        # Convert quaternion to rotation matrix
        R_point = R.from_quat(quaternion).as_matrix()
        R_transformed = R_euler @ R_point
        
        # Convert transformed rotation matrix back to quaternion
        transformed_quaternion = R.from_matrix(R_transformed).as_quat()

        return transformed_position, transformed_quaternion

    def transform_to_sandbox(self, cam_coord, T_pos, R_euler):
        """
        转换 Vive Tracker 相机坐标系到 sandbox 世界坐标系。
        返回欧拉角形式。
        """
        orientation = [0, round(cam_coord[4], 4), 0]
        position = [round(cam_coord[0], 4), 0, round(cam_coord[1], 4)]
        transformed_position, transformed_orientation = self.transform_point_pose(position, orientation, T_pos, R_euler)
        yaw = transformed_orientation[1]

        return transformed_position, yaw
        
    def transform_to_sandbox_quaternion(self, position, quaternion, T_pos, R_euler):
        """
        转换 Vive Tracker 相机坐标系到 sandbox 世界坐标系。
        返回四元数形式。
        """
        # Adjust position format for transformation
        transform_position = [round(position[0], 4), 0, round(position[1], 4)]
        transformed_position, transformed_quaternion = self.transform_point_pose_quaternion(
            transform_position, quaternion, T_pos, R_euler)
            
        return transformed_position, transformed_quaternion

    def get_tracker_pose(self):
        """
        使用 pysurvive 获取最新的 Tracker 位姿
        """
        try:
            if self.actx.Running():
                updated = self.actx.NextUpdated()
                if updated:
                    while "LH" in str(updated.Name()):  
                        # print("\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! \n this is  LH_pose  ")
                        # poseObj = updated.Pose()
                        # poseData = poseObj[0]
                        # poseTimestamp = poseObj[1]
                        # print("%s: T: %f P: % 9f,% 9f,% 9f R: % 9f,% 9f,% 9f,% 9f"%(str(updated.Name(), 'utf-8'), poseTimestamp, poseData.Pos[0], poseData.Pos[1], poseData.Pos[2], poseData.Rot[0], poseData.Rot[1], poseData.Rot[2], poseData.Rot[3]))
                        # print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n")
                        updated = self.actx.NextUpdated()
                
                    pose_obj = updated.Pose()
                    pose_data = pose_obj[0]
                    timestamp = pose_obj[1]

                    position = pose_data.Pos  # [x, y, z]
                    rotation = pose_data.Rot  # 四元数 (w, x, y, z)
                    
                    # 转换为欧拉角 (yxz)
                    r = R.from_quat([rotation[0], rotation[1], rotation[2], rotation[3]])
                    euler_angles = r.as_euler('yzx', degrees=False)
                    pitch, yaw, roll = euler_angles
                    
                    return position, rotation, [roll, yaw, pitch]
                    
            logging.warning("No updated pose data available.")
            return None, None, None

        except Exception as e:
            logging.fatal(f"!!!!!!!!!!!! get_tracker_pose error: {e}", exc_info=True)
            return None, None, None

    def get_transformed_coor(self):
        """
        获取最新的 Tracker 位姿，并转换到 sandbox 坐标系，返回欧拉角形式。
        """
        try:
            position, rotation, euler_angles = self.get_tracker_pose()
            if position is None:
                return None, None
                
            roll, yaw, pitch = euler_angles
            
            transformed_position, transformed_yaw = self.transform_to_sandbox(
                [position[0], position[1], position[2], roll, yaw, pitch],
                self.T_pos, self.R_euler)
                
            return transformed_position, transformed_yaw

        except Exception as e:
            logging.fatal(f"!!!!!!!!!!!! get_transformed_coor error: {e}", exc_info=True)
            return None, None
            
    def get_transformed_coor_with_quaternion(self):
        """
        获取最新的 Tracker 位姿，并转换到 sandbox 坐标系，返回四元数形式。
        """
        try:
            position, quaternion, _ = self.get_tracker_pose()
            if position is None:
                return None, None
                
            transformed_position, transformed_quaternion = self.transform_to_sandbox_quaternion(
                position, quaternion, self.T_pos, self.R_euler)
                
            return transformed_position, transformed_quaternion

        except Exception as e:
            logging.fatal(f"!!!!!!!!!!!! get_transformed_coor_with_quaternion error: {e}", exc_info=True)
            return None, None


if __name__ == '__main__':
    vst = VR2SandBoxTransformer()
    
    # Test both methods
    position, yaw = vst.get_transformed_coor()
    if position is not None:
        print(f"Euler result - Position: {position}, Yaw: {yaw}")
        
    position, quaternion = vst.get_transformed_coor_with_quaternion()
    if position is not None:
        print(f"Quaternion result - Position: {position}, Quaternion: {quaternion}")