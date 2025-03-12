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
        self.T_pos = [
            [-0.88510906, 0., 0.46538367, 3.59162614],
            [0., 1., 0., 0.],
            [-0.46538367, 0., -0.88510906, -4.27316779],
            [0., 0., 0., 1.]
        ]
        self.R_euler = [
            [0.88663003, -0.46247939, 0.],
            [0.46247939, 0.88663003, 0.],
            [0., 0., 1.]
        ]

        self.TRACKER_NAME = tracker_name
        self.actx = pysurvive.SimpleContext(sys.argv)  # 初始化 pysurvive 追踪环境
        time.sleep(10)
        logging.info("pysurvive context initialized.")

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
        orientation = [0, np.radians(round(cam_coord[4], 4)), 0]
        position = [round(cam_coord[0], 4), 0, round(cam_coord[2], 4)]
        transformed_position, transformed_orientation = self.transform_point_pose(position, orientation, T_pos, R_euler)
        yaw = transformed_orientation[1]

        return transformed_position, yaw

    def get_transformed_coor(self):
        """
        使用 pysurvive 获取最新的 Tracker 位姿，并转换到 sandbox 坐标系。
        """
        try:
            updated = self.actx.NextUpdated()  # 获取最新的追踪数据
            if updated:
                poseObj = updated.Pose()
                poseData = poseObj[0]  # 位姿数据
                poseTimestamp = poseObj[1]  # 时间戳

                position = [poseData.Pos[0], poseData.Pos[1], poseData.Pos[2]]
                rotation = [poseData.Rot[0], poseData.Rot[1], poseData.Rot[2], poseData.Rot[3]]

                # 将四元数转换为欧拉角
                euler_angles = R.from_quat(rotation).as_euler('yxz', degrees=True)

                # logging.info(f"Tracker {updated.Name().decode('utf-8')} Pose at T={poseTimestamp:.6f}: "
                #              f"Position={position}, Euler Angles={euler_angles}")

                transformed_position, transformed_yaw = self.transform_to_sandbox(
                    [position[0], position[1], position[2], euler_angles[0], euler_angles[1], euler_angles[2]],
                    self.T_pos, self.R_euler)

                # logging.info(f"Transformed position in Sandbox_coordinate: "
                #              f"[x={transformed_position[0]:.4f}, y={transformed_position[1]:.4f}, "
                #              f"z={transformed_position[2]:.4f}, yaw={np.rad2deg(transformed_yaw):.4f}]")

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