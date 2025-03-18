import pysurvive
import sys
import numpy as np
from scipy.spatial.transform import Rotation as R
import time
import logging
# 位置和角度变化的阈值
POSITION_THRESHOLD = 0.01  # 位置变化阈值（单位：米）
YAW_THRESHOLD = np.radians(3)  # 角度变化阈值（单位：弧度）
logging.basicConfig(level=logging.INFO)

# 创建 pysurvive 上下文
try:
    actx = pysurvive.SimpleContext(sys.argv)  # 初始化 pysurvive 追踪环境
    time.sleep(5)
    logging.info("pysurvive context initialized.")
    updated = actx.NextUpdated()
    while "LH" in str(updated.Name()):   
        poseObj = updated.Pose()
        poseData = poseObj[0]
        poseTimestamp = poseObj[1]
        print("%s: T: %f P: % 9f,% 9f,% 9f R: % 9f,% 9f,% 9f,% 9f"%(str(updated.Name(), 'utf-8'), poseTimestamp, poseData.Pos[0], poseData.Pos[1], poseData.Pos[2], poseData.Rot[0], poseData.Rot[1], poseData.Rot[2], poseData.Rot[3]))
        updated = actx.NextUpdated()
    time.sleep(5)
except Exception as e:
    sys.exit(1)
print("start to print obj")
for obj in actx.Objects():
    print(str(obj.Name(), 'utf-8'))
print("end print")

# 存储上一次的位置和旋转
last_positions = {}
last_rotations = {}

while actx.Running():
    updated = actx.NextUpdated()
    while updated and "LH" in str(updated.Name()):  
        if updated:
            updated = actx.NextUpdated()
    if updated:
        
        poseObj = updated.Pose()
        poseData = poseObj[0]
        poseTimestamp = poseObj[1]

        # 获取当前对象名称
        obj_name = str(updated.Name(), 'utf-8')

        # 获取当前位置信息
        current_position = np.array([poseData.Pos[0], poseData.Pos[1], poseData.Pos[2]])

        # 获取当前四元数
        current_rotation = np.array([poseData.Rot[0], poseData.Rot[1], poseData.Rot[2], poseData.Rot[3]])

        # 计算位置变化
        position_significant = False
        yaw_significant = False

        if obj_name in last_positions:
            distance = np.linalg.norm(current_position - last_positions[obj_name])
            if distance > POSITION_THRESHOLD:
                position_significant = True
        else:
            position_significant = True  # 第一次更新时，强制打印

        # 计算旋转变化（四元数 → 角度变化）
        if obj_name in last_rotations:
            last_rotation = R.from_quat(last_rotations[obj_name])  # 上次的旋转
            current_rotation_matrix = R.from_quat(current_rotation)  # 当前的旋转
            rotation_difference = current_rotation_matrix * last_rotation.inv()  # 计算旋转差
            angle_difference = np.linalg.norm(rotation_difference.as_rotvec())  # 获取角度变化（弧度）

            if angle_difference > YAW_THRESHOLD:
                yaw_significant = True
        else:
            yaw_significant = True  # 第一次更新时，强制打印

        # 仅当位置或角度变化显著时打印
        if position_significant or yaw_significant:
            # 转换四元数为欧拉角（单位：度）
            euler_angles = R.from_quat(current_rotation).as_euler('yzx', degrees=True)

            pitch, roll, yaw = euler_angles
            print("%s: T: %f P: % 9f,% 9f,% 9f Euler: Roll: % 9.3f°, Pitch: % 9.3f°, Yaw: % 9.3f°" % (
                obj_name, poseTimestamp, 
                poseData.Pos[0], poseData.Pos[1], poseData.Pos[2], 
                roll, pitch, yaw
            ))

            # 更新存储的上一次位置信息和旋转信息
            last_positions[obj_name] = current_position
            last_rotations[obj_name] = current_rotation