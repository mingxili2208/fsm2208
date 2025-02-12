import time
import numpy as np
import openvr

class vr_tracked_device():
    def __init__(self, vr_obj, index, device_class):
        self.device_class = device_class
        self.index = index
        self.vr = vr_obj

    def get_pose(self):
        """获取设备位姿"""
        return self.vr.getDeviceToAbsoluteTrackingPose(
            openvr.TrackingUniverseStanding, 0, openvr.k_unMaxTrackedDeviceCount
        )

    def measure_refresh_rate(self, num_samples=100):
        """
        测量 Vive Tracker 3 的刷新率。
        :param num_samples: 采样帧数
        :return: 计算出的刷新率（Hz）
        """
        timestamps = []  # 记录时间戳

        # 采样 num_samples 帧数据
        for _ in range(num_samples):
            timestamps.append(time.time())  # 记录当前时间
            pose = self.get_pose()  # 获取 Tracker 位姿
            # if pose[self.index].bPoseIsValid:
            #     time.sleep(0.01)  # 10ms 采样间隔，避免 CPU 过载

        # 计算相邻时间间隔
        intervals = np.diff(timestamps)  # 计算时间差
        avg_interval = np.mean(intervals)  # 计算平均间隔
        refresh_rate = 1 / avg_interval  # 计算刷新率 Hz

        print(f"Vive Tracker 3 刷新率: {refresh_rate:.2f} Hz")
        return refresh_rate
    
# 初始化 OpenVR
vr = openvr.init(openvr.VRApplication_Other)

# 获取 Tracker ID
tracker_index = None
for i in range(openvr.k_unMaxTrackedDeviceCount):
    device_class = vr.getTrackedDeviceClass(i)
    if device_class == openvr.TrackedDeviceClass_GenericTracker:
        tracker_index = i
        break

if tracker_index is not None:
    tracker = vr_tracked_device(vr, tracker_index, "Tracker")
    tracker.measure_refresh_rate(num_samples=200)  # 采样 200 帧
else:
    print("未找到 Vive Tracker！")

# 关闭 OpenVR
openvr.shutdown()