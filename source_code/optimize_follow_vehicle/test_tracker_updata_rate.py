import time
import openvr

# 初始化 OpenVR
vr_system = openvr.init(openvr.VRApplication_Background)

# 获取 Tracker 设备索引
tracker_index = None
for i in range(openvr.k_unMaxTrackedDeviceCount):
    if vr_system.getTrackedDeviceClass(i) == openvr.TrackedDeviceClass_GenericTracker:
        tracker_index = i
        break

if tracker_index is None:
    print("未找到 Vive Tracker 设备")
    exit(1)

# 测量更新频率
update_count = 0
start_time = time.time()
duration = 5  # 测量 5 秒

print(f"\n the getStringTrackedDeviceProperty(deviceIndex, prop) is {vr_system.getStringTrackedDeviceProperty(tracker_index, openvr.Prop_SerialNumber_String)}")
# print(f"\n the vr_system.getTrackedDeviceClass(1) is {vr_system.getTrackedDeviceClass(1)}")

# while time.time() - start_time < duration:
#     # 获取 Tracker 姿态数据
#     poses = vr_system.getDeviceToAbsoluteTrackingPose(
#         openvr.TrackingUniverseStanding, 0, openvr.k_unMaxTrackedDeviceCount
#     )
#     if poses[tracker_index].bPoseIsValid:
#         print(f"\n Tracker 数据:{poses[tracker_index].mDeviceToAbsoluteTracking}")
#         update_count += 1


#-1.197170376777649
#-1.1971702575683594]
# 计算更新频率

# update_frequency = update_count / duration

# # 关闭 OpenVR
# openvr.shutdown()

# print(f"Vive Tracker 3.0 数据更新频率: {update_frequency:.2f} Hz")