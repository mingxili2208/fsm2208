import openvr

def get_vive_tracker_refresh_rate():
    """获取 SteamVR 设定的 Vive Tracker 3.0 刷新率"""
    vr_system = openvr.init(openvr.VRApplication_Other)

    # 获取系统设定的 VR 刷新率
    refresh_rate = vr_system.getFloatTrackedDeviceProperty(
        openvr.k_unTrackedDeviceIndex_Hmd,  # HMD 设备索引（如果没有 HMD，可能会失败）
        openvr.Prop_DisplayFrequency_Float
    )

    print(f"Vive Tracker 3.0 刷新率: {refresh_rate:.2f} Hz")

    openvr.shutdown()

# if __name__ == "__main__":
#     get_vive_tracker_refresh_rate()

def get_lighthouse_refresh_rate():
    """获取 SteamVR Lighthouse（基站）的刷新率"""
    vr_system = openvr.init(openvr.VRApplication_Other)

    for i in range(openvr.k_unMaxTrackedDeviceCount):
        device_class = vr_system.getTrackedDeviceClass(i)
        if device_class == openvr.TrackedDeviceClass_TrackingReference:
            refresh_rate = vr_system.getFloatTrackedDeviceProperty(
                i, openvr.Prop_DisplayFrequency_Float
            )
            print(f"Lighthouse 刷新率: {refresh_rate:.2f} Hz")
            break

    openvr.shutdown()

if __name__ == "__main__":
    get_lighthouse_refresh_rate()