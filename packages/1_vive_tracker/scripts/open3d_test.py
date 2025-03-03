import numpy as np
import open3d as o3d
import pandas as pd

# === 读取数据 ===
df = pd.read_csv(r"C:\Users\13366\Desktop\fsm_2208\packages\1_vive_tracker\scripts\corrected_tracker_log_0303.csv")
XZ_data = df[["X", "Z"]].values
laser_data = df[["laser_x", "laser_z"]].values

# === 创建 Open3D 点云对象 ===
def create_point_cloud(XZ_data, color):
    """ 创建 Open3D 点云对象 """
    pcd = o3d.geometry.PointCloud()
    points = np.hstack((XZ_data, np.zeros((len(XZ_data), 1))))  # 补充 Y=0
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.paint_uniform_color(color)  # 设定颜色
    return pcd

# === 计算 FPFH 特征 ===
def compute_fpfh_feature(pcd, voxel_size=0.05):
    """ 计算 Open3D 点云的 FPFH 特征 """
    pcd_down = pcd.voxel_down_sample(voxel_size)  # 降采样，提高匹配效率
    pcd_down.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30)
    )  # 计算法向量
    fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 5, max_nn=100)
    )  # 计算 FPFH 特征
    return pcd_down, fpfh

# === 计算 RANSAC 变换 ===
def compute_ransac_transform(source, target, voxel_size=0.05):
    """ 使用 RANSAC 进行初步对齐，获得更好的初始变换 """
    source_down, source_fpfh = compute_fpfh_feature(source, voxel_size)
    target_down, target_fpfh = compute_fpfh_feature(target, voxel_size)

    reg_ransac = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, mutual_filter=True,
        max_correspondence_distance=voxel_size * 2,
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        ransac_n=4,
        checkers=[
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(voxel_size * 2),
        ],
        criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999)
    )
    return reg_ransac.transformation

# === 生成点云 ===
source_pcd = create_point_cloud(XZ_data, [1, 0, 0])  # 红色（原始 XZ_data）
target_pcd = create_point_cloud(laser_data, [0, 1, 0])  # 绿色（laser_data）

# === 计算 RANSAC 预配准变换 ===
trans_init = compute_ransac_transform(source_pcd, target_pcd, voxel_size=0.05)

# === 计算目标点云的法向量（用于 Point-to-Plane ICP） ===
target_pcd.estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.5, max_nn=50)
)

# === 逐步优化 ICP（从大误差到小误差） ===
threshold_list = [0.5, 0.1, 0.05, 0.02]
current_transformation = trans_init
for threshold in threshold_list:
    reg_p2p = o3d.pipelines.registration.registration_icp(
        source_pcd, target_pcd, threshold, current_transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPlane()
    )
    current_transformation = reg_p2p.transformation  # 更新变换矩阵

# === 应用 4×4 变换矩阵 ===
XZ_homo = np.hstack((XZ_data, np.zeros((XZ_data.shape[0], 1)), np.ones((XZ_data.shape[0], 1))))
transformed_XZ_homo = (current_transformation @ XZ_homo.T).T
transformed_XZ = transformed_XZ_homo[:, :2]  # 取前两列（X, Z）

transformed_pcd = create_point_cloud(transformed_XZ, [0, 0, 1])  # 蓝色（ICP 对齐后）

# === 可视化 ===
print("显示点云窗口: 红色 = 原始 XZ, 绿色 = laser_data, 蓝色 = ICP 对齐后")
o3d.visualization.draw_geometries([source_pcd, target_pcd], window_name="ICP 配准前")
o3d.visualization.draw_geometries([transformed_pcd, target_pcd], window_name="ICP 配准后")