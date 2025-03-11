import numpy as np
import open3d as o3d
import pandas as pd
import copy

# === 读取数据 ===
df = pd.read_csv(r"C:\Users\13366\Desktop\fsm_2208\packages\1_vive_tracker\scripts\data\corrected_tracker_log_0303.csv")
XZ_data = df[["X", "Z"]].values
laser_data = df[["laser_x", "laser_z"]].values

# === 改进1: 改进数据预处理 ===
def preprocess_point_cloud(points, voxel_size=0.05):
    """预处理点云：降采样、去除离群点、计算法向量"""
    # 创建点云
    pcd = o3d.geometry.PointCloud()
    
    # 添加少量随机噪声避免共面问题
    points_with_noise = points + np.random.rand(points.shape[0], points.shape[1]) * voxel_size * 0.01
    
    # 将2D点转换为3D点，添加少量随机Y值避免完全共面
    points_3d = np.hstack((
        points_with_noise, 
        np.random.rand(len(points), 1) * voxel_size * 0.01  # 添加微小随机Y值
    ))
    
    pcd.points = o3d.utility.Vector3dVector(points_3d)
    
    # 体素降采样
    pcd_down = pcd.voxel_down_sample(voxel_size)
    
    # 如果点太少，可能无法进行离群点去除
    if len(pcd_down.points) > 30:
        try:
            cl, ind = pcd_down.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
            pcd_clean = pcd_down.select_by_index(ind)
        except RuntimeError:
            print("警告: 离群点去除失败，使用原始降采样点云")
            pcd_clean = pcd_down
    else:
        pcd_clean = pcd_down
    
    # 估计法向量（使用更稳健的参数）
    try:
        pcd_clean.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 3, max_nn=30)
        )
        # 不使用orient_normals_consistent_tangent_plane，它依赖于Qhull
    except RuntimeError as e:
        print(f"警告: 法向量估计失败: {e}")
        # 如果法向量估计失败，手动设置一个默认法向量
        default_normals = np.zeros((len(pcd_clean.points), 3))
        default_normals[:, 2] = 1.0  # 所有法向量指向Z轴正方向
        pcd_clean.normals = o3d.utility.Vector3dVector(default_normals)
    
    return pcd_clean

# === 创建 Open3D 点云对象 ===
def create_colored_point_cloud(points, color):
    """创建带颜色的点云"""
    pcd = o3d.geometry.PointCloud()
    # 添加微小的Y值以避免严格共面
    points_3d = np.hstack((
        points, 
        np.random.rand(len(points), 1) * 0.001  # 添加微小随机Y值
    ))
    pcd.points = o3d.utility.Vector3dVector(points_3d)
    pcd.paint_uniform_color(color)
    return pcd

# === 改进2: 使用简化的特征计算 ===
def compute_enhanced_features(pcd, voxel_size=0.05):
    """计算点云的增强特征，添加错误处理"""
    # 确保点云已降采样
    if len(pcd.points) > 5000:  # 如果点数过多，进行降采样
        pcd = pcd.voxel_down_sample(voxel_size)
    
    # 重新计算法向量（使用更大的搜索半径和更多的邻居）
    try:
        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 3, max_nn=50)
        )
    except RuntimeError as e:
        print(f"警告: 法向量估计失败: {e}")
        # 设置默认法向量
        default_normals = np.zeros((len(pcd.points), 3))
        default_normals[:, 2] = 1.0
        pcd.normals = o3d.utility.Vector3dVector(default_normals)
    
    # 计算FPFH特征
    try:
        fpfh = o3d.pipelines.registration.compute_fpfh_feature(
            pcd,
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 8, max_nn=150)
        )
        return pcd, fpfh
    except RuntimeError as e:
        print(f"警告: FPFH计算失败: {e}，尝试使用更简单的特征")
        # 如果FPFH计算失败，返回简单的坐标作为特征
        # 这不是真正的FPFH，但可以让代码继续运行
        dummy_fpfh = o3d.pipelines.registration.Feature()
        points = np.asarray(pcd.points)
        dummy_fpfh.data = np.hstack([points, np.zeros((len(points), 30-points.shape[1]))])
        return pcd, dummy_fpfh

# === 改进3: 优化RANSAC配准，添加错误处理 ===
def compute_optimized_ransac(source, target, voxel_size=0.05):
    """使用优化后的RANSAC进行初始对齐，添加错误处理"""
    try:
        source_down, source_fpfh = compute_enhanced_features(source, voxel_size)
        target_down, target_fpfh = compute_enhanced_features(target, voxel_size)
        
        # 增加迭代次数和调整参数以获得更好的初始配准
        distance_threshold = voxel_size * 1.5
        
        # 尝试使用RANSAC
        try:
            reg_ransac = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
                source_down, target_down, source_fpfh, target_fpfh, mutual_filter=True,
                max_correspondence_distance=distance_threshold,
                estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
                ransac_n=3,
                checkers=[
                    o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.8),
                    o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)
                    # 移除法向量检查器，因为它可能导致问题
                ],
                criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.9999)
            )
            
            print(f"RANSAC配准结果: fitness={reg_ransac.fitness}, RMSE={reg_ransac.inlier_rmse}")
            return reg_ransac.transformation
            
        except RuntimeError as e:
            print(f"RANSAC失败: {e}，使用默认变换")
            # 如果RANSAC失败，尝试使用质心对齐作为后备方案
            source_center = np.mean(np.asarray(source_down.points), axis=0)
            target_center = np.mean(np.asarray(target_down.points), axis=0)
            
            # 创建平移变换
            init_translation = np.eye(4)
            init_translation[:3, 3] = target_center - source_center
            
            return init_translation
            
    except Exception as e:
        print(f"RANSAC配准过程中出现错误: {e}，使用单位矩阵")
        return np.identity(4)

# === 改进4: 多步骤ICP配准 ===
def multi_stage_icp(source, target, init_transform=np.identity(4), voxel_sizes=[0.1, 0.05, 0.02, 0.01]):
    """多分辨率ICP配准策略，添加错误处理"""
    current_transform = init_transform
    
    # 记录初始配准评估
    try:
        evaluation = o3d.pipelines.registration.evaluate_registration(
            source, target, 0.05, current_transform)
        print(f"初始配准评估: fitness={evaluation.fitness}, RMSE={evaluation.inlier_rmse}")
    except Exception as e:
        print(f"初始配准评估失败: {e}")
    
    # 逐步减小阈值的ICP配准
    for i, voxel_size in enumerate(voxel_sizes):
        threshold = voxel_size * 2
        print(f"第{i+1}阶段ICP: voxel_size={voxel_size}, threshold={threshold}")
        
        try:
            # 选择合适的ICP方法
            if i < len(voxel_sizes) // 2:
                # 早期阶段使用点到点配准
                method = o3d.pipelines.registration.TransformationEstimationPointToPoint()
            else:
                # 后期阶段使用点到平面配准
                method = o3d.pipelines.registration.TransformationEstimationPointToPlane()
            
            reg_p2x = o3d.pipelines.registration.registration_icp(
                source, target, threshold, current_transform,
                method,
                o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=100)
            )
            
            current_transform = reg_p2x.transformation
            print(f"阶段{i+1}完成: fitness={reg_p2x.fitness}, RMSE={reg_p2x.inlier_rmse}")
            
        except Exception as e:
            print(f"阶段{i+1} ICP失败: {e}，继续下一阶段")
    
    # 最终阶段
    try:
        final_icp = o3d.pipelines.registration.registration_icp(
            source, target, voxel_sizes[-1] * 1.5, current_transform,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),  # 使用更稳定的点到点方法
            o3d.pipelines.registration.ICPConvergenceCriteria(
                relative_fitness=1e-6, 
                relative_rmse=1e-6,
                max_iteration=100
            )
        )
        
        print(f"最终ICP结果: fitness={final_icp.fitness}, RMSE={final_icp.inlier_rmse}")
        return final_icp.transformation
        
    except Exception as e:
        print(f"最终ICP阶段失败: {e}，使用当前变换")
        return current_transform

# === 改进5: 评估配准结果 ===
def evaluate_registration(source, target, transformation, threshold=0.02):
    """评估配准结果并可视化重叠情况，添加错误处理"""
    try:
        # 应用变换
        source_transformed = copy.deepcopy(source)
        source_transformed.transform(transformation)
        
        # 评估配准质量
        evaluation = o3d.pipelines.registration.evaluate_registration(
            source_transformed, target, threshold)
        
        print(f"配准评估结果:")
        print(f"  Fitness (重叠比例): {evaluation.fitness}")
        print(f"  Inlier RMSE (均方根误差): {evaluation.inlier_rmse}")
        
        # 计算点到点距离
        try:
            dists = source_transformed.compute_point_cloud_distance(target)
            dists = np.asarray(dists)
            
            # 统计误差分布
            print(f"距离统计:")
            print(f"  最小距离: {np.min(dists):.5f}")
            print(f"  最大距离: {np.max(dists):.5f}")
            print(f"  平均距离: {np.mean(dists):.5f}")
            print(f"  中位数距离: {np.median(dists):.5f}")
            print(f"  标准差: {np.std(dists):.5f}")
            
            # 对于每个点着色以显示误差
            max_dist = threshold
            colors = np.zeros((len(dists), 3))
            for i, d in enumerate(dists):
                normalized_dist = min(d / max_dist, 1.0)
                colors[i] = [normalized_dist, 0, 1 - normalized_dist]
                
            source_transformed.colors = o3d.utility.Vector3dVector(colors)
            
        except Exception as e:
            print(f"距离计算失败: {e}")
            source_transformed.paint_uniform_color([0, 0, 1])  # 默认蓝色
        
        return source_transformed
        
    except Exception as e:
        print(f"评估过程失败: {e}")
        source_copy = copy.deepcopy(source)
        source_copy.paint_uniform_color([0, 0, 1])  # 默认蓝色
        return source_copy

# === 主处理流程 ===
# 设置体素大小
voxel_size = 0.05  # 根据数据特性调整

# 创建原始点云（用于可视化）
source_pcd_raw = create_colored_point_cloud(XZ_data, [1, 0, 0])  # 红色（原始 XZ_data）
target_pcd_raw = create_colored_point_cloud(laser_data, [0, 1, 0])  # 绿色（laser_data）

print("预处理点云...")
# 预处理点云
source_pcd = preprocess_point_cloud(XZ_data, voxel_size)
target_pcd = preprocess_point_cloud(laser_data, voxel_size)
source_pcd.paint_uniform_color([1, 0, 0])  # 红色
target_pcd.paint_uniform_color([0, 1, 0])  # 绿色

print("计算初始变换...")
# 计算初始变换
trans_init = compute_optimized_ransac(source_pcd, target_pcd, voxel_size=voxel_size)

print("应用初始变换...")
# 应用初始变换并可视化
source_pcd_ransac = copy.deepcopy(source_pcd)
source_pcd_ransac.transform(trans_init)
source_pcd_ransac.paint_uniform_color([1, 0.7, 0])  # 橙色

print("执行多阶段ICP配准...")
# 多阶段ICP配准（使用更少的阶段以减少出错机会）
voxel_sizes = [0.1, 0.05, 0.02]  # 简化的多阶段策略
final_transformation = multi_stage_icp(source_pcd, target_pcd, trans_init, voxel_sizes)

print("评估最终配准结果...")
# 评估和可视化最终结果
source_pcd_final = evaluate_registration(source_pcd, target_pcd, final_transformation, 0.05)

# === 应用最终变换到原始点云并可视化 ===
print("应用变换到原始数据...")
# 转换原始数据以便可视化
XZ_homo = np.hstack((XZ_data, np.zeros((XZ_data.shape[0], 1)), np.ones((XZ_data.shape[0], 1))))
transformed_XZ_homo = (final_transformation @ XZ_homo.T).T
transformed_XZ = transformed_XZ_homo[:, :2]  # 取前两列（X, Z）

transformed_pcd_vis = create_colored_point_cloud(transformed_XZ, [0, 0, 1])  # 蓝色（ICP 对齐后）

# === 保存变换结果 ===
try:
    transformed_df = pd.DataFrame(transformed_XZ, columns=["X_aligned", "Z_aligned"])
    result_df = pd.concat([df, transformed_df], axis=1)
    result_df.to_csv(r"C:\Users\13366\Desktop\fsm_2208\packages\1_vive_tracker\scripts\data\aligned_tracker_log.csv", index=False)
    print("已保存变换结果到CSV文件")
except Exception as e:
    print(f"保存CSV失败: {e}")

# === 可视化 ===
print("=== 可视化结果 ===")
print("红色 = 原始 XZ, 绿色 = laser_data")
o3d.visualization.draw_geometries([source_pcd_raw, target_pcd_raw], window_name="原始点云")

print("橙色 = RANSAC初步对齐, 绿色 = laser_data")
o3d.visualization.draw_geometries([source_pcd_ransac, target_pcd], window_name="RANSAC初步对齐")

print("蓝色 = 最终对齐结果(带误差着色), 绿色 = laser_data")
o3d.visualization.draw_geometries([source_pcd_final, target_pcd], window_name="ICP最终对齐")

print("蓝色 = 变换后原始点云, 绿色 = laser_data")
o3d.visualization.draw_geometries([transformed_pcd_vis, target_pcd_raw], window_name="应用于原始点云的最终结果")