#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
优化版混合模型离线训练脚本

本脚本实现了 "已知旋转 + RANSAC仿射 + MLP" 的混合变换模型。
优化内容：
- 增强数值稳定性
- 自适应RANSAC参数
- 丰富的MLP输入特征
- 完善的数据验证
- 改进的错误处理
"""
import os
import datetime
import logging
import joblib
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from sklearn.model_selection import train_test_split
from sklearn.neural_network import MLPRegressor
from sklearn.preprocessing import StandardScaler
from sklearn.metrics import mean_squared_error, mean_absolute_error
import warnings
warnings.filterwarnings('ignore')

# =============================================================================
# 1. 配置与常量
# =============================================================================

# --- 文件与路径配置 ---
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PARENT_DIR = os.path.dirname(SCRIPT_DIR)
LASER_TRACKER_CSV = os.path.join(PARENT_DIR, "data/final_coordinates.csv")

# --- 输出目录配置 ---
TIMESTAMP = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
OUTPUT_DIR = os.path.join(PARENT_DIR, f"results/Optimized_Hybrid_Model_{TIMESTAMP}")
IMG_DIR = os.path.join(OUTPUT_DIR, "img")
MODEL_DIR = os.path.join(OUTPUT_DIR, "models")
LOG_DIR = os.path.join(OUTPUT_DIR, "log")

# --- 数据列名配置 ---
TRACKER_X = "tracker_x"
TRACKER_Z = "tracker_z"
LASER_X = "laser_x"
LASER_Z = "laser_z"

# --- 模型参数配置 ---
TRAIN_TEST_RATIO = 0.8
RANDOM_STATE = 42

# RANSAC 参数 (动态调整)
RANSAC_ITERATIONS = 2000
RANSAC_MIN_SAMPLE_SIZE = 6
RANSAC_MAX_SAMPLE_SIZE = 50
RANSAC_BASE_THRESHOLD = 0.04  # 40mm base threshold

# MLP 参数
MLP_HIDDEN_LAYERS = (256, 128, 64, 32)
MLP_MAX_ITER = 2000
MLP_EARLY_STOPPING = True
MLP_VALIDATION_FRACTION = 0.15

# 数值稳定性参数
REGULARIZATION_LAMBDA = 1e-6
MIN_POINTS_FOR_TRAINING = 20

# --- 已知旋转矩阵 ---
R_EULER_3D = np.array([
    [ 0.67654891, -0.7363977 ,  0.0        ],
    [ 0.7363977 ,  0.67654891,  0.0        ],
    [ 0.0       ,  0.0       ,  1.0        ]
])

# =============================================================================
# 2. 初始化设置
# =============================================================================

def setup_environment():
    """创建所有输出目录并配置日志记录。"""
    for directory in [OUTPUT_DIR, IMG_DIR, MODEL_DIR, LOG_DIR]:
        os.makedirs(directory, exist_ok=True)

    log_file = os.path.join(LOG_DIR, "optimized_training_process.log")
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(message)s',
        handlers=[
            logging.FileHandler(log_file, encoding='utf-8'),
            logging.StreamHandler()
        ]
    )
    logging.info("环境设置完成。")

# =============================================================================
# 3. 数据验证与预处理
# =============================================================================

def validate_data(source_points, target_points):
    """验证输入数据的质量和一致性。"""
    logging.info("开始数据验证...")
    
    if len(source_points) != len(target_points):
        raise ValueError(f"源点和目标点数量不匹配: {len(source_points)} vs {len(target_points)}")
    
    if len(source_points) < MIN_POINTS_FOR_TRAINING:
        raise ValueError(f"数据点太少({len(source_points)})，至少需要{MIN_POINTS_FOR_TRAINING}个点进行可靠训练")
    
    # 检查是否有NaN或无穷值
    if np.any(np.isnan(source_points)) or np.any(np.isnan(target_points)):
        raise ValueError("数据中包含NaN值")
    
    if np.any(np.isinf(source_points)) or np.any(np.isinf(target_points)):
        raise ValueError("数据中包含无穷值")
    
    # 检查数据分布
    src_std = np.std(source_points, axis=0)
    tgt_std = np.std(target_points, axis=0)
    
    if np.any(src_std < 1e-6) or np.any(tgt_std < 1e-6):
        logging.warning("检测到数据方差极小，可能影响训练效果")
    
    # 检查异常值
    src_distances = np.linalg.norm(source_points - np.mean(source_points, axis=0), axis=1)
    tgt_distances = np.linalg.norm(target_points - np.mean(target_points, axis=0), axis=1)
    
    src_outliers = src_distances > (np.mean(src_distances) + 3 * np.std(src_distances))
    tgt_outliers = tgt_distances > (np.mean(tgt_distances) + 3 * np.std(tgt_distances))
    
    outlier_ratio = (np.sum(src_outliers) + np.sum(tgt_outliers)) / (2 * len(source_points))
    if outlier_ratio > 0.1:
        logging.warning(f"检测到{outlier_ratio*100:.1f}%的潜在异常值")
    
    logging.info(f"数据验证完成。源点范围: X[{np.min(source_points[:, 0]):.3f}, {np.max(source_points[:, 0]):.3f}], "
                f"Z[{np.min(source_points[:, 1]):.3f}, {np.max(source_points[:, 1]):.3f}]")

def remove_outliers(source_points, target_points, threshold_std=3.0):
    """移除统计异常值。"""
    initial_count = len(source_points)
    
    # 计算初始距离
    initial_distances = np.linalg.norm(target_points - source_points, axis=1)
    mean_dist = np.mean(initial_distances)
    std_dist = np.std(initial_distances)
    
    # 标记内点
    inlier_mask = initial_distances < (mean_dist + threshold_std * std_dist)
    
    cleaned_source = source_points[inlier_mask]
    cleaned_target = target_points[inlier_mask]
    
    removed_count = initial_count - len(cleaned_source)
    if removed_count > 0:
        logging.info(f"移除了{removed_count}个异常值点 ({removed_count/initial_count*100:.1f}%)")
    
    return cleaned_source, cleaned_target, inlier_mask

# =============================================================================
# 4. 增强的特征提取
# =============================================================================

def extract_enhanced_features(points, center_point=None):
    """提取增强的特征用于MLP训练。"""
    if center_point is None:
        center_point = np.mean(points, axis=0)
    
    features = points.copy()
    
    # 添加径向距离特征
    radial_dist = np.linalg.norm(points - center_point, axis=1, keepdims=True)
    features = np.hstack([features, radial_dist])
    
    # 添加角度特征
    relative_points = points - center_point
    angles = np.arctan2(relative_points[:, 1], relative_points[:, 0]).reshape(-1, 1)
    features = np.hstack([features, angles])
    
    # 添加到中心的相对坐标
    features = np.hstack([features, relative_points])
    
    # 添加二次特征 (x^2, z^2, x*z)
    quad_features = np.column_stack([
        points[:, 0]**2,
        points[:, 1]**2,
        points[:, 0] * points[:, 1]
    ])
    features = np.hstack([features, quad_features])
    
    return features

# =============================================================================
# 5. 改进的变换计算函数
# =============================================================================

def find_affine_transform_robust(source_points, target_points, regularization=REGULARIZATION_LAMBDA):
    """使用正则化最小二乘法计算稳健的仿射变换矩阵。"""
    num_points = source_points.shape[0]
    if num_points < 3:
        return None
    
    # 构建系数矩阵
    A = np.zeros((2 * num_points, 6))
    for i in range(num_points):
        x, z = source_points[i]
        A[2 * i] = [x, z, 1, 0, 0, 0]
        A[2 * i + 1] = [0, 0, 0, x, z, 1]
    
    B = target_points.flatten()
    
    # 正则化求解
    ATA = A.T @ A + regularization * np.eye(6)
    ATB = A.T @ B
    
    try:
        p = np.linalg.solve(ATA, ATB)
        a, b, tx, c, d, ty = p
        return np.array([[a, b, tx], [c, d, ty], [0, 0, 1]])
    except np.linalg.LinAlgError:
        logging.warning("正则化求解失败，尝试SVD方法")
        try:
            p, _, _, _ = np.linalg.lstsq(A, B, rcond=regularization)
            a, b, tx, c, d, ty = p
            return np.array([[a, b, tx], [c, d, ty], [0, 0, 1]])
        except:
            return None

def apply_affine_transform(points, T_affine):
    """将仿射变换矩阵应用到点上。"""
    if points.ndim == 1:
        points = points.reshape(1, -1)
    points_homogeneous = np.hstack([points, np.ones((points.shape[0], 1))])
    transformed_homogeneous = T_affine @ points_homogeneous.T
    return transformed_homogeneous[:2, :].T

def adaptive_ransac_for_affine(source_points, target_points):
    """改进的自适应RANSAC算法寻找稳健的仿射变换。"""
    logging.info("开始自适应RANSAC仿射变换求解...")
    
    total_points = len(source_points)
    
    # 自适应采样大小
    sample_size = max(RANSAC_MIN_SAMPLE_SIZE, 
                     min(RANSAC_MAX_SAMPLE_SIZE, int(0.02 * total_points)))
    
    # 估计初始阈值
    threshold = RANSAC_BASE_THRESHOLD
    if total_points > 100:
        # 使用子集进行初始估计
        subset_indices = np.random.choice(total_points, min(100, total_points), replace=False)
        initial_T = find_affine_transform_robust(
            source_points[subset_indices], 
            target_points[subset_indices]
        )
        if initial_T is not None:
            initial_pred = apply_affine_transform(source_points[subset_indices], initial_T)
            initial_errors = np.linalg.norm(target_points[subset_indices] - initial_pred, axis=1)
            threshold = max(RANSAC_BASE_THRESHOLD, np.percentile(initial_errors, 75))
    
    logging.info(f"RANSAC参数: 采样大小={sample_size}, 内点阈值={threshold*1000:.1f}mm")
    
    best_inlier_count = -1
    best_T_affine = None
    best_inlier_indices = None
    best_score = -np.inf
    
    for i in range(RANSAC_ITERATIONS):
        # 随机采样
        sample_indices = np.random.choice(total_points, sample_size, replace=False)
        sample_source = source_points[sample_indices]
        sample_target = target_points[sample_indices]
        
        # 计算临时模型
        T_temp = find_affine_transform_robust(sample_source, sample_target)
        if T_temp is None:
            continue
        
        # 测试所有点
        predicted_points = apply_affine_transform(source_points, T_temp)
        distances = np.linalg.norm(target_points - predicted_points, axis=1)
        
        # 计算内点
        inlier_indices = np.where(distances < threshold)[0]
        inlier_count = len(inlier_indices)
        
        # 计算综合评分 (内点数 + 内点质量)
        if inlier_count > 0:
            inlier_error = np.mean(distances[inlier_indices])
            score = inlier_count - inlier_error * 1000  # 平衡内点数和误差
        else:
            score = -np.inf
        
        # 更新最佳模型
        if score > best_score and inlier_count > max(6, total_points * 0.1):
            best_score = score
            best_inlier_count = inlier_count
            best_inlier_indices = inlier_indices
            # 使用所有内点重新拟合更精确的模型
            best_T_affine = find_affine_transform_robust(
                source_points[best_inlier_indices], 
                target_points[best_inlier_indices]
            )
            if i % 100 == 0 or i < 50:
                logging.info(f"RANSAC 迭代 {i}: 新最佳模型，内点数={best_inlier_count}/{total_points} "
                           f"({best_inlier_count/total_points*100:.1f}%)")
    
    if best_T_affine is None:
        logging.error("RANSAC未能找到有效的仿射变换")
        return None, None
    
    # 最终优化：使用所有内点进行最后一次拟合
    final_T = find_affine_transform_robust(
        source_points[best_inlier_indices], 
        target_points[best_inlier_indices]
    )
    
    logging.info(f"RANSAC完成。最佳模型有{best_inlier_count}个内点 "
                f"({best_inlier_count/total_points*100:.1f}%)")
    
    return final_T if final_T is not None else best_T_affine, best_inlier_indices

# =============================================================================
# 6. 数据加载与处理
# =============================================================================

def load_and_prepare_data(csv_path):
    """从CSV加载数据并准备为Numpy数组。"""
    logging.info(f"从 {csv_path} 加载数据...")
    
    if not os.path.exists(csv_path):
        logging.warning(f"数据文件未找到: {csv_path}")
        logging.info("生成模拟数据进行演示...")
        return generate_simulation_data()
        
    try:
        df = pd.read_csv(csv_path)
        logging.info(f"成功加载 {len(df)} 个数据点。")
        
        source_points = df[[TRACKER_X, TRACKER_Z]].values
        target_points = df[[LASER_X, LASER_Z]].values
        
        return source_points, target_points
    except Exception as e:
        logging.error(f"加载数据时出错: {e}")
        logging.info("生成模拟数据进行演示...")
        return generate_simulation_data()

def generate_simulation_data(n_points=2500):
    """生成模拟数据用于演示。"""
    logging.info(f"生成{n_points}个模拟数据点...")
    
    np.random.seed(RANDOM_STATE)
    
    # 生成源点云（在一个矩形区域内）
    source_x = np.random.uniform(-2.0, 2.0, n_points)
    source_z = np.random.uniform(-1.5, 1.5, n_points)
    source_points = np.column_stack([source_x, source_z])
    
    # 应用已知旋转
    R_2D = R_EULER_3D[0:2, 0:2]
    rotated_points = (R_2D @ source_points.T).T
    
    # 应用仿射变换（模拟残余变换）
    true_affine = np.array([
        [1.02, 0.05, 0.1],   # 轻微缩放和剪切
        [-0.03, 0.98, -0.05], # 
        [0, 0, 1]
    ])
    affine_points = apply_affine_transform(rotated_points, true_affine)
    
    # 添加非线性变形（模拟投影效果）
    nonlinear_distortion = np.zeros_like(affine_points)
    
    # 径向畸变
    radial_dist = np.linalg.norm(affine_points, axis=1)
    distortion_strength = 0.01
    radial_factor = distortion_strength * radial_dist**2
    
    nonlinear_distortion[:, 0] = radial_factor * np.cos(np.arctan2(affine_points[:, 1], affine_points[:, 0]))
    nonlinear_distortion[:, 1] = radial_factor * np.sin(np.arctan2(affine_points[:, 1], affine_points[:, 0]))
    
    # 添加空间相关的高频变形
    spatial_freq = 0.8
    nonlinear_distortion[:, 0] += 0.005 * np.sin(spatial_freq * affine_points[:, 0]) * np.cos(spatial_freq * affine_points[:, 1])
    nonlinear_distortion[:, 1] += 0.005 * np.cos(spatial_freq * affine_points[:, 0]) * np.sin(spatial_freq * affine_points[:, 1])
    
    target_points = affine_points + nonlinear_distortion
    
    # 添加噪声
    noise_level = 0.002  # 2mm std
    target_points += np.random.normal(0, noise_level, target_points.shape)
    
    logging.info("模拟数据生成完成。包含旋转、仿射变换、非线性畸变和噪声。")
    
    return source_points, target_points

# =============================================================================
# 7. 评估与可视化
# =============================================================================

def comprehensive_evaluation(stage_name, source, target, predicted, save_prefix):
    """计算多种评估指标并生成详细可视化。"""
    # 计算各种误差指标
    errors = np.linalg.norm(target - predicted, axis=1)
    rmse = np.sqrt(np.mean(errors**2))
    mae = np.mean(errors)
    max_error = np.max(errors)
    percentile_95 = np.percentile(errors, 95)
    
    # 转换为毫米
    rmse_mm = rmse * 1000
    mae_mm = mae * 1000
    max_error_mm = max_error * 1000
    percentile_95_mm = percentile_95 * 1000
    
    logging.info(f"[{stage_name}] 性能指标:")
    logging.info(f"  RMSE: {rmse_mm:.3f} mm")
    logging.info(f"  MAE:  {mae_mm:.3f} mm")
    logging.info(f"  Max:  {max_error_mm:.3f} mm")
    logging.info(f"  95%:  {percentile_95_mm:.3f} mm")

    # 创建详细可视化
    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(20, 16))
    
    # 1. 散点图对比
    ax1.scatter(target[:, 0], target[:, 1], c='red', s=15, alpha=0.7, label='目标点 (Laser)')
    ax1.scatter(predicted[:, 0], predicted[:, 1], c='green', s=15, alpha=0.7, label='预测点')
    
    # 绘制误差向量（采样显示）
    sample_indices = np.random.choice(len(target), min(100, len(target)), replace=False)
    for i in sample_indices:
        ax1.arrow(predicted[i, 0], predicted[i, 1], 
                  target[i, 0] - predicted[i, 0], target[i, 1] - predicted[i, 1],
                  head_width=0.005, head_length=0.01, fc='gray', ec='gray', alpha=0.6)
    
    ax1.set_xlabel("X (m)")
    ax1.set_ylabel("Z (m)")
    ax1.set_title(f"{stage_name} - 预测 vs 目标\nRMSE: {rmse_mm:.3f} mm")
    ax1.legend()
    ax1.grid(True, linestyle='--', alpha=0.6)
    ax1.axis('equal')
    
    # 2. 误差分布直方图
    ax2.hist(errors * 1000, bins=50, color='skyblue', edgecolor='black', alpha=0.7)
    ax2.axvline(mae_mm, color='red', linestyle='--', linewidth=2, label=f'MAE: {mae_mm:.2f} mm')
    ax2.axvline(rmse_mm, color='orange', linestyle='--', linewidth=2, label=f'RMSE: {rmse_mm:.2f} mm')
    ax2.axvline(percentile_95_mm, color='purple', linestyle='--', linewidth=2, label=f'95%: {percentile_95_mm:.2f} mm')
    ax2.set_xlabel("误差 (mm)")
    ax2.set_ylabel("频数")
    ax2.set_title("点对点误差分布")
    ax2.legend()
    ax2.grid(True, linestyle='--', alpha=0.6)
    
    # 3. 空间误差分布
    scatter = ax3.scatter(target[:, 0], target[:, 1], c=errors*1000, 
                         cmap='viridis', s=20, alpha=0.8)
    plt.colorbar(scatter, ax=ax3, label='误差 (mm)')
    ax3.set_xlabel("X (m)")
    ax3.set_ylabel("Z (m)")
    ax3.set_title("空间误差分布")
    ax3.grid(True, linestyle='--', alpha=0.6)
    ax3.axis('equal')
    
    # 4. 误差累积分布
    sorted_errors = np.sort(errors * 1000)
    percentiles = np.arange(1, len(sorted_errors) + 1) / len(sorted_errors) * 100
    ax4.plot(sorted_errors, percentiles, 'b-', linewidth=2)
    ax4.axvline(mae_mm, color='red', linestyle='--', label=f'MAE: {mae_mm:.2f} mm')
    ax4.axvline(rmse_mm, color='orange', linestyle='--', label=f'RMSE: {rmse_mm:.2f} mm')
    ax4.axhline(95, color='purple', linestyle='--', alpha=0.7)
    ax4.set_xlabel("误差 (mm)")
    ax4.set_ylabel("累积百分比 (%)")
    ax4.set_title("误差累积分布函数")
    ax4.legend()
    ax4.grid(True, linestyle='--', alpha=0.6)
    
    plt.suptitle(f"'{stage_name}' 阶段性能评估", fontsize=18)
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.savefig(os.path.join(IMG_DIR, f"{save_prefix}_comprehensive_evaluation.png"), dpi=300, bbox_inches='tight')
    plt.close()
    
    return {
        'rmse': rmse,
        'mae': mae,
        'max_error': max_error,
        'percentile_95': percentile_95,
        'rmse_mm': rmse_mm,
        'mae_mm': mae_mm,
        'max_error_mm': max_error_mm,
        'percentile_95_mm': percentile_95_mm
    }

# =============================================================================
# 8. 主执行流程
# =============================================================================

def main():
    """优化版主函数，执行完整的离线训练流程。"""
    setup_environment()
    logging.info("开始优化版混合模型离线训练流程...")

    try:
        # --- 步骤 0: 加载和验证数据 ---
        logging.info("\n" + "="*60)
        logging.info("步骤 0: 数据加载与验证")
        logging.info("="*60)
        
        source_points, target_points = load_and_prepare_data(LASER_TRACKER_CSV)
        if source_points is None:
            logging.error("数据加载失败，退出程序")
            return

        # 数据验证
        validate_data(source_points, target_points)
        
        # 移除异常值
        source_points, target_points, inlier_mask = remove_outliers(source_points, target_points)
        
        # 数据划分
        (src_train, src_test, tgt_train, tgt_test) = train_test_split(
            source_points, target_points, train_size=TRAIN_TEST_RATIO, random_state=RANDOM_STATE
        )
        logging.info(f"数据划分完成: {len(src_train)} 训练点, {len(src_test)} 测试点")

        # --- 步骤 1: 应用已知旋转 ---
        logging.info("\n" + "="*60)
        logging.info("步骤 1: 应用已知旋转变换")
        logging.info("="*60)
        
        R_known_2D = R_EULER_3D[0:2, 0:2]
        logging.info(f"从3D矩阵中提取的2D旋转矩阵:\n{R_known_2D}")
        
        # 验证旋转矩阵的正交性
        orthogonality_check = np.allclose(R_known_2D @ R_known_2D.T, np.eye(2), atol=1e-6)
        det_check = np.allclose(np.linalg.det(R_known_2D), 1.0, atol=1e-6)
        
        if not orthogonality_check or not det_check:
            logging.warning("警告: 提供的旋转矩阵可能不是有效的正交矩阵")
        
        src_train_rotated = (R_known_2D @ src_train.T).T
        logging.info("已将旋转变换应用于训练集源点")

        # --- 步骤 2: 自适应RANSAC求解残余仿射变换 ---
        logging.info("\n" + "="*60)
        logging.info("步骤 2: 自适应RANSAC求解残余仿射变换")
        logging.info("="*60)
        
        T_residual_affine, inlier_indices = adaptive_ransac_for_affine(src_train_rotated, tgt_train)
        if T_residual_affine is None:
            logging.error("RANSAC未能找到有效的仿射变换，退出程序")
            return
        
        logging.info(f"计算出的残余仿射变换矩阵:\n{T_residual_affine}")
        
        # 分析仿射变换的性质
        A = T_residual_affine[:2, :2]
        t = T_residual_affine[:2, 2]
        det_A = np.linalg.det(A)
        U, S, Vt = np.linalg.svd(A)
        
        logging.info(f"仿射变换分析:")
        logging.info(f"  行列式: {det_A:.6f} (面积缩放因子)")
        logging.info(f"  奇异值: [{S[0]:.6f}, {S[1]:.6f}] (主轴缩放)")
        logging.info(f"  平移向量: [{t[0]:.6f}, {t[1]:.6f}] m")

        # --- 步骤 3: 计算非线性残差并准备MLP训练数据 ---
        logging.info("\n" + "="*60)
        logging.info("步骤 3: 计算非线性残差并准备MLP训练数据")
        logging.info("="*60)
        
        # 应用完整线性变换
        train_affine_corrected = apply_affine_transform(src_train_rotated, T_residual_affine)
        
        # 计算最终残差（MLP的训练目标）
        train_final_residuals = tgt_train - train_affine_corrected
        residual_magnitude = np.linalg.norm(train_final_residuals, axis=1)
        
        logging.info(f"非线性残差统计:")
        logging.info(f"  平均幅度: {np.mean(residual_magnitude)*1000:.3f} mm")
        logging.info(f"  标准差: {np.std(residual_magnitude)*1000:.3f} mm")
        logging.info(f"  最大值: {np.max(residual_magnitude)*1000:.3f} mm")
        
        # 提取增强特征
        center_point = np.mean(train_affine_corrected, axis=0)
        train_features = extract_enhanced_features(train_affine_corrected, center_point)
        
        logging.info(f"MLP特征维度: {train_features.shape[1]}")
        logging.info(f"特征包括: 位置(2) + 径向距离(1) + 角度(1) + 相对坐标(2) + 二次项(3)")

        # --- 步骤 4: 训练增强MLP模型 ---
        logging.info("\n" + "="*60)
        logging.info("步骤 4: 训练增强MLP模型")
        logging.info("="*60)
        
        # 特征标准化
        feature_scaler = StandardScaler()
        train_features_scaled = feature_scaler.fit_transform(train_features)
        
        # 输出标准化
        target_scaler = StandardScaler()
        train_residuals_scaled = target_scaler.fit_transform(train_final_residuals)
        
        # 配置并训练MLP
        mlp_model = MLPRegressor(
            hidden_layer_sizes=MLP_HIDDEN_LAYERS,
            activation='relu',
            solver='adam',
            max_iter=MLP_MAX_ITER,
            random_state=RANDOM_STATE,
            early_stopping=MLP_EARLY_STOPPING,
            validation_fraction=MLP_VALIDATION_FRACTION,
            n_iter_no_change=50,
            learning_rate_init=0.001,
            alpha=0.001  # L2正则化
        )
        
        logging.info(f"开始训练MLP模型，网络结构: {MLP_HIDDEN_LAYERS}")
        mlp_model.fit(train_features_scaled, train_residuals_scaled)
        logging.info(f"MLP模型训练完成。最终迭代次数: {mlp_model.n_iter_}")
        
        # 训练性能分析
        train_pred_residuals_scaled = mlp_model.predict(train_features_scaled)
        train_pred_residuals = target_scaler.inverse_transform(train_pred_residuals_scaled)
        train_mlp_rmse = np.sqrt(mean_squared_error(train_final_residuals, train_pred_residuals))
        
        logging.info(f"训练集上MLP残差预测RMSE: {train_mlp_rmse*1000:.3f} mm")

        # --- 步骤 5: 测试集综合评估 ---
        logging.info("\n" + "="*60)
        logging.info("步骤 5: 测试集端到端性能评估")
        logging.info("="*60)
        
        results = {}
        
        # 阶段A: 仅应用已知旋转
        logging.info("评估阶段 A: 仅应用已知旋转...")
        src_test_rotated = (R_known_2D @ src_test.T).T
        results['stage_A'] = comprehensive_evaluation(
            "阶段A - 仅已知旋转", src_test, tgt_test, src_test_rotated, "A_rotation_only"
        )

        # 阶段B: 应用旋转 + 仿射变换
        logging.info("评估阶段 B: 应用旋转 + 仿射变换...")
        test_affine_corrected = apply_affine_transform(src_test_rotated, T_residual_affine)
        results['stage_B'] = comprehensive_evaluation(
            "阶段B - 旋转+仿射", src_test, tgt_test, test_affine_corrected, "B_affine"
        )

        # 阶段C: 应用完整混合模型
        logging.info("评估阶段 C: 应用完整混合模型...")
        test_features = extract_enhanced_features(test_affine_corrected, center_point)
        test_features_scaled = feature_scaler.transform(test_features)
        predicted_residuals_scaled = mlp_model.predict(test_features_scaled)
        predicted_residuals = target_scaler.inverse_transform(predicted_residuals_scaled)
        final_predicted = test_affine_corrected + predicted_residuals
        
        results['stage_C'] = comprehensive_evaluation(
            "阶段C - 完整混合模型", src_test, tgt_test, final_predicted, "C_final_hybrid"
        )

        # --- 步骤 6: 保存所有模型工件 ---
        logging.info("\n" + "="*60)
        logging.info("步骤 6: 保存模型工件和配置")
        logging.info("="*60)
        
        # 保存模型参数
        np.save(os.path.join(MODEL_DIR, "R_known_2D.npy"), R_known_2D)
        np.save(os.path.join(MODEL_DIR, "T_residual_affine.npy"), T_residual_affine)
        np.save(os.path.join(MODEL_DIR, "center_point.npy"), center_point)
        
        # 保存标准化器
        joblib.dump(feature_scaler, os.path.join(MODEL_DIR, "feature_scaler.pkl"))
        joblib.dump(target_scaler, os.path.join(MODEL_DIR, "target_scaler.pkl"))
        
        # 保存MLP模型
        joblib.dump(mlp_model, os.path.join(MODEL_DIR, "mlp_model.pkl"))
        
        # 保存配置信息
        config = {
            'R_EULER_3D': R_EULER_3D.tolist(),
            'MLP_HIDDEN_LAYERS': MLP_HIDDEN_LAYERS,
            'RANSAC_ITERATIONS': RANSAC_ITERATIONS,
            'RANSAC_BASE_THRESHOLD': RANSAC_BASE_THRESHOLD,
            'train_points': len(src_train),
            'test_points': len(src_test),
            'feature_dim': train_features.shape[1],
            'mlp_iterations': int(mlp_model.n_iter_)
        }
        
        import json
        with open(os.path.join(MODEL_DIR, "model_config.json"), "w") as f:
            json.dump(config, f, indent=2)
        
        logging.info(f"所有模型工件已保存到: {MODEL_DIR}")

        # --- 最终总结报告 ---
        improvement_B_to_C = results['stage_B']['rmse_mm'] - results['stage_C']['rmse_mm']
        improvement_A_to_C = results['stage_A']['rmse_mm'] - results['stage_C']['rmse_mm']
        
        summary = f"""
=========================================================
          优化版混合模型离线训练总结
=========================================================
数据源: {os.path.basename(LASER_TRACKER_CSV) if os.path.exists(LASER_TRACKER_CSV) else "模拟数据"}
总数据点: {len(source_points)}
训练点数: {len(src_train)}
测试点数: {len(src_test)}

模型配置:
---------------------------------------------------------
RANSAC迭代次数: {RANSAC_ITERATIONS}
RANSAC内点阈值: {RANSAC_BASE_THRESHOLD*1000:.1f} mm
MLP网络结构: {MLP_HIDDEN_LAYERS}
MLP特征维度: {train_features.shape[1]}
MLP训练迭代: {mlp_model.n_iter_}

测试集性能评估 (RMSE):
---------------------------------------------------------
阶段 A (仅已知旋转):      {results['stage_A']['rmse_mm']:.4f} mm
阶段 B (旋转 + 仿射):     {results['stage_B']['rmse_mm']:.4f} mm  
阶段 C (完整混合模型):    {results['stage_C']['rmse_mm']:.4f} mm
---------------------------------------------------------

性能提升分析:
---------------------------------------------------------
从阶段A到阶段C的提升:    {improvement_A_to_C:.4f} mm ({improvement_A_to_C/results['stage_A']['rmse_mm']*100:.1f}%)
从阶段B到阶段C的提升:    {improvement_B_to_C:.4f} mm ({improvement_B_to_C/results['stage_B']['rmse_mm']*100:.1f}%)

详细性能指标 (阶段C):
---------------------------------------------------------
RMSE: {results['stage_C']['rmse_mm']:.4f} mm
MAE:  {results['stage_C']['mae_mm']:.4f} mm
Max:  {results['stage_C']['max_error_mm']:.4f} mm
95%:  {results['stage_C']['percentile_95_mm']:.4f} mm
=========================================================

模型文件保存位置: {MODEL_DIR}
日志文件位置: {os.path.join(LOG_DIR, "optimized_training_process.log")}
图表保存位置: {IMG_DIR}
=========================================================
"""
        
        print(summary)
        logging.info(summary)
        
        with open(os.path.join(OUTPUT_DIR, "comprehensive_summary.txt"), "w", encoding='utf-8') as f:
            f.write(summary)
        
        logging.info(f"优化版训练流程成功完成！所有结果保存在: {OUTPUT_DIR}")
        
    except Exception as e:
        logging.error(f"训练过程中发生错误: {str(e)}")
        import traceback
        logging.error(f"详细错误信息:\n{traceback.format_exc()}")
        raise


if __name__ == "__main__":
    main()