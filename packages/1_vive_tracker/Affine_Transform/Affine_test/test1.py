#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
混合模型离线训练脚本

本脚本实现了 "已知旋转 + RANSAC仿射 + MLP" 的混合变换模型。
流程:
1. 加载CSV数据。
2. 将数据划分为训练集和测试集。
3. 使用已知的3x3旋转矩阵提取2D旋转。
4. 对源点应用已知旋转。
5. 使用RANSAC在旋转后的点和目标点之间寻找一个稳健的残余仿射变换。
6. 计算经过完整线性变换后的非线性残差。
7. 训练一个MLP模型来学习这些残差。
8. 在独立的测试集上评估完整模型的性能。
9. 保存所有模型工件（旋转矩阵、仿射矩阵、MLP模型）和评估结果。
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
from sklearn.metrics import mean_squared_error

# =============================================================================
# 1. 配置与常量
# =============================================================================

# --- 文件与路径配置 ---
# 假设此脚本保存在 'scripts' 文件夹中
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PARENT_DIR = os.path.dirname(SCRIPT_DIR)
LASER_TRACKER_CSV = os.path.join(PARENT_DIR, "data/final_coordinates.csv")

# --- 输出目录配置 ---
TIMESTAMP = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
OUTPUT_DIR = os.path.join(PARENT_DIR, f"results/Hybrid_Model_Training_{TIMESTAMP}")
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

# RANSAC 参数 (用于寻找稳健的仿射变换)
RANSAC_ITERATIONS = 1000  # 迭代次数
RANSAC_SAMPLE_SIZE = 10      # 每次迭代采样点数
RANSAC_INLIER_THRESHOLD = 0.04 # 40mm, 内点阈值

# MLP 参数
MLP_HIDDEN_LAYERS = (128, 128, 64)
MLP_MAX_ITER = 1000

# --- 已知旋转矩阵  ---
R_EULER_3D = np.array([
    [ 0.67654891, -0.7363977 ,  0.0        ],
    [ 0.7363977 ,  0.67654891,  0.0        ],
    [ 0.0       ,  0.0       ,  1.0        ]
])
# R_EULER_3D = np.array([
#     [ 0.80453205, -0.59390924,  0.0       ],
#     [ 0.59390924,  0.80453205,  0.0       ],
#     [ 0.0       ,  0.0       ,  1.0       ],
# ])

# =============================================================================
# 2. 初始化设置 (日志、目录)
# =============================================================================

def setup_environment():
    """创建所有输出目录并配置日志记录。"""
    for directory in [OUTPUT_DIR, IMG_DIR, MODEL_DIR, LOG_DIR]:
        os.makedirs(directory, exist_ok=True)

    log_file = os.path.join(LOG_DIR, "training_process.log")
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(message)s',
        handlers=[
            logging.FileHandler(log_file),
            logging.StreamHandler()
        ]
    )
    logging.info("环境设置完成。")

# =============================================================================
# 3. 核心变换与模型函数
# =============================================================================

def find_affine_transform(source_points, target_points):
    """使用最小二乘法计算从源点到目标点的最优仿射变换矩阵。"""
    num_points = source_points.shape[0]
    if num_points < 3:
        return None  # 至少需要3个点
    A = np.zeros((2 * num_points, 6))
    for i in range(num_points):
        x, z = source_points[i]
        A[2 * i] = [x, z, 1, 0, 0, 0]
        A[2 * i + 1] = [0, 0, 0, x, z, 1]
    B = target_points.flatten()
    try:
        p, _, _, _ = np.linalg.lstsq(A, B, rcond=None)
        a, b, tx, c, d, ty = p
        return np.array([[a, b, tx], [c, d, ty], [0, 0, 1]])
    except np.linalg.LinAlgError:
        return None

def apply_affine_transform(points, T_affine):
    """将仿射变换矩阵应用到一个或多个点上。"""
    if points.ndim == 1:
        points = points.reshape(1, -1)
    points_homogeneous = np.hstack([points, np.ones((points.shape[0], 1))])
    transformed_homogeneous = T_affine @ points_homogeneous.T
    return transformed_homogeneous[:2, :].T

def ransac_for_affine(source_points, target_points):
    """
    使用RANSAC算法寻找一个稳健的仿射变换。
    返回最佳仿射矩阵和内点索引。
    """
    logging.info("开始为仿射变换执行RANSAC...")
    best_inlier_count = -1
    best_T_affine = None
    best_inlier_indices = None

    total_points = len(source_points)
    for i in range(RANSAC_ITERATIONS):
        # 随机采样
        sample_indices = np.random.choice(total_points, RANSAC_SAMPLE_SIZE, replace=False)
        sample_source = source_points[sample_indices]
        sample_target = target_points[sample_indices]

        # 计算临时模型
        T_temp = find_affine_transform(sample_source, sample_target)
        if T_temp is None:
            continue

        # 在所有点上测试模型
        predicted_points = apply_affine_transform(source_points, T_temp)
        distances = np.linalg.norm(target_points - predicted_points, axis=1)
        
        # 计算内点
        inlier_indices = np.where(distances < RANSAC_INLIER_THRESHOLD)[0]
        inlier_count = len(inlier_indices)

        # 更新最佳模型
        if inlier_count > best_inlier_count:
            best_inlier_count = inlier_count
            best_inlier_indices = inlier_indices
            # 使用所有找到的内点重新计算一个更精确的模型
            best_T_affine = find_affine_transform(source_points[best_inlier_indices], target_points[best_inlier_indices])
            logging.info(f"  RANSAC 迭代 {i}: 找到新的最佳模型，内点数 = {best_inlier_count}/{total_points}")

    logging.info(f"RANSAC完成。最佳模型有 {best_inlier_count} 个内点。")
    return best_T_affine, best_inlier_indices

# =============================================================================
# 4. 数据加载与处理
# =============================================================================

def load_and_prepare_data(csv_path):
    """从CSV加载数据并准备为Numpy数组。"""
    logging.info(f"从 {csv_path} 加载数据...")
    if not os.path.exists(csv_path):
        logging.error(f"数据文件未找到: {csv_path}")
        return None, None
        
    df = pd.read_csv(csv_path)
    logging.info(f"成功加载 {len(df)} 个数据点。")
    
    source_points = df[[TRACKER_X, TRACKER_Z]].values
    target_points = df[[LASER_X, LASER_Z]].values
    
    return source_points, target_points

# =============================================================================
# 5. 可视化与评估
# =============================================================================

def evaluate_and_visualize(stage_name, source, target, predicted, save_prefix):
    """计算RMSE并生成可视化图表。"""
    rmse = np.sqrt(mean_squared_error(target, predicted))
    logging.info(f"[{stage_name}] -> RMSE: {rmse:.6f} m ({rmse*1000:.3f} mm)")

    errors = np.linalg.norm(target - predicted, axis=1) * 1000 # in mm

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 7))
    
    # 散点图
    ax1.scatter(target[:, 0], target[:, 1], c='red', s=20, alpha=0.6, label='目标点 (Laser)')
    ax1.scatter(predicted[:, 0], predicted[:, 1], c='green', s=20, alpha=0.6, label='预测点')
    for i in range(0, len(target), max(1, len(target) // 50)): # 最多画50个误差向量
        ax1.arrow(predicted[i, 0], predicted[i, 1], 
                  target[i, 0] - predicted[i, 0], target[i, 1] - predicted[i, 1],
                  head_width=0.01, head_length=0.02, fc='gray', ec='gray', alpha=0.5)
    ax1.set_xlabel("X (m)")
    ax1.set_ylabel("Z (m)")
    ax1.set_title(f"{stage_name} - 预测 vs 目标\nRMSE: {rmse*1000:.3f} mm")
    ax1.legend()
    ax1.grid(True, linestyle='--', alpha=0.6)
    ax1.axis('equal')

    # 误差分布直方图
    ax2.hist(errors, bins=50, color='skyblue', edgecolor='black')
    ax2.axvline(np.mean(errors), color='red', linestyle='--', label=f'平均误差: {np.mean(errors):.2f} mm')
    ax2.set_xlabel("误差 (mm)")
    ax2.set_ylabel("频数")
    ax2.set_title("点对点误差分布")
    ax2.legend()
    ax2.grid(True, linestyle='--', alpha=0.6)

    plt.suptitle(f"'{stage_name}' 阶段性能评估", fontsize=16)
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.savefig(os.path.join(IMG_DIR, f"{save_prefix}_evaluation.png"), dpi=300)
    plt.close()
    
    return rmse

# =============================================================================
# 6. 主执行流程
# =============================================================================

def main():
    """主函数，执行完整的离线训练流程。"""
    setup_environment()
    logging.info("开始混合模型离线训练流程...")

    # --- 步骤 0: 加载和划分数据 ---
    source_points, target_points = load_and_prepare_data(LASER_TRACKER_CSV)
    if source_points is None:
        return

    (src_train, src_test, 
     tgt_train, tgt_test) = train_test_split(
        source_points, target_points, train_size=TRAIN_TEST_RATIO, random_state=RANDOM_STATE
    )
    logging.info(f"数据划分完成: {len(src_train)} 训练点, {len(src_test)} 测试点。")

    # --- 步骤 1: 提取并应用已知旋转 ---
    logging.info("\n--- 步骤 1: 应用已知旋转 ---")
    R_known_2D = R_EULER_3D[0:2, 0:2]
    logging.info(f"从3D矩阵中提取的2D旋转矩阵:\n{R_known_2D}")
    
    src_train_rotated = (R_known_2D @ src_train.T).T
    logging.info("已将旋转应用于训练集源点。")

    # --- 步骤 2: RANSAC寻找残余仿射变换 ---
    logging.info("\n--- 步骤 2: RANSAC寻找残余仿射变换 ---")
    T_residual_affine, inlier_indices = ransac_for_affine(src_train_rotated, tgt_train)
    if T_residual_affine is None:
        logging.error("RANSAC未能找到有效的仿射变换。正在退出。")
        return
    logging.info(f"计算出的残余仿射变换矩阵:\n{T_residual_affine}")

    # --- 步骤 3: 计算MLP训练数据 ---
    logging.info("\n--- 步骤 3: 计算MLP训练数据 ---")
    # 将完整的线性变换应用到所有训练点上
    train_affine_corrected = apply_affine_transform(src_train_rotated, T_residual_affine)
    
    # MLP的目标是学习线性模型无法解释的最终残差
    train_final_residuals = tgt_train - train_affine_corrected
    logging.info("MLP的输入特征(X)和输出目标(y)已生成。")

    # --- 步骤 4: 训练MLP模型 ---
    logging.info("\n--- 步骤 4: 训练MLP模型 ---")
    mlp_model = MLPRegressor(
        hidden_layer_sizes=MLP_HIDDEN_LAYERS,
        activation='relu',
        solver='adam',
        max_iter=MLP_MAX_ITER,
        random_state=RANDOM_STATE,
        early_stopping=True,
        n_iter_no_change=20
    )
    # MLP的输入是经过线性校正后的点的位置
    mlp_model.fit(train_affine_corrected, train_final_residuals)
    logging.info(f"MLP模型训练完成。最终迭代次数: {mlp_model.n_iter_}")

    # --- 步骤 5: 在测试集上评估完整模型 ---
    logging.info("\n--- 步骤 5: 在测试集上进行端到端评估 ---")
    
    # 阶段A: 仅应用已知旋转
    logging.info("评估阶段 A: 仅应用已知旋转...")
    src_test_rotated = (R_known_2D @ src_test.T).T
    rmse_a = evaluate_and_visualize(
        "阶段A - 仅已知旋转", src_test, tgt_test, src_test_rotated, "A_rotation_only"
    )

    # 阶段B: 应用旋转 + 仿射变换
    logging.info("评估阶段 B: 应用旋转 + 仿射变换...")
    test_affine_corrected = apply_affine_transform(src_test_rotated, T_residual_affine)
    rmse_b = evaluate_and_visualize(
        "阶段B - 旋转+仿射", src_test, tgt_test, test_affine_corrected, "B_affine"
    )

    # 阶段C: 应用完整混合模型 (旋转 + 仿射 + MLP)
    logging.info("评估阶段 C: 应用完整混合模型...")
    predicted_residuals = mlp_model.predict(test_affine_corrected)
    final_predicted = test_affine_corrected + predicted_residuals
    rmse_c = evaluate_and_visualize(
        "阶段C - 完整混合模型", src_test, tgt_test, final_predicted, "C_final_hybrid"
    )

    # --- 步骤 6: 保存所有模型工件 ---
    logging.info("\n--- 步骤 6: 保存模型工件 ---")
    np.save(os.path.join(MODEL_DIR, "R_known_2D.npy"), R_known_2D)
    np.save(os.path.join(MODEL_DIR, "T_residual_affine.npy"), T_residual_affine)
    joblib.dump(mlp_model, os.path.join(MODEL_DIR, "mlp_model.pkl"))
    logging.info(f"所有模型已成功保存到 '{MODEL_DIR}' 文件夹。")

    # --- 最终总结 ---
    summary = f"""
=========================================================
          混合模型离线训练总结
=========================================================
数据源: {os.path.basename(LASER_TRACKER_CSV)}
训练点数: {len(src_train)}
测试点数: {len(src_test)}

RANSAC参数:
  - 迭代次数: {RANSAC_ITERATIONS}
  - 内点阈值: {RANSAC_INLIER_THRESHOLD*1000:.1f} mm

MLP结构:
  - 隐藏层: {MLP_HIDDEN_LAYERS}

测试集性能 (RMSE):
---------------------------------------------------------
阶段 A (仅已知旋转):      {rmse_a*1000:.4f} mm
阶段 B (旋转 + 仿射):     {rmse_b*1000:.4f} mm
阶段 C (完整混合模型):    {rmse_c*1000:.4f} mm
---------------------------------------------------------
从 阶段B 到 阶段C 的提升: { (rmse_b - rmse_c)*1000:.4f} mm
=========================================================
"""
    logging.info(summary)
    with open(os.path.join(OUTPUT_DIR, "final_summary.txt"), "w") as f:
        f.write(summary)
    
    logging.info(f"训练流程结束。所有结果保存在: {OUTPUT_DIR}")


if __name__ == "__main__":
    main()