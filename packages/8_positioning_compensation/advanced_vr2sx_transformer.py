#!/usr/bin/env python

import sys
sys.path.append("/home/cityu-fsm-lab-carla/WorkplaceCarla/op_carla/op_bridge/op_bridge/fsm_lab_simulation")

import logging
import os
import numpy as np
import time
from vive_tracker import ViveTrackerModule
from scipy.spatial.transform import Rotation as R
from scipy.interpolate import RBFInterpolator
from scipy.spatial.distance import cdist

logging.basicConfig(level=logging.INFO)

class AdvancedVR2SandBoxTransformer:
    """
    高级VR到沙箱变换器
    位置变换：使用标定得到的T_svd + RBF修正
    方向变换：使用原始代码中的R_euler矩阵
    """

    def __init__(self, tracker_name="tracker_1", calibration_dir=None, max_control_points=50, rbf_kernel='cubic', epsilon=None):
        """
        初始化高级变换器
        
        Args:
            tracker_name (str): Vive Tracker名称
            calibration_dir (str): 标定结果目录路径
            max_control_points (int): RBF控制点最大数量
            rbf_kernel (str): RBF核函数类型
            epsilon (float): 某些核函数需要的形状参数
        """
        if calibration_dir is None:
            raise ValueError("必须提供标定结果的目录路径 (calibration_dir)!")

        self.TRACKER_NAME = tracker_name
        self.max_control_points = max_control_points
        self.rbf_kernel = rbf_kernel
        self.epsilon = epsilon
        
        # 验证RBF参数
        self._validate_rbf_parameters()
        
        # 兼容原接口的属性
        self.previous_position = None
        self.previous_yaw = None
        self.current_position = None
        self.current_yaw = None
        
        # 方向变换矩阵
        # self.R_euler = np.array([
        #     [ 0.80453205, -0.59390924,  0.0       ],
        #     [ 0.59390924,  0.80453205,  0.0       ],
        #     [ 0.0       ,  0.0       ,  1.0       ],
        # ])
        self.R_euler = np.array([
            [ 0.67654891, -0.7363977 ,  0.0        ],
            [ 0.7363977 ,  0.67654891,  0.0        ],
            [ 0.0       ,  0.0       ,  1.0        ]])
        
        logging.info("使用原始代码中的R_euler矩阵进行方向变换")
        
        # 性能监控
        self.transform_times = []
        self.last_performance_report = time.time()
        
        # 初始化Vive Tracker
        self.vtm = ViveTrackerModule()
        self.vtm.print_discovered_objects()
        logging.info(f'inited vtm {self.vtm.print_discovered_objects()}')
        self.tracker = self.vtm.devices[self.TRACKER_NAME]
        
        # 标定变换矩阵和RBF模型组件
        self.T_svd = None  # 从标定结果加载的变换矩阵（用于位置）
        self.rbf_x = None
        self.rbf_z = None
        self.control_points_xz = None
        self.workspace_bounds = None
        
        # 加载标定结果和RBF模型
        self._load_calibration_and_rbf_model(calibration_dir)

    def _validate_rbf_parameters(self):
        """验证和设置RBF参数"""
        kernel_info = {
            'thin_plate_spline': {'needs_epsilon': False, 'speed': 'slow', 'accuracy': 'high'},
            'cubic': {'needs_epsilon': False, 'speed': 'medium', 'accuracy': 'high'},
            'quintic': {'needs_epsilon': False, 'speed': 'medium', 'accuracy': 'medium'},
            'linear': {'needs_epsilon': False, 'speed': 'fast', 'accuracy': 'low'},
            'multiquadric': {'needs_epsilon': True, 'speed': 'medium', 'accuracy': 'medium'},
            'inverse_multiquadric': {'needs_epsilon': True, 'speed': 'medium', 'accuracy': 'medium'},
            'gaussian': {'needs_epsilon': True, 'speed': 'slow', 'accuracy': 'high'}
        }
        
        if self.rbf_kernel not in kernel_info:
            logging.warning(f"未知的核函数 {self.rbf_kernel}，使用默认的 'cubic'")
            self.rbf_kernel = 'cubic'
        
        kernel_config = kernel_info[self.rbf_kernel]
        
        if kernel_config['needs_epsilon'] and self.epsilon is None:
            self.epsilon = 1.0
            logging.info(f"为核函数 '{self.rbf_kernel}' 自动设置 epsilon = {self.epsilon}")
        
        if not kernel_config['needs_epsilon'] and self.epsilon is not None:
            logging.info(f"核函数 '{self.rbf_kernel}' 不需要 epsilon 参数，忽略提供的值")
            self.epsilon = None
        
        logging.info(f"RBF配置: 核函数='{self.rbf_kernel}', 速度={kernel_config['speed']}, 精度={kernel_config['accuracy']}")

    def _adaptive_control_point_selection(self, control_points, residuals_x, residuals_z):
        """自适应控制点选择"""
        n_points = len(control_points)
        
        if n_points <= self.max_control_points:
            logging.info(f"控制点数量 {n_points} 小于最大值 {self.max_control_points}，使用全部点")
            return control_points, residuals_x, residuals_z
        
        logging.info(f"自适应子采样控制点: {n_points} -> {self.max_control_points}")
        
        # 分析残差分布
        residual_magnitudes = np.sqrt(residuals_x**2 + residuals_z**2)
        residual_std = np.std(residual_magnitudes)
        residual_mean = np.mean(residual_magnitudes)
        
        logging.info(f"残差统计: 均值={residual_mean*1000:.2f}mm, 标准差={residual_std*1000:.2f}mm")
        
        # 使用距离采样
        indices = self._distance_based_sampling(control_points, self.max_control_points)
        
        return control_points[indices], residuals_x[indices], residuals_z[indices]

    def _distance_based_sampling(self, points, target_count):
        """基于距离的贪心采样"""
        n_points = len(points)
        selected_indices = []
        
        # 选择边界点作为起始点
        center = np.mean(points, axis=0)
        distances_to_center = np.linalg.norm(points - center, axis=1)
        selected_indices.append(np.argmax(distances_to_center))
        
        while len(selected_indices) < target_count:
            remaining_indices = [i for i in range(n_points) if i not in selected_indices]
            if not remaining_indices:
                break
            
            max_min_distance = -1
            best_idx = None
            
            for candidate_idx in remaining_indices:
                min_distance = float('inf')
                for selected_idx in selected_indices:
                    dist = np.linalg.norm(points[candidate_idx] - points[selected_idx])
                    min_distance = min(min_distance, dist)
                
                if min_distance > max_min_distance:
                    max_min_distance = min_distance
                    best_idx = candidate_idx
            
            if best_idx is not None:
                selected_indices.append(best_idx)
            else:
                break
        
        return np.array(selected_indices)

    def _load_calibration_and_rbf_model(self, calibration_dir):
        """加载标定结果和RBF模型"""
        logging.info("正在加载标定结果和RBF模型...")
        
        # 文件路径
        svd_matrix_file = os.path.join(calibration_dir, "T_svd.txt")
        inliers_file = os.path.join(calibration_dir, "ransac_inliers.txt")

        if not os.path.exists(svd_matrix_file) or not os.path.exists(inliers_file):
            raise FileNotFoundError(f"标定文件未找到! 路径: {calibration_dir}")

        # 加载标定得到的SVD变换矩阵（用于位置变换）
        self.T_svd = np.loadtxt(svd_matrix_file)
        logging.info("标定T_svd变换矩阵加载成功（用于位置变换）")
        
        # 打印T_svd信息用于验证
        logging.info(f"T_svd矩阵形状: {self.T_svd.shape}")
        logging.info(f"T_svd平移部分: [{self.T_svd[0,3]:.4f}, {self.T_svd[1,3]:.4f}, {self.T_svd[2,3]:.4f}]")

        # 加载并处理内点数据
        inlier_data = np.loadtxt(inliers_file, skiprows=1)
        source_points = inlier_data[:, :3]
        target_points = inlier_data[:, 3:]
        
        # 关键：使用标定得到的T_svd计算基准变换
        logging.info("使用标定得到的T_svd矩阵计算基准变换...")
        
        # 使用T_svd变换源点
        source_homo = np.hstack([source_points, np.ones((source_points.shape[0], 1))])
        tsvd_transformed_homo = (self.T_svd @ source_homo.T).T
        tsvd_transformed = tsvd_transformed_homo[:, :3]
        
        # 计算T_svd变换后与目标点的残差
        residuals = target_points - tsvd_transformed
        
        logging.info(f"基于T_svd的残差统计:")
        logging.info(f"  X方向残差: 均值={np.mean(residuals[:, 0])*1000:.2f}mm, 标准差={np.std(residuals[:, 0])*1000:.2f}mm")
        logging.info(f"  Z方向残差: 均值={np.mean(residuals[:, 2])*1000:.2f}mm, 标准差={np.std(residuals[:, 2])*1000:.2f}mm")
        logging.info(f"  整体残差RMSE: {np.sqrt(np.mean(np.sum(residuals**2, axis=1)))*1000:.2f}mm")
        
        # 准备RBF数据（使用T_svd变换后的点作为控制点）
        control_points_xz = tsvd_transformed[:, [0, 2]]
        residuals_x = residuals[:, 0]
        residuals_z = residuals[:, 2]
        
        # 控制点选择
        control_points_xz, residuals_x, residuals_z = self._adaptive_control_point_selection(
            control_points_xz, residuals_x, residuals_z
        )
        
        # 存储控制点
        self.control_points_xz = control_points_xz
        self._compute_workspace_bounds()
        
        # 创建RBF模型
        try:
            rbf_kwargs = {
                'kernel': self.rbf_kernel,
                'smoothing': 0.001
            }
            
            if self.epsilon is not None:
                rbf_kwargs['epsilon'] = self.epsilon
            
            self.rbf_x = RBFInterpolator(control_points_xz, residuals_x, **rbf_kwargs)
            self.rbf_z = RBFInterpolator(control_points_xz, residuals_z, **rbf_kwargs)
            
            logging.info(f"RBF位置修正模型创建成功:")
            logging.info(f"  - 控制点数量: {len(control_points_xz)}")
            logging.info(f"  - 核函数: {self.rbf_kernel}")
            logging.info(f"  - 位置变换: 标定T_svd + RBF修正")
            logging.info(f"  - 方向变换: 原始R_euler矩阵")
            
            # 性能测试
            self._performance_test()
            
        except Exception as e:
            logging.error(f"RBF模型创建失败: {e}")
            raise

    def _compute_workspace_bounds(self):
        """计算工作空间边界"""
        x_coords = self.control_points_xz[:, 0]
        z_coords = self.control_points_xz[:, 1]
        
        x_buffer = (x_coords.max() - x_coords.min()) * 0.1
        z_buffer = (z_coords.max() - z_coords.min()) * 0.1
        
        self.workspace_bounds = {
            'x_min': x_coords.min() - x_buffer,
            'x_max': x_coords.max() + x_buffer,
            'z_min': z_coords.min() - z_buffer,
            'z_max': z_coords.max() + z_buffer
        }

    def _performance_test(self):
        """性能测试"""
        logging.info("执行性能测试...")
        
        # 生成测试点
        test_points = []
        bounds = self.workspace_bounds
        for _ in range(100):
            x = np.random.uniform(bounds['x_min'], bounds['x_max'])
            z = np.random.uniform(bounds['z_min'], bounds['z_max'])
            test_points.append([x, 0, z])
        
        # 测试变换性能
        start_time = time.time()
        for test_point in test_points:
            _ = self._apply_enhanced_position_transform(test_point)
        end_time = time.time()
        
        avg_time_per_transform = (end_time - start_time) / len(test_points)
        max_frequency = 1.0 / avg_time_per_transform
        
        logging.info(f"性能测试结果:")
        logging.info(f"  - 平均变换时间: {avg_time_per_transform*1000:.2f}ms")
        logging.info(f"  - 理论最大频率: {max_frequency:.1f}Hz")
        
        if max_frequency < 50:
            logging.warning(f"警告: 当前性能可能无法稳定达到50Hz!")

    def _is_outside_workspace(self, point_xz):
        """检查点是否在工作空间外"""
        bounds = self.workspace_bounds
        return (point_xz[0] < bounds['x_min'] or point_xz[0] > bounds['x_max'] or
                point_xz[1] < bounds['z_min'] or point_xz[1] > bounds['z_max'])

    def _apply_enhanced_position_transform(self, position):
        """
        应用增强的位置变换：T_svd + RBF修正
        
        Args:
            position: 输入位置 [x, y, z]
            
        Returns:
            corrected_position: 修正后的位置
        """
        start_time = time.time()
        
        try:
            # 步骤1: 使用标定得到的T_svd进行基础变换
            position_homo = np.append(position, 1)
            tsvd_transformed_homo = self.T_svd @ position_homo
            tsvd_transformed = tsvd_transformed_homo[:3]
            
            # 步骤2: 应用RBF修正
            query_point_xz = tsvd_transformed[[0, 2]]
            
            if self._is_outside_workspace(query_point_xz):
                # 超出工作空间，直接返回T_svd变换结果
                final_position = tsvd_transformed
                logging.debug("查询点超出工作空间，使用T_svd结果")
            else:
                # 在工作空间内，应用RBF修正
                deformation_x = self.rbf_x([query_point_xz])[0]
                deformation_z = self.rbf_z([query_point_xz])[0]
                deformation_vector = np.array([deformation_x, 0, deformation_z])
                final_position = tsvd_transformed + deformation_vector
                
                logging.debug(f"RBF修正: dx={deformation_x*1000:.2f}mm, dz={deformation_z*1000:.2f}mm")
            
            # 性能监控
            transform_time = time.time() - start_time
            self._update_performance_stats(transform_time)
            
            return final_position
            
        except Exception as e:
            logging.error(f"增强位置变换过程出错: {e}")
            # 降级处理：返回T_svd变换结果
            position_homo = np.append(np.array(position), 1)
            return (self.T_svd @ position_homo)[:3]

    def _update_performance_stats(self, transform_time):
        """更新性能统计"""
        self.transform_times.append(transform_time)
        
        if len(self.transform_times) > 1000:
            self.transform_times.pop(0)
        
        current_time = time.time()
        if current_time - self.last_performance_report > 10.0:
            self._report_performance()
            self.last_performance_report = current_time

    def _report_performance(self):
        """报告性能统计"""
        if len(self.transform_times) < 10:
            return
        
        avg_time = np.mean(self.transform_times)
        max_time = np.max(self.transform_times)
        current_frequency = 1.0 / avg_time if avg_time > 0 else 0
        
        logging.info(f"增强位置变换性能 - 平均: {avg_time*1000:.2f}ms, 最大: {max_time*1000:.2f}ms, 频率: {current_frequency:.1f}Hz")

    def transform_point_pose(self, position, orientation, T_pos=None, R_euler=None):
        """
        完全兼容原接口的变换方法
        位置：使用T_svd + RBF修正
        方向：使用原始R_euler
        
        Args:
            position: 位置 [x, y, z]
            orientation: 方向 [roll, pitch, yaw]
            T_pos: 忽略，使用内部的T_svd + RBF
            R_euler: 忽略，使用内部的原始R_euler
        """
        # 位置变换：使用标定T_svd + RBF修正
        transformed_position = self._apply_enhanced_position_transform(position)
        
        # 方向变换：使用原始R_euler矩阵（完全保持原始逻辑）
        R_point = R.from_euler('yxz', [orientation[2], orientation[0], orientation[1]]).as_matrix()
        R_transformed = self.R_euler @ R_point
        transformed_orientation = R.from_matrix(R_transformed).as_euler('yxz')
        transformed_orientation = [transformed_orientation[0], transformed_orientation[2], transformed_orientation[1]]
        
        return transformed_position, transformed_orientation

    def transform_to_sandbox(self, cam_coord, T_pos=None, R_euler=None):
        """
        完全兼容原接口的沙箱变换方法
        忽略传入的T_pos和R_euler参数，使用内部的增强版本
        """
        orientation = [0, np.radians(round(cam_coord[4], 4)), 0]
        position = [round(cam_coord[0], 4), 0, round(cam_coord[2], 4)]
        
        transformed_position, transformed_orientation = self.transform_point_pose(position, orientation)
        yaw = transformed_orientation[1]

        return transformed_position, yaw

    def get_transformed_coor(self):
        """
        完全兼容原接口的坐标获取方法
        """
        try:
            cam_coord = self.tracker.get_pose_euler()
            transformed_position, transformed_yaw = self.transform_to_sandbox(cam_coord)
            return transformed_position, transformed_yaw
            
        except Exception as e:
            logging.error(f'get_transformed_coor 发生错误: {e}')
            return None, None