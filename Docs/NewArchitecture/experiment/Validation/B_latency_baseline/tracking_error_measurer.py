#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import csv
import os
import math
import numpy as np
import yaml
from datetime import datetime
from collections import deque
from scipy import stats
from geometry_msgs.msg import PoseWithCovarianceStamped

class IntelligentTrackingErrorMeasurer(Node):
    """
    智能跟踪误差测量器 - 自动检测直线行驶并进行基线测量
    """
    
    def __init__(self, config_file="config.yaml"):
        super().__init__('intelligent_tracking_error_measurer')
        
        # 加载配置
        self.load_config(config_file)
        
        # 会话设置
        self.session_timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.setup_directories()
        self.setup_logging()
        
        # 智能检测参数
        self.position_history = deque(maxlen=100)  # 保存最近100个位置点
        self.velocity_history = deque(maxlen=50)   # 保存最近50个速度点
        
        # 直线检测状态
        self.in_straight_line = False
        self.straight_line_start_time = None
        self.straight_line_start_position = None
        self.baseline_samples = []
        
        # 检测参数
        self.min_straight_distance = 20.0    # 最小直线距离（米）
        self.max_direction_variance = 5.0    # 最大方向变化（度）
        self.min_velocity_for_baseline = 0.3  # 基线测试最小速度（m/s）
        self.max_velocity_for_baseline = 0.7  # 基线测试最大速度（m/s）
        self.min_straight_duration = 10.0    # 最小直线持续时间（秒）
        
        # 统计信息
        self.stats = {
            'total_samples': 0,
            'straight_line_detections': 0,
            'baseline_measurements': 0,
            'valid_baseline_segments': 0
        }
        
        # 创建订阅器
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, self.pose_topic,
            self.pose_callback, 10)
        
        # 定时器用于检测和分析
        self.analysis_timer = self.create_timer(1.0, self.analyze_motion_pattern)
        
        self.get_logger().info(f"智能跟踪误差测量器启动")
        self.get_logger().info(f"将自动检测直线行驶段并测量基线误差")

    def load_config(self, config_file):
        """加载配置"""
        try:
            with open(config_file, 'r') as f:
                config = yaml.safe_load(f)
            
            # 使用A1的NDT输出作为位姿源
            self.pose_topic = config['experiment_a1']['topics']['ndt_output']
            
            # 基线测试参数（可选，用于覆盖默认值）
            if 'experiment_b' in config and 'baseline_test' in config['experiment_b']:
                baseline_config = config['experiment_b']['baseline_test']
                self.target_velocity_ms = baseline_config.get('target_velocity_ms', 0.5)
                self.min_velocity_for_baseline = self.target_velocity_ms * 0.8
                self.max_velocity_for_baseline = self.target_velocity_ms * 1.2
            else:
                self.target_velocity_ms = 0.5
            
            self.get_logger().info("配置加载成功:")
            self.get_logger().info(f"  位姿话题: {self.pose_topic}")
            self.get_logger().info(f"  目标速度范围: {self.min_velocity_for_baseline:.2f}-{self.max_velocity_for_baseline:.2f} m/s")
            
        except Exception as e:
            self.get_logger().error(f"配置加载失败: {e}")
            self.use_default_config()

    def use_default_config(self):
        """使用默认配置"""
        self.pose_topic = '/localization/pose_estimator/pose'
        self.target_velocity_ms = 0.5
        self.min_velocity_for_baseline = 0.4
        self.max_velocity_for_baseline = 0.6

    def setup_directories(self):
        """设置输出目录"""
        self.data_dir = "data"
        self.logs_dir = "logs"
        
        for directory in [self.data_dir, self.logs_dir]:
            os.makedirs(directory, exist_ok=True)

    def setup_logging(self):
        """设置CSV日志"""
        # 主数据日志
        self.csv_filename = os.path.join(self.data_dir, f'intelligent_tracking_error_{self.session_timestamp}.csv')
        self.csv_file = open(self.csv_filename, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        
        self.csv_writer.writerow([
            'sample_id', 'timestamp_ns', 'session_id',
            'vehicle_x', 'vehicle_y', 'vehicle_z', 'vehicle_yaw_rad',
            'velocity_ms', 'acceleration_ms2', 'heading_deg',
            'in_straight_line', 'straight_line_duration_s',
            'direction_variance_deg', 'velocity_variance_ms',
            'is_baseline_candidate', 'baseline_segment_id'
        ])
        
        # 基线测量日志
        self.baseline_csv_filename = os.path.join(self.data_dir, f'baseline_measurements_{self.session_timestamp}.csv')
        self.baseline_csv_file = open(self.baseline_csv_filename, 'w', newline='')
        self.baseline_csv_writer = csv.writer(self.baseline_csv_file)
        
        self.baseline_csv_writer.writerow([
            'segment_id', 'start_timestamp_ns', 'end_timestamp_ns', 'session_id',
            'start_x', 'start_y', 'end_x', 'end_y',
            'distance_m', 'duration_s', 'avg_velocity_ms',
            'velocity_std_ms', 'direction_variance_deg',
            'theoretical_steady_state_error_m', 'measured_position_drift_m',
            'lateral_deviation_m', 'sample_count'
        ])
        
        self.csv_file.flush()
        self.baseline_csv_file.flush()

    def to_nanoseconds(self, stamp):
        """转换ROS时间戳为纳秒"""
        return stamp.sec * 1_000_000_000 + stamp.nanosec

    def quaternion_to_yaw(self, q):
        """四元数转偏航角"""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def calculate_velocity_and_acceleration(self, current_pos, current_time):
        """计算速度和加速度"""
        if len(self.position_history) < 2:
            return 0.0, 0.0
        
        # 使用最近几个点计算速度
        recent_positions = list(self.position_history)[-5:]  # 最近5个点
        
        if len(recent_positions) < 2:
            return 0.0, 0.0
        
        # 计算平均速度
        velocities = []
        for i in range(1, len(recent_positions)):
            prev_pos = recent_positions[i-1]
            curr_pos = recent_positions[i]
            
            dt = (curr_pos['timestamp'] - prev_pos['timestamp']) / 1e9
            if dt > 0:
                dx = curr_pos['x'] - prev_pos['x']
                dy = curr_pos['y'] - prev_pos['y']
                velocity = math.sqrt(dx**2 + dy**2) / dt
                velocities.append(velocity)
        
        avg_velocity = np.mean(velocities) if velocities else 0.0
        
        # 计算加速度
        self.velocity_history.append({
            'timestamp': current_time,
            'velocity': avg_velocity
        })
        
        acceleration = 0.0
        if len(self.velocity_history) >= 2:
            recent_vel = list(self.velocity_history)[-2:]
            dt = (recent_vel[1]['timestamp'] - recent_vel[0]['timestamp']) / 1e9
            if dt > 0:
                acceleration = (recent_vel[1]['velocity'] - recent_vel[0]['velocity']) / dt
        
        return avg_velocity, acceleration

    def detect_straight_line_motion(self):
        """检测是否在直线行驶"""
        if len(self.position_history) < 10:
            return False, 0.0, 0.0
        
        recent_positions = list(self.position_history)[-20:]  # 分析最近20个点
        
        # 计算方向变化
        headings = []
        for i in range(1, len(recent_positions)):
            prev = recent_positions[i-1]
            curr = recent_positions[i]
            
            dx = curr['x'] - prev['x']
            dy = curr['y'] - prev['y']
            
            if abs(dx) > 0.01 or abs(dy) > 0.01:  # 避免除零
                heading = math.degrees(math.atan2(dy, dx))
                headings.append(heading)
        
        if len(headings) < 5:
            return False, 0.0, 0.0
        
        # 计算方向方差（处理角度的周期性）
        headings = np.array(headings)
        # 将角度转换为单位向量，然后计算方差
        x_components = np.cos(np.radians(headings))
        y_components = np.sin(np.radians(headings))
        
        direction_variance = np.sqrt(np.var(x_components) + np.var(y_components)) * 180 / np.pi
        
        # 计算速度方差
        recent_velocities = [vh['velocity'] for vh in list(self.velocity_history)[-10:]]
        velocity_variance = np.std(recent_velocities) if len(recent_velocities) > 1 else 0.0
        
        # 判断是否为直线行驶
        is_straight = (
            direction_variance < self.max_direction_variance and
            len(recent_velocities) > 0 and
            np.mean(recent_velocities) > self.min_velocity_for_baseline
        )
        
        return is_straight, direction_variance, velocity_variance

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        """位姿回调：处理位姿消息"""
        timestamp_ns = self.to_nanoseconds(msg.header.stamp)
        
        # 提取位置和方向
        position = {
            'timestamp': timestamp_ns,
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'z': msg.pose.pose.position.z
        }
        yaw_rad = self.quaternion_to_yaw(msg.pose.pose.orientation)
        
        # 添加到历史记录
        self.position_history.append(position)
        
        # 计算速度和加速度
        velocity_ms, acceleration_ms2 = self.calculate_velocity_and_acceleration(position, timestamp_ns)
        
        # 检测直线行驶
        is_straight, direction_variance, velocity_variance = self.detect_straight_line_motion()
        
        # 直线行驶状态管理
        straight_line_duration = 0.0
        if is_straight:
            if not self.in_straight_line:
                # 开始直线行驶
                self.in_straight_line = True
                self.straight_line_start_time = timestamp_ns
                self.straight_line_start_position = position
                self.get_logger().info("检测到直线行驶开始")
            
            straight_line_duration = (timestamp_ns - self.straight_line_start_time) / 1e9
        else:
            if self.in_straight_line:
                # 结束直线行驶
                self.end_straight_line_segment(timestamp_ns, position)
                self.in_straight_line = False
        
        # 判断是否为基线测量候选
        is_baseline_candidate = (
            is_straight and
            self.min_velocity_for_baseline <= velocity_ms <= self.max_velocity_for_baseline and
            straight_line_duration >= self.min_straight_duration
        )
        
        # 记录数据
        sample_id = f"intelligent_{self.session_timestamp}_{self.stats['total_samples']:06d}"
        baseline_segment_id = f"seg_{self.stats['straight_line_detections']:03d}" if self.in_straight_line else ""
        
        self.csv_writer.writerow([
            sample_id, timestamp_ns, self.session_timestamp,
            position['x'], position['y'], position['z'], yaw_rad,
            velocity_ms, acceleration_ms2, math.degrees(yaw_rad),
            self.in_straight_line, straight_line_duration,
            direction_variance, velocity_variance,
            is_baseline_candidate, baseline_segment_id
        ])
        self.csv_file.flush()
        
        # 如果是基线候选，添加到基线样本
        if is_baseline_candidate:
            self.baseline_samples.append({
                'timestamp': timestamp_ns,
                'position': position,
                'velocity': velocity_ms,
                'yaw': yaw_rad
            })
        
        self.stats['total_samples'] += 1

    def end_straight_line_segment(self, end_timestamp, end_position):
        """结束直线行驶段，进行基线分析"""
        if not self.straight_line_start_time or not self.baseline_samples:
            return
        
        duration_s = (end_timestamp - self.straight_line_start_time) / 1e9
        
        # 计算距离
        start_pos = self.straight_line_start_position
        distance_m = math.sqrt(
            (end_position['x'] - start_pos['x'])**2 + 
            (end_position['y'] - start_pos['y'])**2
        )
        
        # 只处理足够长的直线段
        if duration_s >= self.min_straight_duration and distance_m >= self.min_straight_distance:
            self.analyze_baseline_segment(end_timestamp, end_position, duration_s, distance_m)
            self.stats['valid_baseline_segments'] += 1
        
        self.stats['straight_line_detections'] += 1
        self.baseline_samples.clear()

    def analyze_baseline_segment(self, end_timestamp, end_position, duration_s, distance_m):
        """分析基线段"""
        if not self.baseline_samples:
            return
        
        # 计算基线统计
        velocities = [sample['velocity'] for sample in self.baseline_samples]
        avg_velocity = np.mean(velocities)
        velocity_std = np.std(velocities)
        
        # 计算理论稳态误差（这里需要从延迟分析获取，暂时使用估算值）
        estimated_total_latency_s = 0.1  # 100ms估算值，实际应从A1实验结果获取
        theoretical_error_m = avg_velocity * estimated_total_latency_s
        
        # 计算实际位置漂移
        start_pos = self.straight_line_start_position
        expected_distance = avg_velocity * duration_s
        actual_distance = distance_m
        measured_drift = actual_distance - expected_distance
        
        # 计算横向偏差（简化版）
        # 实际应该计算到理想直线的距离
        lateral_deviation = 0.0  # 简化处理
        
        # 记录基线测量
        segment_id = f"baseline_{self.session_timestamp}_{self.stats['valid_baseline_segments']:03d}"
        
        self.baseline_csv_writer.writerow([
            segment_id, self.straight_line_start_time, end_timestamp, self.session_timestamp,
            start_pos['x'], start_pos['y'], end_position['x'], end_position['y'],
            distance_m, duration_s, avg_velocity,
            velocity_std, 0.0,  # direction_variance placeholder
            theoretical_error_m, measured_drift,
            lateral_deviation, len(self.baseline_samples)
        ])
        self.baseline_csv_file.flush()
        
        self.get_logger().info(
            f"基线段分析完成: 距离={distance_m:.1f}m, 时长={duration_s:.1f}s, "
            f"平均速度={avg_velocity:.3f}m/s, 理论误差={theoretical_error_m:.6f}m, "
            f"测量漂移={measured_drift:.6f}m"
        )
        
        self.stats['baseline_measurements'] += 1

    def analyze_motion_pattern(self):
        """定期分析运动模式"""
        if self.stats['total_samples'] % 100 == 0 and self.stats['total_samples'] > 0:
            self.get_logger().info(
                f"运动分析统计: 总样本={self.stats['total_samples']}, "
                f"直线检测={self.stats['straight_line_detections']}, "
                f"有效基线段={self.stats['valid_baseline_segments']}, "
                f"基线测量={self.stats['baseline_measurements']}"
            )

    def destroy_node(self):
        """清理关闭"""
        # 如果正在直线行驶中，结束当前段
        if self.in_straight_line and self.position_history:
            current_pos = self.position_history[-1]
            self.end_straight_line_segment(current_pos['timestamp'], current_pos)
        
        if hasattr(self, 'csv_file') and self.csv_file:
            self.csv_file.close()
        if hasattr(self, 'baseline_csv_file') and self.baseline_csv_file:
            self.baseline_csv_file.close()
        
        self.get_logger().info("智能跟踪误差测量完成")
        self.get_logger().info(f"总计检测到 {self.stats['valid_baseline_segments']} 个有效基线段")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', default='config.yaml', help='配置文件路径')
    parsed_args = parser.parse_args()
    
    measurer = IntelligentTrackingErrorMeasurer(parsed_args.config)
    
    try:
        rclpy.spin(measurer)
    except KeyboardInterrupt:
        measurer.get_logger().info("智能跟踪误差测量停止")
    finally:
        measurer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()