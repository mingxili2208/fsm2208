#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from collections import deque
import csv
import os
import datetime
import yaml
import threading

# Import required message types
from sensor_msgs.msg import TimeReference, PointCloud2
from geometry_msgs.msg import PoseWithCovarianceStamped

class LatencyLogRecorderNode(Node):
    """
    A1实验：FSM Sandbox时效性验证节点 (修正版)
    - 从config.yaml加载配置
    - 测量完整的R2V信息管道延迟 (T1→T4)
    - 为实验B提供ΔT_R2V数据
    """
    
    def __init__(self, config_file="config.yaml"):
        super().__init__('latency_log_recorder_node')

        # 加载配置
        self.load_config(config_file)
        
        # 数据结构
        self.t1_t2_map = {}  # {t1_ns: t2_ns}
        self.t3_to_t1_map = {}  # {t3_ns: t1_ns}
        
        # 统计计数器
        self.total_t1_t2_received = 0
        self.total_lidar_received = 0
        self.total_ndt_received = 0
        self.successful_correlations = 0
        self.failed_t1_t3_matches = 0
        self.failed_t3_t4_matches = 0
        
        # 创建订阅器
        self.create_subscriptions()
        
        # 设置CSV日志
        self.setup_csv_logging()
        
        # 定时器
        self.stats_timer = self.create_timer(
            self.stats_report_interval, self.report_statistics)
        self.cleanup_timer = self.create_timer(30.0, self.cleanup_old_entries)
        
        self.get_logger().info("A1时效性验证节点启动")
        self.get_logger().info(f"配置文件: {config_file}")
        self.get_logger().info(f"关联窗口: {self.correlation_window_ns/1e6:.1f}ms")

    def load_config(self, config_file):
        """从config.yaml加载配置"""
        try:
            with open(config_file, 'r') as f:
                config = yaml.safe_load(f)
            
            # 提取A1实验配置
            a1_config = config['experiment_a1']
            
            # 话题配置
            self.timing_sync_topic = a1_config['topics']['timing_sync']
            self.lidar_topic = a1_config['topics']['lidar_input']
            self.ndt_topic = a1_config['topics']['ndt_output']
            
            # 参数配置
            params = a1_config['parameters']
            self.correlation_window_ns = params['correlation_window_ns']
            self.cleanup_threshold_ns = params['cleanup_threshold_ns']
            self.stats_report_interval = params['stats_report_interval_s']
            
            self.get_logger().info("配置加载成功:")
            self.get_logger().info(f"  时序同步话题: {self.timing_sync_topic}")
            self.get_logger().info(f"  LiDAR话题: {self.lidar_topic}")
            self.get_logger().info(f"  NDT输出话题: {self.ndt_topic}")
            
        except Exception as e:
            self.get_logger().error(f"配置加载失败: {e}")
            self.get_logger().info("使用默认配置")
            self.use_default_config()

    def use_default_config(self):
        """使用默认配置"""
        self.timing_sync_topic = "/fsm_sandbox/timing/t1_t2"
        self.lidar_topic = "/carla/follow_adtruck/carla_pointcloud"
        self.ndt_topic = "/localization/pose_estimator/pose"
        self.correlation_window_ns = 50_000_000
        self.cleanup_threshold_ns = 60_000_000_000
        self.stats_report_interval = 5.0

    def create_subscriptions(self):
        """创建订阅器"""
        self.t1_t2_sub = self.create_subscription(
            TimeReference, self.timing_sync_topic, self.t1_t2_callback, 10)
        
        self.lidar_sub = self.create_subscription(
            PointCloud2, self.lidar_topic, self.lidar_callback, 10)
            
        self.ndt_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, self.ndt_topic, self.ndt_pose_callback, 10)

    def setup_csv_logging(self):
        """设置CSV日志"""
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # 确保data目录存在
        data_dir = "data"
        os.makedirs(data_dir, exist_ok=True)
        
        # 使用统一的文件命名：为实验B提供明确的数据源
        self.csv_filename = os.path.join(data_dir, f'r2v_timing_data_{timestamp}.csv')
        self.csv_file = open(self.csv_filename, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        
        # CSV头部 - 为实验B提供清晰的数据结构
        self.csv_writer.writerow([
            'timestamp_iso', 'correlation_id', 'session_id',
            't1_ns', 't2_ns', 't3_ns', 't4_ns',
            'carla_sync_latency_ms', 'carla_render_latency_ms', 
            'ndt_processing_latency_ms', 'r2v_total_latency_ms',
            'sequence_valid'
        ])
        
        self.csv_file.flush()
        self.get_logger().info(f"A1数据将保存到: {os.path.abspath(self.csv_filename)}")

    def to_nanoseconds(self, stamp):
        """转换ROS时间戳为纳秒"""
        return stamp.sec * 1_000_000_000 + stamp.nanosec

    def t1_t2_callback(self, msg: TimeReference):
        """接收(T1, T2)对并存储"""
        t1_ns = self.to_nanoseconds(msg.header.stamp)
        t2_ns = self.to_nanoseconds(msg.time_ref)
        
        # 验证时间戳顺序
        if t2_ns <= t1_ns:
            self.get_logger().warn(f"无效T1-T2顺序: T2({t2_ns}) <= T1({t1_ns})")
            return
            
        self.t1_t2_map[t1_ns] = t2_ns
        self.total_t1_t2_received += 1

    def lidar_callback(self, msg: PointCloud2):
        """接收LiDAR数据并建立T3->T1映射"""
        t3_ns = self.to_nanoseconds(msg.header.stamp)
        self.total_lidar_received += 1
        
        if not self.t1_t2_map:
            return
        
        # 找到最接近的T1时间戳
        closest_t1 = min(self.t1_t2_map.keys(), key=lambda t1: abs(t1 - t3_ns))
        time_diff_ns = abs(closest_t1 - t3_ns)
        
        if time_diff_ns < self.correlation_window_ns:
            self.t3_to_t1_map[t3_ns] = closest_t1
        else:
            self.failed_t1_t3_matches += 1

    def ndt_pose_callback(self, msg: PoseWithCovarianceStamped):
        """接收NDT输出并完成时序链关联"""
        t4_ns = self.to_nanoseconds(self.get_clock().now().to_msg())
        t3_ns = self.to_nanoseconds(msg.header.stamp)
        self.total_ndt_received += 1
        
        # 检查T3是否在映射中
        if t3_ns not in self.t3_to_t1_map:
            self.failed_t3_t4_matches += 1
            return
        
        # 找到对应的T1
        t1_ns = self.t3_to_t1_map.pop(t3_ns)
        
        # 检查T1是否在T1-T2映射中
        if t1_ns not in self.t1_t2_map:
            self.failed_t3_t4_matches += 1
            return
        
        t2_ns = self.t1_t2_map.pop(t1_ns)
        
        # 验证时间戳序列
        sequence_valid = (t1_ns < t2_ns < t3_ns < t4_ns)
        
        # 计算延迟 - 为实验B提供明确的数据
        carla_sync_latency_ms = (t2_ns - t1_ns) / 1e6      # CARLA同步延迟
        carla_render_latency_ms = (t3_ns - t2_ns) / 1e6    # CARLA渲染延迟
        ndt_processing_latency_ms = (t4_ns - t3_ns) / 1e6  # NDT处理延迟
        r2v_total_latency_ms = (t4_ns - t1_ns) / 1e6       # 总R2V延迟 (这是实验B需要的ΔT_R2V)
        
        # 记录到CSV
        timestamp_iso = datetime.datetime.now().isoformat()
        correlation_id = f"A1_{self.successful_correlations:06d}"
        session_id = timestamp_iso[:8].replace('-', '')  # YYYYMMDD
        
        self.csv_writer.writerow([
            timestamp_iso, correlation_id, session_id,
            t1_ns, t2_ns, t3_ns, t4_ns,
            carla_sync_latency_ms, carla_render_latency_ms,
            ndt_processing_latency_ms, r2v_total_latency_ms,
            sequence_valid
        ])
        
        self.csv_file.flush()
        self.successful_correlations += 1
        
        self.get_logger().info(
            f"R2V延迟链记录: 总延迟={r2v_total_latency_ms:.2f}ms "
            f"(同步={carla_sync_latency_ms:.2f}ms, "
            f"渲染={carla_render_latency_ms:.2f}ms, "
            f"NDT={ndt_processing_latency_ms:.2f}ms)"
        )

    def cleanup_old_entries(self):
        """清理旧条目"""
        current_time_ns = self.to_nanoseconds(self.get_clock().now().to_msg())
        
        # 清理T1-T2映射
        old_t1_keys = [t1 for t1 in self.t1_t2_map.keys() 
                       if (current_time_ns - t1) > self.cleanup_threshold_ns]
        for t1 in old_t1_keys:
            del self.t1_t2_map[t1]
        
        # 清理T3-T1映射
        old_t3_keys = [t3 for t3 in self.t3_to_t1_map.keys() 
                       if (current_time_ns - t3) > self.cleanup_threshold_ns]
        for t3 in old_t3_keys:
            del self.t3_to_t1_map[t3]

    def report_statistics(self):
        """报告统计信息"""
        correlation_rate = (self.successful_correlations / max(self.total_ndt_received, 1)) * 100
        
        self.get_logger().info(
            f"A1统计: T1-T2={self.total_t1_t2_received}, "
            f"LiDAR={self.total_lidar_received}, "
            f"NDT={self.total_ndt_received}, "
            f"成功关联={self.successful_correlations} ({correlation_rate:.1f}%)"
        )

    def destroy_node(self):
        """清理关闭"""
        if hasattr(self, 'csv_file') and self.csv_file:
            self.csv_file.close()
            
        final_rate = (self.successful_correlations / max(self.total_ndt_received, 1)) * 100
        self.get_logger().info(f"A1实验完成: {self.successful_correlations} 次成功测量 ({final_rate:.1f}%)")
        self.get_logger().info(f"R2V延迟数据保存至: {self.csv_filename}")
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    # 支持命令行参数指定配置文件
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', default='config.yaml', help='配置文件路径')
    parsed_args = parser.parse_args()
    
    log_recorder_node = LatencyLogRecorderNode(parsed_args.config)
    
    try:
        rclpy.spin(log_recorder_node)
    except KeyboardInterrupt:
        log_recorder_node.get_logger().info("A1数据收集停止")
    finally:
        log_recorder_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()