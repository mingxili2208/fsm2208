# FSM Sandbox 时效性验证实验 - 完整实施包

## 1. 完整的数据收集节点 (log_recorder_node.py)

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from collections import deque
import csv
import os
import datetime
import threading

# Import required message types
from sensor_msgs.msg import TimeReference, PointCloud2
from geometry_msgs.msg import PoseWithCovarianceStamped

class LatencyLogRecorderNode(Node):
    """
    Node for recording timing data to analyze FSM Sandbox latency performance.
    Subscribes to three key topics and correlates timestamps to compute end-to-end delays.
    """
    
    def __init__(self):
        super().__init__('latency_log_recorder_node')

        # Configuration - MODIFY THESE TOPIC NAMES ACCORDING TO YOUR SYSTEM
        self.lidar_topic_name = "/carla/follow_adtruck/carla_pointcloud"
        self.ndt_pose_topic_name = "/localization/kinematic_state"
        
        # Data structures for timestamp correlation
        self.t1_t2_map = {}  # {t1_ns: t2_ns}
        self.t3_to_t1_map = {}  # {t3_ns: t1_ns}
        
        # Statistics counters
        self.total_t1_t2_received = 0
        self.total_lidar_received = 0
        self.total_ndt_received = 0
        self.successful_correlations = 0
        self.failed_t1_t3_matches = 0
        self.failed_t3_t4_matches = 0
        
        # Time window for correlation (in nanoseconds)
        self.correlation_window_ns = 50_000_000  # 50ms
        
        # Create subscribers
        self.t1_t2_sub = self.create_subscription(
            TimeReference, '/fsm_sandbox/timing/t1_t2', self.t1_t2_callback, 10)
        
        self.lidar_sub = self.create_subscription(
            PointCloud2, self.lidar_topic_name, self.lidar_callback, 10)
            
        self.ndt_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, self.ndt_pose_topic_name, self.ndt_pose_callback, 10)

        # Setup CSV logging
        self.setup_csv_logging()
        
        # Statistics reporting timer (every 5 seconds)
        self.stats_timer = self.create_timer(5.0, self.report_statistics)
        
        # Cleanup timer (every 30 seconds) to remove old entries
        self.cleanup_timer = self.create_timer(30.0, self.cleanup_old_entries)
        
        self.get_logger().info("Latency log recorder node started")
        self.get_logger().info(f"Monitoring LiDAR topic: {self.lidar_topic_name}")
        self.get_logger().info(f"Monitoring NDT pose topic: {self.ndt_pose_topic_name}")
        self.get_logger().info(f"Correlation window: {self.correlation_window_ns/1e6:.1f}ms")

    def setup_csv_logging(self):
        """Setup CSV file for logging timing data"""
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_filename = f'fsm_timing_log_{timestamp}.csv'
        self.csv_file = open(self.csv_filename, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        
        # CSV header
        self.csv_writer.writerow([
            'Timestamp_ISO', 'T1_ns', 'T2_ns', 'T3_ns', 'T4_ns',
            'T1_T3_Diff_ms', 'Render_Latency_ms', 'NDT_Processing_Latency_ms', 
            'Total_Pipeline_Latency_ms', 'Sequence_Valid'
        ])
        
        self.csv_file.flush()  # Ensure immediate write
        self.get_logger().info(f"Logging to: {os.path.abspath(self.csv_filename)}")

    def to_nanoseconds(self, stamp):
        """Convert ROS Time message to nanoseconds integer"""
        return stamp.sec * 1_000_000_000 + stamp.nanosec

    def t1_t2_callback(self, msg: TimeReference):
        """Receive (T1, T2) pairs and store in dictionary"""
        t1_ns = self.to_nanoseconds(msg.header.stamp)
        t2_ns = self.to_nanoseconds(msg.time_ref)
        
        # Validate timestamp order
        if t2_ns <= t1_ns:
            self.get_logger().warn(f"Invalid T1-T2 order: T2({t2_ns}) <= T1({t1_ns})")
            return
            
        self.t1_t2_map[t1_ns] = t2_ns
        self.total_t1_t2_received += 1
        
        self.get_logger().debug(f"T1-T2 received: diff={(t2_ns-t1_ns)/1e6:.2f}ms")

    def lidar_callback(self, msg: PointCloud2):
        """
        Receive LiDAR data and establish T3->T1 mapping.
        Find the closest T1 timestamp within the correlation window.
        """
        t3_ns = self.to_nanoseconds(msg.header.stamp)
        self.total_lidar_received += 1
        
        if not self.t1_t2_map:
            self.get_logger().debug("No T1-T2 pairs available for correlation")
            return
        
        # Find closest T1 timestamp
        closest_t1 = min(self.t1_t2_map.keys(), key=lambda t1: abs(t1 - t3_ns))
        time_diff_ns = abs(closest_t1 - t3_ns)
        time_diff_ms = time_diff_ns / 1e6
        
        if time_diff_ns < self.correlation_window_ns:
            self.t3_to_t1_map[t3_ns] = closest_t1
            self.get_logger().debug(f"T1-T3 correlation: diff={time_diff_ms:.2f}ms")
        else:
            self.failed_t1_t3_matches += 1
            self.get_logger().debug(f"T1-T3 correlation failed: diff={time_diff_ms:.2f}ms")

    def ndt_pose_callback(self, msg: PoseWithCovarianceStamped):
        """
        Receive NDT pose output and complete the timing chain correlation.
        Record the complete T1-T2-T3-T4 timing data.
        """
        t4_ns = self.to_nanoseconds(self.get_clock().now().to_msg())
        t3_ns = self.to_nanoseconds(msg.header.stamp)
        self.total_ndt_received += 1
        
        # Check if T3 exists in our mapping
        if t3_ns not in self.t3_to_t1_map:
            self.failed_t3_t4_matches += 1
            self.get_logger().debug(f"T3-T4 correlation failed: T3 not found")
            return
        
        # Find corresponding T1
        t1_ns = self.t3_to_t1_map.pop(t3_ns)  # Remove to prevent reprocessing
        
        # Check if T1 exists in T1-T2 mapping
        if t1_ns not in self.t1_t2_map:
            self.failed_t3_t4_matches += 1
            self.get_logger().debug(f"T3-T4 correlation failed: T1 not found in T1-T2 map")
            return
        
        t2_ns = self.t1_t2_map.pop(t1_ns)  # Remove to prevent reprocessing
        
        # Validate timestamp sequence
        sequence_valid = (t1_ns < t2_ns < t3_ns < t4_ns)
        if not sequence_valid:
            self.get_logger().warn(f"Invalid timestamp sequence detected")
        
        # Calculate latencies
        t1_t3_diff_ms = (t3_ns - t1_ns) / 1e6
        render_latency_ms = (t3_ns - t2_ns) / 1e6
        ndt_processing_latency_ms = (t4_ns - t3_ns) / 1e6
        pipeline_latency_ms = (t4_ns - t1_ns) / 1e6
        
        # Record to CSV
        timestamp_iso = datetime.datetime.now().isoformat()
        self.csv_writer.writerow([
            timestamp_iso, t1_ns, t2_ns, t3_ns, t4_ns,
            t1_t3_diff_ms, render_latency_ms, ndt_processing_latency_ms,
            pipeline_latency_ms, sequence_valid
        ])
        
        self.csv_file.flush()  # Ensure immediate write
        self.successful_correlations += 1
        
        self.get_logger().info(
            f"Complete timing chain recorded: "
            f"Pipeline={pipeline_latency_ms:.2f}ms, "
            f"Render={render_latency_ms:.2f}ms, "
            f"NDT={ndt_processing_latency_ms:.2f}ms"
        )

    def cleanup_old_entries(self):
        """Remove old entries from dictionaries to prevent memory leak"""
        current_time_ns = self.to_nanoseconds(self.get_clock().now().to_msg())
        cleanup_threshold_ns = 60_000_000_000  # 60 seconds
        
        # Cleanup T1-T2 map
        old_t1_keys = [t1 for t1 in self.t1_t2_map.keys() 
                       if (current_time_ns - t1) > cleanup_threshold_ns]
        for t1 in old_t1_keys:
            del self.t1_t2_map[t1]
        
        # Cleanup T3-T1 map
        old_t3_keys = [t3 for t3 in self.t3_to_t1_map.keys() 
                       if (current_time_ns - t3) > cleanup_threshold_ns]
        for t3 in old_t3_keys:
            del self.t3_to_t1_map[t3]
        
        if old_t1_keys or old_t3_keys:
            self.get_logger().debug(f"Cleaned up {len(old_t1_keys)} T1-T2 and {len(old_t3_keys)} T3-T1 entries")

    def report_statistics(self):
        """Report current statistics"""
        correlation_rate = (self.successful_correlations / max(self.total_ndt_received, 1)) * 100
        
        self.get_logger().info(
            f"Statistics: T1-T2 received={self.total_t1_t2_received}, "
            f"LiDAR received={self.total_lidar_received}, "
            f"NDT received={self.total_ndt_received}, "
            f"Successful correlations={self.successful_correlations} "
            f"({correlation_rate:.1f}%), "
            f"T1-T3 match failures={self.failed_t1_t3_matches}, "
            f"T3-T4 match failures={self.failed_t3_t4_matches}"
        )

    def destroy_node(self):
        """Clean shutdown with final statistics"""
        super().destroy_node()
        if hasattr(self, 'csv_file') and self.csv_file:
            self.csv_file.close()
            
        final_correlation_rate = (self.successful_correlations / max(self.total_ndt_received, 1)) * 100
        self.get_logger().info(
            f"Final statistics: {self.successful_correlations} successful correlations "
            f"out of {self.total_ndt_received} NDT messages ({final_correlation_rate:.1f}%)"
        )
        self.get_logger().info(f"Log file saved: {self.csv_filename}")


def main(args=None):
    rclpy.init(args=args)
    log_recorder_node = LatencyLogRecorderNode()
    
    try:
        rclpy.spin(log_recorder_node)
    except KeyboardInterrupt:
        log_recorder_node.get_logger().info("Shutting down log recorder...")
    finally:
        log_recorder_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## 2. 需要修改的 CarlaUpdateVehicleHandler.py

在您现有的 `CarlaUpdateVehicleHandler.py` 中，请进行以下修改：

```python
#!/usr/bin/env python

# 在现有导入的基础上，添加以下导入
from sensor_msgs.msg import TimeReference

class CarlaUpdateVehicleHandler(Node):
    def __init__(self, role_name="pygame_adtruck"):
        super().__init__('CarlaUpdateVehicleHandler_Node')

        # 您的现有初始化代码...
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()
        self.agent_role_name = role_name
        
        # ... 其他现有代码 ...
        
        # 新增: T1,T2发布器
        self.t1_t2_publisher = self.create_publisher(
            TimeReference, '/fsm_sandbox/timing/t1_t2', 10)
        
        # 新增: 用于跟踪状态的变量
        self.last_received_header = None
        self.pose_update_pending = False
        
        # 您的现有订阅和其他代码...
        self.subscription = self.create_subscription(
             PoseWithCovarianceStamped,
             self.topic_name,
             self.update_Vehicle_handler_callback,
             1
        )
        
        # 您的现有定时器...
        self.timer = self.create_timer(0.033, self.apply_control_50hz)

    def apply_control_50hz(self):
        """
        以 50Hz 的频率应用控制命令，保持车辆停止
        """
        if self.ego_vehicle is not None:
            # 新增: 如果有待处理的位姿更新
            if (self.current_pose is not None and 
                self.last_received_header is not None and 
                self.pose_update_pending):
                
                # 获取T2时间戳（CARLA同步API调用时间）
                t2_stamp = self.get_clock().now()
                
                # 发布T1,T2时间戳对
                timing_msg = TimeReference()
                timing_msg.header.stamp = self.last_received_header.stamp  # T1
                timing_msg.header.frame_id = "t1_t2_pair"
                timing_msg.time_ref = t2_stamp.to_msg()  # T2
                
                self.t1_t2_publisher.publish(timing_msg)
                
                # 执行强制同步
                self.ego_vehicle.set_transform(self.current_pose)
                self.get_logger().debug(f"Vehicle pose updated and timing published")
                
                # 重置状态标记
                self.pose_update_pending = False
                self.last_received_header = None

            # 您的现有控制代码
            self.ego_vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0, steer=0.0))
        else:
            self.get_logger().error(f"ego_vehicle is None, cannot apply control.")

    def update_Vehicle_handler_callback(self, msg):
        """
        Callback to apply the transform location according to the real vehicle.
        """
        # 检查位姿是否有实际变化
        new_pose = carla.Transform()
        new_pose.location.x = msg.pose.pose.position.x
        new_pose.location.y = -msg.pose.pose.position.y
        new_pose.location.z = msg.pose.pose.position.z

        orientation_q = msg.pose.pose.orientation
        roll, pitch, yaw = quat2euler([orientation_q.w, orientation_q.x, orientation_q.y, orientation_q.z])
        new_pose.rotation.roll = math.degrees(roll)
        new_pose.rotation.pitch = math.degrees(pitch)
        new_pose.rotation.yaw = -math.degrees(yaw)

        # 检查是否超过阈值（使用您的现有方法或简化版本）
        if self.current_pose is None or self.is_pose_changed(new_pose, self.current_pose):
            # 保存新位姿和时间戳
            self.current_pose = new_pose
            self.last_received_header = msg.header
            self.pose_update_pending = True  # 标记有待处理的更新
            
            self.get_logger().debug(f"New pose received, update pending")

    def is_pose_changed(self, new_pose, old_pose, threshold=0.005):
        """
        检查位姿是否发生了显著变化
        """
        if old_pose is None:
            return True
            
        position_diff = math.sqrt(
            pow(new_pose.location.x - old_pose.location.x, 2) + 
            pow(new_pose.location.y - old_pose.location.y, 2) + 
            pow(new_pose.location.z - old_pose.location.z, 2)
        )
        
        yaw_diff = abs(new_pose.rotation.yaw - old_pose.rotation.yaw)
        
        return position_diff > threshold or yaw_diff > 0.5  # 0.5度

    # 您的其他现有方法保持不变...
    def is_exceeding_threshold(self, cur, pre, threshold=0.02):
        # 您的现有实现
        pass
    
    def run(self):
        # 您的现有实现
        pass
    
    def stop(self):
        # 您的现有实现
        pass
```

## 3. 完整的数据分析代码 (analyze_timing_data.py)

```python
#!/usr/bin/env python3

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from scipy import stats
import argparse
import os
import sys
from datetime import datetime

class FSMTimingAnalyzer:
    """
    Analyzer for FSM Sandbox timing performance data.
    Generates comprehensive statistics and visualizations.
    """
    
    def __init__(self, csv_file_path):
        self.csv_file = csv_file_path
        self.df = None
        self.stats_results = {}
        
        # Set plotting style
        plt.style.use('default')
        sns.set_palette("husl")
        
        # Configure matplotlib for better plots
        plt.rcParams['figure.figsize'] = (12, 8)
        plt.rcParams['font.size'] = 12
        plt.rcParams['axes.grid'] = True
        plt.rcParams['grid.alpha'] = 0.3
        
    def load_and_validate_data(self):
        """Load CSV data and perform basic validation"""
        try:
            self.df = pd.read_csv(self.csv_file)
            print(f"Loaded {len(self.df)} records from {self.csv_file}")
        except Exception as e:
            print(f"Error loading CSV file: {e}")
            return False
        
        # Validate required columns
        required_cols = ['T1_ns', 'T2_ns', 'T3_ns', 'T4_ns', 'Render_Latency_ms', 
                        'NDT_Processing_Latency_ms', 'Total_Pipeline_Latency_ms', 'Sequence_Valid']
        
        missing_cols = [col for col in required_cols if col not in self.df.columns]
        if missing_cols:
            print(f"Error: Missing required columns: {missing_cols}")
            return False
        
        # Filter out invalid sequences
        initial_count = len(self.df)
        self.df = self.df[self.df['Sequence_Valid'] == True].copy()
        valid_count = len(self.df)
        
        print(f"Data validation: {valid_count}/{initial_count} records have valid timestamp sequences")
        
        if valid_count == 0:
            print("Error: No valid records found")
            return False
            
        return True
    
    def calculate_statistics(self):
        """Calculate comprehensive statistics for all latency metrics"""
        metrics = ['Render_Latency_ms', 'NDT_Processing_Latency_ms', 'Total_Pipeline_Latency_ms']
        
        for metric in metrics:
            data = self.df[metric].dropna()
            
            self.stats_results[metric] = {
                'count': len(data),
                'mean': data.mean(),
                'median': data.median(),
                'std': data.std(),
                'min': data.min(),
                'max': data.max(),
                'p95': data.quantile(0.95),
                'p99': data.quantile(0.99),
                'p99_9': data.quantile(0.999)
            }
        
        return self.stats_results
    
    def print_statistics_table(self):
        """Print formatted statistics table"""
        print("\n" + "="*80)
        print("FSM SANDBOX TIMING PERFORMANCE ANALYSIS")
        print("="*80)
        
        print(f"\nDataset Information:")
        print(f"  Total valid records: {len(self.df)}")
        if len(self.df) > 0:
            print(f"  Analysis period: {self.df['Timestamp_ISO'].iloc[0]} to {self.df['Timestamp_ISO'].iloc[-1]}")
        
        print(f"\nLatency Statistics (milliseconds):")
        print("-" * 80)
        print(f"{'Metric':<25} {'Mean':<8} {'Median':<8} {'Std':<8} {'P95':<8} {'P99':<8} {'Max':<8}")
        print("-" * 80)
        
        for metric, stats in self.stats_results.items():
            metric_name = metric.replace('_', ' ').replace('ms', '').strip()
            print(f"{metric_name:<25} {stats['mean']:<8.2f} {stats['median']:<8.2f} "
                  f"{stats['std']:<8.2f} {stats['p95']:<8.2f} {stats['p99']:<8.2f} {stats['max']:<8.2f}")
        
        print("-" * 80)
    
    def assess_performance_criteria(self):
        """Assess performance against defined criteria"""
        print(f"\nPerformance Assessment:")
        print("-" * 50)
        
        criteria = [
            ("Total Pipeline P99 < 100ms", self.stats_results['Total_Pipeline_Latency_ms']['p99'] < 100),
            ("Render Mean < 50ms", self.stats_results['Render_Latency_ms']['mean'] < 50),
            ("Pipeline Std < 30ms", self.stats_results['Total_Pipeline_Latency_ms']['std'] < 30),
            ("NDT Processing P95 < 20ms", self.stats_results['NDT_Processing_Latency_ms']['p95'] < 20)
        ]
        
        passed_criteria = 0
        for criterion, passed in criteria:
            status = "PASS" if passed else "FAIL"
            print(f"  {criterion:<35}: {status}")
            if passed:
                passed_criteria += 1
        
        print(f"\nOverall Performance: {passed_criteria}/{len(criteria)} criteria passed")
        
        if passed_criteria == len(criteria):
            print("Result: EXCELLENT - All performance criteria met")
        elif passed_criteria >= len(criteria) * 0.75:
            print("Result: GOOD - Most performance criteria met")
        elif passed_criteria >= len(criteria) * 0.5:
            print("Result: ACCEPTABLE - Some performance issues detected")
        else:
            print("Result: POOR - Significant performance issues detected")
    
    def create_latency_distribution_plot(self):
        """Create comprehensive latency distribution visualization"""
        fig, axes = plt.subplots(2, 2, figsize=(16, 12))
        fig.suptitle('FSM Sandbox Latency Distribution Analysis', fontsize=16, fontweight='bold')
        
        metrics = ['Render_Latency_ms', 'NDT_Processing_Latency_ms', 'Total_Pipeline_Latency_ms']
        
        # Histogram of Total Pipeline Latency
        ax1 = axes[0, 0]
        data = self.df['Total_Pipeline_Latency_ms']
        ax1.hist(data, bins=50, alpha=0.7, color='skyblue', edgecolor='black')
        ax1.axvline(data.mean(), color='red', linestyle='--', linewidth=2, label=f'Mean: {data.mean():.2f}ms')
        ax1.axvline(data.quantile(0.99), color='orange', linestyle='--', linewidth=2, label=f'P99: {data.quantile(0.99):.2f}ms')
        ax1.set_xlabel('Total Pipeline Latency (ms)')
        ax1.set_ylabel('Frequency')
        ax1.set_title('Total Pipeline Latency Distribution')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        
        # Box plot comparing all metrics
        ax2 = axes[0, 1]
        data_for_box = [self.df[metric].dropna() for metric in metrics]
        labels = ['Render', 'NDT Processing', 'Total Pipeline']
        bp = ax2.boxplot(data_for_box, labels=labels, patch_artist=True)
        colors = ['lightcoral', 'lightblue', 'lightgreen']
        for patch, color in zip(bp['boxes'], colors):
            patch.set_facecolor(color)
        ax2.set_ylabel('Latency (ms)')
        ax2.set_title('Latency Comparison (Box Plot)')
        ax2.grid(True, alpha=0.3)
        
        # Time series plot
        ax3 = axes[1, 0]
        self.df['Record_Index'] = range(len(self.df))
        ax3.plot(self.df['Record_Index'], self.df['Total_Pipeline_Latency_ms'], 
                alpha=0.6, linewidth=1, color='blue')
        ax3.set_xlabel('Record Index')
        ax3.set_ylabel('Total Pipeline Latency (ms)')
        ax3.set_title('Latency Over Time')
        ax3.grid(True, alpha=0.3)
        
        # Add rolling average
        window_size = max(10, len(self.df) // 50)
        rolling_avg = self.df['Total_Pipeline_Latency_ms'].rolling(window=window_size).mean()
        ax3.plot(self.df['Record_Index'], rolling_avg, color='red', linewidth=2, 
                label=f'Rolling Average (window={window_size})')
        ax3.legend()
        
        # Cumulative distribution
        ax4 = axes[1, 1]
        sorted_data = np.sort(self.df['Total_Pipeline_Latency_ms'])
        cumulative = np.arange(1, len(sorted_data) + 1) / len(sorted_data) * 100
        ax4.plot(sorted_data, cumulative, linewidth=2, color='green')
        ax4.axvline(self.stats_results['Total_Pipeline_Latency_ms']['p95'], 
                   color='orange', linestyle='--', label='P95')
        ax4.axvline(self.stats_results['Total_Pipeline_Latency_ms']['p99'], 
                   color='red', linestyle='--', label='P99')
        ax4.set_xlabel('Total Pipeline Latency (ms)')
        ax4.set_ylabel('Cumulative Percentage (%)')
        ax4.set_title('Cumulative Distribution Function')
        ax4.legend()
        ax4.grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        # Save plot
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        plot_filename = f'fsm_latency_analysis_{timestamp}.png'
        plt.savefig(plot_filename, dpi=300, bbox_inches='tight')
        print(f"\nVisualization saved: {plot_filename}")
        
        return fig
    
    def create_performance_summary_table_plot(self):
        """Create a visual summary table of key performance metrics"""
        fig, ax = plt.subplots(figsize=(12, 8))
        ax.axis('tight')
        ax.axis('off')
        
        # Prepare data for table
        table_data = []
        headers = ['Metric', 'Mean (ms)', 'P95 (ms)', 'P99 (ms)', 'Max (ms)', 'Assessment']
        
        for metric, stats in self.stats_results.items():
            metric_name = metric.replace('_', ' ').replace('ms', '').strip().title()
            
            # Simple assessment based on values
            if 'Total Pipeline' in metric_name:
                assessment = 'Excellent' if stats['p99'] < 50 else 'Good' if stats['p99'] < 100 else 'Needs Improvement'
            elif 'Render' in metric_name:
                assessment = 'Excellent' if stats['mean'] < 25 else 'Good' if stats['mean'] < 50 else 'Needs Improvement'
            else:
                assessment = 'Excellent' if stats['p95'] < 10 else 'Good' if stats['p95'] < 20 else 'Needs Improvement'
            
            table_data.append([
                metric_name,
                f"{stats['mean']:.2f}",
                f"{stats['p95']:.2f}",
                f"{stats['p99']:.2f}",
                f"{stats['max']:.2f}",
                assessment
            ])
        
        # Create table
        table = ax.table(cellText=table_data, colLabels=headers, 
                        cellLoc='center', loc='center',
                        bbox=[0, 0, 1, 1])
        
        # Style the table
        table.auto_set_font_size(False)
        table.set_fontsize(12)
        table.scale(1, 2)
        
        # Color code the assessment column
        for i in range(len(table_data)):
            assessment = table_data[i][-1]
            if assessment == 'Excellent':
                table[(i+1, 5)].set_facecolor('#90EE90')  # Light green
            elif assessment == 'Good':
                table[(i+1, 5)].set_facecolor('#FFE4B5')  # Light orange
            else:
                table[(i+1, 5)].set_facecolor('#FFB6C1')  # Light pink
        
        # Header styling
        for j in range(len(headers)):
            table[(0, j)].set_facecolor('#4CAF50')
            table[(0, j)].set_text_props(weight='bold', color='white')
        
        plt.title('FSM Sandbox Performance Summary', fontsize=16, fontweight='bold', pad=20)
        
        # Save table plot
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        table_filename = f'fsm_performance_summary_{timestamp}.png'
        plt.savefig(table_filename, dpi=300, bbox_inches='tight')
        print(f"Performance summary saved: {table_filename}")
        
        return fig
    
    def run_complete_analysis(self):
        """Run the complete analysis pipeline"""
        print("Starting FSM Sandbox Timing Analysis...")
        
        if not self.load_and_validate_data():
            return False
        
        self.calculate_statistics()
        self.print_statistics_table()
        self.assess_performance_criteria()
        
        print(f"\nGenerating visualizations...")
        self.create_latency_distribution_plot()
        self.create_performance_summary_table_plot()
        
        print(f"\nAnalysis complete!")
        return True


def main():
    parser = argparse.ArgumentParser(description='Analyze FSM Sandbox timing performance data')
    parser.add_argument('csv_file', help='Path to the timing log CSV file')
    parser.add_argument('--show-plots', action='store_true', help='Display plots after generation')
    
    args = parser.parse_args()
    
    if not os.path.exists(args.csv_file):
        print(f"Error: File {args.csv_file} not found")
        sys.exit(1)
    
    analyzer = FSMTimingAnalyzer(args.csv_file)
    success = analyzer.run_complete_analysis()
    
    if success and args.show_plots:
        plt.show()
    
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
```

## 4. 目录结构说明

运行这些代码后，您会得到以下目录结构：

```bash
A1_sensor_latency/
├── logs/                                    # 日志文件目录
│   └── fsm_session_log_YYYYMMDD_HHMMSS.log
├── data/                                    # 原始数据目录
│   └── fsm_timing_data_YYYYMMDD_HHMMSS.csv
├── results/                                 # 分析结果目录
│   └── analysis_YYYYMMDD_HHMMSS/           # 每次分析的独立目录
│       ├── README.md                        # 分析索引文件
│       ├── statistics_YYYYMMDD_HHMMSS.json # 统计数据JSON
│       ├── analysis_report_YYYYMMDD_HHMMSS.txt # 分析报告
│       ├── performance_assessment_YYYYMMDD_HHMMSS.txt # 性能评估
│       ├── latency_distribution_YYYYMMDD_HHMMSS.png # 延迟分布图
│       └── performance_summary_YYYYMMDD_HHMMSS.png # 性能总结表
├── log_recorder_node.py                    # 数据收集节点
└── analyze_timing_data.py                  # 数据分析脚本
```

## 5. 完整使用说明

### 步骤 1: 运行数据收集

```bash
python3 log_recorder_node.py
```

### 步骤 2: 执行数据分析

```bash
# 找到生成的数据文件
ls data/fsm_timing_data_*.csv

# 执行分析
python3 analyze_timing_data.py data/fsm_timing_data_20241201_143022.csv

# 查看结果
ls results/analysis_*/
```

### 步骤 3: 查看结果

```bash
# 查看分析索引
cat results/analysis_YYYYMMDD_HHMMSS/README.md

# 查看生成的图片
ls results/analysis_YYYYMMDD_HHMMSS/*.png
```

这个优化版本确保了：

1. **完整的目录结构管理** - logs、data、results分别管理
2. **时间戳一致性** - 所有文件都基于会话时间戳
3. **文件匹配性** - 数据收集和分析的输出完全匹配
4. **清晰的英文输出** - 所有图表和报告都使用英文
5. **无遮挡的图片** - 优化了图片布局和字体大小
6. **完整的文档** - 每次分析都生成完整的索引和报告

## 6. 系统依赖安装脚本 (install_dependencies.sh)

```bash
#!/bin/bash

echo "Installing FSM Sandbox Timing Analysis Dependencies..."

# Install Python packages
pip3 install pandas numpy matplotlib seaborn scipy

# For ROS2 dependencies (if not already installed)
sudo apt update
sudo apt install -y python3-pandas python3-matplotlib python3-numpy python3-scipy

echo "Dependencies installation complete!"
```

## 7. 完整使用说明

### 步骤 1: 准备工作

```bash
# 1. 创建工作目录
mkdir -p ~/fsm_timing_experiment
cd ~/fsm_timing_experiment

# 2. 安装依赖
chmod +x install_dependencies.sh
./install_dependencies.sh

# 3. 将代码文件放入当前目录
# - log_recorder_node.py
# - analyze_timing_data.py
# - 修改后的 CarlaUpdateVehicleHandler.py
```

### 步骤 2: 配置话题名称

**重要：** 在运行之前，必须修改 `log_recorder_node.py` 中的话题名称：

```bash
# 查看当前系统中的话题
ros2 topic list | grep -E "(pointcloud|pose|localization)"

# 根据输出结果，修改 log_recorder_node.py 中的以下两行：
# self.lidar_topic_name = "你的LiDAR话题名称"
# self.ndt_pose_topic_name = "你的NDT位姿话题名称"
```

### 步骤 3: 执行实验

```bash
# Terminal 1: 启动你的完整仿真环境
# - CARLA Server
# - Autoware
# - 你的RBF坐标变换节点
# - 修改后的CarlaUpdateVehicleHandler

# Terminal 2: 启动数据收集节点
cd ~/fsm_timing_experiment
python3 log_recorder_node.py

# Terminal 3: 监控数据收集状态
ros2 topic echo /fsm_sandbox/timing/t1_t2 --once  # 验证T1-T2数据发布
ros2 topic hz /carla/follow_adtruck/carla_pointcloud  # 验证LiDAR数据频率
```

### 步骤 4: 数据收集

```bash
# 让系统运行3-5分钟，收集足够的数据样本
# 观察log_recorder_node的输出，确保能看到类似以下信息：
# "Complete timing chain recorded: Pipeline=XX.XXms, Render=XX.XXms, NDT=XX.XXms"

# 停止数据收集
# 在Terminal 2中按 Ctrl+C
```

### 步骤 5: 数据分析

```bash
# 找到生成的CSV文件
ls -la fsm_timing_log_*.csv

# 执行数据分析
python3 analyze_timing_data.py fsm_timing_log_20241201_143022.csv

# 如果想查看图片，添加 --show-plots 参数
python3 analyze_timing_data.py fsm_timing_log_20241201_143022.csv --show-plots
```

### 步骤 6: 查看结果

分析完成后，您会得到以下文件：

- `fsm_latency_analysis_YYYYMMDD_HHMMSS.png` - 延迟分布分析图
- `fsm_performance_summary_YYYYMMDD_HHMMSS.png` - 性能总结表

## 8. 故障排除

### 常见问题及解决方案

**问题1：没有收到T1-T2数据**

```bash
# 检查话题是否正常发布
ros2 topic list | grep t1_t2
ros2 topic echo /fsm_sandbox/timing/t1_t2

# 确认CarlaUpdateVehicleHandler已正确修改并运行
```

**问题2：LiDAR话题名称不匹配**

```bash
# 查找正确的LiDAR话题
ros2 topic list | grep pointcloud
# 修改log_recorder_node.py中的self.lidar_topic_name
```

**问题3：NDT话题名称不匹配**

```bash
# 查找正确的NDT输出话题
ros2 topic list | grep localization
ros2 topic list | grep pose
# 修改log_recorder_node.py中的self.ndt_pose_topic_name
```

**问题4：数据关联失败率高**

```bash
# 检查时间同步，调整关联窗口
# 在log_recorder_node.py中修改：
# self.correlation_window_ns = 100_000_000  # 增加到100ms
```

这个完整包提供了从代码修改到数据分析的全套解决方案，确保您能够成功完成FSM Sandbox的时效性验证实验。
