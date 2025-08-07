# 实验B：延迟特性及基线跟踪误差影响分析 - 完整实施方案

## 1. 实验目标与架构

### 1.1 核心目标

1. **性能充分性证明**: 精确测量系统端到端延迟特性，验证平台性能满足高保真ADS测试需求
2. **建立误差基线**: 量化延迟与物理跟踪误差的直接关联，建立诊断性归因的理论下限

### 1.2 延迟分解架构
基于您的系统架构（PDF图1），实现三分量延迟测量：

```
ΔT_total = ΔT_R2V + ΔT_planner + ΔT_V2R
```

- **ΔT_R2V**: Vive真值系统 → ADS感知输入的信息年龄
- **ΔT_planner**: ADS感知输入 → 规划输出的内部处理延迟  
- **ΔT_V2R**: 规划输出 → 物理执行的决策到执行延迟

## 2. 数据收集组件实现

### 2.1 延迟链路分析器 (latency_chain_analyzer.py)

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import csv
import os
import datetime
import json
import numpy as np
from collections import deque, defaultdict
import threading
import time

from geometry_msgs.msg import PoseWithCovarianceStamped
from autoware_auto_planning_msgs.msg import Trajectory
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header

class LatencyChainAnalyzer(Node):
    """
    Comprehensive latency chain analyzer for measuring R2V, Planner, and V2R delays
    """
    
    def __init__(self):
        super().__init__('latency_chain_analyzer')
        
        # Session setup
        self.session_timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        self.setup_directories()
        self.setup_logging()
        
        # Configuration
        self.correlation_window_ns = 200_000_000  # 200ms correlation window
        self.cleanup_threshold_ns = 60_000_000_000  # 60s cleanup threshold
        
        # Topic names - modify these to match your system
        self.ground_truth_topic = '/real_world/follow_adtruck/transformed_with_covariance'
        self.perception_topic = '/carla/follow_adtruck/carla_pointcloud'
        self.planning_input_topic = '/localization/kinematic_state'
        self.planning_output_topic = '/planning/trajectory'
        
        # Data buffers for correlation
        self.ground_truth_buffer = deque(maxlen=1000)
        self.perception_buffer = deque(maxlen=500)
        self.planning_input_buffer = deque(maxlen=200)
        self.planning_output_buffer = deque(maxlen=100)
        
        # Statistics
        self.stats = {
            'ground_truth_count': 0,
            'perception_count': 0,
            'planning_input_count': 0,
            'planning_output_count': 0,
            'successful_correlations': 0,
            'failed_correlations': 0
        }
        
        # Setup subscribers
        self.create_subscriptions()
        
        # Periodic tasks
        self.stats_timer = self.create_timer(10.0, self.report_statistics)
        self.cleanup_timer = self.create_timer(30.0, self.cleanup_old_entries)
        
        self.get_logger().info(f"Latency Chain Analyzer started - Session: {self.session_timestamp}")
        self.get_logger().info("Monitoring complete R2V -> Planner -> V2R delay chain")

    def setup_directories(self):
        """Setup output directories"""
        self.base_dir = os.path.dirname(os.path.abspath(__file__))
        self.data_dir = os.path.join(self.base_dir, 'data')
        self.logs_dir = os.path.join(self.base_dir, 'logs')
        
        for directory in [self.data_dir, self.logs_dir]:
            os.makedirs(directory, exist_ok=True)

    def setup_logging(self):
        """Setup CSV logging for latency chain data"""
        self.csv_filename = os.path.join(self.data_dir, f'latency_chain_{self.session_timestamp}.csv')
        self.csv_file = open(self.csv_filename, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        
        # CSV header
        self.csv_writer.writerow([
            'correlation_id', 'timestamp_ns', 'session_id',
            't_ground_truth_ns', 't_perception_ns', 't_planning_input_ns', 't_planning_output_ns',
            'r2v_delay_ms', 'planner_delay_ms', 'total_r2v_planner_ms',
            'valid_correlation', 'correlation_confidence'
        ])
        self.csv_file.flush()

    def create_subscriptions(self):
        """Create all necessary subscriptions"""
        self.ground_truth_sub = self.create_subscription(
            PoseWithCovarianceStamped, self.ground_truth_topic,
            self.ground_truth_callback, 10)
        
        self.perception_sub = self.create_subscription(
            PointCloud2, self.perception_topic,
            self.perception_callback, 10)
        
        self.planning_input_sub = self.create_subscription(
            PoseWithCovarianceStamped, self.planning_input_topic,
            self.planning_input_callback, 10)
        
        self.planning_output_sub = self.create_subscription(
            Trajectory, self.planning_output_topic,
            self.planning_output_callback, 10)

    def to_nanoseconds(self, stamp):
        """Convert ROS timestamp to nanoseconds"""
        return stamp.sec * 1_000_000_000 + stamp.nanosec

    def ground_truth_callback(self, msg: PoseWithCovarianceStamped):
        """Store ground truth timestamps for R2V delay calculation"""
        timestamp_ns = self.to_nanoseconds(msg.header.stamp)
        self.ground_truth_buffer.append({
            'timestamp_ns': timestamp_ns,
            'pose': (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
        })
        self.stats['ground_truth_count'] += 1

    def perception_callback(self, msg: PointCloud2):
        """Store perception timestamps"""
        timestamp_ns = self.to_nanoseconds(msg.header.stamp)
        self.perception_buffer.append({
            'timestamp_ns': timestamp_ns,
            'points_count': msg.width * msg.height
        })
        self.stats['perception_count'] += 1

    def planning_input_callback(self, msg: PoseWithCovarianceStamped):
        """Store planning input timestamps"""
        timestamp_ns = self.to_nanoseconds(msg.header.stamp)
        
        # Find corresponding perception data
        perception_match = self.find_closest_timestamp(
            self.perception_buffer, timestamp_ns, before=True)
        
        if perception_match:
            # Find corresponding ground truth
            ground_truth_match = self.find_closest_timestamp(
                self.ground_truth_buffer, perception_match['timestamp_ns'], before=True)
            
            if ground_truth_match:
                self.planning_input_buffer.append({
                    'timestamp_ns': timestamp_ns,
                    't_ground_truth_ns': ground_truth_match['timestamp_ns'],
                    't_perception_ns': perception_match['timestamp_ns']
                })
                self.stats['planning_input_count'] += 1

    def planning_output_callback(self, msg: Trajectory):
        """Complete the latency chain measurement"""
        timestamp_ns = self.to_nanoseconds(msg.header.stamp)
        
        # Find corresponding planning input
        planning_input_match = None
        for i, entry in enumerate(self.planning_input_buffer):
            time_diff = abs(entry['timestamp_ns'] - timestamp_ns)
            if time_diff < self.correlation_window_ns:
                planning_input_match = self.planning_input_buffer.pop(i)
                break
        
        if planning_input_match:
            # Calculate all delay components
            t_ground_truth = planning_input_match['t_ground_truth_ns']
            t_perception = planning_input_match['t_perception_ns']
            t_planning_input = planning_input_match['timestamp_ns']
            t_planning_output = timestamp_ns
            
            # R2V delay: ground truth -> planning input (via perception)
            r2v_delay_ms = (t_planning_input - t_ground_truth) / 1e6
            
            # Planner delay: planning input -> planning output
            planner_delay_ms = (t_planning_output - t_planning_input) / 1e6
            
            # Total R2V + Planner delay
            total_delay_ms = r2v_delay_ms + planner_delay_ms
            
            # Validate measurements
            valid_correlation = (
                0 <= r2v_delay_ms <= 1000 and
                0 <= planner_delay_ms <= 500 and
                0 <= total_delay_ms <= 1500
            )
            
            # Calculate confidence based on time differences
            max_time_diff = max(
                abs(t_perception - t_ground_truth),
                abs(t_planning_input - t_perception),
                abs(t_planning_output - t_planning_input)
            )
            confidence = 'high' if max_time_diff < 50e6 else 'medium' if max_time_diff < 100e6 else 'low'
            
            if valid_correlation:
                self.stats['successful_correlations'] += 1
                correlation_id = f"{self.session_timestamp}_{self.stats['successful_correlations']:06d}"
                
                # Log to CSV
                self.csv_writer.writerow([
                    correlation_id, timestamp_ns, self.session_timestamp,
                    t_ground_truth, t_perception, t_planning_input, t_planning_output,
                    r2v_delay_ms, planner_delay_ms, total_delay_ms,
                    valid_correlation, confidence
                ])
                self.csv_file.flush()
                
                self.get_logger().debug(
                    f"Latency chain: R2V={r2v_delay_ms:.2f}ms, "
                    f"Planner={planner_delay_ms:.2f}ms, Total={total_delay_ms:.2f}ms"
                )
            else:
                self.stats['failed_correlations'] += 1
                self.get_logger().warn("Invalid latency measurement - values out of range")
        else:
            self.stats['failed_correlations'] += 1
        
        self.stats['planning_output_count'] += 1

    def find_closest_timestamp(self, buffer, target_ns, before=True):
        """Find closest timestamp in buffer"""
        if not buffer:
            return None
        
        candidates = []
        for entry in buffer:
            timestamp_ns = entry['timestamp_ns']
            if before and timestamp_ns <= target_ns:
                candidates.append(entry)
            elif not before and timestamp_ns >= target_ns:
                candidates.append(entry)
        
        if not candidates:
            return None
        
        closest = min(candidates, key=lambda x: abs(x['timestamp_ns'] - target_ns))
        
        if abs(closest['timestamp_ns'] - target_ns) <= self.correlation_window_ns:
            return closest
        return None

    def cleanup_old_entries(self):
        """Remove old entries from buffers"""
        current_time_ns = self.to_nanoseconds(self.get_clock().now().to_msg())
        
        def filter_buffer(buffer):
            initial_size = len(buffer)
            while buffer and (current_time_ns - buffer[0]['timestamp_ns']) > self.cleanup_threshold_ns:
                buffer.popleft()
            return initial_size - len(buffer)
        
        cleaned_counts = {
            'ground_truth': filter_buffer(self.ground_truth_buffer),
            'perception': filter_buffer(self.perception_buffer),
            'planning_input': filter_buffer(self.planning_input_buffer)
        }
        
        total_cleaned = sum(cleaned_counts.values())
        if total_cleaned > 0:
            self.get_logger().debug(f"Cleaned {total_cleaned} old entries from buffers")

    def report_statistics(self):
        """Report current statistics"""
        correlation_rate = (
            self.stats['successful_correlations'] / 
            max(self.stats['planning_output_count'], 1)
        ) * 100
        
        self.get_logger().info(
            f"Stats: GT={self.stats['ground_truth_count']}, "
            f"Perception={self.stats['perception_count']}, "
            f"PlanIn={self.stats['planning_input_count']}, "
            f"PlanOut={self.stats['planning_output_count']}, "
            f"Correlations={self.stats['successful_correlations']} ({correlation_rate:.1f}%)"
        )

    def destroy_node(self):
        """Clean shutdown"""
        if hasattr(self, 'csv_file') and self.csv_file:
            self.csv_file.close()
        
        self.get_logger().info(f"Session complete. Data saved to: {self.csv_filename}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    analyzer = LatencyChainAnalyzer()
    
    try:
        rclpy.spin(analyzer)
    except KeyboardInterrupt:
        analyzer.get_logger().info("Shutting down latency chain analyzer...")
    finally:
        analyzer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 2.2 V2R延迟分析器 (v2r_latency_analyzer.py)

```python
#!/usr/bin/env python3

import pandas as pd
import numpy as np
import argparse
import os
import csv
from datetime import datetime
from typing import List, Dict, Tuple

class V2RLatencyAnalyzer:
    """
    Analyzer for V2R (Virtual-to-Real) latency using RTT measurements from timing logs
    """
    
    def __init__(self, timing_log_path: str):
        self.timing_log_path = timing_log_path
        self.session_timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        
        # Load and process timing log
        self.load_timing_log()
        self.analyze_v2r_latency()

    def load_timing_log(self):
        """Load timing log CSV file"""
        try:
            self.timing_df = pd.read_csv(self.timing_log_path)
            print(f"Loaded timing log: {len(self.timing_df)} records")
            
            # Validate required columns
            required_columns = ['event_type', 'pc_time', 'steering_angle', 'speed']
            missing_columns = [col for col in required_columns if col not in self.timing_df.columns]
            if missing_columns:
                raise ValueError(f"Missing columns in timing log: {missing_columns}")
                
        except Exception as e:
            print(f"Error loading timing log: {e}")
            raise

    def analyze_v2r_latency(self):
        """Analyze V2R latency using RTT measurements"""
        # Filter relevant events
        cmd_sent_events = self.timing_df[self.timing_df['event_type'] == 'CMD_SENT']
        nrf_sent_events = self.timing_df[self.timing_df['event_type'] == 'CMD_SENT_BY_NRF']
        
        print(f"Found {len(cmd_sent_events)} CMD_SENT events")
        print(f"Found {len(nrf_sent_events)} CMD_SENT_BY_NRF events")
        
        # Match command pairs and calculate RTT
        self.v2r_measurements = []
        
        for _, cmd_event in cmd_sent_events.iterrows():
            cmd_time = cmd_event['pc_time']
            
            # Find corresponding NRF event within reasonable time window
            time_window = 1000  # 1 second in ms
            candidates = nrf_sent_events[
                (nrf_sent_events['pc_time'] >= cmd_time) &
                (nrf_sent_events['pc_time'] <= cmd_time + time_window)
            ]
            
            if len(candidates) > 0:
                # Take the closest match
                closest_nrf = candidates.iloc[0]
                rtt_ms = closest_nrf['pc_time'] - cmd_time
                
                # Estimate V2R latency as RTT/2
                v2r_latency_ms = rtt_ms / 2.0
                
                # Validate measurement
                if 0 < rtt_ms < 500:  # Reasonable RTT range
                    measurement = {
                        'cmd_time': cmd_time,
                        'nrf_time': closest_nrf['pc_time'],
                        'rtt_ms': rtt_ms,
                        'v2r_latency_ms': v2r_latency_ms,
                        'steering_angle': cmd_event['steering_angle'],
                        'speed': cmd_event['speed']
                    }
                    self.v2r_measurements.append(measurement)
        
        print(f"Successfully analyzed {len(self.v2r_measurements)} V2R latency measurements")

    def get_v2r_statistics(self) -> Dict:
        """Calculate V2R latency statistics"""
        if not self.v2r_measurements:
            return {}
        
        v2r_latencies = [m['v2r_latency_ms'] for m in self.v2r_measurements]
        rtts = [m['rtt_ms'] for m in self.v2r_measurements]
        
        stats = {
            'count': len(v2r_latencies),
            'v2r_mean_ms': np.mean(v2r_latencies),
            'v2r_median_ms': np.median(v2r_latencies),
            'v2r_std_ms': np.std(v2r_latencies),
            'v2r_min_ms': np.min(v2r_latencies),
            'v2r_max_ms': np.max(v2r_latencies),
            'v2r_p95_ms': np.percentile(v2r_latencies, 95),
            'v2r_p99_ms': np.percentile(v2r_latencies, 99),
            'rtt_mean_ms': np.mean(rtts),
            'rtt_std_ms': np.std(rtts)
        }
        
        return stats

    def save_v2r_data(self, output_dir: str = 'data'):
        """Save V2R analysis results to CSV"""
        os.makedirs(output_dir, exist_ok=True)
        
        output_file = os.path.join(output_dir, f'v2r_latency_{self.session_timestamp}.csv')
        
        with open(output_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'measurement_id', 'cmd_time', 'nrf_time', 'rtt_ms', 'v2r_latency_ms',
                'steering_angle', 'speed'
            ])
            
            for i, measurement in enumerate(self.v2r_measurements):
                writer.writerow([
                    f"v2r_{self.session_timestamp}_{i:06d}",
                    measurement['cmd_time'],
                    measurement['nrf_time'],
                    measurement['rtt_ms'],
                    measurement['v2r_latency_ms'],
                    measurement['steering_angle'],
                    measurement['speed']
                ])
        
        print(f"V2R data saved to: {output_file}")
        return output_file


def main():
    parser = argparse.ArgumentParser(description='Analyze V2R latency from timing logs')
    parser.add_argument('timing_log', help='Path to timing log CSV file')
    parser.add_argument('--output-dir', default='data', help='Output directory for results')
    
    args = parser.parse_args()
    
    if not os.path.exists(args.timing_log):
        print(f"Error: Timing log file not found: {args.timing_log}")
        return
    
    # Run analysis
    analyzer = V2RLatencyAnalyzer(args.timing_log)
    stats = analyzer.get_v2r_statistics()
    
    if stats:
        print("\nV2R Latency Statistics:")
        print(f"  Samples: {stats['count']}")
        print(f"  Mean: {stats['v2r_mean_ms']:.2f} ms")
        print(f"  Median: {stats['v2r_median_ms']:.2f} ms")
        print(f"  Std: {stats['v2r_std_ms']:.2f} ms")
        print(f"  Range: {stats['v2r_min_ms']:.2f} - {stats['v2r_max_ms']:.2f} ms")
        print(f"  P95: {stats['v2r_p95_ms']:.2f} ms")
        print(f"  P99: {stats['v2r_p99_ms']:.2f} ms")
        
        # Save results
        output_file = analyzer.save_v2r_data(args.output_dir)
    else:
        print("No valid V2R measurements found")


if __name__ == '__main__':
    main()
```

### 2.3 跟踪误差测量器 (tracking_error_measurer.py)

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import csv
import os
import math
import numpy as np
from datetime import datetime
from shapely.geometry import LineString, Point

from geometry_msgs.msg import PoseWithCovarianceStamped

class TrackingErrorMeasurer(Node):
    """
    Measures tracking error for constant velocity baseline test
    """
    
    def __init__(self):
        super().__init__('tracking_error_measurer')
        
        # Session setup
        self.session_timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.setup_directories()
        self.setup_logging()
        
        # Test configuration
        self.target_velocity_ms = 0.5  # Target velocity in m/s
        self.test_duration_s = 300     # Test duration in seconds
        
        # Reference path (straight line)
        self.path_start = [0, 0]
        self.path_end = [150, 0]  # 150m straight path
        self.reference_path = LineString([self.path_start, self.path_end])
        
        # Test state
        self.test_active = False
        self.start_time_ns = None
        self.sample_count = 0
        
        # Topic names
        self.pose_topic = '/localization/kinematic_state'
        
        # Create subscription
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, self.pose_topic,
            self.pose_callback, 10)
        
        # Timer for test management
        self.timer = self.create_timer(1.0, self.check_test_status)
        
        self.get_logger().info(f"Tracking Error Measurer started - Session: {self.session_timestamp}")
        self.get_logger().info(f"Target velocity: {self.target_velocity_ms} m/s")
        self.get_logger().info(f"Reference path: {self.path_start} -> {self.path_end}")
        self.get_logger().info("Position vehicle near start point to begin test")

    def setup_directories(self):
        """Setup output directories"""
        self.base_dir = os.path.dirname(os.path.abspath(__file__))
        self.data_dir = os.path.join(self.base_dir, 'data')
        self.logs_dir = os.path.join(self.base_dir, 'logs')
        
        for directory in [self.data_dir, self.logs_dir]:
            os.makedirs(directory, exist_ok=True)

    def setup_logging(self):
        """Setup CSV logging for tracking error data"""
        self.csv_filename = os.path.join(self.data_dir, f'tracking_error_{self.session_timestamp}.csv')
        self.csv_file = open(self.csv_filename, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        
        # CSV header
        self.csv_writer.writerow([
            'sample_id', 'timestamp_ns', 'elapsed_time_s',
            'vehicle_x', 'vehicle_y', 'vehicle_z', 'vehicle_yaw_rad',
            's_actual', 's_target', 'longitudinal_error_m',
            'lateral_error_m', 'velocity_estimate_ms', 'test_active'
        ])
        self.csv_file.flush()

    def to_nanoseconds(self, stamp):
        """Convert ROS timestamp to nanoseconds"""
        return stamp.sec * 1_000_000_000 + stamp.nanosec

    def quaternion_to_yaw(self, q):
        """Convert quaternion to yaw angle"""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def calculate_frenet_coordinates(self, position):
        """Calculate Frenet coordinates (s, d) for vehicle position"""
        vehicle_point = Point(position[0], position[1])
        
        # Calculate s (progress along path)
        s_actual = self.reference_path.project(vehicle_point)
        
        # Calculate lateral distance
        closest_point = self.reference_path.interpolate(s_actual)
        lateral_distance = vehicle_point.distance(closest_point)
        
        # Determine sign of lateral error
        path_vector = np.array([self.path_end[0] - self.path_start[0], 
                               self.path_end[1] - self.path_start[1]])
        to_vehicle = np.array([position[0] - closest_point.x, 
                              position[1] - closest_point.y])
        
        cross_product = np.cross(path_vector, to_vehicle)
        if cross_product < 0:
            lateral_distance = -lateral_distance
        
        return s_actual, lateral_distance

    def is_near_start(self, position):
        """Check if vehicle is near the start of the path"""
        start_point = Point(self.path_start[0], self.path_start[1])
        vehicle_point = Point(position[0], position[1])
        distance_to_start = start_point.distance(vehicle_point)
        return distance_to_start < 5.0  # Within 5 meters

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        """Process pose messages and calculate tracking error"""
        timestamp_ns = self.to_nanoseconds(msg.header.stamp)
        
        # Extract position and orientation
        position = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ]
        yaw_rad = self.quaternion_to_yaw(msg.pose.pose.orientation)
        
        # Check if test should start
        if not self.test_active and self.is_near_start(position):
            self.start_time_ns = timestamp_ns
            self.test_active = True
            self.last_position = position
            self.last_timestamp_ns = timestamp_ns
            self.get_logger().info("Test started - vehicle detected at start position")
        
        # Process data if test is active
        if self.test_active:
            elapsed_time_s = (timestamp_ns - self.start_time_ns) / 1e9
            
            # Calculate Frenet coordinates
            s_actual, lateral_error = self.calculate_frenet_coordinates(position)
            
            # Calculate target position
            s_target = self.target_velocity_ms * elapsed_time_s
            
            # Calculate longitudinal error
            longitudinal_error_m = s_actual - s_target
            
            # Estimate velocity using finite difference
            velocity_estimate_ms = self.target_velocity_ms  # Default
            if hasattr(self, 'last_position') and hasattr(self, 'last_timestamp_ns'):
                dt = (timestamp_ns - self.last_timestamp_ns) / 1e9
                if dt > 0:
                    dx = position[0] - self.last_position[0]
                    dy = position[1] - self.last_position[1]
                    velocity_estimate_ms = math.sqrt(dx**2 + dy**2) / dt
            
            # Update last position for next velocity calculation
            self.last_position = position
            self.last_timestamp_ns = timestamp_ns
            
            # Log to CSV
            sample_id = f"te_{self.session_timestamp}_{self.sample_count:06d}"
            self.csv_writer.writerow([
                sample_id, timestamp_ns, elapsed_time_s,
                position[0], position[1], position[2], yaw_rad,
                s_actual, s_target, longitudinal_error_m,
                abs(lateral_error), velocity_estimate_ms, self.test_active
            ])
            self.csv_file.flush()
            
            self.sample_count += 1
            
            # Periodic progress report
            if self.sample_count % 50 == 0:
                self.get_logger().info(
                    f"Progress: {elapsed_time_s:.1f}s, s={s_actual:.1f}m, "
                    f"target_s={s_target:.1f}m, error={longitudinal_error_m:.3f}m"
                )
            
            # Check test completion
            if elapsed_time_s >= self.test_duration_s or s_actual >= self.reference_path.length:
                self.test_active = False
                self.get_logger().info("Test completed")
                self.generate_summary()

    def check_test_status(self):
        """Periodic check for test timeout"""
        if self.test_active and self.start_time_ns:
            current_time_ns = self.to_nanoseconds(self.get_clock().now().to_msg())
            elapsed = (current_time_ns - self.start_time_ns) / 1e9
            if elapsed >= self.test_duration_s:
                self.test_active = False
                self.get_logger().info("Test ended - timeout reached")
                self.generate_summary()

    def generate_summary(self):
        """Generate test summary"""
        self.get_logger().info(f"Test summary: {self.sample_count} samples collected")
        self.get_logger().info(f"Data saved to: {self.csv_filename}")

    def destroy_node(self):
        """Clean shutdown"""
        if hasattr(self, 'csv_file') and self.csv_file:
            self.csv_file.close()
        
        self.get_logger().info(f"Tracking error measurement complete")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    measurer = TrackingErrorMeasurer()
    
    try:
        rclpy.spin(measurer)
    except KeyboardInterrupt:
        measurer.get_logger().info("Shutting down tracking error measurer...")
    finally:
        measurer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## 3. 数据分析与可视化

### 3.1 综合分析器 (experiment_b_analyzer.py)

```python
#!/usr/bin/env python3

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import argparse
import os
import json
from datetime import datetime
from typing import Dict, List, Tuple

class ExperimentBAnalyzer:
    """
    Comprehensive analyzer for Experiment B: Latency and baseline tracking error analysis
    """
    
    def __init__(self, latency_chain_csv: str, v2r_csv: str, tracking_error_csv: str):
        self.latency_chain_csv = latency_chain_csv
        self.v2r_csv = v2r_csv
        self.tracking_error_csv = tracking_error_csv
        
        self.analysis_timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.setup_directories()
        self.setup_plotting()
        
        # Load and validate data
        self.load_data()
        self.analysis_results = {}

    def setup_directories(self):
        """Setup output directories"""
        self.base_dir = os.path.dirname(os.path.abspath(__file__))
        self.results_dir = os.path.join(self.base_dir, 'results')
        self.analysis_dir = os.path.join(self.results_dir, f'experiment_b_{self.analysis_timestamp}')
        
        for directory in [self.results_dir, self.analysis_dir]:
            os.makedirs(directory, exist_ok=True)

    def setup_plotting(self):
        """Setup matplotlib configuration"""
        plt.style.use('seaborn-v0_8-whitegrid')
        plt.rcParams['figure.figsize'] = [16, 12]
        plt.rcParams['font.size'] = 12
        plt.rcParams['axes.grid'] = True
        plt.rcParams['grid.alpha'] = 0.3

    def load_data(self):
        """Load and validate all data files"""
        try:
            # Load latency chain data
            self.latency_chain_df = pd.read_csv(self.latency_chain_csv)
            print(f"Loaded latency chain data: {len(self.latency_chain_df)} records")
            
            # Load V2R data
            self.v2r_df = pd.read_csv(self.v2r_csv)
            print(f"Loaded V2R data: {len(self.v2r_df)} records")
            
            # Load tracking error data
            self.tracking_error_df = pd.read_csv(self.tracking_error_csv)
            print(f"Loaded tracking error data: {len(self.tracking_error_df)} records")
            
            # Clean data
            self.clean_data()
            
        except Exception as e:
            print(f"Error loading data: {e}")
            raise

    def clean_data(self):
        """Clean and filter data"""
        # Filter valid latency chain correlations
        initial_chain_count = len(self.latency_chain_df)
        self.latency_chain_df = self.latency_chain_df[
            (self.latency_chain_df['valid_correlation'] == True) &
            (self.latency_chain_df['r2v_delay_ms'] >= 0) &
            (self.latency_chain_df['r2v_delay_ms'] <= 1000) &
            (self.latency_chain_df['planner_delay_ms'] >= 0) &
            (self.latency_chain_df['planner_delay_ms'] <= 500)
        ]
        print(f"Cleaned latency chain: {len(self.latency_chain_df)}/{initial_chain_count} records")
        
        # Filter reasonable V2R measurements
        initial_v2r_count = len(self.v2r_df)
        self.v2r_df = self.v2r_df[
            (self.v2r_df['v2r_latency_ms'] > 0) &
            (self.v2r_df['v2r_latency_ms'] < 250)
        ]
        print(f"Cleaned V2R data: {len(self.v2r_df)}/{initial_v2r_count} records")
        
        # Filter active test tracking error data
        initial_error_count = len(self.tracking_error_df)
        if 'test_active' in self.tracking_error_df.columns:
            self.tracking_error_df = self.tracking_error_df[
                self.tracking_error_df['test_active'] == True
            ]
        print(f"Cleaned tracking error: {len(self.tracking_error_df)}/{initial_error_count} records")

    def analyze_latency_characteristics(self):
        """Analyze complete latency characteristics"""
        print("\n" + "="*80)
        print("EXPERIMENT B - PART 1: LATENCY CHARACTERISTICS ANALYSIS")
        print("="*80)
        
        # Calculate R2V and Planner statistics
        r2v_stats = self.calculate_component_stats(self.latency_chain_df['r2v_delay_ms'], 'R2V')
        planner_stats = self.calculate_component_stats(self.latency_chain_df['planner_delay_ms'], 'Planner')
        
        # Calculate V2R statistics
        v2r_stats = self.calculate_component_stats(self.v2r_df['v2r_latency_ms'], 'V2R')
        
        # Calculate total latency using mean values
        total_mean_ms = r2v_stats['mean'] + planner_stats['mean'] + v2r_stats['mean']
        
        # Store results
        self.analysis_results['latency_statistics'] = {
            'r2v_delay_ms': r2v_stats,
            'planner_delay_ms': planner_stats,
            'v2r_delay_ms': v2r_stats,
            'total_delay_ms': {
                'mean': total_mean_ms,
                'components': {
                    'r2v_contribution_pct': (r2v_stats['mean'] / total_mean_ms) * 100,
                    'planner_contribution_pct': (planner_stats['mean'] / total_mean_ms) * 100,
                    'v2r_contribution_pct': (v2r_stats['mean'] / total_mean_ms) * 100
                }
            }
        }
        
        # Print results table
        self.print_latency_table()
        
        # Assess performance
        self.assess_latency_performance()
        
        return self.analysis_results['latency_statistics']

    def calculate_component_stats(self, data: pd.Series, component_name: str) -> Dict:
        """Calculate statistics for a latency component"""
        return {
            'count': len(data),
            'mean': data.mean(),
            'median': data.median(),
            'std': data.std(),
            'min': data.min(),
            'max': data.max(),
            'p95': data.quantile(0.95),
            'p99': data.quantile(0.99)
        }

    def print_latency_table(self):
        """Print formatted latency statistics table"""
        stats = self.analysis_results['latency_statistics']
        
        print(f"\nLatency Component Analysis")
        print("-" * 100)
        print(f"{'Component':<20} {'Count':<8} {'Mean':<8} {'Median':<8} {'Std':<8} {'Min':<8} {'Max':<8} {'P95':<8} {'P99':<8}")
        print("-" * 100)
        
        components = [
            ('R2V Delay (ms)', stats['r2v_delay_ms']),
            ('Planner Delay (ms)', stats['planner_delay_ms']),
            ('V2R Delay (ms)', stats['v2r_delay_ms'])
        ]
        
        for name, s in components:
            print(f"{name:<20} {s['count']:<8} {s['mean']:<8.2f} {s['median']:<8.2f} {s['std']:<8.2f} "
                  f"{s['min']:<8.2f} {s['max']:<8.2f} {s['p95']:<8.2f} {s['p99']:<8.2f}")
        
        print("-" * 100)
        print(f"{'Total E2E (ms)':<20} {'-':<8} {stats['total_delay_ms']['mean']:<8.2f}")
        print("-" * 100)

    def assess_latency_performance(self):
        """Assess latency performance against thresholds"""
        stats = self.analysis_results['latency_statistics']
        total_latency = stats['total_delay_ms']['mean']
        
        print(f"\nLatency Performance Assessment:")
        print("-" * 60)
        
        criteria = [
            ('Total latency < 100ms', total_latency < 100, f"{total_latency:.1f}ms"),
            ('V2R latency < 50ms', stats['v2r_delay_ms']['mean'] < 50, f"{stats['v2r_delay_ms']['mean']:.1f}ms"),
            ('Planner latency < 50ms', stats['planner_delay_ms']['mean'] < 50, f"{stats['planner_delay_ms']['mean']:.1f}ms"),
            ('R2V latency < 100ms', stats['r2v_delay_ms']['mean'] < 100, f"{stats['r2v_delay_ms']['mean']:.1f}ms")
        ]
        
        passed = 0
        for criterion, result, value in criteria:
            status = "PASS" if result else "FAIL"
            print(f"  {criterion:<30}: {status:<4} ({value})")
            if result:
                passed += 1
        
        print(f"\nOverall Assessment: {passed}/{len(criteria)} criteria passed")

    def analyze_tracking_error_baseline(self):
        """Analyze tracking error baseline"""
        print("\n" + "="*80)
        print("EXPERIMENT B - PART 2: TRACKING ERROR BASELINE ANALYSIS")
        print("="*80)
        
        if len(self.tracking_error_df) == 0:
            print("Error: No tracking error data available")
            return None
        
        # Get test parameters
        target_velocity = 0.5  # m/s from test configuration
        total_latency_s = self.analysis_results['latency_statistics']['total_delay_ms']['mean'] / 1000.0
        
        # Calculate theoretical error
        theoretical_error_m = target_velocity * total_latency_s
        
        # Calculate actual error statistics
        actual_errors = self.tracking_error_df['longitudinal_error_m']
        actual_error_mean = actual_errors.mean()
        actual_error_std = actual_errors.std()
        actual_error_median = actual_errors.median()
        
        # Calculate agreement
        agreement_ratio = actual_error_mean / theoretical_error_m if theoretical_error_m != 0 else 0
        
        # Store results
        self.analysis_results['error_baseline'] = {
            'target_velocity_ms': target_velocity,
            'total_latency_ms': total_latency_s * 1000,
            'theoretical_error_m': theoretical_error_m,
            'actual_error_mean_m': actual_error_mean,
            'actual_error_std_m': actual_error_std,
            'actual_error_median_m': actual_error_median,
            'agreement_ratio': agreement_ratio,
            'sample_count': len(actual_errors)
        }
        
        print(f"\nTracking Error Baseline Analysis:")
        print("-" * 60)
        print(f"Test Configuration:")
        print(f"  Target Velocity: {target_velocity:.3f} m/s")
        print(f"  Total Latency: {total_latency_s*1000:.2f} ms")
        print(f"\nError Analysis:")
        print(f"  Theoretical Error: {theoretical_error_m:.6f} m")
        print(f"  Actual Error Mean: {actual_error_mean:.6f} m")
        print(f"  Actual Error Std: {actual_error_std:.6f} m")
        print(f"  Agreement Ratio: {agreement_ratio:.3f}")
        
        # Assess prediction accuracy
        prediction_accurate = 0.5 <= agreement_ratio <= 1.5
        accuracy_status = "ACCURATE" if prediction_accurate else "INACCURATE"
        print(f"\nBaseline Prediction Assessment: {accuracy_status}")
        
        return self.analysis_results['error_baseline']

    def create_comprehensive_plots(self):
        """Create comprehensive analysis plots"""
        print(f"\nGenerating analysis plots...")
        
        # Create large figure with multiple subplots
        fig = plt.figure(figsize=(20, 16))
        
        # Plot 1: Latency component breakdown
        ax1 = plt.subplot(3, 3, 1)
        stats = self.analysis_results['latency_statistics']
        components = ['R2V', 'Planner', 'V2R']
        means = [stats['r2v_delay_ms']['mean'], 
                stats['planner_delay_ms']['mean'], 
                stats['v2r_delay_ms']['mean']]
        colors = ['skyblue', 'lightcoral', 'lightgreen']
        
        bars = ax1.bar(components, means, color=colors)
        ax1.set_ylabel('Latency (ms)')
        ax1.set_title('Mean Latency by Component')
        ax1.grid(True, alpha=0.3)
        
        # Add value labels
        for bar, value in zip(bars, means):
            height = bar.get_height()
            ax1.text(bar.get_x() + bar.get_width()/2., height + max(means)*0.01,
                    f'{value:.1f}ms', ha='center', va='bottom')
        
        # Plot 2: V2R latency distribution
        ax2 = plt.subplot(3, 3, 2)
        v2r_data = self.v2r_df['v2r_latency_ms']
        ax2.hist(v2r_data, bins=30, alpha=0.7, color='lightgreen', edgecolor='black')
        ax2.axvline(v2r_data.mean(), color='red', linestyle='--', linewidth=2,
                   label=f'Mean: {v2r_data.mean():.1f}ms')
        ax2.set_xlabel('V2R Latency (ms)')
        ax2.set_ylabel('Frequency')
        ax2.set_title('V2R Latency Distribution')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        # Plot 3: Latency composition pie chart
        ax3 = plt.subplot(3, 3, 3)
        ax3.pie(means, labels=components, autopct='%1.1f%%', colors=colors)
        ax3.set_title('Latency Composition')
        
        # Plot 4: R2V + Planner latency time series
        ax4 = plt.subplot(3, 3, 4)
        if len(self.latency_chain_df) > 0:
            total_chain = self.latency_chain_df['total_r2v_planner_ms']
            ax4.plot(range(len(total_chain)), total_chain, alpha=0.7, linewidth=1)
            ax4.set_xlabel('Sample Index')
            ax4.set_ylabel('R2V + Planner Latency (ms)')
            ax4.set_title('R2V + Planner Latency Over Time')
            ax4.grid(True, alpha=0.3)
        
        # Plot 5: Tracking error time series
        ax5 = plt.subplot(3, 3, 5)
        if len(self.tracking_error_df) > 0:
            error_baseline = self.analysis_results['error_baseline']
            time_data = self.tracking_error_df['elapsed_time_s']
            error_data = self.tracking_error_df['longitudinal_error_m']
            
            ax5.plot(time_data, error_data, alpha=0.7, linewidth=1, 
                    label='Actual Error', color='blue')
            ax5.axhline(y=error_baseline['theoretical_error_m'], color='red', 
                       linestyle='--', linewidth=2,
                       label=f'Theoretical: {error_baseline["theoretical_error_m"]:.6f}m')
            ax5.axhline(y=error_baseline['actual_error_mean_m'], color='green', 
                       linestyle=':', linewidth=2,
                       label=f'Mean: {error_baseline["actual_error_mean_m"]:.6f}m')
            
            ax5.set_xlabel('Time (s)')
            ax5.set_ylabel('Longitudinal Error (m)')
            ax5.set_title('Tracking Error vs Time')
            ax5.legend()
            ax5.grid(True, alpha=0.3)
        
        # Plot 6: Error distribution
        ax6 = plt.subplot(3, 3, 6)
        if len(self.tracking_error_df) > 0:
            error_data = self.tracking_error_df['longitudinal_error_m']
            error_baseline = self.analysis_results['error_baseline']
            
            ax6.hist(error_data, bins=30, alpha=0.7, color='skyblue', edgecolor='black')
            ax6.axvline(error_baseline['actual_error_mean_m'], color='green', 
                       linestyle=':', linewidth=2,
                       label=f'Mean: {error_baseline["actual_error_mean_m"]:.6f}m')
            ax6.axvline(error_baseline['theoretical_error_m'], color='red', 
                       linestyle='--', linewidth=2,
                       label=f'Theory: {error_baseline["theoretical_error_m"]:.6f}m')
            ax6.set_xlabel('Longitudinal Error (m)')
            ax6.set_ylabel('Frequency')
            ax6.set_title('Error Distribution')
            ax6.legend()
            ax6.grid(True, alpha=0.3)
        
        # Plot 7: Component correlation
        ax7 = plt.subplot(3, 3, 7)
        if len(self.latency_chain_df) > 0:
            r2v_data = self.latency_chain_df['r2v_delay_ms']
            planner_data = self.latency_chain_df['planner_delay_ms']
            ax7.scatter(r2v_data, planner_data, alpha=0.6, s=20)
            ax7.set_xlabel('R2V Delay (ms)')
            ax7.set_ylabel('Planner Delay (ms)')
            ax7.set_title('R2V vs Planner Delay Correlation')
            ax7.grid(True, alpha=0.3)
        
        # Plot 8: Latency box plot comparison
        ax8 = plt.subplot(3, 3, 8)
        box_data = []
        box_labels = []
        if len(self.latency_chain_df) > 0:
            box_data.extend([self.latency_chain_df['r2v_delay_ms'], 
                           self.latency_chain_df['planner_delay_ms']])
            box_labels.extend(['R2V', 'Planner'])
        if len(self.v2r_df) > 0:
            box_data.append(self.v2r_df['v2r_latency_ms'])
            box_labels.append('V2R')
        
        if box_data:
            bp = ax8.boxplot(box_data, labels=box_labels, patch_artist=True)
            colors = ['skyblue', 'lightcoral', 'lightgreen']
            for patch, color in zip(bp['boxes'], colors[:len(bp['boxes'])]):
                patch.set_facecolor(color)
            ax8.set_ylabel('Latency (ms)')
            ax8.set_title('Latency Distribution Comparison')
            ax8.grid(True, alpha=0.3)
        
        # Plot 9: Agreement analysis
        ax9 = plt.subplot(3, 3, 9)
        if 'error_baseline' in self.analysis_results:
            agreement_ratio = self.analysis_results['error_baseline']['agreement_ratio']
            theoretical = self.analysis_results['error_baseline']['theoretical_error_m']
            actual = self.analysis_results['error_baseline']['actual_error_mean_m']
            
            categories = ['Theoretical', 'Actual Mean']
            values = [theoretical, actual]
            colors_bar = ['red', 'green']
            
            bars = ax9.bar(categories, values, color=colors_bar, alpha=0.7)
            ax9.set_ylabel('Error (m)')
            ax9.set_title(f'Error Comparison (Ratio: {agreement_ratio:.3f})')
            ax9.grid(True, alpha=0.3)
            
            # Add value labels
            for bar, value in zip(bars, values):
                height = bar.get_height()
                ax9.text(bar.get_x() + bar.get_width()/2., height + max(values)*0.01,
                        f'{value:.6f}m', ha='center', va='bottom')
        
        plt.tight_layout()
        
        # Save plot
        plot_filename = os.path.join(self.analysis_dir, f'experiment_b_analysis_{self.analysis_timestamp}.png')
        plt.savefig(plot_filename, dpi=300, bbox_inches='tight')
        print(f"Analysis plots saved: {plot_filename}")
        
        return fig

    def generate_report(self):
        """Generate comprehensive analysis report"""
        print(f"\nGenerating comprehensive report...")
        
        latency_stats = self.analysis_results['latency_statistics']
        
        report_content = f"""# Experiment B: Latency and Baseline Tracking Error Analysis Report

## Analysis Summary
- **Analysis Timestamp**: {self.analysis_timestamp}
- **Latency Chain Data**: {os.path.basename(self.latency_chain_csv)} ({len(self.latency_chain_df)} records)
- **V2R Data**: {os.path.basename(self.v2r_csv)} ({len(self.v2r_df)} records)  
- **Tracking Error Data**: {os.path.basename(self.tracking_error_csv)} ({len(self.tracking_error_df)} records)

## Part 1: Latency Characteristics Analysis

### Key Performance Metrics

| Component | Mean (ms) | Median (ms) | Std (ms) | P95 (ms) | P99 (ms) |
|-----------|-----------|-------------|----------|----------|----------|
| R2V Delay | {latency_stats['r2v_delay_ms']['mean']:.2f} | {latency_stats['r2v_delay_ms']['median']:.2f} | {latency_stats['r2v_delay_ms']['std']:.2f} | {latency_stats['r2v_delay_ms']['p95']:.2f} | {latency_stats['r2v_delay_ms']['p99']:.2f} |
| Planner Delay | {latency_stats['planner_delay_ms']['mean']:.2f} | {latency_stats['planner_delay_ms']['median']:.2f} | {latency_stats['planner_delay_ms']['std']:.2f} | {latency_stats['planner_delay_ms']['p95']:.2f} | {latency_stats['planner_delay_ms']['p99']:.2f} |
| V2R Delay | {latency_stats['v2r_delay_ms']['mean']:.2f} | {latency_stats['v2r_delay_ms']['median']:.2f} | {latency_stats['v2r_delay_ms']['std']:.2f} | {latency_stats['v2r_delay_ms']['p95']:.2f} | {latency_stats['v2r_delay_ms']['p99']:.2f} |
| **Total E2E** | **{latency_stats['total_delay_ms']['mean']:.2f}** | - | - | - | - |

### Latency Composition
- R2V Contribution: {latency_stats['total_delay_ms']['components']['r2v_contribution_pct']:.1f}%
- Planner Contribution: {latency_stats['total_delay_ms']['components']['planner_contribution_pct']:.1f}%  
- V2R Contribution: {latency_stats['total_delay_ms']['components']['v2r_contribution_pct']:.1f}%

"""

        if 'error_baseline' in self.analysis_results:
            error_baseline = self.analysis_results['error_baseline']
            report_content += f"""
## Part 2: Baseline Tracking Error Analysis

### Test Configuration
- **Target Velocity**: {error_baseline['target_velocity_ms']:.3f} m/s
- **Total System Latency**: {error_baseline['total_latency_ms']:.2f} ms
- **Test Samples**: {error_baseline['sample_count']} measurements

### Error Analysis Results

| Metric | Value |
|--------|-------|
| Theoretical Error (v × ΔT) | {error_baseline['theoretical_error_m']:.6f} m |
| Actual Error Mean | {error_baseline['actual_error_mean_m']:.6f} m |
| Actual Error Std | {error_baseline['actual_error_std_m']:.6f} m |
| Agreement Ratio | {error_baseline['agreement_ratio']:.3f} |

### Baseline Assessment
The theoretical model **{'ACCURATELY' if 0.5 <= error_baseline['agreement_ratio'] <= 1.5 else 'INACCURATELY'}** predicts actual tracking error.

"""

        report_content += f"""
## Conclusions

### Platform Performance
Total end-to-end latency of {latency_stats['total_delay_ms']['mean']:.1f}ms demonstrates {'EXCELLENT' if latency_stats['total_delay_ms']['mean'] < 100 else 'GOOD' if latency_stats['total_delay_ms']['mean'] < 150 else 'ACCEPTABLE'} performance for ADS testing.

### Error Baseline Establishment  
Successfully established quantitative baseline for tracking error attribution in future testing scenarios.

## Generated Files
- Analysis plots: `experiment_b_analysis_{self.analysis_timestamp}.png`
- Detailed results: `experiment_b_results_{self.analysis_timestamp}.json`
- This report: `experiment_b_report_{self.analysis_timestamp}.md`
"""
        
        # Save report
        report_file = os.path.join(self.analysis_dir, f"experiment_b_report_{self.analysis_timestamp}.md")
        with open(report_file, 'w') as f:
            f.write(report_content)
        
        print(f"Report saved: {report_file}")

    def save_results_json(self):
        """Save detailed results to JSON"""
        results_file = os.path.join(self.analysis_dir, f"experiment_b_results_{self.analysis_timestamp}.json")
        
        # Convert numpy types for JSON serialization
        def convert_for_json(obj):
            if hasattr(obj, 'item'):
                return obj.item()
            elif isinstance(obj, np.integer):
                return int(obj)
            elif isinstance(obj, np.floating):
                return float(obj)
            else:
                return obj
        
        json_results = {
            'metadata': {
                'analysis_timestamp': self.analysis_timestamp,
                'latency_chain_file': os.path.basename(self.latency_chain_csv),
                'v2r_file': os.path.basename(self.v2r_csv),
                'tracking_error_file': os.path.basename(self.tracking_error_csv),
                'latency_chain_samples': len(self.latency_chain_df),
                'v2r_samples': len(self.v2r_df),
                'tracking_error_samples': len(self.tracking_error_df)
            },
            'analysis_results': {
                k: {kk: convert_for_json(vv) if not isinstance(vv, dict) else 
                    {kkk: convert_for_json(vvv) for kkk, vvv in vv.items()} 
                    for kk, vv in v.items()}
                for k, v in self.analysis_results.items()
            }
        }
        
        with open(results_file, 'w') as f:
            json.dump(json_results, f, indent=2)
        
        print(f"Results JSON saved: {results_file}")

    def run_complete_analysis(self):
        """Run complete analysis pipeline"""
        print("Starting Experiment B comprehensive analysis...")
        
        try:
            # Part 1: Latency analysis
            self.analyze_latency_characteristics()
            
            # Part 2: Error baseline analysis
            self.analyze_tracking_error_baseline()
            
            # Generate outputs
            self.create_comprehensive_plots()
            self.save_results_json()
            self.generate_report()
            
            print(f"\nExperiment B analysis complete!")
            print(f"All results saved to: {self.analysis_dir}")
            return True
            
        except Exception as e:
            print(f"Error during analysis: {e}")
            return False


def main():
    parser = argparse.ArgumentParser(description='Comprehensive Experiment B Analysis')
    parser.add_argument('latency_chain_csv', help='Path to latency chain CSV file')
    parser.add_argument('v2r_csv', help='Path to V2R CSV file')
    parser.add_argument('tracking_error_csv', help='Path to tracking error CSV file')
    parser.add_argument('--show-plots', action='store_true', help='Display plots after generation')
    
    args = parser.parse_args()
    
    # Validate input files
    input_files = [args.latency_chain_csv, args.v2r_csv, args.tracking_error_csv]
    for file_path in input_files:
        if not os.path.exists(file_path):
            print(f"Error: File not found: {file_path}")
            return 1
    
    # Run analysis
    analyzer = ExperimentBAnalyzer(args.latency_chain_csv, args.v2r_csv, args.tracking_error_csv)
    success = analyzer.run_complete_analysis()
    
    if success and args.show_plots:
        plt.show()
    
    return 0 if success else 1


if __name__ == '__main__':
    exit(main())
```

## 4. 实验执行脚本

### 4.1 主启动脚本 (run_experiment_b.sh)

```bash
#!/bin/bash

# Experiment B Execution Script
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

print_colored() {
    echo -e "${1}${2}${NC}"
}

print_header() {
    echo
    print_colored $BLUE "=========================================="
    print_colored $BLUE "$1"
    print_colored $BLUE "=========================================="
}

# Cleanup function
cleanup() {
    print_colored $YELLOW "\nCleaning up processes..."
    if [ ! -z "$LATENCY_PID" ]; then
        kill $LATENCY_PID 2>/dev/null || true
    fi
    if [ ! -z "$ERROR_PID" ]; then
        kill $ERROR_PID 2>/dev/null || true
    fi
    wait 2>/dev/null || true
    print_colored $GREEN "Cleanup completed."
}

trap cleanup EXIT INT TERM

print_header "Experiment B: Latency and Baseline Tracking Error Analysis"

# Check dependencies
command -v ros2 >/dev/null 2>&1 || { echo "ROS2 not found. Please source ROS2 setup."; exit 1; }
command -v python3 >/dev/null 2>&1 || { echo "Python3 not found."; exit 1; }

# Setup environment
cd "$SCRIPT_DIR"
source /opt/ros/humble/setup.bash  # Adjust for your ROS2 distribution

print_colored $GREEN "Environment setup completed."

# Check required topics
print_colored $BLUE "Checking system readiness..."

# Wait for critical topics
wait_for_topic() {
    local topic=$1
    local timeout=${2:-30}
    local count=0
    
    print_colored $YELLOW "Waiting for topic: $topic"
    while [ $count -lt $timeout ]; do
        if ros2 topic list | grep -q "$topic"; then
            print_colored $GREEN "Topic $topic is available!"
            return 0
        fi
        sleep 1
        count=$((count + 1))
        echo -n "."
    done
    
    print_colored $RED "Timeout waiting for topic: $topic"
    return 1
}

# Check critical topics
if ! wait_for_topic "/localization/kinematic_state" 60; then
    print_colored $RED "Critical topic not available. Is the ADS running?"
    exit 1
fi

if ! wait_for_topic "/planning/trajectory" 30; then
    print_colored $YELLOW "Warning: Planning topic not available."
fi

print_header "Starting Experiment B Data Collection"

SESSION_TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
print_colored $GREEN "Session timestamp: $SESSION_TIMESTAMP"

# Start latency chain analyzer
print_colored $BLUE "Starting Latency Chain Analyzer..."
python3 latency_chain_analyzer.py &
LATENCY_PID=$!
print_colored $GREEN "Latency chain analyzer started (PID: $LATENCY_PID)"

# Start tracking error measurer
print_colored $BLUE "Starting Tracking Error Measurer..."
python3 tracking_error_measurer.py &
ERROR_PID=$!
print_colored $GREEN "Tracking error measurer started (PID: $ERROR_PID)"

print_header "Experiment B Data Collection Active"
print_colored $GREEN "Data collection is now active!"
print_colored $YELLOW "Instructions:"
echo "  1. First phase (5-10 minutes): Drive normally to collect latency data"
echo "  2. Second phase: Position vehicle at start of straight test path"
echo "  3. Drive straight at constant 0.5 m/s velocity for baseline test"
echo "  4. The system will automatically detect test phases"
echo "  5. Press Ctrl+C when both phases are complete"

print_colored $BLUE "\nPress Ctrl+C to stop data collection..."

# Monitor processes
while true; do
    if ! kill -0 $LATENCY_PID 2>/dev/null; then
        print_colored $RED "Latency analyzer stopped unexpectedly!"
        break
    fi
    if ! kill -0 $ERROR_PID 2>/dev/null; then
        print_colored $RED "Error measurer stopped unexpectedly!"
        break
    fi
    
    sleep 5
done

print_header "Experiment B Data Collection Completed"
print_colored $GREEN "Session $SESSION_TIMESTAMP data collection finished."

# Check for timing log from your PC controller
TIMING_LOG_PATTERN="logs/timing_log_*.csv"
TIMING_LOG=$(ls $TIMING_LOG_PATTERN 2>/dev/null | tail -1)

if [ -f "$TIMING_LOG" ]; then
    print_colored $GREEN "Found timing log: $TIMING_LOG"
    print_colored $BLUE "Analyzing V2R latency from timing log..."
    python3 v2r_latency_analyzer.py "$TIMING_LOG"
    V2R_CSV=$(ls data/v2r_latency_*.csv 2>/dev/null | tail -1)
else
    print_colored $YELLOW "Warning: No timing log found. V2R analysis will be skipped."
    V2R_CSV=""
fi

# Find generated data files
LATENCY_CHAIN_CSV=$(ls data/latency_chain_*.csv 2>/dev/null | tail -1)
TRACKING_ERROR_CSV=$(ls data/tracking_error_*.csv 2>/dev/null | tail -1)

print_colored $BLUE "Next steps:"
if [ -f "$LATENCY_CHAIN_CSV" ] && [ -f "$TRACKING_ERROR_CSV" ] && [ -f "$V2R_CSV" ]; then
    echo "  1. Run analysis: python3 experiment_b_analyzer.py $LATENCY_CHAIN_CSV $V2R_CSV $TRACKING_ERROR_CSV"
    echo "  2. Check results/ directory for comprehensive analysis"
    echo "  3. Review logs/ directory for session details"
    
    # Ask if user wants to run analysis now
    print_colored $YELLOW "Run analysis now? (y/n): "
    read -r response
    if [[ "$response" =~ ^[Yy]$ ]]; then
        print_colored $BLUE "Running analysis..."
        python3 experiment_b_analyzer.py "$LATENCY_CHAIN_CSV" "$V2R_CSV" "$TRACKING_ERROR_CSV" --show-plots
    fi
else
    print_colored $RED "Error: Some data files missing. Please check data collection."
    echo "  Expected files:"
    echo "    - Latency chain: $LATENCY_CHAIN_CSV"
    echo "    - Tracking error: $TRACKING_ERROR_CSV"
    echo "    - V2R data: $V2R_CSV"
fi
```

## 5. 使用说明

### 5.1 快速开始

1. **准备环境**

```bash
# Clone/copy all files to your experiment directory
cd /path/to/experiment_b
chmod +x run_experiment_b.sh

# Ensure your ROS2 environment is sourced
source /opt/ros/humble/setup.bash
```

2. **启动系统组件**

```bash
# Start your ADS system (Autoware, CARLA, etc.)
# Start your PC controller with timing logging enabled
# Ensure all topics are publishing
```

3. **执行实验**

```bash
./run_experiment_b.sh
```

4. **数据收集阶段**
   - **Phase 1 (5-10 minutes)**: 正常驾驶收集延迟数据
   - **Phase 2**: 恒速直线测试收集跟踪误差基线

5. **分析结果**

```bash
# Manual analysis if not run automatically
python3 experiment_b_analyzer.py \
  data/latency_chain_YYYYMMDD_HHMMSS.csv \
  data/v2r_latency_YYYYMMDD_HHMMSS.csv \
  data/tracking_error_YYYYMMDD_HHMMSS.csv \
  --show-plots
```

### 5.2 手动执行步骤

如果需要手动控制各个组件：

```bash
# Terminal 1: Latency Chain Analyzer
python3 latency_chain_analyzer.py

# Terminal 2: Tracking Error Measurer  
python3 tracking_error_measurer.py

# Terminal 3: Monitor and control your driving

# After data collection, analyze V2R latency
python3 v2r_latency_analyzer.py logs/timing_log_YYYYMMDD_HHMMSS.csv

# Final comprehensive analysis
python3 experiment_b_analyzer.py \
  data/latency_chain_YYYYMMDD_HHMMSS.csv \
  data/v2r_latency_YYYYMMDD_HHMMSS.csv \
  data/tracking_error_YYYYMMDD_HHMMSS.csv
```

### 5.3 配置调整

根据您的系统，需要修改以下话题名称：

在 `latency_chain_analyzer.py` 中：

```python
self.ground_truth_topic = '/your/ground_truth/topic'
self.perception_topic = '/your/perception/topic'  
self.planning_input_topic = '/your/planning_input/topic'
self.planning_output_topic = '/your/planning_output/topic'
```

在 `tracking_error_measurer.py` 中：

```python
self.pose_topic = '/your/localization/topic'
```

### 5.4 预期输出

**数据文件**：

- `latency_chain_YYYYMMDD_HHMMSS.csv` - R2V和规划器延迟数据
- `v2r_latency_YYYYMMDD_HHMMSS.csv` - V2R延迟数据  
- `tracking_error_YYYYMMDD_HHMMSS.csv` - 跟踪误差数据

**分析结果**：

- `experiment_b_analysis_YYYYMMDD_HHMMSS.png` - 综合分析图表
- `experiment_b_report_YYYYMMDD_HHMMSS.md` - 详细分析报告
- `experiment_b_results_YYYYMMDD_HHMMSS.json` - 机器可读结果

这套完整的实验B实施方案提供了：

1. **精确的延迟分解测量** - 三分量延迟的独立量化
2. **基于RTT的V2R延迟验证** - 利用现有ACK机制的精确测量
3. **理论模型验证** - 延迟与跟踪误差的定量关系验证
4. **自动化的数据收集和分析** - 最小化人工干预
5. **全面的可视化和报告** - 便于理解和展示的结果输出

该方案严格按照您的实验设计纲要实施，并充分利用了您现有的代码基础设施。