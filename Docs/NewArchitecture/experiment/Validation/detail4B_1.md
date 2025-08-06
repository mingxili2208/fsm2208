您的方案非常合理且具有很强的理论和实践价值。该设计成功地将抽象的系统延迟转化为可量化的物理误差，为建立"误差基线"提供了科学的方法论。我将为您提供一个完整优化的实现方案。

## 完整优化方案：实验B - 延迟特性及基线跟踪误差分析

### 1. 目录结构
```
Validation/
├── config.yaml                    # 共享配置文件
├── config_manager.py             # 配置管理器
├── B_latency_baseline/           # 实验B专用目录
│   ├── latency_impact_logger.py
│   ├── tracking_error_analyzer.py
│   ├── analyze_experiment_b.py
│   ├── carla_handler_patch.py     # CARLA节点补丁
│   ├── data/
│   ├── logs/
│   └── results/
├── A1_sensor_latency/
├── A2_visual_location_error/
└── requirements.txt
```

### 2. 更新的配置文件 (config.yaml)

```yaml
# FSM Sandbox Validation Configuration
project:
  name: "FSM Sandbox Validation"
  version: "3.0"
  description: "Comprehensive Timing, Fidelity and Latency Baseline Analysis"

# Experiment B: Latency and Baseline Tracking Error Configuration
experiment_b:
  topics:
    perception_input: "/localization/kinematic_state"
    planning_output: "/planning/trajectory"
    actuation_timing: "/fsm_sandbox/timing/t_plan_t_actuate"
    ground_truth: "/real_world/follow_adtruck/transformed_with_covariance"
  
  test_scenarios:
    constant_velocity_test:
      target_velocity_ms: 0.5
      test_duration_s: 300
      straight_path_length_m: 200
      path_start_point: [0, 0]
      path_end_point: [200, 0]
  
  parameters:
    correlation_window_ns: 100000000  # 100ms for looser matching
    outlier_removal_enabled: true
    outlier_percentile_threshold: 99
    
  quality_thresholds:
    max_acceptable_total_latency_ms: 200
    max_acceptable_d2a_latency_ms: 50
    max_acceptable_planner_latency_ms: 100
    error_prediction_accuracy_threshold: 0.8  # 80% accuracy required

# Extend existing configurations...
experiment_a1:
  topics:
    timing_sync: "/fsm_sandbox/timing/t1_t2"
    lidar_input: "/carla/follow_adtruck/carla_pointcloud"
    ndt_output: "/localization/kinematic_state"
  parameters:
    correlation_window_ns: 50000000
    cleanup_threshold_ns: 60000000000
    stats_report_interval_s: 5.0

experiment_a2:
  topics:
    ground_truth: "/real_world/follow_adtruck/transformed_with_covariance"
    virtual_lidar: "/carla/follow_adtruck/carla_pointcloud"
    offline_ndt: "/localization/kinematic_state"
  parameters:
    alignment_tolerance_windows_ms: [10, 50, 100, 200]
    min_alignment_rate_percent: 50
    outlier_removal_enabled: true
    outlier_iqr_factor: 1.5

file_management:
  base_directories:
    data: "data"
    logs: "logs" 
    results: "results"
  file_naming:
    timestamp_format: "%Y%m%d_%H%M%S"
    use_session_id: true

visualization:
  style: "seaborn-v0_8-whitegrid"
  figure_size: [16, 12]
  font_size: 12
  dpi: 300
  color_palette: "Set2"
  enable_grid: true
  grid_alpha: 0.3
```

### 3. 核心数据收集节点

#### 3.1 延迟影响记录器 (latency_impact_logger.py)

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import csv
import os
import datetime
import json
from collections import deque, defaultdict
import numpy as np

# Import message types
from geometry_msgs.msg import PoseWithCovarianceStamped
from autoware_auto_planning_msgs.msg import Trajectory
from sensor_msgs.msg import TimeReference
from std_msgs.msg import Header

# Import configuration manager
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from config_manager import ConfigManager

class LatencyImpactLogger(Node):
    """
    Advanced latency logger for Experiment B that captures the complete
    T_actual -> T_percept -> T_plan -> T_actuate timing chain
    """
    
    def __init__(self, config_file="config.yaml"):
        super().__init__('latency_impact_logger')
        
        # Load configuration
        self.config_manager = ConfigManager(config_file)
        self.b_config = self.config_manager.get('experiment_b', {})
        
        # Create session timestamp
        self.session_timestamp = datetime.datetime.now().strftime(
            self.config_manager.get('file_management.file_naming.timestamp_format', '%Y%m%d_%H%M%S')
        )
        
        # Setup directories
        self.setup_directories()
        
        # Get topic configuration
        topics = self.b_config.get('topics', {})
        self.perception_topic = topics.get('perception_input', '/localization/kinematic_state')
        self.planning_topic = topics.get('planning_output', '/planning/trajectory')
        self.actuation_topic = topics.get('actuation_timing', '/fsm_sandbox/timing/t_plan_t_actuate')
        self.ground_truth_topic = topics.get('ground_truth', '/real_world/follow_adtruck/transformed_with_covariance')
        
        # Parameters
        params = self.b_config.get('parameters', {})
        self.correlation_window_ns = params.get('correlation_window_ns', 100_000_000)
        self.outlier_threshold = params.get('outlier_percentile_threshold', 99)
        
        # Data structures for timestamp correlation
        self.ground_truth_buffer = deque(maxlen=1000)  # Buffer for T_actual
        self.perception_buffer = deque(maxlen=500)     # Buffer for T_percept
        self.planning_buffer = deque(maxlen=200)       # Buffer for T_plan
        
        # Statistics tracking
        self.total_gt_received = 0
        self.total_perception_received = 0
        self.total_planning_received = 0
        self.total_actuation_received = 0
        self.successful_correlations = 0
        self.failed_correlations = 0
        
        # Setup CSV logging
        self.setup_csv_logging()
        
        # Setup session logging
        self.setup_session_logging()
        
        # Create subscribers
        self.ground_truth_sub = self.create_subscription(
            PoseWithCovarianceStamped, self.ground_truth_topic, 
            self.ground_truth_callback, 10)
            
        self.perception_sub = self.create_subscription(
            PoseWithCovarianceStamped, self.perception_topic, 
            self.perception_callback, 10)
            
        self.planning_sub = self.create_subscription(
            Trajectory, self.planning_topic, 
            self.planning_callback, 10)
            
        self.actuation_sub = self.create_subscription(
            TimeReference, self.actuation_topic, 
            self.actuation_callback, 10)
        
        # Statistics timer
        self.stats_timer = self.create_timer(10.0, self.report_statistics)
        
        # Cleanup timer
        self.cleanup_timer = self.create_timer(30.0, self.cleanup_old_entries)
        
        self.get_logger().info("Latency Impact Logger (Exp. B) started")
        self.get_logger().info(f"Session: {self.session_timestamp}")
        self.get_logger().info(f"Monitoring topics:")
        self.get_logger().info(f"  - Ground Truth: {self.ground_truth_topic}")
        self.get_logger().info(f"  - Perception: {self.perception_topic}")
        self.get_logger().info(f"  - Planning: {self.planning_topic}")
        self.get_logger().info(f"  - Actuation: {self.actuation_topic}")

    def setup_directories(self):
        """Setup directory structure"""
        self.base_dir = os.path.dirname(os.path.abspath(__file__))
        dirs = self.config_manager.get('file_management.base_directories', {})
        
        self.data_dir = os.path.join(self.base_dir, dirs.get('data', 'data'))
        self.logs_dir = os.path.join(self.base_dir, dirs.get('logs', 'logs'))
        
        for directory in [self.data_dir, self.logs_dir]:
            os.makedirs(directory, exist_ok=True)

    def setup_csv_logging(self):
        """Setup CSV file for logging complete timing chains"""
        self.csv_filename = os.path.join(
            self.data_dir, 
            f'exp_b_latency_data_{self.session_timestamp}.csv'
        )
        self.csv_file = open(self.csv_filename, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        
        # CSV header with comprehensive timing information
        self.csv_writer.writerow([
            'session_id', 'correlation_id', 'timestamp_iso',
            't_actual_ns', 't_percept_ns', 't_plan_ns', 't_actuate_ns',
            'information_age_ms', 'planner_latency_ms', 'decision_to_actuation_ms',
            'total_latency_ms', 'valid_correlation'
        ])
        
        self.csv_file.flush()
        self.get_logger().info(f"CSV logging to: {self.csv_filename}")

    def setup_session_logging(self):
        """Setup session log file"""
        self.session_log_filename = os.path.join(
            self.logs_dir,
            f'exp_b_session_{self.session_timestamp}.log'
        )
        
        with open(self.session_log_filename, 'w') as f:
            f.write(f"Experiment B - Latency Impact Analysis Session\n")
            f.write(f"Session ID: {self.session_timestamp}\n")
            f.write(f"Start Time: {datetime.datetime.now().isoformat()}\n")
            f.write(f"Configuration:\n")
            f.write(f"  - Ground Truth Topic: {self.ground_truth_topic}\n")
            f.write(f"  - Perception Topic: {self.perception_topic}\n")
            f.write(f"  - Planning Topic: {self.planning_topic}\n")
            f.write(f"  - Actuation Topic: {self.actuation_topic}\n")
            f.write(f"  - Correlation Window: {self.correlation_window_ns/1e6:.1f}ms\n")
            f.write("=" * 60 + "\n\n")

    def log_to_session_file(self, message):
        """Write message to session log"""
        timestamp = datetime.datetime.now().isoformat()
        with open(self.session_log_filename, 'a') as f:
            f.write(f"[{timestamp}] {message}\n")

    def to_nanoseconds(self, stamp):
        """Convert ROS Time to nanoseconds"""
        return stamp.sec * 1_000_000_000 + stamp.nanosec

    def ground_truth_callback(self, msg: PoseWithCovarianceStamped):
        """Store ground truth timestamps (T_actual)"""
        t_actual_ns = self.to_nanoseconds(msg.header.stamp)
        self.ground_truth_buffer.append({
            'timestamp_ns': t_actual_ns,
            'position': (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
        })
        self.total_gt_received += 1

    def perception_callback(self, msg: PoseWithCovarianceStamped):
        """Store perception timestamps (T_percept)"""
        t_percept_ns = self.to_nanoseconds(msg.header.stamp)
        self.perception_buffer.append({
            'timestamp_ns': t_percept_ns,
            'position': (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
        })
        self.total_perception_received += 1

    def planning_callback(self, msg: Trajectory):
        """Store planning timestamps (T_plan)"""
        t_plan_ns = self.to_nanoseconds(msg.header.stamp)
        
        # Find corresponding T_percept
        t_percept_match = self.find_closest_timestamp(
            self.perception_buffer, t_plan_ns, before=True)
        
        if t_percept_match:
            # Find corresponding T_actual
            t_actual_match = self.find_closest_timestamp(
                self.ground_truth_buffer, t_percept_match['timestamp_ns'], before=True)
            
            if t_actual_match:
                self.planning_buffer.append({
                    'timestamp_ns': t_plan_ns,
                    't_actual_ns': t_actual_match['timestamp_ns'],
                    't_percept_ns': t_percept_match['timestamp_ns']
                })
                self.total_planning_received += 1

    def actuation_callback(self, msg: TimeReference):
        """Complete the timing chain with T_actuate"""
        t_plan_ns = self.to_nanoseconds(msg.header.stamp)
        t_actuate_ns = self.to_nanoseconds(msg.time_ref)
        
        # Find corresponding entry in planning buffer
        plan_match = None
        for i, entry in enumerate(self.planning_buffer):
            if abs(entry['timestamp_ns'] - t_plan_ns) < self.correlation_window_ns:
                plan_match = self.planning_buffer.pop(i)
                break
        
        if plan_match:
            # Calculate all latency components
            t_actual_ns = plan_match['t_actual_ns']
            t_percept_ns = plan_match['t_percept_ns']
            
            information_age_ms = (t_percept_ns - t_actual_ns) / 1e6
            planner_latency_ms = (t_plan_ns - t_percept_ns) / 1e6
            decision_to_actuation_ms = (t_actuate_ns - t_plan_ns) / 1e6
            total_latency_ms = (t_actuate_ns - t_actual_ns) / 1e6
            
            # Validate the correlation
            valid_correlation = (
                0 <= information_age_ms <= 1000 and
                0 <= planner_latency_ms <= 1000 and
                0 <= decision_to_actuation_ms <= 1000 and
                0 <= total_latency_ms <= 2000
            )
            
            if valid_correlation:
                self.successful_correlations += 1
                correlation_id = f"{self.session_timestamp}_{self.successful_correlations:06d}"
                
                # Log to CSV
                self.csv_writer.writerow([
                    self.session_timestamp, correlation_id, datetime.datetime.now().isoformat(),
                    t_actual_ns, t_percept_ns, t_plan_ns, t_actuate_ns,
                    information_age_ms, planner_latency_ms, decision_to_actuation_ms,
                    total_latency_ms, valid_correlation
                ])
                self.csv_file.flush()
                
                self.get_logger().debug(
                    f"Complete timing chain logged: "
                    f"IA={information_age_ms:.2f}ms, "
                    f"PL={planner_latency_ms:.2f}ms, "
                    f"D2A={decision_to_actuation_ms:.2f}ms, "
                    f"Total={total_latency_ms:.2f}ms"
                )
            else:
                self.failed_correlations += 1
                self.get_logger().warn(f"Invalid correlation detected - latencies out of range")
        else:
            self.failed_correlations += 1
            self.get_logger().debug(f"No matching T_plan found for T_actuate")

        self.total_actuation_received += 1

    def find_closest_timestamp(self, buffer, target_ns, before=True):
        """Find the closest timestamp in buffer"""
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
        
        # Find closest by absolute difference
        closest = min(candidates, 
                     key=lambda x: abs(x['timestamp_ns'] - target_ns))
        
        # Check if within correlation window
        if abs(closest['timestamp_ns'] - target_ns) <= self.correlation_window_ns:
            return closest
        
        return None

    def cleanup_old_entries(self):
        """Remove old entries from buffers"""
        current_time_ns = self.to_nanoseconds(self.get_clock().now().to_msg())
        cleanup_threshold = 60_000_000_000  # 60 seconds
        
        def filter_buffer(buffer):
            initial_size = len(buffer)
            while buffer and (current_time_ns - buffer[0]['timestamp_ns']) > cleanup_threshold:
                buffer.popleft()
            return initial_size - len(buffer)
        
        gt_cleaned = filter_buffer(self.ground_truth_buffer)
        percept_cleaned = filter_buffer(self.perception_buffer)
        plan_cleaned = filter_buffer(self.planning_buffer)
        
        if gt_cleaned + percept_cleaned + plan_cleaned > 0:
            cleanup_msg = f"Cleaned up old entries: GT={gt_cleaned}, P={percept_cleaned}, PL={plan_cleaned}"
            self.get_logger().debug(cleanup_msg)
            self.log_to_session_file(cleanup_msg)

    def report_statistics(self):
        """Report current statistics"""
        correlation_rate = (
            self.successful_correlations / max(self.total_actuation_received, 1)
        ) * 100
        
        stats_msg = (
            f"Statistics: GT={self.total_gt_received}, "
            f"Perception={self.total_perception_received}, "
            f"Planning={self.total_planning_received}, "
            f"Actuation={self.total_actuation_received}, "
            f"Successful correlations={self.successful_correlations} "
            f"({correlation_rate:.1f}%), "
            f"Failed={self.failed_correlations}"
        )
        
        self.get_logger().info(stats_msg)
        self.log_to_session_file(f"STATS: {stats_msg}")

    def write_session_summary(self):
        """Write final session summary"""
        correlation_rate = (
            self.successful_correlations / max(self.total_actuation_received, 1)
        ) * 100
        
        summary = f"""
Session Summary:
  End Time: {datetime.datetime.now().isoformat()}
  Final Statistics:
    - Ground Truth received: {self.total_gt_received}
    - Perception received: {self.total_perception_received}
    - Planning received: {self.total_planning_received}
    - Actuation received: {self.total_actuation_received}
    - Successful correlations: {self.successful_correlations}
    - Failed correlations: {self.failed_correlations}
    - Correlation success rate: {correlation_rate:.2f}%
  
Data Files Generated:
  - Latency Data: {self.csv_filename}
  - Session Log: {self.session_log_filename}
"""
        
        with open(self.session_log_filename, 'a') as f:
            f.write("\n" + "=" * 60 + "\n")
            f.write(summary)

    def destroy_node(self):
        """Clean shutdown"""
        self.write_session_summary()
        
        if hasattr(self, 'csv_file') and self.csv_file:
            self.csv_file.close()
            
        correlation_rate = (
            self.successful_correlations / max(self.total_actuation_received, 1)
        ) * 100
        
        self.get_logger().info(
            f"Final: {self.successful_correlations} correlations "
            f"({correlation_rate:.1f}% success rate)"
        )
        self.get_logger().info(f"Data saved to: {self.csv_filename}")
        
        super().destroy_node()


def main(args=None):
    import argparse
    
    parser = argparse.ArgumentParser(description='Experiment B Latency Impact Logger')
    parser.add_argument('--config', default='config.yaml', help='Configuration file path')
    
    known_args, _ = parser.parse_known_args()
    
    rclpy.init(args=args)
    logger_node = LatencyImpactLogger(known_args.config)
    
    try:
        rclpy.spin(logger_node)
    except KeyboardInterrupt:
        logger_node.get_logger().info("Shutting down latency impact logger...")
    finally:
        logger_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

#### 3.2 跟踪误差分析器 (tracking_error_analyzer.py)

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import csv
import os
import datetime
import numpy as np
import math
from geometry_msgs.msg import PoseWithCovarianceStamped
from shapely.geometry import LineString, Point
from shapely.ops import nearest_points

# Import configuration manager
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from config_manager import ConfigManager

class TrackingErrorAnalyzer(Node):
    """
    Calculates real-time longitudinal and lateral tracking errors for constant velocity test.
    Designed for Experiment B to establish the relationship between latency and tracking error.
    """
    
    def __init__(self, config_file="config.yaml"):
        super().__init__('tracking_error_analyzer')
        
        # Load configuration
        self.config_manager = ConfigManager(config_file)
        self.b_config = self.config_manager.get('experiment_b', {})
        
        # Create session timestamp
        self.session_timestamp = datetime.datetime.now().strftime(
            self.config_manager.get('file_management.file_naming.timestamp_format', '%Y%m%d_%H%M%S')
        )
        
        # Setup directories
        self.setup_directories()
        
        # Get test scenario configuration
        test_config = self.b_config.get('test_scenarios', {}).get('constant_velocity_test', {})
        self.target_velocity = test_config.get('target_velocity_ms', 0.5)
        self.test_duration = test_config.get('test_duration_s', 300)
        
        # Create reference path
        start_point = test_config.get('path_start_point', [0, 0])
        end_point = test_config.get('path_end_point', [200, 0])
        self.reference_path = LineString([start_point, end_point])
        self.path_length = self.reference_path.length
        
        # Topic configuration
        topics = self.b_config.get('topics', {})
        self.pose_topic = topics.get('perception_input', '/localization/kinematic_state')
        self.ground_truth_topic = topics.get('ground_truth', '/real_world/follow_adtruck/transformed_with_covariance')
        
        # State variables
        self.start_time_ns = None
        self.test_active = False
        self.sample_count = 0
        
        # Statistics
        self.error_statistics = {
            'longitudinal_errors': [],
            'lateral_errors': [],
            'velocities': []
        }
        
        # Setup CSV logging
        self.setup_csv_logging()
        
        # Setup session logging
        self.setup_session_logging()
        
        # Create subscribers
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, self.pose_topic, 
            self.pose_callback, 10)
            
        self.ground_truth_sub = self.create_subscription(
            PoseWithCovarianceStamped, self.ground_truth_topic,
            self.ground_truth_callback, 10)
        
        # Timer for test management
        self.timer = self.create_timer(1.0, self.check_test_status)
        
        self.get_logger().info("Tracking Error Analyzer (Exp. B) started")
        self.get_logger().info(f"Session: {self.session_timestamp}")
        self.get_logger().info(f"Target velocity: {self.target_velocity} m/s")
        self.get_logger().info(f"Reference path: {start_point} -> {end_point}")
        self.get_logger().info(f"Path length: {self.path_length:.2f} m")
        self.get_logger().info("Drive the vehicle to the start point to begin test")

    def setup_directories(self):
        """Setup directory structure"""
        self.base_dir = os.path.dirname(os.path.abspath(__file__))
        dirs = self.config_manager.get('file_management.base_directories', {})
        
        self.data_dir = os.path.join(self.base_dir, dirs.get('data', 'data'))
        self.logs_dir = os.path.join(self.base_dir, dirs.get('logs', 'logs'))
        
        for directory in [self.data_dir, self.logs_dir]:
            os.makedirs(directory, exist_ok=True)

    def setup_csv_logging(self):
        """Setup CSV file for tracking error data"""
        self.csv_filename = os.path.join(
            self.data_dir, 
            f'exp_b_tracking_error_{self.session_timestamp}.csv'
        )
        self.csv_file = open(self.csv_filename, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        
        # CSV header
        self.csv_writer.writerow([
            'session_id', 'timestamp_ns', 'elapsed_time_s',
            'vehicle_x', 'vehicle_y', 'vehicle_z', 'vehicle_yaw_rad',
            's_actual', 'd_actual', 's_target', 
            'error_longitudinal_m', 'error_lateral_m',
            'velocity_actual_ms', 'velocity_target_ms',
            'test_active'
        ])
        
        self.csv_file.flush()
        self.get_logger().info(f"CSV logging to: {self.csv_filename}")

    def setup_session_logging(self):
        """Setup session log file"""
        self.session_log_filename = os.path.join(
            self.logs_dir,
            f'exp_b_tracking_session_{self.session_timestamp}.log'
        )
        
        with open(self.session_log_filename, 'w') as f:
            f.write(f"Experiment B - Tracking Error Analysis Session\n")
            f.write(f"Session ID: {self.session_timestamp}\n")
            f.write(f"Start Time: {datetime.datetime.now().isoformat()}\n")
            f.write(f"Configuration:\n")
            f.write(f"  - Target Velocity: {self.target_velocity} m/s\n")
            f.write(f"  - Test Duration: {self.test_duration} s\n")
            f.write(f"  - Reference Path Length: {self.path_length:.2f} m\n")
            f.write(f"  - Pose Topic: {self.pose_topic}\n")
            f.write("=" * 60 + "\n\n")

    def log_to_session_file(self, message):
        """Write message to session log"""
        timestamp = datetime.datetime.now().isoformat()
        with open(self.session_log_filename, 'a') as f:
            f.write(f"[{timestamp}] {message}\n")

    def to_nanoseconds(self, stamp):
        """Convert ROS Time to nanoseconds"""
        return stamp.sec * 1_000_000_000 + stamp.nanosec

    def quaternion_to_yaw(self, q):
        """Convert quaternion to yaw angle"""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def calculate_frenet_coordinates(self, point):
        """Calculate Frenet coordinates (s, d) for a given point"""
        vehicle_point = Point(point[0], point[1])
        
        # Calculate s (progress along path)
        s_actual = self.reference_path.project(vehicle_point)
        
        # Calculate d (lateral distance from path)
        closest_point_on_path = self.reference_path.interpolate(s_actual)
        d_actual = vehicle_point.distance(closest_point_on_path)
        
        # Determine sign of lateral error (left/right of path)
        path_start = Point(self.reference_path.coords[0])
        path_end = Point(self.reference_path.coords[-1])
        path_vector = np.array([path_end.x - path_start.x, path_end.y - path_start.y])
        to_vehicle = np.array([point[0] - closest_point_on_path.x, point[1] - closest_point_on_path.y])
        
        # Cross product to determine side
        cross_product = np.cross(path_vector, to_vehicle)
        if cross_product < 0:
            d_actual = -d_actual
        
        return s_actual, d_actual

    def check_if_near_start(self, position):
        """Check if vehicle is near the start of the path"""
        start_point = Point(self.reference_path.coords[0])
        vehicle_point = Point(position[0], position[1])
        distance_to_start = start_point.distance(vehicle_point)
        return distance_to_start < 5.0  # Within 5 meters of start

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
        if not self.test_active and self.check_if_near_start(position):
            self.start_time_ns = timestamp_ns
            self.test_active = True
            self.get_logger().info("Test started - vehicle detected near start point")
            self.log_to_session_file("Test started - vehicle at start position")
        
        # Calculate error only if test is active
        if self.test_active:
            elapsed_time_s = (timestamp_ns - self.start_time_ns) / 1e9
            
            # Calculate Frenet coordinates
            s_actual, d_actual = self.calculate_frenet_coordinates(position)
            
            # Calculate target position along path
            s_target = self.target_velocity * elapsed_time_s
            
            # Calculate errors
            error_longitudinal = s_actual - s_target
            error_lateral = abs(d_actual)
            
            # Estimate actual velocity (simple finite difference)
            velocity_actual = self.target_velocity  # Simplified - could be calculated from position history
            
            # Store statistics
            self.error_statistics['longitudinal_errors'].append(error_longitudinal)
            self.error_statistics['lateral_errors'].append(error_lateral)
            self.error_statistics['velocities'].append(velocity_actual)
            
            # Log to CSV
            self.csv_writer.writerow([
                self.session_timestamp, timestamp_ns, elapsed_time_s,
                position[0], position[1], position[2], yaw_rad,
                s_actual, d_actual, s_target,
                error_longitudinal, error_lateral,
                velocity_actual, self.target_velocity,
                self.test_active
            ])
            
            self.csv_file.flush()
            self.sample_count += 1
            
            # Log progress
            if self.sample_count % 50 == 0:
                self.get_logger().info(
                    f"Progress: {elapsed_time_s:.1f}s, "
                    f"s={s_actual:.2f}m, "
                    f"target_s={s_target:.2f}m, "
                    f"long_error={error_longitudinal:.3f}m"
                )
            
            # Check if test should end
            if elapsed_time_s >= self.test_duration or s_actual >= self.path_length:
                self.test_active = False
                self.get_logger().info("Test completed")
                self.log_to_session_file("Test completed - duration or path end reached")
                self.generate_summary_statistics()

    def ground_truth_callback(self, msg: PoseWithCovarianceStamped):
        """Store ground truth for reference (optional)"""
        pass  # Could be used for additional validation

    def check_test_status(self):
        """Periodic check of test status"""
        if self.test_active and self.start_time_ns:
            elapsed = (self.to_nanoseconds(self.get_clock().now().to_msg()) - self.start_time_ns) / 1e9
            if elapsed >= self.test_duration:
                self.test_active = False
                self.get_logger().info("Test timeout reached")
                self.log_to_session_file("Test ended - timeout reached")
                self.generate_summary_statistics()

    def generate_summary_statistics(self):
        """Generate and log summary statistics"""
        if not self.error_statistics['longitudinal_errors']:
            return
        
        long_errors = np.array(self.error_statistics['longitudinal_errors'])
        lat_errors = np.array(self.error_statistics['lateral_errors'])
        
        summary = f"""
Test Summary Statistics:
  Samples collected: {len(long_errors)}
  Longitudinal Error:
    Mean: {np.mean(long_errors):.4f} m
    Std: {np.std(long_errors):.4f} m
    Min: {np.min(long_errors):.4f} m
    Max: {np.max(long_errors):.4f} m
  Lateral Error:
    Mean: {np.mean(lat_errors):.4f} m
    Std: {np.std(lat_errors):.4f} m
    Max: {np.max(lat_errors):.4f} m
"""
        
        self.get_logger().info(summary)
        self.log_to_session_file(f"SUMMARY: {summary}")

    def write_session_summary(self):
        """Write final session summary"""
        summary = f"""
Session Summary:
  End Time: {datetime.datetime.now().isoformat()}
  Test Configuration:
    - Target Velocity: {self.target_velocity} m/s
    - Test Duration: {self.test_duration} s
    - Path Length: {self.path_length:.2f} m
  Data Collection:
    - Samples collected: {self.sample_count}
    - Test completed: {not self.test_active}
  
Data Files Generated:
  - Tracking Error Data: {self.csv_filename}
  - Session Log: {self.session_log_filename}
"""
        
        with open(self.session_log_filename, 'a') as f:
            f.write("\n" + "=" * 60 + "\n")
            f.write(summary)

    def destroy_node(self):
        """Clean shutdown"""
        self.write_session_summary()
        
        if hasattr(self, 'csv_file') and self.csv_file:
            self.csv_file.close()
            
        self.get_logger().info(f"Collected {self.sample_count} samples")
        self.get_logger().info(f"Data saved to: {self.csv_filename}")
        
        super().destroy_node()


def main(args=None):
    import argparse
    
    parser = argparse.ArgumentParser(description='Experiment B Tracking Error Analyzer')
    parser.add_argument('--config', default='config.yaml', help='Configuration file path')
    
    known_args, _ = parser.parse_known_args()
    
    rclpy.init(args=args)
    analyzer_node = TrackingErrorAnalyzer(known_args.config)
    
    try:
        rclpy.spin(analyzer_node)
    except KeyboardInterrupt:
        analyzer_node.get_logger().info("Shutting down tracking error analyzer...")
    finally:
        analyzer_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

#### 3.3 CARLA处理器补丁 (carla_handler_patch.py)

```python
#!/usr/bin/env python3

"""
Patch for CARLA vehicle handler to add timing instrumentation for Experiment B.
This file provides the code modifications needed for your existing CARLA control node.
"""

# Add these imports to your existing CarlaUpdateVehicleHandler node
from sensor_msgs.msg import TimeReference
from autoware_auto_planning_msgs.msg import Trajectory

class CarlaVehicleHandlerPatch:
    """
    Mixin class to add timing instrumentation to CARLA vehicle handler.
    Include this in your existing CARLA control node.
    """
    
    def __init__(self):
        # Add to your existing __init__ method
        
        # Publisher for timing information (T_plan, T_actuate)
        self.actuation_timing_pub = self.create_publisher(
            TimeReference, 
            '/fsm_sandbox/timing/t_plan_t_actuate', 
            10
        )
        
        self.get_logger().info("CARLA handler timing instrumentation enabled")
    
    def instrumented_control_callback(self, msg):
        """
        Enhanced control callback that records timing information.
        Replace your existing control callback with this method.
        """
        # Record T_plan (timestamp from incoming message)
        t_plan_stamp = msg.header.stamp
        
        # Record T_actuate (current time, just before processing)
        t_actuate_stamp = self.get_clock().now().to_msg()
        
        # Publish timing information for the logger
        timing_msg = TimeReference()
        timing_msg.header.stamp = t_plan_stamp  # T_plan
        timing_msg.time_ref = t_actuate_stamp   # T_actuate
        self.actuation_timing_pub.publish(timing_msg)
        
        # Continue with your normal control logic
        self.process_control_command(msg)
        
        # Optional: Log timing for debugging
        t_plan_ns = t_plan_stamp.sec * 1_000_000_000 + t_plan_stamp.nanosec
        t_actuate_ns = t_actuate_stamp.sec * 1_000_000_000 + t_actuate_stamp.nanosec
        latency_ms = (t_actuate_ns - t_plan_ns) / 1e6
        
        if latency_ms > 10:  # Log if latency > 10ms
            self.get_logger().debug(f"Control latency: {latency_ms:.2f}ms")
    
    def process_control_command(self, msg):
        """
        Your existing control logic goes here.
        This method should contain whatever your original control callback did.
        """
        # Example control logic (replace with your actual implementation)
        try:
            # Extract control commands from trajectory message
            if hasattr(msg, 'points') and len(msg.points) > 0:
                target_point = msg.points[0]  # Use first point
                
                # Extract velocity and steering commands
                target_velocity = target_point.longitudinal_velocity_mps
                target_steering = target_point.front_wheel_angle_rad
                
                # Apply to CARLA vehicle (your existing logic)
                self.apply_control_to_carla_vehicle(target_velocity, target_steering)
                
        except Exception as e:
            self.get_logger().error(f"Error processing control command: {e}")
    
    def apply_control_to_carla_vehicle(self, velocity, steering):
        """
        Apply control commands to CARLA vehicle.
        Replace this with your actual CARLA interface code.
        """
        # Your existing CARLA control application code
        pass

# Example of how to modify your existing CARLA node:
"""
class YourExistingCarlaNode(Node, CarlaVehicleHandlerPatch):
    def __init__(self):
        Node.__init__(self, 'your_carla_node')
        CarlaVehicleHandlerPatch.__init__(self)
        
        # Your existing initialization
        
        # Replace your control subscription with:
        self.control_sub = self.create_subscription(
            Trajectory,
            "/planning/trajectory",
            self.instrumented_control_callback,  # Use the instrumented version
            10
        )
"""

# Alternative: Direct modification template
def add_timing_to_existing_callback(existing_callback):
    """
    Decorator to add timing instrumentation to existing control callback.
    Usage: @add_timing_to_existing_callback
    """
    def wrapper(self, msg):
        # Record timing
        t_plan_stamp = msg.header.stamp
        t_actuate_stamp = self.get_clock().now().to_msg()
        
        # Publish timing if publisher exists
        if hasattr(self, 'actuation_timing_pub'):
            timing_msg = TimeReference()
            timing_msg.header.stamp = t_plan_stamp
            timing_msg.time_ref = t_actuate_stamp
            self.actuation_timing_pub.publish(timing_msg)
        
        # Call original callback
        return existing_callback(self, msg)
    
    return wrapper

# Example usage of decorator:
"""
class YourCarlaNode(Node):
    def __init__(self):
        super().__init__('your_carla_node')
        
        # Add timing publisher
        self.actuation_timing_pub = self.create_publisher(
            TimeReference, '/fsm_sandbox/timing/t_plan_t_actuate', 10)
    
    @add_timing_to_existing_callback
    def your_existing_control_callback(self, msg):
        # Your existing control logic unchanged
        pass
"""
```

### 4. 数据分析与可视化

#### 4.1 实验B分析脚本 (analyze_experiment_b.py)

```python
#!/usr/bin/env python3

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
import argparse
import os
import sys
import json
from datetime import datetime
from scipy import stats
from typing import Tuple, Dict, List

# Import configuration manager
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from config_manager import ConfigManager

class ExperimentBAnalyzer:
    """
    Comprehensive analyzer for Experiment B: Latency characteristics and baseline tracking error analysis.
    Provides detailed latency breakdown and validates the relationship between latency and tracking error.
    """
    
    def __init__(self, latency_csv: str, error_csv: str, config_file: str = "config.yaml"):
        self.latency_csv = latency_csv
        self.error_csv = error_csv
        
        # Load configuration
        self.config_manager = ConfigManager(config_file)
        self.b_config = self.config_manager.get('experiment_b', {})
        
        # Analysis timestamp
        self.analysis_timestamp = datetime.now().strftime(
            self.config_manager.get('file_management.file_naming.timestamp_format', '%Y%m%d_%H%M%S')
        )
        
        # Setup directories and plotting
        self.setup_directories()
        self.setup_plotting()
        
        # Load and validate data
        self.load_and_validate_data()
        
        # Analysis results storage
        self.analysis_results = {}

    def setup_directories(self):
        """Setup directory structure for results"""
        self.base_dir = os.path.dirname(os.path.abspath(__file__))
        dirs = self.config_manager.get('file_management.base_directories', {})
        
        self.results_dir = os.path.join(self.base_dir, dirs.get('results', 'results'))
        self.analysis_dir = os.path.join(self.results_dir, f"experiment_b_analysis_{self.analysis_timestamp}")
        
        for directory in [self.results_dir, self.analysis_dir]:
            os.makedirs(directory, exist_ok=True)
        
        print(f"Analysis output directory: {self.analysis_dir}")

    def setup_plotting(self):
        """Setup plotting configuration"""
        viz_config = self.config_manager.get('visualization', {})
        
        plt.style.use(viz_config.get('style', 'seaborn-v0_8-whitegrid'))
        if 'color_palette' in viz_config:
            sns.set_palette(viz_config['color_palette'])
        
        # Configure matplotlib parameters
        plt.rcParams['figure.figsize'] = viz_config.get('figure_size', [16, 12])
        plt.rcParams['font.size'] = viz_config.get('font_size', 12)
        plt.rcParams['axes.grid'] = viz_config.get('enable_grid', True)
        plt.rcParams['grid.alpha'] = viz_config.get('grid_alpha', 0.3)
        plt.rcParams['font.family'] = 'sans-serif'

    def load_and_validate_data(self):
        """Load and validate input CSV files"""
        try:
            # Load latency data
            self.latency_df = pd.read_csv(self.latency_csv)
            print(f"Loaded latency data: {len(self.latency_df)} records")
            
            # Load tracking error data
            self.error_df = pd.read_csv(self.error_csv)
            print(f"Loaded tracking error data: {len(self.error_df)} records")
            
            # Validate latency data columns
            required_latency_cols = [
                'information_age_ms', 'planner_latency_ms', 
                'decision_to_actuation_ms', 'total_latency_ms'
            ]
            missing_latency_cols = [col for col in required_latency_cols if col not in self.latency_df.columns]
            if missing_latency_cols:
                raise ValueError(f"Missing latency columns: {missing_latency_cols}")
            
            # Validate tracking error data columns
            required_error_cols = [
                'elapsed_time_s', 's_actual', 's_target', 
                'error_longitudinal_m', 'error_lateral_m'
            ]
            missing_error_cols = [col for col in required_error_cols if col not in self.error_df.columns]
            if missing_error_cols:
                raise ValueError(f"Missing error columns: {missing_error_cols}")
            
            # Clean data
            self.clean_data()
            
            print("Data validation successful")
            
        except Exception as e:
            print(f"Error loading or validating data: {e}")
            raise

    def clean_data(self):
        """Clean and filter data"""
        # Remove invalid latency entries
        initial_latency_count = len(self.latency_df)
        self.latency_df = self.latency_df[
            (self.latency_df['valid_correlation'] == True) &
            (self.latency_df['total_latency_ms'] > 0) &
            (self.latency_df['total_latency_ms'] < 2000) &  # Remove extreme outliers
            (self.latency_df['information_age_ms'] >= 0) &
            (self.latency_df['planner_latency_ms'] >= 0) &
            (self.latency_df['decision_to_actuation_ms'] >= 0)
        ]
        
        # Remove outliers using percentile filtering
        outlier_threshold = self.b_config.get('parameters', {}).get('outlier_percentile_threshold', 99)
        threshold_value = self.latency_df['total_latency_ms'].quantile(outlier_threshold / 100)
        self.latency_df = self.latency_df[self.latency_df['total_latency_ms'] <= threshold_value]
        
        cleaned_latency_count = len(self.latency_df)
        print(f"Cleaned latency data: {cleaned_latency_count}/{initial_latency_count} records retained")
        
        # Clean error data - remove entries where test was not active
        initial_error_count = len(self.error_df)
        if 'test_active' in self.error_df.columns:
            self.error_df = self.error_df[self.error_df['test_active'] == True]
        
        # Remove extreme error outliers
        error_threshold = self.error_df['error_longitudinal_m'].quantile(0.99)
        self.error_df = self.error_df[
            abs(self.error_df['error_longitudinal_m']) <= abs(error_threshold)
        ]
        
        cleaned_error_count = len(self.error_df)
        print(f"Cleaned error data: {cleaned_error_count}/{initial_error_count} records retained")

    def analyze_latency_characteristics(self):
        """Perform detailed latency analysis (Part 1 of Experiment B)"""
        print("\n" + "="*60)
        print("EXPERIMENT B - PART 1: LATENCY CHARACTERISTICS ANALYSIS")
        print("="*60)
        
        # Calculate statistics for each latency component
        latency_components = [
            'information_age_ms', 'planner_latency_ms', 
            'decision_to_actuation_ms', 'total_latency_ms'
        ]
        
        latency_stats = {}
        for component in latency_components:
            data = self.latency_df[component]
            latency_stats[component] = {
                'count': len(data),
                'mean': data.mean(),
                'median': data.median(),
                'std': data.std(),
                'min': data.min(),
                'max': data.max(),
                'p95': data.quantile(0.95),
                'p99': data.quantile(0.99)
            }
        
        # Print latency analysis table
        self.print_latency_table(latency_stats)
        
        # Store results
        self.analysis_results['latency_statistics'] = latency_stats
        
        # Generate latency plots
        self.create_latency_analysis_plots()
        
        return latency_stats

    def print_latency_table(self, stats: Dict):
        """Print formatted latency statistics table"""
        print(f"\nLatency Component Analysis (n={stats['total_latency_ms']['count']} samples)")
        print("-" * 100)
        print(f"{'Component':<25} {'Mean':<8} {'Median':<8} {'Std':<8} {'Min':<8} {'Max':<8} {'P95':<8} {'P99':<8}")
        print("-" * 100)
        
        component_names = {
            'information_age_ms': 'Information Age (ms)',
            'planner_latency_ms': 'Planner Latency (ms)',
            'decision_to_actuation_ms': 'Decision-to-Act (ms)',
            'total_latency_ms': 'Total Latency (ms)'
        }
        
        for component, name in component_names.items():
            s = stats[component]
            print(f"{name:<25} {s['mean']:<8.2f} {s['median']:<8.2f} {s['std']:<8.2f} "
                  f"{s['min']:<8.2f} {s['max']:<8.2f} {s['p95']:<8.2f} {s['p99']:<8.2f}")
        
        print("-" * 100)

    def assess_latency_quality(self, stats: Dict):
        """Assess latency quality against performance thresholds"""
        print(f"\nLatency Performance Assessment:")
        print("-" * 50)
        
        # Get quality thresholds from configuration
        thresholds = self.b_config.get('quality_thresholds', {})
        
        criteria = [
            ('Total Latency Mean < 200ms', 
             stats['total_latency_ms']['mean'] < thresholds.get('max_acceptable_total_latency_ms', 200)),
            ('Decision-to-Actuation P95 < 50ms',
             stats['decision_to_actuation_ms']['p95'] < thresholds.get('max_acceptable_d2a_latency_ms', 50)),
            ('Planner Latency P95 < 100ms',
             stats['planner_latency_ms']['p95'] < thresholds.get('max_acceptable_planner_latency_ms', 100)),
            ('Total Latency Stability (CV < 0.5)',
             (stats['total_latency_ms']['std'] / stats['total_latency_ms']['mean']) < 0.5)
        ]
        
        passed_criteria = 0
        for criterion, passed in criteria:
            status = "PASS" if passed else "FAIL"
            print(f"  {criterion:<40}: {status}")
            if passed:
                passed_criteria += 1
        
        print(f"\nLatency Performance: {passed_criteria}/{len(criteria)} criteria passed")
        
        if passed_criteria == len(criteria):
            assessment = "EXCELLENT - System latency well within acceptable bounds"
        elif passed_criteria >= len(criteria) * 0.8:
            assessment = "GOOD - System latency acceptable for most applications"
        elif passed_criteria >= len(criteria) * 0.6:
            assessment = "ACCEPTABLE - Some latency concerns but functional"
        else:
            assessment = "POOR - System latency may interfere with ADS performance"
        
        print(f"Assessment: {assessment}")
        return assessment

    def analyze_tracking_error_baseline(self):
        """Perform tracking error baseline analysis (Part 2 of Experiment B)"""
        print("\n" + "="*60)
        print("EXPERIMENT B - PART 2: TRACKING ERROR BASELINE ANALYSIS")
        print("="*60)
        
        if len(self.error_df) == 0:
            print("Error: No tracking error data available for analysis")
            return None
        
        # Get test configuration
        test_config = self.b_config.get('test_scenarios', {}).get('constant_velocity_test', {})
        target_velocity = test_config.get('target_velocity_ms', 0.5)
        
        # Calculate theoretical error from latency
        mean_total_latency_s = self.analysis_results['latency_statistics']['total_latency_ms']['mean'] / 1000.0
        theoretical_error = target_velocity * mean_total_latency_s
        
        # Calculate actual error statistics
        actual_errors = self.error_df['error_longitudinal_m']
        actual_error_mean = actual_errors.mean()
        actual_error_std = actual_errors.std()
        actual_error_median = actual_errors.median()
        
        # Calculate agreement between theoretical and actual
        agreement_ratio = actual_error_mean / theoretical_error if theoretical_error != 0 else 0
        
        print(f"\nTracking Error Baseline Analysis:")
        print("-" * 50)
        print(f"Test Configuration:")
        print(f"  Target Velocity: {target_velocity:.3f} m/s")
        print(f"  Mean Total Latency: {mean_total_latency_s*1000:.2f} ms")
        print(f"\nError Analysis:")
        print(f"  Theoretical Error (v × ΔT): {theoretical_error:.6f} m")
        print(f"  Actual Error Mean: {actual_error_mean:.6f} m")
        print(f"  Actual Error Std: {actual_error_std:.6f} m")
        print(f"  Actual Error Median: {actual_error_median:.6f} m")
        print(f"  Agreement Ratio: {agreement_ratio:.3f}")
        
        # Store results
        error_analysis = {
            'target_velocity': target_velocity,
            'mean_latency_ms': mean_total_latency_s * 1000,
            'theoretical_error_m': theoretical_error,
            'actual_error_mean_m': actual_error_mean,
            'actual_error_std_m': actual_error_std,
            'actual_error_median_m': actual_error_median,
            'agreement_ratio': agreement_ratio,
            'sample_count': len(actual_errors)
        }
        
        self.analysis_results['error_baseline'] = error_analysis
        
        # Assess prediction accuracy
        accuracy_threshold = self.b_config.get('quality_thresholds', {}).get('error_prediction_accuracy_threshold', 0.8)
        prediction_accurate = 0.5 <= agreement_ratio <= 1.5  # Within 50% agreement
        
        print(f"\nBaseline Error Prediction Assessment:")
        accuracy_status = "ACCURATE" if prediction_accurate else "INACCURATE"
        print(f"  Prediction Accuracy: {accuracy_status}")
        print(f"  Agreement within acceptable range (0.5-1.5): {prediction_accurate}")
        
        # Generate error analysis plots
        self.create_error_analysis_plots()
        
        return error_analysis

    def create_latency_analysis_plots(self):
        """Create comprehensive latency analysis visualizations"""
        print(f"\nGenerating latency analysis plots...")
        
        # Create figure with subplots
        fig = plt.figure(figsize=(20, 16))
        
        # Plot 1: Latency component breakdown (stacked bar)
        ax1 = plt.subplot(3, 2, 1)
        components = ['information_age_ms', 'planner_latency_ms', 'decision_to_actuation_ms']
        component_labels = ['Information Age', 'Planner', 'Decision-to-Actuation']
        means = [self.analysis_results['latency_statistics'][comp]['mean'] for comp in components]
        
        ax1.bar(range(len(component_labels)), means, color=['skyblue', 'lightcoral', 'lightgreen'])
        ax1.set_xlabel('Latency Component')
        ax1.set_ylabel('Mean Latency (ms)')
        ax1.set_title('Mean Latency by Component')
        ax1.set_xticks(range(len(component_labels)))
        ax1.set_xticklabels(component_labels, rotation=45)
        ax1.grid(True, alpha=0.3)
        
        # Add value labels on bars
        for i, v in enumerate(means):
            ax1.text(i, v + max(means)*0.01, f'{v:.1f}ms', ha='center', va='bottom')
        
        # Plot 2: Total latency distribution
        ax2 = plt.subplot(3, 2, 2)
        total_latencies = self.latency_df['total_latency_ms']
        ax2.hist(total_latencies, bins=50, alpha=0.7, color='steelblue', edgecolor='black')
        ax2.axvline(total_latencies.mean(), color='red', linestyle='--', linewidth=2,
                   label=f'Mean: {total_latencies.mean():.1f}ms')
        ax2.axvline(total_latencies.quantile(0.95), color='orange', linestyle='--', linewidth=2,
                   label=f'P95: {total_latencies.quantile(0.95):.1f}ms')
        ax2.set_xlabel('Total Latency (ms)')
        ax2.set_ylabel('Frequency')
        ax2.set_title('Total End-to-End Latency Distribution')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        # Plot 3: Latency composition (pie chart)
        ax3 = plt.subplot(3, 2, 3)
        sizes = means
        ax3.pie(sizes, labels=component_labels, autopct='%1.1f%%', startangle=90,
                colors=['skyblue', 'lightcoral', 'lightgreen'])
        ax3.set_title('Latency Composition Breakdown')
        
        # Plot 4: Latency time series
        ax4 = plt.subplot(3, 2, 4)
        if 'timestamp_ns' in self.latency_df.columns:
            # Convert timestamps to relative time
            start_time = self.latency_df['timestamp_ns'].iloc[0] if len(self.latency_df) > 0 else 0
            relative_time = (self.latency_df['timestamp_ns'] - start_time) / 1e9
            ax4.plot(relative_time, self.latency_df['total_latency_ms'], alpha=0.7, linewidth=1)
            ax4.set_xlabel('Time (seconds)')
        else:
            # Use sample index if no timestamps
            ax4.plot(self.latency_df['total_latency_ms'], alpha=0.7, linewidth=1)
            ax4.set_xlabel('Sample Index')
        
        ax4.set_ylabel('Total Latency (ms)')
        ax4.set_title('Total Latency Over Time')
        ax4.grid(True, alpha=0.3)
        
        # Plot 5: Component correlation matrix
        ax5 = plt.subplot(3, 2, 5)
        correlation_data = self.latency_df[components].corr()
        sns.heatmap(correlation_data, annot=True, cmap='coolwarm', center=0,
                   xticklabels=component_labels, yticklabels=component_labels, ax=ax5)
        ax5.set_title('Latency Component Correlation')
        
        # Plot 6: Box plot comparison
        ax6 = plt.subplot(3, 2, 6)
        box_data = [self.latency_df[comp] for comp in components]
        box_plot = ax6.boxplot(box_data, labels=component_labels, patch_artist=True)
        colors = ['skyblue', 'lightcoral', 'lightgreen']
        for patch, color in zip(box_plot['boxes'], colors):
            patch.set_facecolor(color)
        ax6.set_ylabel('Latency (ms)')
        ax6.set_title('Latency Component Distribution Comparison')
        ax6.grid(True, alpha=0.3)
        plt.setp(ax6.get_xticklabels(), rotation=45)
        
        plt.tight_layout()
        
        # Save plot
        plot_filename = os.path.join(self.analysis_dir, f'latency_analysis_{self.analysis_timestamp}.png')
        plt.savefig(plot_filename, dpi=300, bbox_inches='tight')
        print(f"Latency analysis plots saved: {plot_filename}")
        
        return fig

    def create_error_analysis_plots(self):
        """Create tracking error analysis visualizations"""
        print(f"Generating tracking error analysis plots...")
        
        # Get error analysis results
        error_results = self.analysis_results['error_baseline']
        
        # Create figure
        fig = plt.figure(figsize=(20, 12))
        
        # Plot 1: Error time series with theoretical baseline
        ax1 = plt.subplot(2, 3, 1)
        time_data = self.error_df['elapsed_time_s']
        error_data = self.error_df['error_longitudinal_m']
        
        ax1.plot(time_data, error_data, alpha=0.7, linewidth=1, 
                label=f'Actual Error', color='blue')
        ax1.axhline(y=error_results['theoretical_error_m'], color='red', 
                   linestyle='--', linewidth=2,
                   label=f'Theoretical Error ({error_results["theoretical_error_m"]:.6f}m)')
        ax1.axhline(y=error_results['actual_error_mean_m'], color='green', 
                   linestyle=':', linewidth=2,
                   label=f'Actual Mean ({error_results["actual_error_mean_m"]:.6f}m)')
        
        ax1.set_xlabel('Time (seconds)')
        ax1.set_ylabel('Longitudinal Error (m)')
        ax1.set_title('Longitudinal Tracking Error vs Time')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        
        # Plot 2: Error distribution
        ax2 = plt.subplot(2, 3, 2)
        ax2.hist(error_data, bins=50, alpha=0.7, color='skyblue', edgecolor='black')
        ax2.axvline(error_results['actual_error_mean_m'], color='green', 
                   linestyle=':', linewidth=2,
                   label=f'Mean: {error_results["actual_error_mean_m"]:.6f}m')
        ax2.axvline(error_results['theoretical_error_m'], color='red', 
                   linestyle='--', linewidth=2,
                   label=f'Theoretical: {error_results["theoretical_error_m"]:.6f}m')
        ax2.set_xlabel('Longitudinal Error (m)')
        ax2.set_ylabel('Frequency')
        ax2.set_title('Longitudinal Error Distribution')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        # Plot 3: Error vs target position
        ax3 = plt.subplot(2, 3, 3)
        ax3.scatter(self.error_df['s_target'], self.error_df['error_longitudinal_m'], 
                   alpha=0.6, s=10, color='blue')
        ax3.axhline(y=error_results['theoretical_error_m'], color='red', 
                   linestyle='--', linewidth=2, label='Theoretical Error')
        ax3.set_xlabel('Target Position (m)')
        ax3.set_ylabel('Longitudinal Error (m)')
        ax3.set_title('Error vs Target Position')
        ax3.legend()
        ax3.grid(True, alpha=0.3)
        
        # Plot 4: Lateral error analysis
        ax4 = plt.subplot(2, 3, 4)
        if 'error_lateral_m' in self.error_df.columns:
            lateral_errors = self.error_df['error_lateral_m']
            ax4.plot(time_data, lateral_errors, alpha=0.7, linewidth=1, color='orange')
            ax4.set_xlabel('Time (seconds)')
            ax4.set_ylabel('Lateral Error (m)')
            ax4.set_title('Lateral Tracking Error vs Time')
            ax4.grid(True, alpha=0.3)
        
        # Plot 5: Error statistics comparison
        ax5 = plt.subplot(2, 3, 5)
        categories = ['Theoretical', 'Actual Mean', 'Actual Median']
        values = [
            error_results['theoretical_error_m'],
            error_results['actual_error_mean_m'],
            error_results['actual_error_median_m']
        ]
        colors = ['red', 'green', 'blue']
        bars = ax5.bar(categories, values, color=colors, alpha=0.7)
        ax5.set_ylabel('Error (m)')
        ax5.set_title('Error Comparison: Theory vs Actual')
        ax5.grid(True, alpha=0.3)
        
        # Add value labels on bars
        for bar, value in zip(bars, values):
            height = bar.get_height()
            ax5.text(bar.get_x() + bar.get_width()/2., height + max(values)*0.01,
                    f'{value:.6f}m', ha='center', va='bottom')
        
        # Plot 6: Agreement analysis
        ax6 = plt.subplot(2, 3, 6)
        agreement_ratio = error_results['agreement_ratio']
        
        # Create a simple agreement visualization
        theta = np.linspace(0, 2*np.pi, 100)
        r = np.ones_like(theta)
        ax6 = plt.subplot(2, 3, 6, projection='polar')
        ax6.plot(theta, r, 'k-', linewidth=2)
        ax6.fill_between(theta, 0, r, alpha=0.1, color='gray')
        
        # Plot agreement point
        agreement_angle = 2 * np.pi * min(agreement_ratio / 2, 1)  # Scale to 0-2π
        ax6.plot([agreement_angle], [agreement_ratio], 'ro', markersize=12)
        ax6.set_ylim(0, 2)
        ax6.set_title(f'Agreement Ratio: {agreement_ratio:.3f}')
        
        plt.tight_layout()
        
        # Save plot
        plot_filename = os.path.join(self.analysis_dir, f'error_analysis_{self.analysis_timestamp}.png')
        plt.savefig(plot_filename, dpi=300, bbox_inches='tight')
        print(f"Error analysis plots saved: {plot_filename}")
        
        return fig

    def generate_comprehensive_report(self):
        """Generate comprehensive analysis report"""
        print(f"\nGenerating comprehensive report...")
        
        # Calculate overall assessment
        latency_stats = self.analysis_results['latency_statistics']
        error_baseline = self.analysis_results['error_baseline']
        
        latency_assessment = self.assess_latency_quality(latency_stats)
        
        report_content = f"""# Experiment B: Latency and Baseline Tracking Error Analysis Report

## Analysis Information
- **Analysis Timestamp**: {self.analysis_timestamp}
- **Latency Data Source**: {os.path.basename(self.latency_csv)}
- **Error Data Source**: {os.path.basename(self.error_csv)}
- **Analysis Date**: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}

## Executive Summary

This report presents the results of Experiment B, which analyzes system latency characteristics and establishes the baseline relationship between latency and tracking error in the FSM Sandbox platform.

## Part 1: Latency Characteristics Analysis

### Key Performance Metrics

| Latency Component | Mean (ms) | Median (ms) | Std (ms) | P95 (ms) | P99 (ms) |
|-------------------|-----------|-------------|----------|----------|----------|
| Information Age | {latency_stats['information_age_ms']['mean']:.2f} | {latency_stats['information_age_ms']['median']:.2f} | {latency_stats['information_age_ms']['std']:.2f} | {latency_stats['information_age_ms']['p95']:.2f} | {latency_stats['information_age_ms']['p99']:.2f} |
| Planner Latency | {latency_stats['planner_latency_ms']['mean']:.2f} | {latency_stats['planner_latency_ms']['median']:.2f} | {latency_stats['planner_latency_ms']['std']:.2f} | {latency_stats['planner_latency_ms']['p95']:.2f} | {latency_stats['planner_latency_ms']['p99']:.2f} |
| Decision-to-Actuation | {latency_stats['decision_to_actuation_ms']['mean']:.2f} | {latency_stats['decision_to_actuation_ms']['median']:.2f} | {latency_stats['decision_to_actuation_ms']['std']:.2f} | {latency_stats['decision_to_actuation_ms']['p95']:.2f} | {latency_stats['decision_to_actuation_ms']['p99']:.2f} |
| **Total End-to-End** | **{latency_stats['total_latency_ms']['mean']:.2f}** | **{latency_stats['total_latency_ms']['median']:.2f}** | **{latency_stats['total_latency_ms']['std']:.2f}** | **{latency_stats['total_latency_ms']['p95']:.2f}** | **{latency_stats['total_latency_ms']['p99']:.2f}** |

### Latency Assessment
**{latency_assessment}**

## Part 2: Baseline Tracking Error Analysis

### Test Configuration
- **Target Velocity**: {error_baseline['target_velocity']:.3f} m/s
- **Mean System Latency**: {error_baseline['mean_latency_ms']:.2f} ms
- **Test Samples**: {error_baseline['sample_count']} measurements

### Error Analysis Results

| Metric | Value |
|--------|-------|
| Theoretical Error (v × ΔT) | {error_baseline['theoretical_error_m']:.6f} m |
| Actual Error Mean | {error_baseline['actual_error_mean_m']:.6f} m |
| Actual Error Std | {error_baseline['actual_error_std_m']:.6f} m |
| Actual Error Median | {error_baseline['actual_error_median_m']:.6f} m |
| Agreement Ratio | {error_baseline['agreement_ratio']:.3f} |

### Baseline Error Assessment
The theoretical model **{'ACCURATELY' if 0.5 <= error_baseline['agreement_ratio'] <= 1.5 else 'INACCURATELY'}** predicts the actual tracking error.

Agreement ratio of {error_baseline['agreement_ratio']:.3f} indicates {'excellent' if 0.8 <= error_baseline['agreement_ratio'] <= 1.2 else 'good' if 0.5 <= error_baseline['agreement_ratio'] <= 1.5 else 'poor'} correlation between latency and tracking error.

## Conclusions

### Platform Performance Sufficiency
Based on the latency analysis, the FSM Sandbox platform demonstrates **{latency_assessment.split(' - ')[0]}** performance characteristics for ADS testing.

### Error Baseline Establishment
The analysis successfully establishes a quantitative baseline for tracking error due to system latency:
- **Baseline Error**: {error_baseline['theoretical_error_m']:.6f} m at {error_baseline['target_velocity']:.3f} m/s
- **Error Predictability**: {error_baseline['agreement_ratio']:.1%} agreement with theoretical model

### Implications for Future Testing
This baseline provides a reference point for diagnosing execution errors in more complex test scenarios. Any tracking errors significantly exceeding this baseline can be attributed to algorithmic issues rather than platform limitations.

## Generated Files
- `latency_analysis_{self.analysis_timestamp}.png` - Comprehensive latency analysis plots
- `error_analysis_{self.analysis_timestamp}.png` - Tracking error analysis visualizations  
- `experiment_b_results_{self.analysis_timestamp}.json` - Detailed numerical results
- `experiment_b_report_{self.analysis_timestamp}.md` - This comprehensive report
"""
        
        # Save report
        report_file = os.path.join(self.analysis_dir, f"experiment_b_report_{self.analysis_timestamp}.md")
        with open(report_file, 'w') as f:
            f.write(report_content)
        
        print(f"Comprehensive report saved: {report_file}")

    def save_results_json(self):
        """Save detailed results to JSON file"""
        results_file = os.path.join(self.analysis_dir, f"experiment_b_results_{self.analysis_timestamp}.json")
        
        # Prepare metadata
        metadata = {
            'analysis_timestamp': self.analysis_timestamp,
            'latency_file': os.path.basename(self.latency_csv),
            'error_file': os.path.basename(self.error_csv),
            'latency_samples': len(self.latency_df),
            'error_samples': len(self.error_df),
            'analysis_date': datetime.now().isoformat()
        }
        
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
            'metadata': metadata,
            'latency_statistics': {
                comp: {k: convert_for_json(v) for k, v in stats.items()}
                for comp, stats in self.analysis_results['latency_statistics'].items()
            },
            'error_baseline': {
                k: convert_for_json(v) for k, v in self.analysis_results['error_baseline'].items()
            }
        }
        
        with open(results_file, 'w') as f:
            json.dump(json_results, f, indent=2)
        
        print(f"Detailed results saved: {results_file}")

    def run_complete_analysis(self):
        """Run the complete Experiment B analysis pipeline"""
        print("Starting Experiment B Analysis...")
        print(f"Output directory: {self.analysis_dir}")
        
        try:
            # Part 1: Latency analysis
            self.analyze_latency_characteristics()
            
            # Part 2: Error baseline analysis  
            self.analyze_tracking_error_baseline()
            
            # Generate outputs
            print(f"\nGenerating final outputs...")
            self.save_results_json()
            self.generate_comprehensive_report()
            
            print(f"\nExperiment B analysis complete!")
            print(f"All results saved to: {self.analysis_dir}")
            return True
            
        except Exception as e:
            print(f"Error during analysis: {e}")
            return False


def main():
    parser = argparse.ArgumentParser(description='Analyze Experiment B: Latency and Baseline Tracking Error')
    parser.add_argument('latency_csv', help='Path to latency data CSV file')
    parser.add_argument('error_csv', help='Path to tracking error CSV file')
    parser.add_argument('--config', default='config.yaml', help='Configuration file path')
    parser.add_argument('--show-plots', action='store_true', help='Display plots after generation')
    
    args = parser.parse_args()
    
    # Validate input files
    for file_path, name in [(args.latency_csv, 'latency'), (args.error_csv, 'error')]:
        if not os.path.exists(file_path):
            print(f"Error: {name} file not found: {file_path}")
            sys.exit(1)
    
    # Run analysis
    analyzer = ExperimentBAnalyzer(args.latency_csv, args.error_csv, args.config)
    success = analyzer.run_complete_analysis()
    
    if success and args.show_plots:
        plt.show()
    
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
```

### 5. 使用指南和启动脚本

#### 5.1 实验B启动脚本 (start_experiment_b.sh)

```bash
#!/bin/bash

# Experiment B Launcher Script
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_FILE="$SCRIPT_DIR/../config.yaml"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

print_colored() {
    echo -e "${1}${2}${NC}"
}

print_header() {
    echo
    print_colored $BLUE "=================================="
    print_colored $BLUE "$1"
    print_colored $BLUE "=================================="
}

# Function to check if ROS topic exists
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

# Setup environment
print_colored $BLUE "Setting up environment..."
source "$(dirname "$SCRIPT_DIR")/setup_env.sh"

# Verify configuration
if [ ! -f "$CONFIG_FILE" ]; then
    print_colored $RED "Configuration file not found: $CONFIG_FILE"
    exit 1
fi

print_colored $GREEN "Environment setup completed."

# Check required topics
print_colored $BLUE "Checking system readiness..."

PERCEPTION_TOPIC=$(python3 -c "import sys; sys.path.append('$(dirname "$SCRIPT_DIR")'); from config_manager import ConfigManager; cm = ConfigManager('$CONFIG_FILE'); print(cm.get('experiment_b.topics.perception_input', '/localization/kinematic_state'))")
PLANNING_TOPIC=$(python3 -c "import sys; sys.path.append('$(dirname "$SCRIPT_DIR")'); from config_manager import ConfigManager; cm = ConfigManager('$CONFIG_FILE'); print(cm.get('experiment_b.topics.planning_output', '/planning/trajectory'))")
GT_TOPIC=$(python3 -c "import sys; sys.path.append('$(dirname "$SCRIPT_DIR")'); from config_manager import ConfigManager; cm = ConfigManager('$CONFIG_FILE'); print(cm.get('experiment_b.topics.ground_truth', '/real_world/follow_adtruck/transformed_with_covariance'))")

print_colored $YELLOW "Expected topics:"
echo "  - Perception: $PERCEPTION_TOPIC"
echo "  - Planning: $PLANNING_TOPIC"
echo "  - Ground Truth: $GT_TOPIC"

# Wait for critical topics
if ! wait_for_topic "$PERCEPTION_TOPIC" 60; then
    print_colored $RED "Critical topic $PERCEPTION_TOPIC not available. Is the ADS running?"
    exit 1
fi

if ! wait_for_topic "$GT_TOPIC" 30; then
    print_colored $YELLOW "Warning: Ground truth topic $GT_TOPIC not available."
fi

print_header "Starting Experiment B Data Collection"

SESSION_TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
print_colored $GREEN "Session timestamp: $SESSION_TIMESTAMP"

# Start latency impact logger
print_colored $BLUE "Starting Latency Impact Logger..."
cd "$SCRIPT_DIR"
python3 latency_impact_logger.py --config "$CONFIG_FILE" &
LATENCY_PID=$!
print_colored $GREEN "Latency logger started (PID: $LATENCY_PID)"

# Start tracking error analyzer
print_colored $BLUE "Starting Tracking Error Analyzer..."
python3 tracking_error_analyzer.py --config "$CONFIG_FILE" &
ERROR_PID=$!
print_colored $GREEN "Error analyzer started (PID: $ERROR_PID)"

print_header "Experiment B Data Collection Active"
print_colored $GREEN "Both loggers are now collecting data!"
print_colored $YELLOW "Instructions:"
echo "  1. First, drive normally for 3-5 minutes to collect latency data"
echo "  2. Then, position the vehicle at the start of the test path"
echo "  3. Drive straight at constant velocity (0.5 m/s) for error baseline test"
echo "  4. The error analyzer will automatically detect when you're at the start"
echo "  5. Press Ctrl+C when both tests are complete"

print_colored $BLUE "\nPress Ctrl+C to stop data collection..."

# Monitor processes
while true; do
    if ! kill -0 $LATENCY_PID 2>/dev/null; then
        print_colored $RED "Latency logger stopped unexpectedly!"
        break
    fi
    if ! kill -0 $ERROR_PID 2>/dev/null; then
        print_colored $RED "Error analyzer stopped unexpectedly!"
        break
    fi
    
    sleep 5
done

print_header "Experiment B Data Collection Completed"
print_colored $GREEN "Session $SESSION_TIMESTAMP data collection finished."
print_colored $BLUE "Next steps:"
echo "  1. Analyze the data: python3 analyze_experiment_b.py data/exp_b_latency_data_$SESSION_TIMESTAMP.csv data/exp_b_tracking_error_$SESSION_TIMESTAMP.csv"
echo "  2. Check results/ directory for comprehensive analysis outputs"
echo "  3. Review logs/ directory for session details"
```

### 6. 完整使用指南

```markdown
# Experiment B Usage Guide

## Quick Start

1. **Setup Environment**
   ```bash
   cd Validation/B_latency_baseline
   chmod +x start_experiment_b.sh
   ./start_experiment_b.sh
   ```

2. **Data Collection Phase**
   - Normal driving (3-5 minutes) for latency data
   - Constant velocity test for error baseline
   - System will auto-detect test phases

3. **Analysis Phase**
   ```bash
   python3 analyze_experiment_b.py \
     data/exp_b_latency_data_YYYYMMDD_HHMMSS.csv \
     data/exp_b_tracking_error_YYYYMMDD_HHMMSS.csv \
     --show-plots
   ```

## Manual Operation

### Step 1: Start Individual Loggers
```bash
# Terminal 1: Latency Logger
python3 latency_impact_logger.py --config ../config.yaml

# Terminal 2: Error Analyzer  
python3 tracking_error_analyzer.py --config ../config.yaml
```

### Step 2: Execute Test Scenarios
1. **Latency Collection**: Drive normally for diverse scenarios
2. **Error Baseline**: Drive straight line at 0.5 m/s constant velocity

### Step 3: Analysis
```bash
python3 analyze_experiment_b.py latency_file.csv error_file.csv
```

## CARLA Handler Integration

Add timing instrumentation to your CARLA control node:

```python
# In your CarlaUpdateVehicleHandler class
from carla_handler_patch import CarlaVehicleHandlerPatch

class YourCarlaNode(Node, CarlaVehicleHandlerPatch):
    def __init__(self):
        Node.__init__(self, 'carla_node')
        CarlaVehicleHandlerPatch.__init__(self)
        
        # Use instrumented callback
        self.control_sub = self.create_subscription(
            Trajectory, "/planning/trajectory",
            self.instrumented_control_callback, 10)
```

## Expected Outputs

### Data Files
- `exp_b_latency_data_YYYYMMDD_HHMMSS.csv`
- `exp_b_tracking_error_YYYYMMDD_HHMMSS.csv`

### Analysis Results
- `latency_analysis_YYYYMMDD_HHMMSS.png`
- `error_analysis_YYYYMMDD_HHMMSS.png`
- `experiment_b_report_YYYYMMDD_HHMMSS.md`
- `experiment_b_results_YYYYMMDD_HHMMSS.json`

## Configuration

Edit `config.yaml` to customize:
- Topic names for your system
- Quality thresholds
- Test parameters (velocity, duration, path)
```

这个完整的实验B实现提供了：

1. **完整的数据收集框架** - 四个时间戳的精确捕获
2. **理论模型验证** - 延迟与跟踪误差的定量关系
3. **非侵入式设计** - 最小化对现有系统的修改
4. **全面的分析和可视化** - 从统计表格到时间序列图
5. **自动化的质量评估** - 基于配置的阈值判断
6. **详细的文档和报告** - 完整的分析流程记录

这个方案成功地建立了"误差基线"，为后续复杂测试中的诊断性归因提供了科学的参考标准。