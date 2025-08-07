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