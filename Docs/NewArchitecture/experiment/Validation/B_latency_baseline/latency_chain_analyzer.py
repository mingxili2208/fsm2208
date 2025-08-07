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