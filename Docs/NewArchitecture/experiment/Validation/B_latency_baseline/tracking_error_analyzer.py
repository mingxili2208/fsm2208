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