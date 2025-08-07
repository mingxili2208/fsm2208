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