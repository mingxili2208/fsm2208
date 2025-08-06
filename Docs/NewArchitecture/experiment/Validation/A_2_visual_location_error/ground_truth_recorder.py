#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import csv
import os
import datetime
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from transforms3d.euler import quat2euler
import numpy as np

class GroundTruthRecorder(Node):
    """
    Records ground truth poses for fidelity analysis.
    This node runs concurrently with timing analysis (A.1) to collect data for both experiments.
    """
    
    def __init__(self):
        super().__init__('ground_truth_recorder')
        
        # Create session timestamp matching A.1 experiment
        self.session_timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # Setup directory structure (matching A.1)
        self.setup_directories()
        
        # Configuration - MODIFY ACCORDING TO YOUR SYSTEM
        self.ground_truth_topic = "/real_world/follow_adtruck/transformed_with_covariance"
        self.message_type = PoseWithCovarianceStamped
        
        # Create subscription
        self.subscription = self.create_subscription(
            self.message_type,
            self.ground_truth_topic,
            self.pose_callback,
            10)
        
        # Setup CSV logging
        self.setup_csv_logging()
        
        # Statistics
        self.total_poses_recorded = 0
        self.last_timestamp_ns = 0
        
        # Statistics timer
        self.stats_timer = self.create_timer(5.0, self.report_statistics)
        
        self.get_logger().info("Ground truth recorder started")
        self.get_logger().info(f"Session timestamp: {self.session_timestamp}")
        self.get_logger().info(f"Listening to topic: {self.ground_truth_topic}")
        self.get_logger().info(f"Data directory: {self.data_dir}")

    def setup_directories(self):
        """Setup directory structure matching A.1 experiment"""
        self.base_dir = os.path.dirname(os.path.abspath(__file__))
        self.data_dir = os.path.join(self.base_dir, "data")
        self.logs_dir = os.path.join(self.base_dir, "logs")
        
        for directory in [self.data_dir, self.logs_dir]:
            os.makedirs(directory, exist_ok=True)

    def setup_csv_logging(self):
        """Setup CSV file for ground truth poses"""
        self.csv_filename = os.path.join(
            self.data_dir,
            f'ground_truth_poses_{self.session_timestamp}.csv'
        )
        
        self.csv_file = open(self.csv_filename, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        
        # CSV header with comprehensive information
        self.csv_writer.writerow([
            'timestamp_ns', 'session_id', 'x', 'y', 'z', 
            'roll_rad', 'pitch_rad', 'yaw_rad', 'qw', 'qx', 'qy', 'qz'
        ])
        
        self.csv_file.flush()
        self.get_logger().info(f"CSV logging to: {self.csv_filename}")

    def pose_callback(self, msg):
        """Process incoming pose messages"""
        try:
            # Extract pose based on message type
            if isinstance(msg, PoseWithCovarianceStamped):
                pose = msg.pose.pose
            elif isinstance(msg, PoseStamped):
                pose = msg.pose
            else:
                self.get_logger().error(f"Unsupported message type: {type(msg)}")
                return
            
            # Extract timestamp
            timestamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            
            # Validate timestamp (should be increasing)
            if timestamp_ns <= self.last_timestamp_ns:
                self.get_logger().warn(f"Non-increasing timestamp detected: {timestamp_ns}")
                return
            
            self.last_timestamp_ns = timestamp_ns
            
            # Extract position
            pos = pose.position
            q = pose.orientation
            
            # Convert quaternion to euler angles
            try:
                roll, pitch, yaw = quat2euler([q.w, q.x, q.y, q.z])
            except Exception as e:
                self.get_logger().error(f"Quaternion conversion failed: {e}")
                return
            
            # Write to CSV
            self.csv_writer.writerow([
                timestamp_ns, self.session_timestamp,
                pos.x, pos.y, pos.z,
                roll, pitch, yaw,
                q.w, q.x, q.y, q.z
            ])
            
            self.csv_file.flush()
            self.total_poses_recorded += 1
            
            self.get_logger().debug(f"Recorded pose: x={pos.x:.3f}, y={pos.y:.3f}, yaw={np.degrees(yaw):.1f}deg")
            
        except Exception as e:
            self.get_logger().error(f"Error processing pose: {e}")

    def report_statistics(self):
        """Report recording statistics"""
        self.get_logger().info(f"Ground truth poses recorded: {self.total_poses_recorded}")

    def destroy_node(self):
        """Clean shutdown"""
        if hasattr(self, 'csv_file') and self.csv_file:
            self.csv_file.close()
            
        self.get_logger().info(f"Ground truth recorder stopped")
        self.get_logger().info(f"Total poses recorded: {self.total_poses_recorded}")
        self.get_logger().info(f"Data saved to: {self.csv_filename}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    recorder = GroundTruthRecorder()
    
    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        recorder.get_logger().info("Shutting down ground truth recorder...")
    finally:
        recorder.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()