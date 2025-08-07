#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import csv
import os
import datetime
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from transforms3d.euler import quat2euler
import numpy as np

class OfflineNdtRecorder(Node):
    """
    Records NDT localization output during offline bag playback for fidelity analysis.
    """
    
    def __init__(self):
        super().__init__('offline_ndt_recorder')
        
        # Create analysis timestamp
        self.analysis_timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # Setup directory structure
        self.setup_directories()
        
        # Configuration - MODIFY ACCORDING TO YOUR NDT OUTPUT
        self.ndt_output_topic = "/localization/kinematic_state"
        self.message_type = PoseWithCovarianceStamped
        
        # Create subscription
        self.subscription = self.create_subscription(
            self.message_type,
            self.ndt_output_topic,
            self.ndt_callback,
            10)
        
        # Setup CSV logging
        self.setup_csv_logging()
        
        # Statistics
        self.total_ndt_poses_recorded = 0
        self.last_timestamp_ns = 0
        self.first_pose_time = None
        self.last_pose_time = None
        
        # Statistics timer
        self.stats_timer = self.create_timer(5.0, self.report_statistics)
        
        self.get_logger().info("Offline NDT recorder started")
        self.get_logger().info(f"Analysis timestamp: {self.analysis_timestamp}")
        self.get_logger().info(f"Listening to NDT topic: {self.ndt_output_topic}")
        self.get_logger().info(f"Data directory: {self.data_dir}")

    def setup_directories(self):
        """Setup directory structure"""
        self.base_dir = os.path.dirname(os.path.abspath(__file__))
        self.data_dir = os.path.join(self.base_dir, "data")
        self.logs_dir = os.path.join(self.base_dir, "logs")
        
        for directory in [self.data_dir, self.logs_dir]:
            os.makedirs(directory, exist_ok=True)

    def setup_csv_logging(self):
        """Setup CSV file for NDT poses"""
        self.csv_filename = os.path.join(
            self.data_dir,
            f'offline_ndt_poses_{self.analysis_timestamp}.csv'
        )
        
        self.csv_file = open(self.csv_filename, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        
        # CSV header matching ground truth format
        self.csv_writer.writerow([
            'timestamp_ns', 'analysis_id', 'x', 'y', 'z',
            'roll_rad', 'pitch_rad', 'yaw_rad', 'qw', 'qx', 'qy', 'qz'
        ])
        
        self.csv_file.flush()
        self.get_logger().info(f"CSV logging to: {self.csv_filename}")

    def ndt_callback(self, msg):
        """Process incoming NDT pose messages"""
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
            
            # Track timing statistics
            if self.first_pose_time is None:
                self.first_pose_time = timestamp_ns
            self.last_pose_time = timestamp_ns
            
            # Validate timestamp order (allow some tolerance for bag playback)
            if timestamp_ns < self.last_timestamp_ns - 1_000_000:  # 1ms tolerance
                self.get_logger().warn(f"Timestamp ordering issue: {timestamp_ns} < {self.last_timestamp_ns}")
            
            self.last_timestamp_ns = timestamp_ns
            
            # Extract position and orientation
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
                timestamp_ns, self.analysis_timestamp,
                pos.x, pos.y, pos.z,
                roll, pitch, yaw,
                q.w, q.x, q.y, q.z
            ])
            
            self.csv_file.flush()
            self.total_ndt_poses_recorded += 1
            
            self.get_logger().debug(f"Recorded NDT pose: x={pos.x:.3f}, y={pos.y:.3f}, yaw={np.degrees(yaw):.1f}deg")
            
        except Exception as e:
            self.get_logger().error(f"Error processing NDT pose: {e}")

    def report_statistics(self):
        """Report recording statistics"""
        duration_s = 0
        if self.first_pose_time and self.last_pose_time:
            duration_s = (self.last_pose_time - self.first_pose_time) / 1e9
        
        self.get_logger().info(
            f"NDT poses recorded: {self.total_ndt_poses_recorded}, "
            f"Duration: {duration_s:.1f}s"
        )

    def destroy_node(self):
        """Clean shutdown with final statistics"""
        if hasattr(self, 'csv_file') and self.csv_file:
            self.csv_file.close()
        
        duration_s = 0
        if self.first_pose_time and self.last_pose_time:
            duration_s = (self.last_pose_time - self.first_pose_time) / 1e9
        
        self.get_logger().info(f"Offline NDT recorder stopped")
        self.get_logger().info(f"Total NDT poses recorded: {self.total_ndt_poses_recorded}")
        self.get_logger().info(f"Recording duration: {duration_s:.1f}s")
        self.get_logger().info(f"Data saved to: {self.csv_filename}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    recorder = OfflineNdtRecorder()
    
    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        recorder.get_logger().info("Shutting down offline NDT recorder...")
    finally:
        recorder.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()