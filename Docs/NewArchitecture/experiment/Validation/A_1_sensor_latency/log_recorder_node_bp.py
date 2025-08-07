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

# Import configuration manager
from config_manager import ConfigManager

class LatencyLogRecorderNode(Node):
    """
    Node for recording timing data to analyze FSM Sandbox latency performance.
    Now uses external configuration management for better flexibility.
    """
    
    def __init__(self, config_file: str = "config.yaml"):
        super().__init__('latency_log_recorder_node')

        # Load configuration
        self.config_manager = ConfigManager(config_file)
        self.a1_config = self.config_manager.get_a1_config()
        
        # Create session timestamp
        self.session_timestamp = datetime.datetime.now().strftime(
            self.config_manager.get('file_management.file_naming.timestamp_format', '%Y%m%d_%H%M%S')
        )
        
        # Setup directory structure
        self.setup_directories()

        # Get topic configuration
        topics = self.a1_config.get('topics', {})
        self.timing_sync_topic = topics.get('timing_sync', '/fsm_sandbox/timing/t1_t2')
        self.lidar_topic = topics.get('lidar_input', '/carla/follow_adtruck/carla_pointcloud')
        self.ndt_topic = topics.get('ndt_output', '/localization/kinematic_state')
        
        # Get parameters
        params = self.a1_config.get('parameters', {})
        self.correlation_window_ns = params.get('correlation_window_ns', 50_000_000)
        self.cleanup_threshold_ns = params.get('cleanup_threshold_ns', 60_000_000_000)
        self.stats_interval = params.get('stats_report_interval_s', 5.0)
        
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
        
        # Create subscribers
        self.t1_t2_sub = self.create_subscription(
            TimeReference, self.timing_sync_topic, self.t1_t2_callback, 10)
        
        self.lidar_sub = self.create_subscription(
            PointCloud2, self.lidar_topic, self.lidar_callback, 10)
            
        self.ndt_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, self.ndt_topic, self.ndt_pose_callback, 10)

        # Setup CSV logging
        self.setup_csv_logging()
        
        # Setup session logging
        self.setup_session_logging()
        
        # Statistics reporting timer
        self.stats_timer = self.create_timer(self.stats_interval, self.report_statistics)
        
        # Cleanup timer
        self.cleanup_timer = self.create_timer(30.0, self.cleanup_old_entries)
        
        self.get_logger().info("Latency log recorder node started")
        self.get_logger().info(f"Session timestamp: {self.session_timestamp}")
        self.get_logger().info(f"Data directory: {self.data_dir}")
        self.get_logger().info(f"Monitoring topics:")
        self.get_logger().info(f"  - Timing sync: {self.timing_sync_topic}")
        self.get_logger().info(f"  - LiDAR: {self.lidar_topic}")
        self.get_logger().info(f"  - NDT: {self.ndt_topic}")
        self.get_logger().info(f"Correlation window: {self.correlation_window_ns/1e6:.1f}ms")

    def setup_directories(self):
        """Setup directory structure using configuration"""
        self.base_dir = os.path.dirname(os.path.abspath(__file__))
        
        # Get directory configuration
        dirs = self.config_manager.get_directories()
        self.log_dir = os.path.join(self.base_dir, dirs.get('logs', 'logs'))
        self.data_dir = os.path.join(self.base_dir, dirs.get('data', 'data'))
        self.results_dir = os.path.join(self.base_dir, dirs.get('results', 'results'))
        
        # Create directories
        for directory in [self.log_dir, self.data_dir, self.results_dir]:
            os.makedirs(directory, exist_ok=True)

    def setup_csv_logging(self):
        """Setup CSV file for logging timing data"""
        self.csv_filename = os.path.join(
            self.data_dir, 
            f'fsm_timing_data_{self.session_timestamp}.csv'
        )
        self.csv_file = open(self.csv_filename, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        
        # CSV header
        self.csv_writer.writerow([
            'Timestamp_ISO', 'Session_ID', 'T1_ns', 'T2_ns', 'T3_ns', 'T4_ns',
            'T1_T3_Diff_ms', 'Render_Latency_ms', 'NDT_Processing_Latency_ms', 
            'Total_Pipeline_Latency_ms', 'Sequence_Valid'
        ])
        
        self.csv_file.flush()
        self.get_logger().info(f"CSV data logging to: {self.csv_filename}")

    def setup_session_logging(self):
        """Setup session log file"""
        self.session_log_filename = os.path.join(
            self.log_dir,
            f'fsm_session_log_{self.session_timestamp}.log'
        )
        
        # Write session header
        with open(self.session_log_filename, 'w') as f:
            f.write(f"FSM Sandbox Timing Analysis Session Log\n")
            f.write(f"Session ID: {self.session_timestamp}\n")
            f.write(f"Start Time: {datetime.datetime.now().isoformat()}\n")
            f.write(f"Configuration:\n")
            f.write(f"  - Timing Sync Topic: {self.timing_sync_topic}\n")
            f.write(f"  - LiDAR Topic: {self.lidar_topic}\n")
            f.write(f"  - NDT Topic: {self.ndt_topic}\n")
            f.write(f"  - Correlation Window: {self.correlation_window_ns/1e6:.1f}ms\n")
            f.write("="*60 + "\n\n")
        
        self.get_logger().info(f"Session logging to: {self.session_log_filename}")

    def log_to_session_file(self, message):
        """Write message to session log file"""
        timestamp = datetime.datetime.now().isoformat()
        with open(self.session_log_filename, 'a') as f:
            f.write(f"[{timestamp}] {message}\n")

    def to_nanoseconds(self, stamp):
        """Convert ROS Time message to nanoseconds integer"""
        return stamp.sec * 1_000_000_000 + stamp.nanosec

    def t1_t2_callback(self, msg: TimeReference):
        """Receive (T1, T2) pairs and store in dictionary"""
        t1_ns = self.to_nanoseconds(msg.header.stamp)
        t2_ns = self.to_nanoseconds(msg.time_ref)
        
        # Validate timestamp order
        if t2_ns <= t1_ns:
            warning_msg = f"Invalid T1-T2 order: T2({t2_ns}) <= T1({t1_ns})"
            self.get_logger().warn(warning_msg)
            self.log_to_session_file(f"WARNING: {warning_msg}")
            return
            
        self.t1_t2_map[t1_ns] = t2_ns
        self.total_t1_t2_received += 1
        
        debug_msg = f"T1-T2 received: diff={(t2_ns-t1_ns)/1e6:.2f}ms"
        self.get_logger().debug(debug_msg)
        
        # Log every 10th T1-T2 pair to session file
        if self.total_t1_t2_received % 10 == 0:
            self.log_to_session_file(f"T1-T2 pairs received: {self.total_t1_t2_received}")

    def lidar_callback(self, msg: PointCloud2):
        """Receive LiDAR data and establish T3->T1 mapping"""
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
            debug_msg = f"T1-T3 correlation failed: diff={time_diff_ms:.2f}ms"
            self.get_logger().debug(debug_msg)
            
            # Log correlation failures periodically
            if self.failed_t1_t3_matches % 20 == 0:
                self.log_to_session_file(f"T1-T3 correlation failures: {self.failed_t1_t3_matches}")

    def ndt_pose_callback(self, msg: PoseWithCovarianceStamped):
        """Receive NDT pose output and complete timing chain correlation"""
        t4_ns = self.to_nanoseconds(self.get_clock().now().to_msg())
        t3_ns = self.to_nanoseconds(msg.header.stamp)
        self.total_ndt_received += 1
        
        # Check if T3 exists in our mapping
        if t3_ns not in self.t3_to_t1_map:
            self.failed_t3_t4_matches += 1
            self.get_logger().debug(f"T3-T4 correlation failed: T3 not found")
            return
        
        # Find corresponding T1
        t1_ns = self.t3_to_t1_map.pop(t3_ns)
        
        # Check if T1 exists in T1-T2 mapping
        if t1_ns not in self.t1_t2_map:
            self.failed_t3_t4_matches += 1
            self.get_logger().debug(f"T3-T4 correlation failed: T1 not found in T1-T2 map")
            return
        
        t2_ns = self.t1_t2_map.pop(t1_ns)
        
        # Validate timestamp sequence
        sequence_valid = (t1_ns < t2_ns < t3_ns < t4_ns)
        if not sequence_valid:
            warning_msg = f"Invalid timestamp sequence detected"
            self.get_logger().warn(warning_msg)
            self.log_to_session_file(f"WARNING: {warning_msg}")
        
        # Calculate latencies
        t1_t3_diff_ms = (t3_ns - t1_ns) / 1e6
        render_latency_ms = (t3_ns - t2_ns) / 1e6
        ndt_processing_latency_ms = (t4_ns - t3_ns) / 1e6
        pipeline_latency_ms = (t4_ns - t1_ns) / 1e6
        
        # Record to CSV
        timestamp_iso = datetime.datetime.now().isoformat()
        self.csv_writer.writerow([
            timestamp_iso, self.session_timestamp, t1_ns, t2_ns, t3_ns, t4_ns,
            t1_t3_diff_ms, render_latency_ms, ndt_processing_latency_ms,
            pipeline_latency_ms, sequence_valid
        ])
        
        self.csv_file.flush()
        self.successful_correlations += 1
        
        success_msg = (
            f"Complete timing chain recorded: "
            f"Pipeline={pipeline_latency_ms:.2f}ms, "
            f"Render={render_latency_ms:.2f}ms, "
            f"NDT={ndt_processing_latency_ms:.2f}ms"
        )
        self.get_logger().info(success_msg)
        self.log_to_session_file(f"SUCCESS: {success_msg}")

    def cleanup_old_entries(self):
        """Remove old entries from dictionaries to prevent memory leak"""
        current_time_ns = self.to_nanoseconds(self.get_clock().now().to_msg())
        
        # Cleanup T1-T2 map
        old_t1_keys = [t1 for t1 in self.t1_t2_map.keys() 
                       if (current_time_ns - t1) > self.cleanup_threshold_ns]
        for t1 in old_t1_keys:
            del self.t1_t2_map[t1]
        
        # Cleanup T3-T1 map
        old_t3_keys = [t3 for t3 in self.t3_to_t1_map.keys() 
                       if (current_time_ns - t3) > self.cleanup_threshold_ns]
        for t3 in old_t3_keys:
            del self.t3_to_t1_map[t3]
        
        if old_t1_keys or old_t3_keys:
            cleanup_msg = f"Cleaned up {len(old_t1_keys)} T1-T2 and {len(old_t3_keys)} T3-T1 entries"
            self.get_logger().debug(cleanup_msg)
            self.log_to_session_file(f"CLEANUP: {cleanup_msg}")

    def report_statistics(self):
        """Report current statistics"""
        correlation_rate = (self.successful_correlations / max(self.total_ndt_received, 1)) * 100
        
        stats_msg = (
            f"Statistics: T1-T2 received={self.total_t1_t2_received}, "
            f"LiDAR received={self.total_lidar_received}, "
            f"NDT received={self.total_ndt_received}, "
            f"Successful correlations={self.successful_correlations} "
            f"({correlation_rate:.1f}%), "
            f"T1-T3 match failures={self.failed_t1_t3_matches}, "
            f"T3-T4 match failures={self.failed_t3_t4_matches}"
        )
        
        self.get_logger().info(stats_msg)
        self.log_to_session_file(f"STATS: {stats_msg}")

    def write_session_summary(self):
        """Write final session summary"""
        end_time = datetime.datetime.now()
        final_correlation_rate = (self.successful_correlations / max(self.total_ndt_received, 1)) * 100
        
        summary = f"""
Session Summary:
  End Time: {end_time.isoformat()}
  Configuration Used:
    - Timing Sync Topic: {self.timing_sync_topic}
    - LiDAR Topic: {self.lidar_topic}
    - NDT Topic: {self.ndt_topic}
    - Correlation Window: {self.correlation_window_ns/1e6:.1f}ms
  Final Statistics:
    - T1-T2 pairs received: {self.total_t1_t2_received}
    - LiDAR messages received: {self.total_lidar_received}
    - NDT messages received: {self.total_ndt_received}
    - Successful correlations: {self.successful_correlations}
    - Correlation success rate: {final_correlation_rate:.2f}%
    - T1-T3 match failures: {self.failed_t1_t3_matches}
    - T3-T4 match failures: {self.failed_t3_t4_matches}
  
Data Files Generated:
  - CSV Data: {self.csv_filename}
  - Session Log: {self.session_log_filename}
"""
        
        with open(self.session_log_filename, 'a') as f:
            f.write("\n" + "="*60 + "\n")
            f.write(summary)

    def destroy_node(self):
        """Clean shutdown with final statistics"""
        self.write_session_summary()
        
        super().destroy_node()
        if hasattr(self, 'csv_file') and self.csv_file:
            self.csv_file.close()
            
        final_correlation_rate = (self.successful_correlations / max(self.total_ndt_received, 1)) * 100
        final_msg = (
            f"Final statistics: {self.successful_correlations} successful correlations "
            f"out of {self.total_ndt_received} NDT messages ({final_correlation_rate:.1f}%)"
        )
        self.get_logger().info(final_msg)
        self.get_logger().info(f"Data saved to: {self.csv_filename}")
        self.get_logger().info(f"Session log saved to: {self.session_log_filename}")


def main(args=None):
    import argparse
    
    parser = argparse.ArgumentParser(description='FSM Sandbox Timing Analysis Data Recorder')
    parser.add_argument('--config', default='config.yaml', help='Configuration file path')
    
    # Parse known args to avoid ROS2 argument conflicts
    known_args, _ = parser.parse_known_args()
    
    rclpy.init(args=args)
    log_recorder_node = LatencyLogRecorderNode(known_args.config)
    
    try:
        rclpy.spin(log_recorder_node)
    except KeyboardInterrupt:
        log_recorder_node.get_logger().info("Shutting down log recorder...")
    finally:
        log_recorder_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()