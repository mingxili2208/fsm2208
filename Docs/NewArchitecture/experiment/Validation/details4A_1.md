您的优化建议**非常合理且专业**！这些建议确实能显著提升工具的可维护性、鲁棒性和分析深度。我将基于这些建议提供一个完整的优化方案。

## 完整优化方案

### 1. 配置文件 (config.yaml)

```yaml
# FSM Sandbox Experiment Configuration
project:
  name: "FSM Sandbox Validation"
  version: "1.0"
  description: "Timing and Fidelity Analysis for FSM Sandbox Architecture"

# Experiment A.1: Timing Analysis Configuration
experiment_a1:
  topics:
    timing_sync: "/fsm_sandbox/timing/t1_t2"
    lidar_input: "/carla/follow_adtruck/carla_pointcloud"
    ndt_output: "/localization/kinematic_state"
  
  parameters:
    correlation_window_ns: 50000000  # 50ms
    cleanup_threshold_ns: 60000000000  # 60s
    stats_report_interval_s: 5.0
    
  quality_thresholds:
    pipeline_p99_ms: 100
    render_mean_ms: 50
    pipeline_std_ms: 30
    ndt_p95_ms: 20

# Experiment A.2: Fidelity Analysis Configuration  
experiment_a2:
  topics:
    ground_truth: "/real_world/follow_adtruck/transformed_with_covariance"
    virtual_lidar: "/carla/follow_adtruck/carla_pointcloud"
    offline_ndt: "/localization/kinematic_state"
  
  message_types:
    ground_truth_type: "PoseWithCovarianceStamped"  # or "PoseStamped"
    ndt_output_type: "PoseWithCovarianceStamped"
  
  parameters:
    alignment_tolerance_windows_ms: [10, 50, 100, 200]
    min_alignment_rate_percent: 50
    outlier_removal_enabled: true
    outlier_iqr_factor: 1.5
    
  quality_thresholds:
    translation_mean_mm: 5
    translation_p99_mm: 20
    yaw_mean_deg: 0.1
    yaw_p95_deg: 0.5
    translation_std_mm: 10

# File Management
file_management:
  base_directories:
    data: "data"
    logs: "logs" 
    results: "results"
    config: "config"
  
  file_naming:
    timestamp_format: "%Y%m%d_%H%M%S"
    use_session_id: true

# Visualization Settings
visualization:
  style: "default"
  figure_size: [16, 12]
  font_size: 12
  dpi: 300
  color_palette: "husl"
  enable_grid: true
  grid_alpha: 0.3

# Analysis Settings
analysis:
  enable_outlier_removal: true
  enable_spatial_analysis: true
  enable_confidence_intervals: true
  confidence_level: 0.95
```

### 2. 配置管理器 (config_manager.py)

```python
#!/usr/bin/env python3

import yaml
import os
from pathlib import Path
from typing import Dict, Any, Optional

class ConfigManager:
    """
    Centralized configuration management for FSM Sandbox experiments.
    """
    
    def __init__(self, config_file: str = "config.yaml"):
        self.config_file = config_file
        self.config = self._load_config()
        self._validate_config()
        
    def _load_config(self) -> Dict[str, Any]:
        """Load configuration from YAML file"""
        # Try to find config file in multiple locations
        search_paths = [
            self.config_file,
            os.path.join(os.path.dirname(__file__), self.config_file),
            os.path.join(os.path.dirname(__file__), "config", self.config_file),
            os.path.join(os.getcwd(), self.config_file)
        ]
        
        for path in search_paths:
            if os.path.exists(path):
                try:
                    with open(path, 'r') as f:
                        config = yaml.safe_load(f)
                    print(f"Configuration loaded from: {os.path.abspath(path)}")
                    return config
                except Exception as e:
                    print(f"Error loading config from {path}: {e}")
                    continue
        
        # If no config file found, create default
        print("No configuration file found. Creating default config.yaml...")
        default_config = self._create_default_config()
        self._save_config(default_config, self.config_file)
        return default_config
    
    def _create_default_config(self) -> Dict[str, Any]:
        """Create default configuration"""
        return {
            'project': {
                'name': 'FSM Sandbox Validation',
                'version': '1.0'
            },
            'experiment_a1': {
                'topics': {
                    'timing_sync': '/fsm_sandbox/timing/t1_t2',
                    'lidar_input': '/carla/follow_adtruck/carla_pointcloud',
                    'ndt_output': '/localization/kinematic_state'
                },
                'parameters': {
                    'correlation_window_ns': 50000000,
                    'cleanup_threshold_ns': 60000000000,
                    'stats_report_interval_s': 5.0
                }
            },
            'experiment_a2': {
                'topics': {
                    'ground_truth': '/real_world/follow_adtruck/transformed_with_covariance',
                    'virtual_lidar': '/carla/follow_adtruck/carla_pointcloud',
                    'offline_ndt': '/localization/kinematic_state'
                },
                'message_types': {
                    'ground_truth_type': 'PoseWithCovarianceStamped',
                    'ndt_output_type': 'PoseWithCovarianceStamped'
                }
            },
            'file_management': {
                'base_directories': {
                    'data': 'data',
                    'logs': 'logs',
                    'results': 'results'
                }
            }
        }
    
    def _save_config(self, config: Dict[str, Any], filepath: str):
        """Save configuration to file"""
        try:
            with open(filepath, 'w') as f:
                yaml.dump(config, f, default_flow_style=False, indent=2)
            print(f"Default configuration saved to: {os.path.abspath(filepath)}")
        except Exception as e:
            print(f"Error saving config: {e}")
    
    def _validate_config(self):
        """Validate configuration structure"""
        required_sections = ['experiment_a1', 'experiment_a2', 'file_management']
        for section in required_sections:
            if section not in self.config:
                raise ValueError(f"Missing required configuration section: {section}")
    
    def get(self, key_path: str, default: Any = None) -> Any:
        """
        Get configuration value using dot notation.
        Example: get('experiment_a1.topics.lidar_input')
        """
        keys = key_path.split('.')
        value = self.config
        
        try:
            for key in keys:
                value = value[key]
            return value
        except (KeyError, TypeError):
            return default
    
    def get_a1_config(self) -> Dict[str, Any]:
        """Get Experiment A.1 configuration"""
        return self.config.get('experiment_a1', {})
    
    def get_a2_config(self) -> Dict[str, Any]:
        """Get Experiment A.2 configuration"""
        return self.config.get('experiment_a2', {})
    
    def get_directories(self) -> Dict[str, str]:
        """Get base directory configuration"""
        return self.config.get('file_management', {}).get('base_directories', {})
    
    def update_topic(self, experiment: str, topic_key: str, new_value: str):
        """Update topic configuration"""
        if experiment in self.config and 'topics' in self.config[experiment]:
            self.config[experiment]['topics'][topic_key] = new_value
            print(f"Updated {experiment}.topics.{topic_key} = {new_value}")
        else:
            print(f"Error: Cannot update {experiment}.topics.{topic_key}")
```

### 3. 优化的时效性记录节点 (log_recorder_node.py)

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
```

### 4. 优化的保真度分析脚本 (analyze_fidelity_data.py)

```python
#!/usr/bin/env python3

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
import argparse
import os
import sys
from datetime import datetime
import json
from scipy import stats

# Import configuration manager
from config_manager import ConfigManager

class FSMFidelityAnalyzer:
    """
    Enhanced analyzer for FSM Sandbox fidelity and self-consistency validation.
    Includes outlier removal, spatial error analysis, and configurable parameters.
    """
    
    def __init__(self, ground_truth_csv, ndt_csv, config_file="config.yaml"):
        self.ground_truth_csv = ground_truth_csv
        self.ndt_csv = ndt_csv
        self.df_aligned = None
        self.analysis_results = {}
        
        # Load configuration
        self.config_manager = ConfigManager(config_file)
        self.a2_config = self.config_manager.get_a2_config()
        
        # Create analysis timestamp
        self.analysis_timestamp = datetime.now().strftime(
            self.config_manager.get('file_management.file_naming.timestamp_format', '%Y%m%d_%H%M%S')
        )
        
        # Setup directory structure
        self.setup_directories()
        
        # Configure plotting
        self.setup_plotting()
        
        # Get analysis parameters
        self.outlier_removal_enabled = self.a2_config.get('parameters', {}).get('outlier_removal_enabled', True)
        self.outlier_iqr_factor = self.a2_config.get('parameters', {}).get('outlier_iqr_factor', 1.5)
        self.alignment_windows = self.a2_config.get('parameters', {}).get('alignment_tolerance_windows_ms', [10, 50, 100, 200])
        self.min_alignment_rate = self.a2_config.get('parameters', {}).get('min_alignment_rate_percent', 50)

    def setup_directories(self):
        """Setup directory structure for results"""
        self.base_dir = os.path.dirname(os.path.abspath(__file__))
        
        # Get directory configuration
        dirs = self.config_manager.get_directories()
        self.results_dir = os.path.join(self.base_dir, dirs.get('results', 'results'))
        self.analysis_dir = os.path.join(self.results_dir, f"fidelity_analysis_{self.analysis_timestamp}")
        
        for directory in [self.results_dir, self.analysis_dir]:
            os.makedirs(directory, exist_ok=True)
        
        print(f"Fidelity analysis output directory: {self.analysis_dir}")

    def setup_plotting(self):
        """Setup plotting configuration"""
        viz_config = self.config_manager.get('visualization', {})
        
        plt.style.use(viz_config.get('style', 'default'))
        if 'color_palette' in viz_config:
            sns.set_palette(viz_config['color_palette'])
        
        # Configure matplotlib
        plt.rcParams['figure.figsize'] = viz_config.get('figure_size', [16, 12])
        plt.rcParams['font.size'] = viz_config.get('font_size', 12)
        plt.rcParams['axes.grid'] = viz_config.get('enable_grid', True)
        plt.rcParams['grid.alpha'] = viz_config.get('grid_alpha', 0.3)
        plt.rcParams['font.family'] = 'sans-serif'

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi] range"""
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def load_and_validate_data(self):
        """Load and validate input CSV files"""
        try:
            # Load data
            self.df_gt = pd.read_csv(self.ground_truth_csv)
            self.df_ndt = pd.read_csv(self.ndt_csv)
            
            print(f"Loaded ground truth data: {len(self.df_gt)} records")
            print(f"Loaded NDT data: {len(self.df_ndt)} records")
            
            # Validate required columns
            required_cols = ['timestamp_ns', 'x', 'y', 'z', 'roll_rad', 'pitch_rad', 'yaw_rad']
            
            for df, name in [(self.df_gt, 'ground truth'), (self.df_ndt, 'NDT')]:
                missing_cols = [col for col in required_cols if col not in df.columns]
                if missing_cols:
                    print(f"Error: Missing columns in {name} data: {missing_cols}")
                    return False
            
            # Sort by timestamp
            self.df_gt.sort_values('timestamp_ns', inplace=True)
            self.df_ndt.sort_values('timestamp_ns', inplace=True)
            
            # Remove duplicates
            initial_gt = len(self.df_gt)
            initial_ndt = len(self.df_ndt)
            self.df_gt.drop_duplicates('timestamp_ns', inplace=True)
            self.df_ndt.drop_duplicates('timestamp_ns', inplace=True)
            
            print(f"After deduplication: GT {len(self.df_gt)}/{initial_gt}, NDT {len(self.df_ndt)}/{initial_ndt}")
            
            return True
            
        except FileNotFoundError as e:
            print(f"Error: File not found -> {e}")
            return False
        except Exception as e:
            print(f"Error loading data: {e}")
            return False

    def align_data_with_adaptive_window(self):
        """Align ground truth and NDT data using adaptive time window"""
        # Convert alignment windows from ms to ns
        tolerance_windows_ns = [w * 1_000_000 for w in self.alignment_windows]
        
        for tolerance_ns in tolerance_windows_ns:
            tolerance_ms = tolerance_ns / 1e6
            
            # Perform alignment
            df_aligned = pd.merge_asof(
                self.df_ndt,
                self.df_gt,
                on='timestamp_ns',
                direction='nearest',
                tolerance=tolerance_ns,
                suffixes=('_ndt', '_gt')
            )
            
            # Remove rows where alignment failed
            df_aligned.dropna(subset=['x_gt', 'y_gt'], inplace=True)
            
            alignment_rate = len(df_aligned) / len(self.df_ndt) * 100
            
            print(f"Alignment with {tolerance_ms:.0f}ms window: {len(df_aligned)}/{len(self.df_ndt)} "
                  f"({alignment_rate:.1f}%) records aligned")
            
            # Accept if we get good alignment rate
            if alignment_rate >= 80:
                self.df_aligned = df_aligned
                self.alignment_tolerance_ms = tolerance_ms
                return True
            elif alignment_rate >= self.min_alignment_rate:
                # Store as backup if we don't find better
                if not hasattr(self, 'df_aligned') or len(df_aligned) > len(self.df_aligned):
                    self.df_aligned = df_aligned
                    self.alignment_tolerance_ms = tolerance_ms
        
        if hasattr(self, 'df_aligned') and len(self.df_aligned) > 10:
            final_rate = len(self.df_aligned) / len(self.df_ndt) * 100
            print(f"Warning: Low alignment rate ({final_rate:.1f}%), but proceeding with analysis")
            return True
        else:
            print("Error: Data alignment failed completely. Check timestamp consistency.")
            return False

    def calculate_errors(self):
        """Calculate translation and rotation errors"""
        if self.df_aligned is None or len(self.df_aligned) == 0:
            print("Error: No aligned data available for error calculation")
            return False
        
        # Translation errors
        self.df_aligned['error_x'] = self.df_aligned['x_ndt'] - self.df_aligned['x_gt']
        self.df_aligned['error_y'] = self.df_aligned['y_ndt'] - self.df_aligned['y_gt']
        self.df_aligned['error_z'] = self.df_aligned['z_ndt'] - self.df_aligned['z_gt']
        
        # Translation error magnitude (Euclidean distance)
        self.df_aligned['translation_error_m'] = np.sqrt(
            self.df_aligned['error_x']**2 + 
            self.df_aligned['error_y']**2 + 
            self.df_aligned['error_z']**2
        )
        
        # Rotation errors (normalized and converted to degrees)
        self.df_aligned['roll_error_deg'] = np.degrees(
            self.normalize_angle(self.df_aligned['roll_rad_ndt'] - self.df_aligned['roll_rad_gt'])
        )
        self.df_aligned['pitch_error_deg'] = np.degrees(
            self.normalize_angle(self.df_aligned['pitch_rad_ndt'] - self.df_aligned['pitch_rad_gt'])
        )
        self.df_aligned['yaw_error_deg'] = np.degrees(
            self.normalize_angle(self.df_aligned['yaw_rad_ndt'] - self.df_aligned['yaw_rad_gt'])
        )
        
        # Absolute rotation errors
        self.df_aligned['abs_roll_error_deg'] = np.abs(self.df_aligned['roll_error_deg'])
        self.df_aligned['abs_pitch_error_deg'] = np.abs(self.df_aligned['pitch_error_deg'])
        self.df_aligned['abs_yaw_error_deg'] = np.abs(self.df_aligned['yaw_error_deg'])
        
        # Overall rotation error magnitude
        self.df_aligned['rotation_error_deg'] = np.sqrt(
            self.df_aligned['roll_error_deg']**2 + 
            self.df_aligned['pitch_error_deg']**2 + 
            self.df_aligned['yaw_error_deg']**2
        )
        
        return True

    def remove_outliers(self, df, column):
        """Remove outliers using IQR method"""
        if not self.outlier_removal_enabled:
            return df
            
        Q1 = df[column].quantile(0.25)
        Q3 = df[column].quantile(0.75)
        IQR = Q3 - Q1
        lower_bound = Q1 - self.outlier_iqr_factor * IQR
        upper_bound = Q3 + self.outlier_iqr_factor * IQR
        
        initial_count = len(df)
        df_filtered = df[(df[column] >= lower_bound) & (df[column] <= upper_bound)].copy()
        removed_count = initial_count - len(df_filtered)
        
        if removed_count > 0:
            print(f"Outlier removal for {column}: removed {removed_count}/{initial_count} "
                  f"({removed_count/initial_count*100:.1f}%) outliers")
        
        return df_filtered

    def apply_outlier_removal(self):
        """Apply outlier removal to key error metrics"""
        if not self.outlier_removal_enabled:
            return
            
        print("\nApplying outlier removal...")
        initial_count = len(self.df_aligned)
        
        # Remove outliers for translation error
        self.df_aligned = self.remove_outliers(self.df_aligned, 'translation_error_m')
        
        # Remove outliers for rotation errors
        self.df_aligned = self.remove_outliers(self.df_aligned, 'abs_yaw_error_deg')
        
        final_count = len(self.df_aligned)
        print(f"Final dataset: {final_count}/{initial_count} records retained "
              f"({final_count/initial_count*100:.1f}%)")

    def calculate_statistics(self):
        """Calculate comprehensive error statistics"""
        error_metrics = [
            'translation_error_m', 'abs_roll_error_deg', 
            'abs_pitch_error_deg', 'abs_yaw_error_deg', 'rotation_error_deg'
        ]
        
        self.analysis_results = {}
        
        for metric in error_metrics:
            data = self.df_aligned[metric].dropna()
            
            self.analysis_results[metric] = {
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
        
        # Save detailed results to JSON
        self.save_detailed_results()
        
        return self.analysis_results

    def save_detailed_results(self):
        """Save detailed analysis results to JSON file"""
        results_file = os.path.join(self.analysis_dir, f"fidelity_statistics_{self.analysis_timestamp}.json")
        
        # Prepare metadata
        metadata = {
            'analysis_timestamp': self.analysis_timestamp,
            'ground_truth_file': os.path.basename(self.ground_truth_csv),
            'ndt_file': os.path.basename(self.ndt_csv),
            'total_ground_truth_records': len(self.df_gt),
            'total_ndt_records': len(self.df_ndt),
            'aligned_records': len(self.df_aligned),
            'alignment_rate_percent': len(self.df_aligned) / len(self.df_ndt) * 100,
            'alignment_tolerance_ms': self.alignment_tolerance_ms,
            'outlier_removal_enabled': self.outlier_removal_enabled,
            'outlier_iqr_factor': self.outlier_iqr_factor,
            'analysis_date': datetime.now().isoformat()
        }
        
        # Convert numpy types for JSON serialization
        json_results = {}
        for metric, stats in self.analysis_results.items():
            json_results[metric] = {k: float(v) if hasattr(v, 'item') else int(v) if k == 'count' else v 
                                  for k, v in stats.items()}
        
        output = {
            'metadata': metadata,
            'error_statistics': json_results
        }
        
        with open(results_file, 'w') as f:
            json.dump(output, f, indent=2)
        
        print(f"Detailed results saved to: {results_file}")

    def print_statistics_table(self):
        """Print formatted statistics table"""
        print("\n" + "="*80)
        print("FSM SANDBOX FIDELITY AND SELF-CONSISTENCY ANALYSIS")
        print("="*80)
        
        print(f"\nDataset Information:")
        print(f"  Ground Truth Records: {len(self.df_gt)}")
        print(f"  NDT Output Records: {len(self.df_ndt)}")
        print(f"  Successfully Aligned: {len(self.df_aligned)} ({len(self.df_aligned)/len(self.df_ndt)*100:.1f}%)")
        print(f"  Alignment Tolerance: {self.alignment_tolerance_ms:.0f}ms")
        print(f"  Outlier Removal: {'Enabled' if self.outlier_removal_enabled else 'Disabled'}")
        
        print(f"\nVirtual Perception Localization Error Statistics:")
        print("-" * 80)
        print(f"{'Metric':<25} {'Mean':<10} {'Median':<10} {'Std':<10} {'P95':<10} {'P99':<10} {'Max':<10}")
        print("-" * 80)
        
        # Translation error (convert to mm for better readability)
        trans_stats = self.analysis_results['translation_error_m']
        print(f"{'Translation Error (mm)':<25} {trans_stats['mean']*1000:<10.2f} {trans_stats['median']*1000:<10.2f} "
              f"{trans_stats['std']*1000:<10.2f} {trans_stats['p95']*1000:<10.2f} {trans_stats['p99']*1000:<10.2f} "
              f"{trans_stats['max']*1000:<10.2f}")
        
        # Rotation errors
        for metric in ['abs_yaw_error_deg', 'abs_roll_error_deg', 'abs_pitch_error_deg']:
            stats = self.analysis_results[metric]
            metric_name = metric.replace('abs_', '').replace('_', ' ').replace('deg', '(deg)').title()
            print(f"{metric_name:<25} {stats['mean']:<10.3f} {stats['median']:<10.3f} "
                  f"{stats['std']:<10.3f} {stats['p95']:<10.3f} {stats['p99']:<10.3f} {stats['max']:<10.3f}")
        
        print("-" * 80)

    def assess_fidelity_quality(self):
        """Assess fidelity quality against configurable criteria"""
        print(f"\nFidelity Quality Assessment:")
        print("-" * 50)
        
        # Get quality thresholds from configuration
        thresholds = self.a2_config.get('quality_thresholds', {})
        
        # Define quality criteria using configuration
        criteria = [
            (f"Translation Error Mean < {thresholds.get('translation_mean_mm', 5)}mm", 
             self.analysis_results['translation_error_m']['mean'] < thresholds.get('translation_mean_mm', 5)/1000),
            (f"Translation Error P99 < {thresholds.get('translation_p99_mm', 20)}mm", 
             self.analysis_results['translation_error_m']['p99'] < thresholds.get('translation_p99_mm', 20)/1000),
            (f"Yaw Error Mean < {thresholds.get('yaw_mean_deg', 0.1)}deg", 
             self.analysis_results['abs_yaw_error_deg']['mean'] < thresholds.get('yaw_mean_deg', 0.1)),
            (f"Yaw Error P95 < {thresholds.get('yaw_p95_deg', 0.5)}deg", 
             self.analysis_results['abs_yaw_error_deg']['p95'] < thresholds.get('yaw_p95_deg', 0.5)),
            (f"Translation Std < {thresholds.get('translation_std_mm', 10)}mm", 
             self.analysis_results['translation_error_m']['std'] < thresholds.get('translation_std_mm', 10)/1000)
        ]
        
        passed_criteria = 0
        for criterion, passed in criteria:
            status = "PASS" if passed else "FAIL"
            print(f"  {criterion:<35}: {status}")
            if passed:
                passed_criteria += 1
        
        print(f"\nOverall Fidelity: {passed_criteria}/{len(criteria)} criteria passed")
        
        if passed_criteria == len(criteria):
            result = "EXCELLENT - Virtual LiDAR demonstrates exceptional fidelity"
        elif passed_criteria >= len(criteria) * 0.8:
            result = "GOOD - Virtual LiDAR shows high fidelity with minor deviations"
        elif passed_criteria >= len(criteria) * 0.6:
            result = "ACCEPTABLE - Virtual LiDAR fidelity meets basic requirements"
        else:
            result = "POOR - Virtual LiDAR fidelity requires significant improvement"
        
        print(f"Assessment: {result}")

    def create_error_analysis_plots(self):
        """Create comprehensive error analysis visualizations including spatial distribution"""
        fig, axes = plt.subplots(3, 2, figsize=(16, 18))
        fig.suptitle('FSM Sandbox Virtual LiDAR Fidelity Analysis', fontsize=16, fontweight='bold')
        
        # Convert timestamp to relative time in seconds for better visualization
        start_time = self.df_aligned['timestamp_ns'].iloc[0]
        self.df_aligned['time_sec'] = (self.df_aligned['timestamp_ns'] - start_time) / 1e9
        
        # Plot 1: Translation Error Over Time
        ax1 = axes[0, 0]
        ax1.plot(self.df_aligned['time_sec'], self.df_aligned['translation_error_m'] * 1000, 
                 color='blue', linewidth=1, alpha=0.7)
        ax1.axhline(y=self.analysis_results['translation_error_m']['mean'] * 1000, 
                   color='red', linestyle='--', linewidth=2, 
                   label=f'Mean: {self.analysis_results["translation_error_m"]["mean"]*1000:.2f}mm')
        ax1.axhline(y=self.analysis_results['translation_error_m']['p99'] * 1000, 
                   color='orange', linestyle='--', linewidth=2,
                   label=f'P99: {self.analysis_results["translation_error_m"]["p99"]*1000:.2f}mm')
        ax1.set_xlabel('Time (seconds)')
        ax1.set_ylabel('Translation Error (mm)')
        ax1.set_title('Translation Error Over Time')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        
        # Plot 2: Yaw Error Over Time
        ax2 = axes[0, 1]
        ax2.plot(self.df_aligned['time_sec'], self.df_aligned['abs_yaw_error_deg'], 
                 color='red', linewidth=1, alpha=0.7)
        ax2.axhline(y=self.analysis_results['abs_yaw_error_deg']['mean'], 
                   color='darkred', linestyle='--', linewidth=2,
                   label=f'Mean: {self.analysis_results["abs_yaw_error_deg"]["mean"]:.3f}deg')
        ax2.axhline(y=self.analysis_results['abs_yaw_error_deg']['p95'], 
                   color='orange', linestyle='--', linewidth=2,
                   label=f'P95: {self.analysis_results["abs_yaw_error_deg"]["p95"]:.3f}deg')
        ax2.set_xlabel('Time (seconds)')
        ax2.set_ylabel('Absolute Yaw Error (degrees)')
        ax2.set_title('Yaw Error Over Time')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        # Plot 3: Translation Error Distribution
        ax3 = axes[1, 0]
        trans_error_mm = self.df_aligned['translation_error_m'] * 1000
        ax3.hist(trans_error_mm, bins=50, alpha=0.7, color='skyblue', edgecolor='black')
        ax3.axvline(trans_error_mm.mean(), color='red', linestyle='--', linewidth=2, 
                   label=f'Mean: {trans_error_mm.mean():.2f}mm')
        ax3.axvline(trans_error_mm.quantile(0.99), color='orange', linestyle='--', linewidth=2,
                   label=f'P99: {trans_error_mm.quantile(0.99):.2f}mm')
        ax3.set_xlabel('Translation Error (mm)')
        ax3.set_ylabel('Frequency')
        ax3.set_title('Translation Error Distribution')
        ax3.legend()
        ax3.grid(True, alpha=0.3)
        
        # Plot 4: Yaw Error Distribution
        ax4 = axes[1, 1]
        yaw_error = self.df_aligned['abs_yaw_error_deg']
        ax4.hist(yaw_error, bins=50, alpha=0.7, color='lightcoral', edgecolor='black')
        ax4.axvline(yaw_error.mean(), color='darkred', linestyle='--', linewidth=2,
                   label=f'Mean: {yaw_error.mean():.3f}deg')
        ax4.axvline(yaw_error.quantile(0.95), color='orange', linestyle='--', linewidth=2,
                   label=f'P95: {yaw_error.quantile(0.95):.3f}deg')
        ax4.set_xlabel('Absolute Yaw Error (degrees)')
        ax4.set_ylabel('Frequency')
        ax4.set_title('Yaw Error Distribution')
        ax4.legend()
        ax4.grid(True, alpha=0.3)
        
        # Plot 5: Spatial Error Distribution (Top-down view)
        ax5 = axes[2, 0]
        sc = ax5.scatter(self.df_aligned['x_gt'], self.df_aligned['y_gt'], 
                        c=self.df_aligned['translation_error_m'] * 1000,  # color by error in mm
                        cmap='viridis', s=15, alpha=0.7)
        ax5.set_xlabel('X Coordinate (m)')
        ax5.set_ylabel('Y Coordinate (m)')
        ax5.set_title('Spatial Distribution of Translation Error')
        ax5.set_aspect('equal', adjustable='box')
        cbar = plt.colorbar(sc, ax=ax5)
        cbar.set_label('Translation Error (mm)')
        ax5.grid(True, alpha=0.3)
        
        # Plot 6: Error Correlation Analysis
        ax6 = axes[2, 1]
        ax6.scatter(self.df_aligned['translation_error_m'] * 1000, 
                   self.df_aligned['abs_yaw_error_deg'], 
                   alpha=0.6, s=10)
        ax6.set_xlabel('Translation Error (mm)')
        ax6.set_ylabel('Absolute Yaw Error (degrees)')
        ax6.set_title('Translation vs Rotation Error Correlation')
        ax6.grid(True, alpha=0.3)
        
        # Calculate and display correlation
        correlation = np.corrcoef(self.df_aligned['translation_error_m'], 
                                 self.df_aligned['abs_yaw_error_deg'])[0, 1]
        ax6.text(0.05, 0.95, f'Correlation: {correlation:.3f}', 
                transform=ax6.transAxes, bbox=dict(boxstyle="round", facecolor='wheat'))
        
        plt.tight_layout()
        
        # Save plot
        plot_filename = os.path.join(self.analysis_dir, f'fidelity_error_analysis_{self.analysis_timestamp}.png')
        plt.savefig(plot_filename, dpi=self.config_manager.get('visualization.dpi', 300), bbox_inches='tight')
        print(f"Error analysis plots saved: {plot_filename}")
        
        return fig

    def create_summary_report(self):
        """Create a comprehensive summary report"""
        # Get quality thresholds for report
        thresholds = self.a2_config.get('quality_thresholds', {})
        
        report_content = f"""# FSM Sandbox Fidelity Analysis Report

## Analysis Information
- **Analysis Timestamp**: {self.analysis_timestamp}
- **Ground Truth Source**: {os.path.basename(self.ground_truth_csv)}
- **NDT Output Source**: {os.path.basename(self.ndt_csv)}
- **Analysis Date**: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}

## Configuration
- **Outlier Removal**: {'Enabled' if self.outlier_removal_enabled else 'Disabled'}
- **IQR Factor**: {self.outlier_iqr_factor}
- **Alignment Tolerance**: {self.alignment_tolerance_ms:.0f}ms

## Data Summary
- **Ground Truth Records**: {len(self.df_gt)}
- **NDT Output Records**: {len(self.df_ndt)}
- **Successfully Aligned**: {len(self.df_aligned)} ({len(self.df_aligned)/len(self.df_ndt)*100:.1f}%)

## Key Performance Metrics

### Translation Error
- **Mean**: {self.analysis_results['translation_error_m']['mean']*1000:.2f}mm
- **Standard Deviation**: {self.analysis_results['translation_error_m']['std']*1000:.2f}mm
- **99th Percentile**: {self.analysis_results['translation_error_m']['p99']*1000:.2f}mm
- **Maximum**: {self.analysis_results['translation_error_m']['max']*1000:.2f}mm

### Yaw Error
- **Mean**: {self.analysis_results['abs_yaw_error_deg']['mean']:.3f}°
- **Standard Deviation**: {self.analysis_results['abs_yaw_error_deg']['std']:.3f}°
- **95th Percentile**: {self.analysis_results['abs_yaw_error_deg']['p95']:.3f}°
- **Maximum**: {self.analysis_results['abs_yaw_error_deg']['max']:.3f}°

## Quality Assessment
"""
        
        # Add quality assessment
        criteria_passed = 0
        total_criteria = 5
        
        trans_mean_mm = self.analysis_results['translation_error_m']['mean'] * 1000
        trans_p99_mm = self.analysis_results['translation_error_m']['p99'] * 1000
        trans_std_mm = self.analysis_results['translation_error_m']['std'] * 1000
        yaw_mean_deg = self.analysis_results['abs_yaw_error_deg']['mean']
        yaw_p95_deg = self.analysis_results['abs_yaw_error_deg']['p95']
        
        # Check criteria
        if trans_mean_mm < thresholds.get('translation_mean_mm', 5):
            criteria_passed += 1
        if trans_p99_mm < thresholds.get('translation_p99_mm', 20):
            criteria_passed += 1
        if yaw_mean_deg < thresholds.get('yaw_mean_deg', 0.1):
            criteria_passed += 1
        if yaw_p95_deg < thresholds.get('yaw_p95_deg', 0.5):
            criteria_passed += 1
        if trans_std_mm < thresholds.get('translation_std_mm', 10):
            criteria_passed += 1
        
        report_content += f"\n**Quality Score**: {criteria_passed}/{total_criteria} criteria passed\n\n"
        
        if criteria_passed == total_criteria:
            conclusion = "The virtual LiDAR demonstrates **EXCELLENT** fidelity with exceptional precision."
        elif criteria_passed >= 4:
            conclusion = "The virtual LiDAR shows **GOOD** fidelity suitable for most applications."
        elif criteria_passed >= 3:
            conclusion = "The virtual LiDAR demonstrates **ACCEPTABLE** fidelity for basic applications."
        else:
            conclusion = "The virtual LiDAR fidelity requires **IMPROVEMENT** for reliable localization."
        
        report_content += f"## Conclusion\n\n{conclusion}\n\n"
        
        report_content += f"""## Generated Files
- `fidelity_statistics_{self.analysis_timestamp}.json` - Detailed statistics
- `fidelity_error_analysis_{self.analysis_timestamp}.png` - Comprehensive error analysis plots
- `fidelity_report_{self.analysis_timestamp}.md` - This summary report
"""
        
        # Save report
        report_file = os.path.join(self.analysis_dir, f"fidelity_report_{self.analysis_timestamp}.md")
        with open(report_file, 'w') as f:
            f.write(report_content)
        
        print(f"Summary report saved: {report_file}")

    def run_complete_analysis(self):
        """Run the complete fidelity analysis pipeline"""
        print("Starting FSM Sandbox Fidelity Analysis...")
        print(f"Output directory: {self.analysis_dir}")
        
        # Step 1: Load and validate data
        if not self.load_and_validate_data():
            return False
        
        # Step 2: Align data
        if not self.align_data_with_adaptive_window():
            return False
        
        # Step 3: Calculate errors
        if not self.calculate_errors():
            return False
        
        # Step 4: Apply outlier removal
        self.apply_outlier_removal()
        
        # Step 5: Calculate statistics
        self.calculate_statistics()
        
        # Step 6: Generate outputs
        self.print_statistics_table()
        self.assess_fidelity_quality()
        
        print(f"\nGenerating visualizations...")
        self.create_error_analysis_plots()
        
        print(f"\nGenerating summary report...")
        self.create_summary_report()
        
        print(f"\nFidelity analysis complete!")
        print(f"All results saved to: {self.analysis_dir}")
        return True


def main():
    parser = argparse.ArgumentParser(description='Analyze FSM Sandbox virtual LiDAR fidelity')
    parser.add_argument('ground_truth_csv', help='Path to ground truth poses CSV file')
    parser.add_argument('ndt_csv', help='Path to offline NDT poses CSV file')
    parser.add_argument('--config', default='config.yaml', help='Configuration file path')
    parser.add_argument('--show-plots', action='store_true', help='Display plots after generation')
    
    args = parser.parse_args()
    
    # Validate input files
    for file_path, name in [(args.ground_truth_csv, 'ground truth'), (args.ndt_csv, 'NDT')]:
        if not os.path.exists(file_path):
            print(f"Error: {name} file not found: {file_path}")
            sys.exit(1)
    
    # Run analysis
    analyzer = FSMFidelityAnalyzer(args.ground_truth_csv, args.ndt_csv, args.config)
    success = analyzer.run_complete_analysis()
    
    if success and args.show_plots:
        plt.show()
    
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
```

### 5. 依赖管理 (requirements.txt)

```txt
# FSM Sandbox Experiment Dependencies
pandas>=1.3.0
numpy>=1.20.0
matplotlib>=3.3.0
seaborn>=0.11.0
transforms3d>=0.3.1
pyyaml>=5.4.0
scipy>=1.7.0

# Optional: For better performance
numba>=0.53.0

# ROS2 dependencies (if not already installed)
# rclpy (usually installed with ROS2)
```

### 6. 安装脚本 (install_dependencies.sh)

```bash
#!/bin/bash

echo "Installing FSM Sandbox Experiment Dependencies..."

# Check if virtual environment should be used
if [ "$1" = "--venv" ]; then
    echo "Setting up virtual environment..."
    python3 -m venv fsm_env
    source fsm_env/bin/activate
    echo "Virtual environment activated."
fi

# Upgrade pip first
python3 -m pip install --upgrade pip

# Install dependencies from requirements.txt
if [ -f "requirements.txt" ]; then
    echo "Installing packages from requirements.txt..."
    pip3 install -r requirements.txt
else
    echo "requirements.txt not found, installing packages individually..."
    pip3 install pandas numpy matplotlib seaborn transforms3d pyyaml scipy
fi

# Install ROS2 dependencies if needed
echo "Checking ROS2 dependencies..."
sudo apt update
sudo apt install -y python3-pandas python3-matplotlib python3-numpy python3-scipy python3-yaml

# Verify installation
echo "Verifying installation..."
python3 -c "import pandas, numpy, matplotlib, seaborn, transforms3d, yaml, scipy; print('All dependencies installed successfully!')"

echo "Installation complete!"
echo ""
echo "Usage:"
echo "  Normal installation: ./install_dependencies.sh"
echo "  With virtual env: ./install_dependencies.sh --venv"
```

### 7. 完整使用指南

#### 配置设置

```bash
# 1. 首次运行会自动生成config.yaml，然后编辑它
python3 log_recorder_node.py --config config.yaml
# 编辑生成的config.yaml文件，设置正确的话题名称

# 2. 验证配置
python3 -c "from config_manager import ConfigManager; cm = ConfigManager(); print('Configuration loaded successfully')"
```

#### 运行实验

```bash
# 阶段一：在线数据收集（A.1 + A.2同步）
# Terminal 1: A.1时效性分析
python3 log_recorder_node.py --config config.yaml

# Terminal 2: A.2保真度分析 - 真值记录
python3 ground_truth_recorder.py

# Terminal 3: ROS bag录制
ros2 bag record -o fidelity_lidar_bag /carla/follow_adtruck/carla_pointcloud

# 阶段二：离线NDT处理
python3 offline_ndt_recorder.py
ros2 bag play fidelity_lidar_bag

# 阶段三：数据分析
python3 analyze_fidelity_data.py data/ground_truth_poses_*.csv data/offline_ndt_poses_*.csv --config config.yaml
```

这个优化方案整合了所有建议，提供了：

1. **外部化配置管理** - 通过YAML文件和专门的配置管理器
2. **智能异常值处理** - 可配置的IQR方法
3. **空间误差分析** - 新增的空间分布可视化
4. **标准依赖管理** - requirements.txt和安装脚本
5. **增强的鲁棒性** - 更好的错误处理和数据验证
6. **完全的英文输出** - 所有图表和报告都使用英文

该方案现在具备了生产环境的质量标准，易于维护和扩展。