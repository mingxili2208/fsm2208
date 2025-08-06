# 主要改进包括

1. **统一目录结构管理** - 与A.1实验保持一致的文件组织
2. **完善的错误处理和数据验证**
3. **更robust的时间戳对齐算法**
4. **英文输出和清晰的可视化**
5. **与A.1实验的完美集成**

## 优化后的完整实验方案

### 1. 实验设计优化点

#### 1.1 与A.1实验的集成策略

- **同时启动两个数据收集节点**，一次运行收集所有需要的数据
- **统一的目录结构**，便于数据管理和分析
- **共享时间戳**，确保数据一致性

#### 1.2 数据质量保证

- **多重数据验证**机制
- **自适应时间窗口对齐**算法
- **异常值检测和处理**

## 2. 优化后的完整代码

### 代码1: `ground_truth_recorder.py` (优化版)

```python
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
```

### 代码2: `offline_ndt_recorder.py` (优化版)

```python
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
```

### 代码3: `analyze_fidelity_data.py` (优化版)

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

class FSMFidelityAnalyzer:
    """
    Analyzer for FSM Sandbox fidelity and self-consistency validation.
    Compares offline NDT localization results with ground truth poses to assess virtual LiDAR quality.
    """
    
    def __init__(self, ground_truth_csv, ndt_csv):
        self.ground_truth_csv = ground_truth_csv
        self.ndt_csv = ndt_csv
        self.df_aligned = None
        self.analysis_results = {}
        
        # Create analysis timestamp
        self.analysis_timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # Setup directory structure
        self.setup_directories()
        
        # Configure plotting
        plt.style.use('default')
        sns.set_palette("husl")
        plt.rcParams['figure.figsize'] = (14, 10)
        plt.rcParams['font.size'] = 12
        plt.rcParams['axes.grid'] = True
        plt.rcParams['grid.alpha'] = 0.3
        plt.rcParams['font.family'] = 'sans-serif'

    def setup_directories(self):
        """Setup directory structure for results"""
        self.base_dir = os.path.dirname(os.path.abspath(__file__))
        self.results_dir = os.path.join(self.base_dir, "results")
        self.analysis_dir = os.path.join(self.results_dir, f"fidelity_analysis_{self.analysis_timestamp}")
        
        for directory in [self.results_dir, self.analysis_dir]:
            os.makedirs(directory, exist_ok=True)
        
        print(f"Fidelity analysis output directory: {self.analysis_dir}")

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
        """
        Align ground truth and NDT data using adaptive time window.
        Uses progressively larger windows if initial alignment fails.
        """
        tolerance_windows = [10_000_000, 50_000_000, 100_000_000, 200_000_000]  # 10ms to 200ms
        
        for tolerance_ns in tolerance_windows:
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
            elif alignment_rate >= 50:
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
        """Assess fidelity quality against predefined criteria"""
        print(f"\nFidelity Quality Assessment:")
        print("-" * 50)
        
        # Define quality criteria
        criteria = [
            ("Translation Error Mean < 5mm", self.analysis_results['translation_error_m']['mean'] < 0.005),
            ("Translation Error P99 < 20mm", self.analysis_results['translation_error_m']['p99'] < 0.020),
            ("Yaw Error Mean < 0.1deg", self.analysis_results['abs_yaw_error_deg']['mean'] < 0.1),
            ("Yaw Error P95 < 0.5deg", self.analysis_results['abs_yaw_error_deg']['p95'] < 0.5),
            ("Translation Std < 10mm", self.analysis_results['translation_error_m']['std'] < 0.010)
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
        """Create comprehensive error analysis visualizations"""
        fig, axes = plt.subplots(2, 2, figsize=(16, 12))
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
        
        plt.tight_layout()
        
        # Save plot
        plot_filename = os.path.join(self.analysis_dir, f'fidelity_error_analysis_{self.analysis_timestamp}.png')
        plt.savefig(plot_filename, dpi=300, bbox_inches='tight')
        print(f"Error analysis plots saved: {plot_filename}")
        
        return fig

    def create_summary_report(self):
        """Create a comprehensive summary report"""
        report_content = f"""# FSM Sandbox Fidelity Analysis Report

## Analysis Information
- **Analysis Timestamp**: {self.analysis_timestamp}
- **Ground Truth Source**: {os.path.basename(self.ground_truth_csv)}
- **NDT Output Source**: {os.path.basename(self.ndt_csv)}
- **Analysis Date**: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}

## Data Summary
- **Ground Truth Records**: {len(self.df_gt)}
- **NDT Output Records**: {len(self.df_ndt)}
- **Successfully Aligned**: {len(self.df_aligned)} ({len(self.df_aligned)/len(self.df_ndt)*100:.1f}%)
- **Alignment Tolerance**: {self.alignment_tolerance_ms:.0f}ms

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

## Generated Files
- `fidelity_statistics_{self.analysis_timestamp}.json` - Detailed statistics
- `fidelity_error_analysis_{self.analysis_timestamp}.png` - Error analysis plots
- `fidelity_report_{self.analysis_timestamp}.md` - This summary report

## Conclusion
"""
        
        # Add quality assessment
        trans_mean_mm = self.analysis_results['translation_error_m']['mean'] * 1000
        yaw_mean_deg = self.analysis_results['abs_yaw_error_deg']['mean']
        
        if trans_mean_mm < 5 and yaw_mean_deg < 0.1:
            conclusion = "The virtual LiDAR demonstrates **EXCELLENT** fidelity with sub-millimeter precision."
        elif trans_mean_mm < 20 and yaw_mean_deg < 0.5:
            conclusion = "The virtual LiDAR shows **GOOD** fidelity suitable for most applications."
        else:
            conclusion = "The virtual LiDAR fidelity may require **IMPROVEMENT** for high-precision applications."
        
        report_content += conclusion
        
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
        
        # Step 4: Calculate statistics
        self.calculate_statistics()
        
        # Step 5: Generate outputs
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
    parser.add_argument('--show-plots', action='store_true', help='Display plots after generation')
    
    args = parser.parse_args()
    
    # Validate input files
    for file_path, name in [(args.ground_truth_csv, 'ground truth'), (args.ndt_csv, 'NDT')]:
        if not os.path.exists(file_path):
            print(f"Error: {name} file not found: {file_path}")
            sys.exit(1)
    
    # Run analysis
    analyzer = FSMFidelityAnalyzer(args.ground_truth_csv, args.ndt_csv)
    success = analyzer.run_complete_analysis()
    
    if success and args.show_plots:
        plt.show()
    
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
```

## 3. 完整的使用指南

### 阶段一：在线数据收集（与A.1同步）

```bash
# Terminal 1: 启动A.1的时效性记录节点
python3 log_recorder_node.py

# Terminal 2: 启动保真度实验的真值记录节点
python3 ground_truth_recorder.py

# Terminal 3: 启动ROS bag录制LiDAR数据
ros2 bag record -o fidelity_lidar_bag /carla/follow_adtruck/carla_pointcloud

# Terminal 4: 启动你的完整FSM Sandbox系统
# - CARLA Server
# - Autoware
# - 修改后的CarlaUpdateVehicleHandler
# - RBF坐标变换节点

# 让系统运行3-5分钟，然后停止所有记录
```

### 阶段二：离线NDT处理

```bash
# Terminal 1: 启动独立NDT模块（确保使用相同地图）
ros2 launch your_ndt_package ndt_localization.launch.py

# Terminal 2: 启动离线NDT记录节点
python3 offline_ndt_recorder.py

# Terminal 3: 回放LiDAR数据
ros2 bag play fidelity_lidar_bag

# 等待回放完成，停止所有节点
```

### 阶段三：数据分析

```bash
# 执行保真度分析
python3 analyze_fidelity_data.py data/ground_truth_poses_YYYYMMDD_HHMMSS.csv data/offline_ndt_poses_YYYYMMDD_HHMMSS.csv

# 查看结果
ls results/fidelity_analysis_*/
```

## 4. 预期输出文件结构

```bash
A1_sensor_latency/
├── data/
│   ├── fsm_timing_data_YYYYMMDD_HHMMSS.csv      # A.1实验数据
│   ├── ground_truth_poses_YYYYMMDD_HHMMSS.csv   # A.2真值数据
│   └── offline_ndt_poses_YYYYMMDD_HHMMSS.csv    # A.2离线NDT数据
├── results/
│   ├── analysis_YYYYMMDD_HHMMSS/                # A.1分析结果
│   └── fidelity_analysis_YYYYMMDD_HHMMSS/       # A.2分析结果
│       ├── fidelity_statistics_YYYYMMDD_HHMMSS.json
│       ├── fidelity_error_analysis_YYYYMMDD_HHMMSS.png
│       └── fidelity_report_YYYYMMDD_HHMMSS.md
└── logs/
    ├── fsm_session_log_YYYYMMDD_HHMMSS.log
    └── ... (其他日志文件)
```

这个优化方案提供了：

1. **完整的错误处理**和数据验证
2. **自适应时间窗口对齐**算法
3. **清晰的英文输出**和专业的可视化
4. **与A.1实验的完美集成**
5. **全面的质量评估**指标和报告生成
