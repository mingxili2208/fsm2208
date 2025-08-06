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