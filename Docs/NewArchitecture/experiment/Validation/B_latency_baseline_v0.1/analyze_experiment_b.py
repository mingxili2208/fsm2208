#!/usr/bin/env python3

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
import argparse
import os
import sys
import json
from datetime import datetime
from scipy import stats
from typing import Tuple, Dict, List

# Import configuration manager
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from config_manager import ConfigManager

class ExperimentBAnalyzer:
    """
    Comprehensive analyzer for Experiment B: Latency characteristics and baseline tracking error analysis.
    Provides detailed latency breakdown and validates the relationship between latency and tracking error.
    """
    
    def __init__(self, latency_csv: str, error_csv: str, config_file: str = "config.yaml"):
        self.latency_csv = latency_csv
        self.error_csv = error_csv
        
        # Load configuration
        self.config_manager = ConfigManager(config_file)
        self.b_config = self.config_manager.get('experiment_b', {})
        
        # Analysis timestamp
        self.analysis_timestamp = datetime.now().strftime(
            self.config_manager.get('file_management.file_naming.timestamp_format', '%Y%m%d_%H%M%S')
        )
        
        # Setup directories and plotting
        self.setup_directories()
        self.setup_plotting()
        
        # Load and validate data
        self.load_and_validate_data()
        
        # Analysis results storage
        self.analysis_results = {}

    def setup_directories(self):
        """Setup directory structure for results"""
        self.base_dir = os.path.dirname(os.path.abspath(__file__))
        dirs = self.config_manager.get('file_management.base_directories', {})
        
        self.results_dir = os.path.join(self.base_dir, dirs.get('results', 'results'))
        self.analysis_dir = os.path.join(self.results_dir, f"experiment_b_analysis_{self.analysis_timestamp}")
        
        for directory in [self.results_dir, self.analysis_dir]:
            os.makedirs(directory, exist_ok=True)
        
        print(f"Analysis output directory: {self.analysis_dir}")

    def setup_plotting(self):
        """Setup plotting configuration"""
        viz_config = self.config_manager.get('visualization', {})
        
        plt.style.use(viz_config.get('style', 'seaborn-v0_8-whitegrid'))
        if 'color_palette' in viz_config:
            sns.set_palette(viz_config['color_palette'])
        
        # Configure matplotlib parameters
        plt.rcParams['figure.figsize'] = viz_config.get('figure_size', [16, 12])
        plt.rcParams['font.size'] = viz_config.get('font_size', 12)
        plt.rcParams['axes.grid'] = viz_config.get('enable_grid', True)
        plt.rcParams['grid.alpha'] = viz_config.get('grid_alpha', 0.3)
        plt.rcParams['font.family'] = 'sans-serif'

    def load_and_validate_data(self):
        """Load and validate input CSV files"""
        try:
            # Load latency data
            self.latency_df = pd.read_csv(self.latency_csv)
            print(f"Loaded latency data: {len(self.latency_df)} records")
            
            # Load tracking error data
            self.error_df = pd.read_csv(self.error_csv)
            print(f"Loaded tracking error data: {len(self.error_df)} records")
            
            # Validate latency data columns
            required_latency_cols = [
                'information_age_ms', 'planner_latency_ms', 
                'decision_to_actuation_ms', 'total_latency_ms'
            ]
            missing_latency_cols = [col for col in required_latency_cols if col not in self.latency_df.columns]
            if missing_latency_cols:
                raise ValueError(f"Missing latency columns: {missing_latency_cols}")
            
            # Validate tracking error data columns
            required_error_cols = [
                'elapsed_time_s', 's_actual', 's_target', 
                'error_longitudinal_m', 'error_lateral_m'
            ]
            missing_error_cols = [col for col in required_error_cols if col not in self.error_df.columns]
            if missing_error_cols:
                raise ValueError(f"Missing error columns: {missing_error_cols}")
            
            # Clean data
            self.clean_data()
            
            print("Data validation successful")
            
        except Exception as e:
            print(f"Error loading or validating data: {e}")
            raise

    def clean_data(self):
        """Clean and filter data"""
        # Remove invalid latency entries
        initial_latency_count = len(self.latency_df)
        self.latency_df = self.latency_df[
            (self.latency_df['valid_correlation'] == True) &
            (self.latency_df['total_latency_ms'] > 0) &
            (self.latency_df['total_latency_ms'] < 2000) &  # Remove extreme outliers
            (self.latency_df['information_age_ms'] >= 0) &
            (self.latency_df['planner_latency_ms'] >= 0) &
            (self.latency_df['decision_to_actuation_ms'] >= 0)
        ]
        
        # Remove outliers using percentile filtering
        outlier_threshold = self.b_config.get('parameters', {}).get('outlier_percentile_threshold', 99)
        threshold_value = self.latency_df['total_latency_ms'].quantile(outlier_threshold / 100)
        self.latency_df = self.latency_df[self.latency_df['total_latency_ms'] <= threshold_value]
        
        cleaned_latency_count = len(self.latency_df)
        print(f"Cleaned latency data: {cleaned_latency_count}/{initial_latency_count} records retained")
        
        # Clean error data - remove entries where test was not active
        initial_error_count = len(self.error_df)
        if 'test_active' in self.error_df.columns:
            self.error_df = self.error_df[self.error_df['test_active'] == True]
        
        # Remove extreme error outliers
        error_threshold = self.error_df['error_longitudinal_m'].quantile(0.99)
        self.error_df = self.error_df[
            abs(self.error_df['error_longitudinal_m']) <= abs(error_threshold)
        ]
        
        cleaned_error_count = len(self.error_df)
        print(f"Cleaned error data: {cleaned_error_count}/{initial_error_count} records retained")

    def analyze_latency_characteristics(self):
        """Perform detailed latency analysis (Part 1 of Experiment B)"""
        print("\n" + "="*60)
        print("EXPERIMENT B - PART 1: LATENCY CHARACTERISTICS ANALYSIS")
        print("="*60)
        
        # Calculate statistics for each latency component
        latency_components = [
            'information_age_ms', 'planner_latency_ms', 
            'decision_to_actuation_ms', 'total_latency_ms'
        ]
        
        latency_stats = {}
        for component in latency_components:
            data = self.latency_df[component]
            latency_stats[component] = {
                'count': len(data),
                'mean': data.mean(),
                'median': data.median(),
                'std': data.std(),
                'min': data.min(),
                'max': data.max(),
                'p95': data.quantile(0.95),
                'p99': data.quantile(0.99)
            }
        
        # Print latency analysis table
        self.print_latency_table(latency_stats)
        
        # Store results
        self.analysis_results['latency_statistics'] = latency_stats
        
        # Generate latency plots
        self.create_latency_analysis_plots()
        
        return latency_stats

    def print_latency_table(self, stats: Dict):
        """Print formatted latency statistics table"""
        print(f"\nLatency Component Analysis (n={stats['total_latency_ms']['count']} samples)")
        print("-" * 100)
        print(f"{'Component':<25} {'Mean':<8} {'Median':<8} {'Std':<8} {'Min':<8} {'Max':<8} {'P95':<8} {'P99':<8}")
        print("-" * 100)
        
        component_names = {
            'information_age_ms': 'Information Age (ms)',
            'planner_latency_ms': 'Planner Latency (ms)',
            'decision_to_actuation_ms': 'Decision-to-Act (ms)',
            'total_latency_ms': 'Total Latency (ms)'
        }
        
        for component, name in component_names.items():
            s = stats[component]
            print(f"{name:<25} {s['mean']:<8.2f} {s['median']:<8.2f} {s['std']:<8.2f} "
                  f"{s['min']:<8.2f} {s['max']:<8.2f} {s['p95']:<8.2f} {s['p99']:<8.2f}")
        
        print("-" * 100)

    def assess_latency_quality(self, stats: Dict):
        """Assess latency quality against performance thresholds"""
        print(f"\nLatency Performance Assessment:")
        print("-" * 50)
        
        # Get quality thresholds from configuration
        thresholds = self.b_config.get('quality_thresholds', {})
        
        criteria = [
            ('Total Latency Mean < 200ms', 
             stats['total_latency_ms']['mean'] < thresholds.get('max_acceptable_total_latency_ms', 200)),
            ('Decision-to-Actuation P95 < 50ms',
             stats['decision_to_actuation_ms']['p95'] < thresholds.get('max_acceptable_d2a_latency_ms', 50)),
            ('Planner Latency P95 < 100ms',
             stats['planner_latency_ms']['p95'] < thresholds.get('max_acceptable_planner_latency_ms', 100)),
            ('Total Latency Stability (CV < 0.5)',
             (stats['total_latency_ms']['std'] / stats['total_latency_ms']['mean']) < 0.5)
        ]
        
        passed_criteria = 0
        for criterion, passed in criteria:
            status = "PASS" if passed else "FAIL"
            print(f"  {criterion:<40}: {status}")
            if passed:
                passed_criteria += 1
        
        print(f"\nLatency Performance: {passed_criteria}/{len(criteria)} criteria passed")
        
        if passed_criteria == len(criteria):
            assessment = "EXCELLENT - System latency well within acceptable bounds"
        elif passed_criteria >= len(criteria) * 0.8:
            assessment = "GOOD - System latency acceptable for most applications"
        elif passed_criteria >= len(criteria) * 0.6:
            assessment = "ACCEPTABLE - Some latency concerns but functional"
        else:
            assessment = "POOR - System latency may interfere with ADS performance"
        
        print(f"Assessment: {assessment}")
        return assessment

    def analyze_tracking_error_baseline(self):
        """Perform tracking error baseline analysis (Part 2 of Experiment B)"""
        print("\n" + "="*60)
        print("EXPERIMENT B - PART 2: TRACKING ERROR BASELINE ANALYSIS")
        print("="*60)
        
        if len(self.error_df) == 0:
            print("Error: No tracking error data available for analysis")
            return None
        
        # Get test configuration
        test_config = self.b_config.get('test_scenarios', {}).get('constant_velocity_test', {})
        target_velocity = test_config.get('target_velocity_ms', 0.5)
        
        # Calculate theoretical error from latency
        mean_total_latency_s = self.analysis_results['latency_statistics']['total_latency_ms']['mean'] / 1000.0
        theoretical_error = target_velocity * mean_total_latency_s
        
        # Calculate actual error statistics
        actual_errors = self.error_df['error_longitudinal_m']
        actual_error_mean = actual_errors.mean()
        actual_error_std = actual_errors.std()
        actual_error_median = actual_errors.median()
        
        # Calculate agreement between theoretical and actual
        agreement_ratio = actual_error_mean / theoretical_error if theoretical_error != 0 else 0
        
        print(f"\nTracking Error Baseline Analysis:")
        print("-" * 50)
        print(f"Test Configuration:")
        print(f"  Target Velocity: {target_velocity:.3f} m/s")
        print(f"  Mean Total Latency: {mean_total_latency_s*1000:.2f} ms")
        print(f"\nError Analysis:")
        print(f"  Theoretical Error (v × ΔT): {theoretical_error:.6f} m")
        print(f"  Actual Error Mean: {actual_error_mean:.6f} m")
        print(f"  Actual Error Std: {actual_error_std:.6f} m")
        print(f"  Actual Error Median: {actual_error_median:.6f} m")
        print(f"  Agreement Ratio: {agreement_ratio:.3f}")
        
        # Store results
        error_analysis = {
            'target_velocity': target_velocity,
            'mean_latency_ms': mean_total_latency_s * 1000,
            'theoretical_error_m': theoretical_error,
            'actual_error_mean_m': actual_error_mean,
            'actual_error_std_m': actual_error_std,
            'actual_error_median_m': actual_error_median,
            'agreement_ratio': agreement_ratio,
            'sample_count': len(actual_errors)
        }
        
        self.analysis_results['error_baseline'] = error_analysis
        
        # Assess prediction accuracy
        accuracy_threshold = self.b_config.get('quality_thresholds', {}).get('error_prediction_accuracy_threshold', 0.8)
        prediction_accurate = 0.5 <= agreement_ratio <= 1.5  # Within 50% agreement
        
        print(f"\nBaseline Error Prediction Assessment:")
        accuracy_status = "ACCURATE" if prediction_accurate else "INACCURATE"
        print(f"  Prediction Accuracy: {accuracy_status}")
        print(f"  Agreement within acceptable range (0.5-1.5): {prediction_accurate}")
        
        # Generate error analysis plots
        self.create_error_analysis_plots()
        
        return error_analysis

    def create_latency_analysis_plots(self):
        """Create comprehensive latency analysis visualizations"""
        print(f"\nGenerating latency analysis plots...")
        
        # Create figure with subplots
        fig = plt.figure(figsize=(20, 16))
        
        # Plot 1: Latency component breakdown (stacked bar)
        ax1 = plt.subplot(3, 2, 1)
        components = ['information_age_ms', 'planner_latency_ms', 'decision_to_actuation_ms']
        component_labels = ['Information Age', 'Planner', 'Decision-to-Actuation']
        means = [self.analysis_results['latency_statistics'][comp]['mean'] for comp in components]
        
        ax1.bar(range(len(component_labels)), means, color=['skyblue', 'lightcoral', 'lightgreen'])
        ax1.set_xlabel('Latency Component')
        ax1.set_ylabel('Mean Latency (ms)')
        ax1.set_title('Mean Latency by Component')
        ax1.set_xticks(range(len(component_labels)))
        ax1.set_xticklabels(component_labels, rotation=45)
        ax1.grid(True, alpha=0.3)
        
        # Add value labels on bars
        for i, v in enumerate(means):
            ax1.text(i, v + max(means)*0.01, f'{v:.1f}ms', ha='center', va='bottom')
        
        # Plot 2: Total latency distribution
        ax2 = plt.subplot(3, 2, 2)
        total_latencies = self.latency_df['total_latency_ms']
        ax2.hist(total_latencies, bins=50, alpha=0.7, color='steelblue', edgecolor='black')
        ax2.axvline(total_latencies.mean(), color='red', linestyle='--', linewidth=2,
                   label=f'Mean: {total_latencies.mean():.1f}ms')
        ax2.axvline(total_latencies.quantile(0.95), color='orange', linestyle='--', linewidth=2,
                   label=f'P95: {total_latencies.quantile(0.95):.1f}ms')
        ax2.set_xlabel('Total Latency (ms)')
        ax2.set_ylabel('Frequency')
        ax2.set_title('Total End-to-End Latency Distribution')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        # Plot 3: Latency composition (pie chart)
        ax3 = plt.subplot(3, 2, 3)
        sizes = means
        ax3.pie(sizes, labels=component_labels, autopct='%1.1f%%', startangle=90,
                colors=['skyblue', 'lightcoral', 'lightgreen'])
        ax3.set_title('Latency Composition Breakdown')
        
        # Plot 4: Latency time series
        ax4 = plt.subplot(3, 2, 4)
        if 'timestamp_ns' in self.latency_df.columns:
            # Convert timestamps to relative time
            start_time = self.latency_df['timestamp_ns'].iloc[0] if len(self.latency_df) > 0 else 0
            relative_time = (self.latency_df['timestamp_ns'] - start_time) / 1e9
            ax4.plot(relative_time, self.latency_df['total_latency_ms'], alpha=0.7, linewidth=1)
            ax4.set_xlabel('Time (seconds)')
        else:
            # Use sample index if no timestamps
            ax4.plot(self.latency_df['total_latency_ms'], alpha=0.7, linewidth=1)
            ax4.set_xlabel('Sample Index')
        
        ax4.set_ylabel('Total Latency (ms)')
        ax4.set_title('Total Latency Over Time')
        ax4.grid(True, alpha=0.3)
        
        # Plot 5: Component correlation matrix
        ax5 = plt.subplot(3, 2, 5)
        correlation_data = self.latency_df[components].corr()
        sns.heatmap(correlation_data, annot=True, cmap='coolwarm', center=0,
                   xticklabels=component_labels, yticklabels=component_labels, ax=ax5)
        ax5.set_title('Latency Component Correlation')
        
        # Plot 6: Box plot comparison
        ax6 = plt.subplot(3, 2, 6)
        box_data = [self.latency_df[comp] for comp in components]
        box_plot = ax6.boxplot(box_data, labels=component_labels, patch_artist=True)
        colors = ['skyblue', 'lightcoral', 'lightgreen']
        for patch, color in zip(box_plot['boxes'], colors):
            patch.set_facecolor(color)
        ax6.set_ylabel('Latency (ms)')
        ax6.set_title('Latency Component Distribution Comparison')
        ax6.grid(True, alpha=0.3)
        plt.setp(ax6.get_xticklabels(), rotation=45)
        
        plt.tight_layout()
        
        # Save plot
        plot_filename = os.path.join(self.analysis_dir, f'latency_analysis_{self.analysis_timestamp}.png')
        plt.savefig(plot_filename, dpi=300, bbox_inches='tight')
        print(f"Latency analysis plots saved: {plot_filename}")
        
        return fig

    def create_error_analysis_plots(self):
        """Create tracking error analysis visualizations"""
        print(f"Generating tracking error analysis plots...")
        
        # Get error analysis results
        error_results = self.analysis_results['error_baseline']
        
        # Create figure
        fig = plt.figure(figsize=(20, 12))
        
        # Plot 1: Error time series with theoretical baseline
        ax1 = plt.subplot(2, 3, 1)
        time_data = self.error_df['elapsed_time_s']
        error_data = self.error_df['error_longitudinal_m']
        
        ax1.plot(time_data, error_data, alpha=0.7, linewidth=1, 
                label=f'Actual Error', color='blue')
        ax1.axhline(y=error_results['theoretical_error_m'], color='red', 
                   linestyle='--', linewidth=2,
                   label=f'Theoretical Error ({error_results["theoretical_error_m"]:.6f}m)')
        ax1.axhline(y=error_results['actual_error_mean_m'], color='green', 
                   linestyle=':', linewidth=2,
                   label=f'Actual Mean ({error_results["actual_error_mean_m"]:.6f}m)')
        
        ax1.set_xlabel('Time (seconds)')
        ax1.set_ylabel('Longitudinal Error (m)')
        ax1.set_title('Longitudinal Tracking Error vs Time')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        
        # Plot 2: Error distribution
        ax2 = plt.subplot(2, 3, 2)
        ax2.hist(error_data, bins=50, alpha=0.7, color='skyblue', edgecolor='black')
        ax2.axvline(error_results['actual_error_mean_m'], color='green', 
                   linestyle=':', linewidth=2,
                   label=f'Mean: {error_results["actual_error_mean_m"]:.6f}m')
        ax2.axvline(error_results['theoretical_error_m'], color='red', 
                   linestyle='--', linewidth=2,
                   label=f'Theoretical: {error_results["theoretical_error_m"]:.6f}m')
        ax2.set_xlabel('Longitudinal Error (m)')
        ax2.set_ylabel('Frequency')
        ax2.set_title('Longitudinal Error Distribution')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        # Plot 3: Error vs target position
        ax3 = plt.subplot(2, 3, 3)
        ax3.scatter(self.error_df['s_target'], self.error_df['error_longitudinal_m'], 
                   alpha=0.6, s=10, color='blue')
        ax3.axhline(y=error_results['theoretical_error_m'], color='red', 
                   linestyle='--', linewidth=2, label='Theoretical Error')
        ax3.set_xlabel('Target Position (m)')
        ax3.set_ylabel('Longitudinal Error (m)')
        ax3.set_title('Error vs Target Position')
        ax3.legend()
        ax3.grid(True, alpha=0.3)
        
        # Plot 4: Lateral error analysis
        ax4 = plt.subplot(2, 3, 4)
        if 'error_lateral_m' in self.error_df.columns:
            lateral_errors = self.error_df['error_lateral_m']
            ax4.plot(time_data, lateral_errors, alpha=0.7, linewidth=1, color='orange')
            ax4.set_xlabel('Time (seconds)')
            ax4.set_ylabel('Lateral Error (m)')
            ax4.set_title('Lateral Tracking Error vs Time')
            ax4.grid(True, alpha=0.3)
        
        # Plot 5: Error statistics comparison
        ax5 = plt.subplot(2, 3, 5)
        categories = ['Theoretical', 'Actual Mean', 'Actual Median']
        values = [
            error_results['theoretical_error_m'],
            error_results['actual_error_mean_m'],
            error_results['actual_error_median_m']
        ]
        colors = ['red', 'green', 'blue']
        bars = ax5.bar(categories, values, color=colors, alpha=0.7)
        ax5.set_ylabel('Error (m)')
        ax5.set_title('Error Comparison: Theory vs Actual')
        ax5.grid(True, alpha=0.3)
        
        # Add value labels on bars
        for bar, value in zip(bars, values):
            height = bar.get_height()
            ax5.text(bar.get_x() + bar.get_width()/2., height + max(values)*0.01,
                    f'{value:.6f}m', ha='center', va='bottom')
        
        # Plot 6: Agreement analysis
        ax6 = plt.subplot(2, 3, 6)
        agreement_ratio = error_results['agreement_ratio']
        
        # Create a simple agreement visualization
        theta = np.linspace(0, 2*np.pi, 100)
        r = np.ones_like(theta)
        ax6 = plt.subplot(2, 3, 6, projection='polar')
        ax6.plot(theta, r, 'k-', linewidth=2)
        ax6.fill_between(theta, 0, r, alpha=0.1, color='gray')
        
        # Plot agreement point
        agreement_angle = 2 * np.pi * min(agreement_ratio / 2, 1)  # Scale to 0-2π
        ax6.plot([agreement_angle], [agreement_ratio], 'ro', markersize=12)
        ax6.set_ylim(0, 2)
        ax6.set_title(f'Agreement Ratio: {agreement_ratio:.3f}')
        
        plt.tight_layout()
        
        # Save plot
        plot_filename = os.path.join(self.analysis_dir, f'error_analysis_{self.analysis_timestamp}.png')
        plt.savefig(plot_filename, dpi=300, bbox_inches='tight')
        print(f"Error analysis plots saved: {plot_filename}")
        
        return fig

    def generate_comprehensive_report(self):
        """Generate comprehensive analysis report"""
        print(f"\nGenerating comprehensive report...")
        
        # Calculate overall assessment
        latency_stats = self.analysis_results['latency_statistics']
        error_baseline = self.analysis_results['error_baseline']
        
        latency_assessment = self.assess_latency_quality(latency_stats)
        
        report_content = f"""# Experiment B: Latency and Baseline Tracking Error Analysis Report

## Analysis Information
- **Analysis Timestamp**: {self.analysis_timestamp}
- **Latency Data Source**: {os.path.basename(self.latency_csv)}
- **Error Data Source**: {os.path.basename(self.error_csv)}
- **Analysis Date**: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}

## Executive Summary

This report presents the results of Experiment B, which analyzes system latency characteristics and establishes the baseline relationship between latency and tracking error in the FSM Sandbox platform.

## Part 1: Latency Characteristics Analysis

### Key Performance Metrics

| Latency Component | Mean (ms) | Median (ms) | Std (ms) | P95 (ms) | P99 (ms) |
|-------------------|-----------|-------------|----------|----------|----------|
| Information Age | {latency_stats['information_age_ms']['mean']:.2f} | {latency_stats['information_age_ms']['median']:.2f} | {latency_stats['information_age_ms']['std']:.2f} | {latency_stats['information_age_ms']['p95']:.2f} | {latency_stats['information_age_ms']['p99']:.2f} |
| Planner Latency | {latency_stats['planner_latency_ms']['mean']:.2f} | {latency_stats['planner_latency_ms']['median']:.2f} | {latency_stats['planner_latency_ms']['std']:.2f} | {latency_stats['planner_latency_ms']['p95']:.2f} | {latency_stats['planner_latency_ms']['p99']:.2f} |
| Decision-to-Actuation | {latency_stats['decision_to_actuation_ms']['mean']:.2f} | {latency_stats['decision_to_actuation_ms']['median']:.2f} | {latency_stats['decision_to_actuation_ms']['std']:.2f} | {latency_stats['decision_to_actuation_ms']['p95']:.2f} | {latency_stats['decision_to_actuation_ms']['p99']:.2f} |
| **Total End-to-End** | **{latency_stats['total_latency_ms']['mean']:.2f}** | **{latency_stats['total_latency_ms']['median']:.2f}** | **{latency_stats['total_latency_ms']['std']:.2f}** | **{latency_stats['total_latency_ms']['p95']:.2f}** | **{latency_stats['total_latency_ms']['p99']:.2f}** |

### Latency Assessment
**{latency_assessment}**

## Part 2: Baseline Tracking Error Analysis

### Test Configuration
- **Target Velocity**: {error_baseline['target_velocity']:.3f} m/s
- **Mean System Latency**: {error_baseline['mean_latency_ms']:.2f} ms
- **Test Samples**: {error_baseline['sample_count']} measurements

### Error Analysis Results

| Metric | Value |
|--------|-------|
| Theoretical Error (v × ΔT) | {error_baseline['theoretical_error_m']:.6f} m |
| Actual Error Mean | {error_baseline['actual_error_mean_m']:.6f} m |
| Actual Error Std | {error_baseline['actual_error_std_m']:.6f} m |
| Actual Error Median | {error_baseline['actual_error_median_m']:.6f} m |
| Agreement Ratio | {error_baseline['agreement_ratio']:.3f} |

### Baseline Error Assessment
The theoretical model **{'ACCURATELY' if 0.5 <= error_baseline['agreement_ratio'] <= 1.5 else 'INACCURATELY'}** predicts the actual tracking error.

Agreement ratio of {error_baseline['agreement_ratio']:.3f} indicates {'excellent' if 0.8 <= error_baseline['agreement_ratio'] <= 1.2 else 'good' if 0.5 <= error_baseline['agreement_ratio'] <= 1.5 else 'poor'} correlation between latency and tracking error.

## Conclusions

### Platform Performance Sufficiency
Based on the latency analysis, the FSM Sandbox platform demonstrates **{latency_assessment.split(' - ')[0]}** performance characteristics for ADS testing.

### Error Baseline Establishment
The analysis successfully establishes a quantitative baseline for tracking error due to system latency:
- **Baseline Error**: {error_baseline['theoretical_error_m']:.6f} m at {error_baseline['target_velocity']:.3f} m/s
- **Error Predictability**: {error_baseline['agreement_ratio']:.1%} agreement with theoretical model

### Implications for Future Testing
This baseline provides a reference point for diagnosing execution errors in more complex test scenarios. Any tracking errors significantly exceeding this baseline can be attributed to algorithmic issues rather than platform limitations.

## Generated Files
- `latency_analysis_{self.analysis_timestamp}.png` - Comprehensive latency analysis plots
- `error_analysis_{self.analysis_timestamp}.png` - Tracking error analysis visualizations  
- `experiment_b_results_{self.analysis_timestamp}.json` - Detailed numerical results
- `experiment_b_report_{self.analysis_timestamp}.md` - This comprehensive report
"""
        
        # Save report
        report_file = os.path.join(self.analysis_dir, f"experiment_b_report_{self.analysis_timestamp}.md")
        with open(report_file, 'w') as f:
            f.write(report_content)
        
        print(f"Comprehensive report saved: {report_file}")

    def save_results_json(self):
        """Save detailed results to JSON file"""
        results_file = os.path.join(self.analysis_dir, f"experiment_b_results_{self.analysis_timestamp}.json")
        
        # Prepare metadata
        metadata = {
            'analysis_timestamp': self.analysis_timestamp,
            'latency_file': os.path.basename(self.latency_csv),
            'error_file': os.path.basename(self.error_csv),
            'latency_samples': len(self.latency_df),
            'error_samples': len(self.error_df),
            'analysis_date': datetime.now().isoformat()
        }
        
        # Convert numpy types for JSON serialization
        def convert_for_json(obj):
            if hasattr(obj, 'item'):
                return obj.item()
            elif isinstance(obj, np.integer):
                return int(obj)
            elif isinstance(obj, np.floating):
                return float(obj)
            else:
                return obj
        
        json_results = {
            'metadata': metadata,
            'latency_statistics': {
                comp: {k: convert_for_json(v) for k, v in stats.items()}
                for comp, stats in self.analysis_results['latency_statistics'].items()
            },
            'error_baseline': {
                k: convert_for_json(v) for k, v in self.analysis_results['error_baseline'].items()
            }
        }
        
        with open(results_file, 'w') as f:
            json.dump(json_results, f, indent=2)
        
        print(f"Detailed results saved: {results_file}")

    def run_complete_analysis(self):
        """Run the complete Experiment B analysis pipeline"""
        print("Starting Experiment B Analysis...")
        print(f"Output directory: {self.analysis_dir}")
        
        try:
            # Part 1: Latency analysis
            self.analyze_latency_characteristics()
            
            # Part 2: Error baseline analysis  
            self.analyze_tracking_error_baseline()
            
            # Generate outputs
            print(f"\nGenerating final outputs...")
            self.save_results_json()
            self.generate_comprehensive_report()
            
            print(f"\nExperiment B analysis complete!")
            print(f"All results saved to: {self.analysis_dir}")
            return True
            
        except Exception as e:
            print(f"Error during analysis: {e}")
            return False


def main():
    parser = argparse.ArgumentParser(description='Analyze Experiment B: Latency and Baseline Tracking Error')
    parser.add_argument('latency_csv', help='Path to latency data CSV file')
    parser.add_argument('error_csv', help='Path to tracking error CSV file')
    parser.add_argument('--config', default='config.yaml', help='Configuration file path')
    parser.add_argument('--show-plots', action='store_true', help='Display plots after generation')
    
    args = parser.parse_args()
    
    # Validate input files
    for file_path, name in [(args.latency_csv, 'latency'), (args.error_csv, 'error')]:
        if not os.path.exists(file_path):
            print(f"Error: {name} file not found: {file_path}")
            sys.exit(1)
    
    # Run analysis
    analyzer = ExperimentBAnalyzer(args.latency_csv, args.error_csv, args.config)
    success = analyzer.run_complete_analysis()
    
    if success and args.show_plots:
        plt.show()
    
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()