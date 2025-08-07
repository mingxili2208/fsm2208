#!/usr/bin/env python3

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import argparse
import os
import json
from datetime import datetime
from typing import Dict, List, Tuple

class ExperimentBAnalyzer:
    """
    Comprehensive analyzer for Experiment B: Latency and baseline tracking error analysis
    """
    
    def __init__(self, latency_chain_csv: str, v2r_csv: str, tracking_error_csv: str):
        self.latency_chain_csv = latency_chain_csv
        self.v2r_csv = v2r_csv
        self.tracking_error_csv = tracking_error_csv
        
        self.analysis_timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.setup_directories()
        self.setup_plotting()
        
        # Load and validate data
        self.load_data()
        self.analysis_results = {}

    def setup_directories(self):
        """Setup output directories"""
        self.base_dir = os.path.dirname(os.path.abspath(__file__))
        self.results_dir = os.path.join(self.base_dir, 'results')
        self.analysis_dir = os.path.join(self.results_dir, f'experiment_b_{self.analysis_timestamp}')
        
        for directory in [self.results_dir, self.analysis_dir]:
            os.makedirs(directory, exist_ok=True)

    def setup_plotting(self):
        """Setup matplotlib configuration"""
        plt.style.use('seaborn-v0_8-whitegrid')
        plt.rcParams['figure.figsize'] = [16, 12]
        plt.rcParams['font.size'] = 12
        plt.rcParams['axes.grid'] = True
        plt.rcParams['grid.alpha'] = 0.3

    def load_data(self):
        """Load and validate all data files"""
        try:
            # Load latency chain data
            self.latency_chain_df = pd.read_csv(self.latency_chain_csv)
            print(f"Loaded latency chain data: {len(self.latency_chain_df)} records")
            
            # Load V2R data
            self.v2r_df = pd.read_csv(self.v2r_csv)
            print(f"Loaded V2R data: {len(self.v2r_df)} records")
            
            # Load tracking error data
            self.tracking_error_df = pd.read_csv(self.tracking_error_csv)
            print(f"Loaded tracking error data: {len(self.tracking_error_df)} records")
            
            # Clean data
            self.clean_data()
            
        except Exception as e:
            print(f"Error loading data: {e}")
            raise

    def clean_data(self):
        """Clean and filter data"""
        # Filter valid latency chain correlations
        initial_chain_count = len(self.latency_chain_df)
        self.latency_chain_df = self.latency_chain_df[
            (self.latency_chain_df['valid_correlation'] == True) &
            (self.latency_chain_df['r2v_delay_ms'] >= 0) &
            (self.latency_chain_df['r2v_delay_ms'] <= 1000) &
            (self.latency_chain_df['planner_delay_ms'] >= 0) &
            (self.latency_chain_df['planner_delay_ms'] <= 500)
        ]
        print(f"Cleaned latency chain: {len(self.latency_chain_df)}/{initial_chain_count} records")
        
        # Filter reasonable V2R measurements
        initial_v2r_count = len(self.v2r_df)
        self.v2r_df = self.v2r_df[
            (self.v2r_df['v2r_latency_ms'] > 0) &
            (self.v2r_df['v2r_latency_ms'] < 250)
        ]
        print(f"Cleaned V2R data: {len(self.v2r_df)}/{initial_v2r_count} records")
        
        # Filter active test tracking error data
        initial_error_count = len(self.tracking_error_df)
        if 'test_active' in self.tracking_error_df.columns:
            self.tracking_error_df = self.tracking_error_df[
                self.tracking_error_df['test_active'] == True
            ]
        print(f"Cleaned tracking error: {len(self.tracking_error_df)}/{initial_error_count} records")

    def analyze_latency_characteristics(self):
        """Analyze complete latency characteristics"""
        print("\n" + "="*80)
        print("EXPERIMENT B - PART 1: LATENCY CHARACTERISTICS ANALYSIS")
        print("="*80)
        
        # Calculate R2V and Planner statistics
        r2v_stats = self.calculate_component_stats(self.latency_chain_df['r2v_delay_ms'], 'R2V')
        planner_stats = self.calculate_component_stats(self.latency_chain_df['planner_delay_ms'], 'Planner')
        
        # Calculate V2R statistics
        v2r_stats = self.calculate_component_stats(self.v2r_df['v2r_latency_ms'], 'V2R')
        
        # Calculate total latency using mean values
        total_mean_ms = r2v_stats['mean'] + planner_stats['mean'] + v2r_stats['mean']
        
        # Store results
        self.analysis_results['latency_statistics'] = {
            'r2v_delay_ms': r2v_stats,
            'planner_delay_ms': planner_stats,
            'v2r_delay_ms': v2r_stats,
            'total_delay_ms': {
                'mean': total_mean_ms,
                'components': {
                    'r2v_contribution_pct': (r2v_stats['mean'] / total_mean_ms) * 100,
                    'planner_contribution_pct': (planner_stats['mean'] / total_mean_ms) * 100,
                    'v2r_contribution_pct': (v2r_stats['mean'] / total_mean_ms) * 100
                }
            }
        }
        
        # Print results table
        self.print_latency_table()
        
        # Assess performance
        self.assess_latency_performance()
        
        return self.analysis_results['latency_statistics']

    def calculate_component_stats(self, data: pd.Series, component_name: str) -> Dict:
        """Calculate statistics for a latency component"""
        return {
            'count': len(data),
            'mean': data.mean(),
            'median': data.median(),
            'std': data.std(),
            'min': data.min(),
            'max': data.max(),
            'p95': data.quantile(0.95),
            'p99': data.quantile(0.99)
        }

    def print_latency_table(self):
        """Print formatted latency statistics table"""
        stats = self.analysis_results['latency_statistics']
        
        print(f"\nLatency Component Analysis")
        print("-" * 100)
        print(f"{'Component':<20} {'Count':<8} {'Mean':<8} {'Median':<8} {'Std':<8} {'Min':<8} {'Max':<8} {'P95':<8} {'P99':<8}")
        print("-" * 100)
        
        components = [
            ('R2V Delay (ms)', stats['r2v_delay_ms']),
            ('Planner Delay (ms)', stats['planner_delay_ms']),
            ('V2R Delay (ms)', stats['v2r_delay_ms'])
        ]
        
        for name, s in components:
            print(f"{name:<20} {s['count']:<8} {s['mean']:<8.2f} {s['median']:<8.2f} {s['std']:<8.2f} "
                  f"{s['min']:<8.2f} {s['max']:<8.2f} {s['p95']:<8.2f} {s['p99']:<8.2f}")
        
        print("-" * 100)
        print(f"{'Total E2E (ms)':<20} {'-':<8} {stats['total_delay_ms']['mean']:<8.2f}")
        print("-" * 100)

    def assess_latency_performance(self):
        """Assess latency performance against thresholds"""
        stats = self.analysis_results['latency_statistics']
        total_latency = stats['total_delay_ms']['mean']
        
        print(f"\nLatency Performance Assessment:")
        print("-" * 60)
        
        criteria = [
            ('Total latency < 100ms', total_latency < 100, f"{total_latency:.1f}ms"),
            ('V2R latency < 50ms', stats['v2r_delay_ms']['mean'] < 50, f"{stats['v2r_delay_ms']['mean']:.1f}ms"),
            ('Planner latency < 50ms', stats['planner_delay_ms']['mean'] < 50, f"{stats['planner_delay_ms']['mean']:.1f}ms"),
            ('R2V latency < 100ms', stats['r2v_delay_ms']['mean'] < 100, f"{stats['r2v_delay_ms']['mean']:.1f}ms")
        ]
        
        passed = 0
        for criterion, result, value in criteria:
            status = "PASS" if result else "FAIL"
            print(f"  {criterion:<30}: {status:<4} ({value})")
            if result:
                passed += 1
        
        print(f"\nOverall Assessment: {passed}/{len(criteria)} criteria passed")

    def analyze_tracking_error_baseline(self):
        """Analyze tracking error baseline"""
        print("\n" + "="*80)
        print("EXPERIMENT B - PART 2: TRACKING ERROR BASELINE ANALYSIS")
        print("="*80)
        
        if len(self.tracking_error_df) == 0:
            print("Error: No tracking error data available")
            return None
        
        # Get test parameters
        target_velocity = 0.5  # m/s from test configuration
        total_latency_s = self.analysis_results['latency_statistics']['total_delay_ms']['mean'] / 1000.0
        
        # Calculate theoretical error
        theoretical_error_m = target_velocity * total_latency_s
        
        # Calculate actual error statistics
        actual_errors = self.tracking_error_df['longitudinal_error_m']
        actual_error_mean = actual_errors.mean()
        actual_error_std = actual_errors.std()
        actual_error_median = actual_errors.median()
        
        # Calculate agreement
        agreement_ratio = actual_error_mean / theoretical_error_m if theoretical_error_m != 0 else 0
        
        # Store results
        self.analysis_results['error_baseline'] = {
            'target_velocity_ms': target_velocity,
            'total_latency_ms': total_latency_s * 1000,
            'theoretical_error_m': theoretical_error_m,
            'actual_error_mean_m': actual_error_mean,
            'actual_error_std_m': actual_error_std,
            'actual_error_median_m': actual_error_median,
            'agreement_ratio': agreement_ratio,
            'sample_count': len(actual_errors)
        }
        
        print(f"\nTracking Error Baseline Analysis:")
        print("-" * 60)
        print(f"Test Configuration:")
        print(f"  Target Velocity: {target_velocity:.3f} m/s")
        print(f"  Total Latency: {total_latency_s*1000:.2f} ms")
        print(f"\nError Analysis:")
        print(f"  Theoretical Error: {theoretical_error_m:.6f} m")
        print(f"  Actual Error Mean: {actual_error_mean:.6f} m")
        print(f"  Actual Error Std: {actual_error_std:.6f} m")
        print(f"  Agreement Ratio: {agreement_ratio:.3f}")
        
        # Assess prediction accuracy
        prediction_accurate = 0.5 <= agreement_ratio <= 1.5
        accuracy_status = "ACCURATE" if prediction_accurate else "INACCURATE"
        print(f"\nBaseline Prediction Assessment: {accuracy_status}")
        
        return self.analysis_results['error_baseline']

    def create_comprehensive_plots(self):
        """Create comprehensive analysis plots"""
        print(f"\nGenerating analysis plots...")
        
        # Create large figure with multiple subplots
        fig = plt.figure(figsize=(20, 16))
        
        # Plot 1: Latency component breakdown
        ax1 = plt.subplot(3, 3, 1)
        stats = self.analysis_results['latency_statistics']
        components = ['R2V', 'Planner', 'V2R']
        means = [stats['r2v_delay_ms']['mean'], 
                stats['planner_delay_ms']['mean'], 
                stats['v2r_delay_ms']['mean']]
        colors = ['skyblue', 'lightcoral', 'lightgreen']
        
        bars = ax1.bar(components, means, color=colors)
        ax1.set_ylabel('Latency (ms)')
        ax1.set_title('Mean Latency by Component')
        ax1.grid(True, alpha=0.3)
        
        # Add value labels
        for bar, value in zip(bars, means):
            height = bar.get_height()
            ax1.text(bar.get_x() + bar.get_width()/2., height + max(means)*0.01,
                    f'{value:.1f}ms', ha='center', va='bottom')
        
        # Plot 2: V2R latency distribution
        ax2 = plt.subplot(3, 3, 2)
        v2r_data = self.v2r_df['v2r_latency_ms']
        ax2.hist(v2r_data, bins=30, alpha=0.7, color='lightgreen', edgecolor='black')
        ax2.axvline(v2r_data.mean(), color='red', linestyle='--', linewidth=2,
                   label=f'Mean: {v2r_data.mean():.1f}ms')
        ax2.set_xlabel('V2R Latency (ms)')
        ax2.set_ylabel('Frequency')
        ax2.set_title('V2R Latency Distribution')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        # Plot 3: Latency composition pie chart
        ax3 = plt.subplot(3, 3, 3)
        ax3.pie(means, labels=components, autopct='%1.1f%%', colors=colors)
        ax3.set_title('Latency Composition')
        
        # Plot 4: R2V + Planner latency time series
        ax4 = plt.subplot(3, 3, 4)
        if len(self.latency_chain_df) > 0:
            total_chain = self.latency_chain_df['total_r2v_planner_ms']
            ax4.plot(range(len(total_chain)), total_chain, alpha=0.7, linewidth=1)
            ax4.set_xlabel('Sample Index')
            ax4.set_ylabel('R2V + Planner Latency (ms)')
            ax4.set_title('R2V + Planner Latency Over Time')
            ax4.grid(True, alpha=0.3)
        
        # Plot 5: Tracking error time series
        ax5 = plt.subplot(3, 3, 5)
        if len(self.tracking_error_df) > 0:
            error_baseline = self.analysis_results['error_baseline']
            time_data = self.tracking_error_df['elapsed_time_s']
            error_data = self.tracking_error_df['longitudinal_error_m']
            
            ax5.plot(time_data, error_data, alpha=0.7, linewidth=1, 
                    label='Actual Error', color='blue')
            ax5.axhline(y=error_baseline['theoretical_error_m'], color='red', 
                       linestyle='--', linewidth=2,
                       label=f'Theoretical: {error_baseline["theoretical_error_m"]:.6f}m')
            ax5.axhline(y=error_baseline['actual_error_mean_m'], color='green', 
                       linestyle=':', linewidth=2,
                       label=f'Mean: {error_baseline["actual_error_mean_m"]:.6f}m')
            
            ax5.set_xlabel('Time (s)')
            ax5.set_ylabel('Longitudinal Error (m)')
            ax5.set_title('Tracking Error vs Time')
            ax5.legend()
            ax5.grid(True, alpha=0.3)
        
        # Plot 6: Error distribution
        ax6 = plt.subplot(3, 3, 6)
        if len(self.tracking_error_df) > 0:
            error_data = self.tracking_error_df['longitudinal_error_m']
            error_baseline = self.analysis_results['error_baseline']
            
            ax6.hist(error_data, bins=30, alpha=0.7, color='skyblue', edgecolor='black')
            ax6.axvline(error_baseline['actual_error_mean_m'], color='green', 
                       linestyle=':', linewidth=2,
                       label=f'Mean: {error_baseline["actual_error_mean_m"]:.6f}m')
            ax6.axvline(error_baseline['theoretical_error_m'], color='red', 
                       linestyle='--', linewidth=2,
                       label=f'Theory: {error_baseline["theoretical_error_m"]:.6f}m')
            ax6.set_xlabel('Longitudinal Error (m)')
            ax6.set_ylabel('Frequency')
            ax6.set_title('Error Distribution')
            ax6.legend()
            ax6.grid(True, alpha=0.3)
        
        # Plot 7: Component correlation
        ax7 = plt.subplot(3, 3, 7)
        if len(self.latency_chain_df) > 0:
            r2v_data = self.latency_chain_df['r2v_delay_ms']
            planner_data = self.latency_chain_df['planner_delay_ms']
            ax7.scatter(r2v_data, planner_data, alpha=0.6, s=20)
            ax7.set_xlabel('R2V Delay (ms)')
            ax7.set_ylabel('Planner Delay (ms)')
            ax7.set_title('R2V vs Planner Delay Correlation')
            ax7.grid(True, alpha=0.3)
        
        # Plot 8: Latency box plot comparison
        ax8 = plt.subplot(3, 3, 8)
        box_data = []
        box_labels = []
        if len(self.latency_chain_df) > 0:
            box_data.extend([self.latency_chain_df['r2v_delay_ms'], 
                           self.latency_chain_df['planner_delay_ms']])
            box_labels.extend(['R2V', 'Planner'])
        if len(self.v2r_df) > 0:
            box_data.append(self.v2r_df['v2r_latency_ms'])
            box_labels.append('V2R')
        
        if box_data:
            bp = ax8.boxplot(box_data, labels=box_labels, patch_artist=True)
            colors = ['skyblue', 'lightcoral', 'lightgreen']
            for patch, color in zip(bp['boxes'], colors[:len(bp['boxes'])]):
                patch.set_facecolor(color)
            ax8.set_ylabel('Latency (ms)')
            ax8.set_title('Latency Distribution Comparison')
            ax8.grid(True, alpha=0.3)
        
        # Plot 9: Agreement analysis
        ax9 = plt.subplot(3, 3, 9)
        if 'error_baseline' in self.analysis_results:
            agreement_ratio = self.analysis_results['error_baseline']['agreement_ratio']
            theoretical = self.analysis_results['error_baseline']['theoretical_error_m']
            actual = self.analysis_results['error_baseline']['actual_error_mean_m']
            
            categories = ['Theoretical', 'Actual Mean']
            values = [theoretical, actual]
            colors_bar = ['red', 'green']
            
            bars = ax9.bar(categories, values, color=colors_bar, alpha=0.7)
            ax9.set_ylabel('Error (m)')
            ax9.set_title(f'Error Comparison (Ratio: {agreement_ratio:.3f})')
            ax9.grid(True, alpha=0.3)
            
            # Add value labels
            for bar, value in zip(bars, values):
                height = bar.get_height()
                ax9.text(bar.get_x() + bar.get_width()/2., height + max(values)*0.01,
                        f'{value:.6f}m', ha='center', va='bottom')
        
        plt.tight_layout()
        
        # Save plot
        plot_filename = os.path.join(self.analysis_dir, f'experiment_b_analysis_{self.analysis_timestamp}.png')
        plt.savefig(plot_filename, dpi=300, bbox_inches='tight')
        print(f"Analysis plots saved: {plot_filename}")
        
        return fig

    def generate_report(self):
        """Generate comprehensive analysis report"""
        print(f"\nGenerating comprehensive report...")
        
        latency_stats = self.analysis_results['latency_statistics']
        
        report_content = f"""# Experiment B: Latency and Baseline Tracking Error Analysis Report

## Analysis Summary
- **Analysis Timestamp**: {self.analysis_timestamp}
- **Latency Chain Data**: {os.path.basename(self.latency_chain_csv)} ({len(self.latency_chain_df)} records)
- **V2R Data**: {os.path.basename(self.v2r_csv)} ({len(self.v2r_df)} records)  
- **Tracking Error Data**: {os.path.basename(self.tracking_error_csv)} ({len(self.tracking_error_df)} records)

## Part 1: Latency Characteristics Analysis

### Key Performance Metrics

| Component | Mean (ms) | Median (ms) | Std (ms) | P95 (ms) | P99 (ms) |
|-----------|-----------|-------------|----------|----------|----------|
| R2V Delay | {latency_stats['r2v_delay_ms']['mean']:.2f} | {latency_stats['r2v_delay_ms']['median']:.2f} | {latency_stats['r2v_delay_ms']['std']:.2f} | {latency_stats['r2v_delay_ms']['p95']:.2f} | {latency_stats['r2v_delay_ms']['p99']:.2f} |
| Planner Delay | {latency_stats['planner_delay_ms']['mean']:.2f} | {latency_stats['planner_delay_ms']['median']:.2f} | {latency_stats['planner_delay_ms']['std']:.2f} | {latency_stats['planner_delay_ms']['p95']:.2f} | {latency_stats['planner_delay_ms']['p99']:.2f} |
| V2R Delay | {latency_stats['v2r_delay_ms']['mean']:.2f} | {latency_stats['v2r_delay_ms']['median']:.2f} | {latency_stats['v2r_delay_ms']['std']:.2f} | {latency_stats['v2r_delay_ms']['p95']:.2f} | {latency_stats['v2r_delay_ms']['p99']:.2f} |
| **Total E2E** | **{latency_stats['total_delay_ms']['mean']:.2f}** | - | - | - | - |

### Latency Composition
- R2V Contribution: {latency_stats['total_delay_ms']['components']['r2v_contribution_pct']:.1f}%
- Planner Contribution: {latency_stats['total_delay_ms']['components']['planner_contribution_pct']:.1f}%  
- V2R Contribution: {latency_stats['total_delay_ms']['components']['v2r_contribution_pct']:.1f}%

"""

        if 'error_baseline' in self.analysis_results:
            error_baseline = self.analysis_results['error_baseline']
            report_content += f"""
## Part 2: Baseline Tracking Error Analysis

### Test Configuration
- **Target Velocity**: {error_baseline['target_velocity_ms']:.3f} m/s
- **Total System Latency**: {error_baseline['total_latency_ms']:.2f} ms
- **Test Samples**: {error_baseline['sample_count']} measurements

### Error Analysis Results

| Metric | Value |
|--------|-------|
| Theoretical Error (v × ΔT) | {error_baseline['theoretical_error_m']:.6f} m |
| Actual Error Mean | {error_baseline['actual_error_mean_m']:.6f} m |
| Actual Error Std | {error_baseline['actual_error_std_m']:.6f} m |
| Agreement Ratio | {error_baseline['agreement_ratio']:.3f} |

### Baseline Assessment
The theoretical model **{'ACCURATELY' if 0.5 <= error_baseline['agreement_ratio'] <= 1.5 else 'INACCURATELY'}** predicts actual tracking error.

"""

        report_content += f"""
## Conclusions

### Platform Performance
Total end-to-end latency of {latency_stats['total_delay_ms']['mean']:.1f}ms demonstrates {'EXCELLENT' if latency_stats['total_delay_ms']['mean'] < 100 else 'GOOD' if latency_stats['total_delay_ms']['mean'] < 150 else 'ACCEPTABLE'} performance for ADS testing.

### Error Baseline Establishment  
Successfully established quantitative baseline for tracking error attribution in future testing scenarios.

## Generated Files
- Analysis plots: `experiment_b_analysis_{self.analysis_timestamp}.png`
- Detailed results: `experiment_b_results_{self.analysis_timestamp}.json`
- This report: `experiment_b_report_{self.analysis_timestamp}.md`
"""
        
        # Save report
        report_file = os.path.join(self.analysis_dir, f"experiment_b_report_{self.analysis_timestamp}.md")
        with open(report_file, 'w') as f:
            f.write(report_content)
        
        print(f"Report saved: {report_file}")

    def save_results_json(self):
        """Save detailed results to JSON"""
        results_file = os.path.join(self.analysis_dir, f"experiment_b_results_{self.analysis_timestamp}.json")
        
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
            'metadata': {
                'analysis_timestamp': self.analysis_timestamp,
                'latency_chain_file': os.path.basename(self.latency_chain_csv),
                'v2r_file': os.path.basename(self.v2r_csv),
                'tracking_error_file': os.path.basename(self.tracking_error_csv),
                'latency_chain_samples': len(self.latency_chain_df),
                'v2r_samples': len(self.v2r_df),
                'tracking_error_samples': len(self.tracking_error_df)
            },
            'analysis_results': {
                k: {kk: convert_for_json(vv) if not isinstance(vv, dict) else 
                    {kkk: convert_for_json(vvv) for kkk, vvv in vv.items()} 
                    for kk, vv in v.items()}
                for k, v in self.analysis_results.items()
            }
        }
        
        with open(results_file, 'w') as f:
            json.dump(json_results, f, indent=2)
        
        print(f"Results JSON saved: {results_file}")

    def run_complete_analysis(self):
        """Run complete analysis pipeline"""
        print("Starting Experiment B comprehensive analysis...")
        
        try:
            # Part 1: Latency analysis
            self.analyze_latency_characteristics()
            
            # Part 2: Error baseline analysis
            self.analyze_tracking_error_baseline()
            
            # Generate outputs
            self.create_comprehensive_plots()
            self.save_results_json()
            self.generate_report()
            
            print(f"\nExperiment B analysis complete!")
            print(f"All results saved to: {self.analysis_dir}")
            return True
            
        except Exception as e:
            print(f"Error during analysis: {e}")
            return False


def main():
    parser = argparse.ArgumentParser(description='Comprehensive Experiment B Analysis')
    parser.add_argument('latency_chain_csv', help='Path to latency chain CSV file')
    parser.add_argument('v2r_csv', help='Path to V2R CSV file')
    parser.add_argument('tracking_error_csv', help='Path to tracking error CSV file')
    parser.add_argument('--show-plots', action='store_true', help='Display plots after generation')
    
    args = parser.parse_args()
    
    # Validate input files
    input_files = [args.latency_chain_csv, args.v2r_csv, args.tracking_error_csv]
    for file_path in input_files:
        if not os.path.exists(file_path):
            print(f"Error: File not found: {file_path}")
            return 1
    
    # Run analysis
    analyzer = ExperimentBAnalyzer(args.latency_chain_csv, args.v2r_csv, args.tracking_error_csv)
    success = analyzer.run_complete_analysis()
    
    if success and args.show_plots:
        plt.show()
    
    return 0 if success else 1


if __name__ == '__main__':
    exit(main())