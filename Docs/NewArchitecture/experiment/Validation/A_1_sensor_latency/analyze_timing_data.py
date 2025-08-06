#!/usr/bin/env python3

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from scipy import stats
import argparse
import os
import sys
from datetime import datetime
import json

class FSMTimingAnalyzer:
    """
    Analyzer for FSM Sandbox timing performance data.
    Generates comprehensive statistics and visualizations with organized output structure.
    """
    
    def __init__(self, csv_file_path):
        self.csv_file = csv_file_path
        self.df = None
        self.stats_results = {}
        
        # Create analysis timestamp
        self.analysis_timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # Setup directory structure
        self.setup_directories()
        
        # Set plotting style
        plt.style.use('default')
        sns.set_palette("husl")
        
        # Configure matplotlib for better plots
        plt.rcParams['figure.figsize'] = (12, 8)
        plt.rcParams['font.size'] = 12
        plt.rcParams['axes.grid'] = True
        plt.rcParams['grid.alpha'] = 0.3
        plt.rcParams['font.family'] = 'sans-serif'
        
    def setup_directories(self):
        """Setup directory structure for organized output"""
        # Get current script directory
        self.base_dir = os.path.dirname(os.path.abspath(__file__))
        
        # Create subdirectories
        self.results_dir = os.path.join(self.base_dir, "results")
        self.logs_dir = os.path.join(self.base_dir, "logs")
        
        # Create analysis-specific subdirectory
        self.analysis_dir = os.path.join(self.results_dir, f"analysis_{self.analysis_timestamp}")
        
        # Create directories if they don't exist
        for directory in [self.results_dir, self.logs_dir, self.analysis_dir]:
            os.makedirs(directory, exist_ok=True)
            
        print(f"Analysis output directory: {self.analysis_dir}")
        
    def load_and_validate_data(self):
        """Load CSV data and perform basic validation"""
        try:
            self.df = pd.read_csv(self.csv_file)
            print(f"Loaded {len(self.df)} records from {self.csv_file}")
        except Exception as e:
            print(f"Error loading CSV file: {e}")
            return False
        
        # Validate required columns
        required_cols = ['T1_ns', 'T2_ns', 'T3_ns', 'T4_ns', 'Render_Latency_ms', 
                        'NDT_Processing_Latency_ms', 'Total_Pipeline_Latency_ms', 'Sequence_Valid']
        
        missing_cols = [col for col in required_cols if col not in self.df.columns]
        if missing_cols:
            print(f"Error: Missing required columns: {missing_cols}")
            return False
        
        # Extract session information if available
        if 'Session_ID' in self.df.columns and len(self.df) > 0:
            self.session_id = self.df['Session_ID'].iloc[0]
            print(f"Analyzing data from session: {self.session_id}")
        else:
            self.session_id = "unknown"
        
        # Filter out invalid sequences
        initial_count = len(self.df)
        self.df = self.df[self.df['Sequence_Valid'] == True].copy()
        valid_count = len(self.df)
        
        print(f"Data validation: {valid_count}/{initial_count} records have valid timestamp sequences")
        
        if valid_count == 0:
            print("Error: No valid records found")
            return False
            
        return True
    
    def calculate_statistics(self):
        """Calculate comprehensive statistics for all latency metrics"""
        metrics = ['Render_Latency_ms', 'NDT_Processing_Latency_ms', 'Total_Pipeline_Latency_ms']
        
        for metric in metrics:
            data = self.df[metric].dropna()
            
            self.stats_results[metric] = {
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
        
        # Save statistics to JSON file
        stats_file = os.path.join(self.analysis_dir, f"statistics_{self.analysis_timestamp}.json")
        with open(stats_file, 'w') as f:
            # Convert numpy types to native Python types for JSON serialization
            json_stats = {}
            for metric, stats in self.stats_results.items():
                json_stats[metric] = {k: float(v) if hasattr(v, 'item') else v 
                                    for k, v in stats.items()}
            
            analysis_metadata = {
                'analysis_timestamp': self.analysis_timestamp,
                'session_id': self.session_id,
                'source_file': os.path.basename(self.csv_file),
                'total_records': len(self.df),
                'analysis_date': datetime.now().isoformat()
            }
            
            output = {
                'metadata': analysis_metadata,
                'statistics': json_stats
            }
            
            json.dump(output, f, indent=2)
        
        print(f"Statistics saved to: {stats_file}")
        return self.stats_results
    
    def print_statistics_table(self):
        """Print formatted statistics table and save to file"""
        output_lines = []
        
        header = "="*80
        title = "FSM SANDBOX TIMING PERFORMANCE ANALYSIS"
        
        output_lines.extend(["\n" + header, title, header])
        
        info_section = [
            f"\nDataset Information:",
            f"  Session ID: {self.session_id}",
            f"  Analysis Timestamp: {self.analysis_timestamp}",
            f"  Total valid records: {len(self.df)}",
            f"  Source file: {os.path.basename(self.csv_file)}"
        ]
        
        if len(self.df) > 0 and 'Timestamp_ISO' in self.df.columns:
            info_section.append(f"  Data period: {self.df['Timestamp_ISO'].iloc[0]} to {self.df['Timestamp_ISO'].iloc[-1]}")
        
        output_lines.extend(info_section)
        
        stats_header = [
            f"\nLatency Statistics (milliseconds):",
            "-" * 80,
            f"{'Metric':<25} {'Mean':<8} {'Median':<8} {'Std':<8} {'P95':<8} {'P99':<8} {'Max':<8}",
            "-" * 80
        ]
        output_lines.extend(stats_header)
        
        for metric, stats in self.stats_results.items():
            metric_name = metric.replace('_', ' ').replace('ms', '').strip()
            stats_line = (f"{metric_name:<25} {stats['mean']:<8.2f} {stats['median']:<8.2f} "
                         f"{stats['std']:<8.2f} {stats['p95']:<8.2f} {stats['p99']:<8.2f} {stats['max']:<8.2f}")
            output_lines.append(stats_line)
        
        output_lines.append("-" * 80)
        
        # Print to console
        for line in output_lines:
            print(line)
        
        # Save to file
        report_file = os.path.join(self.analysis_dir, f"analysis_report_{self.analysis_timestamp}.txt")
        with open(report_file, 'w') as f:
            f.write('\n'.join(output_lines))
        
        print(f"\nAnalysis report saved to: {report_file}")
    
    def assess_performance_criteria(self):
        """Assess performance against defined criteria"""
        criteria_lines = [
            f"\nPerformance Assessment:",
            "-" * 50
        ]
        
        criteria = [
            ("Total Pipeline P99 < 100ms", self.stats_results['Total_Pipeline_Latency_ms']['p99'] < 100),
            ("Render Mean < 50ms", self.stats_results['Render_Latency_ms']['mean'] < 50),
            ("Pipeline Std < 30ms", self.stats_results['Total_Pipeline_Latency_ms']['std'] < 30),
            ("NDT Processing P95 < 20ms", self.stats_results['NDT_Processing_Latency_ms']['p95'] < 20)
        ]
        
        passed_criteria = 0
        for criterion, passed in criteria:
            status = "PASS" if passed else "FAIL"
            criterion_line = f"  {criterion:<35}: {status}"
            criteria_lines.append(criterion_line)
            if passed:
                passed_criteria += 1
        
        overall_line = f"\nOverall Performance: {passed_criteria}/{len(criteria)} criteria passed"
        criteria_lines.append(overall_line)
        
        if passed_criteria == len(criteria):
            result = "Result: EXCELLENT - All performance criteria met"
        elif passed_criteria >= len(criteria) * 0.75:
            result = "Result: GOOD - Most performance criteria met"
        elif passed_criteria >= len(criteria) * 0.5:
            result = "Result: ACCEPTABLE - Some performance issues detected"
        else:
            result = "Result: POOR - Significant performance issues detected"
        
        criteria_lines.append(result)
        
        # Print to console
        for line in criteria_lines:
            print(line)
        
        # Save assessment to file
        assessment_file = os.path.join(self.analysis_dir, f"performance_assessment_{self.analysis_timestamp}.txt")
        with open(assessment_file, 'w') as f:
            f.write('\n'.join(criteria_lines))
        
        print(f"Performance assessment saved to: {assessment_file}")
    
    def create_latency_distribution_plot(self):
        """Create comprehensive latency distribution visualization"""
        fig, axes = plt.subplots(2, 2, figsize=(16, 12))
        fig.suptitle('FSM Sandbox Latency Distribution Analysis', fontsize=16, fontweight='bold')
        
        metrics = ['Render_Latency_ms', 'NDT_Processing_Latency_ms', 'Total_Pipeline_Latency_ms']
        
        # Histogram of Total Pipeline Latency
        ax1 = axes[0, 0]
        data = self.df['Total_Pipeline_Latency_ms']
        ax1.hist(data, bins=50, alpha=0.7, color='skyblue', edgecolor='black')
        ax1.axvline(data.mean(), color='red', linestyle='--', linewidth=2, label=f'Mean: {data.mean():.2f}ms')
        ax1.axvline(data.quantile(0.99), color='orange', linestyle='--', linewidth=2, label=f'P99: {data.quantile(0.99):.2f}ms')
        ax1.set_xlabel('Total Pipeline Latency (ms)')
        ax1.set_ylabel('Frequency')
        ax1.set_title('Total Pipeline Latency Distribution')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        
        # Box plot comparing all metrics
        ax2 = axes[0, 1]
        data_for_box = [self.df[metric].dropna() for metric in metrics]
        labels = ['Render', 'NDT Processing', 'Total Pipeline']
        bp = ax2.boxplot(data_for_box, labels=labels, patch_artist=True)
        colors = ['lightcoral', 'lightblue', 'lightgreen']
        for patch, color in zip(bp['boxes'], colors):
            patch.set_facecolor(color)
        ax2.set_ylabel('Latency (ms)')
        ax2.set_title('Latency Comparison (Box Plot)')
        ax2.grid(True, alpha=0.3)
        
        # Time series plot
        ax3 = axes[1, 0]
        self.df['Record_Index'] = range(len(self.df))
        ax3.plot(self.df['Record_Index'], self.df['Total_Pipeline_Latency_ms'], 
                alpha=0.6, linewidth=1, color='blue')
        ax3.set_xlabel('Record Index')
        ax3.set_ylabel('Total Pipeline Latency (ms)')
        ax3.set_title('Latency Over Time')
        ax3.grid(True, alpha=0.3)
        
        # Add rolling average
        window_size = max(10, len(self.df) // 50)
        rolling_avg = self.df['Total_Pipeline_Latency_ms'].rolling(window=window_size).mean()
        ax3.plot(self.df['Record_Index'], rolling_avg, color='red', linewidth=2, 
                label=f'Rolling Average (window={window_size})')
        ax3.legend()
        
        # Cumulative distribution
        ax4 = axes[1, 1]
        sorted_data = np.sort(self.df['Total_Pipeline_Latency_ms'])
        cumulative = np.arange(1, len(sorted_data) + 1) / len(sorted_data) * 100
        ax4.plot(sorted_data, cumulative, linewidth=2, color='green')
        ax4.axvline(self.stats_results['Total_Pipeline_Latency_ms']['p95'], 
                   color='orange', linestyle='--', label='P95')
        ax4.axvline(self.stats_results['Total_Pipeline_Latency_ms']['p99'], 
                   color='red', linestyle='--', label='P99')
        ax4.set_xlabel('Total Pipeline Latency (ms)')
        ax4.set_ylabel('Cumulative Percentage (%)')
        ax4.set_title('Cumulative Distribution Function')
        ax4.legend()
        ax4.grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        # Save plot
        plot_filename = os.path.join(self.analysis_dir, f'latency_distribution_{self.analysis_timestamp}.png')
        plt.savefig(plot_filename, dpi=300, bbox_inches='tight')
        print(f"Latency distribution plot saved: {plot_filename}")
        
        return fig
    
    def create_performance_summary_table_plot(self):
        """Create a visual summary table of key performance metrics"""
        fig, ax = plt.subplots(figsize=(14, 8))
        ax.axis('tight')
        ax.axis('off')
        
        # Prepare data for table
        table_data = []
        headers = ['Metric', 'Mean (ms)', 'P95 (ms)', 'P99 (ms)', 'Max (ms)', 'Assessment']
        
        for metric, stats in self.stats_results.items():
            metric_name = metric.replace('_', ' ').replace('ms', '').strip().title()
            
            # Assessment based on values
            if 'Total Pipeline' in metric_name:
                assessment = 'Excellent' if stats['p99'] < 50 else 'Good' if stats['p99'] < 100 else 'Needs Improvement'
            elif 'Render' in metric_name:
                assessment = 'Excellent' if stats['mean'] < 25 else 'Good' if stats['mean'] < 50 else 'Needs Improvement'
            else:
                assessment = 'Excellent' if stats['p95'] < 10 else 'Good' if stats['p95'] < 20 else 'Needs Improvement'
            
            table_data.append([
                metric_name,
                f"{stats['mean']:.2f}",
                f"{stats['p95']:.2f}",
                f"{stats['p99']:.2f}",
                f"{stats['max']:.2f}",
                assessment
            ])
        
        # Create table
        table = ax.table(cellText=table_data, colLabels=headers, 
                        cellLoc='center', loc='center',
                        bbox=[0, 0, 1, 1])
        
        # Style the table
        table.auto_set_font_size(False)
        table.set_fontsize(11)
        table.scale(1, 2.5)
        
        # Color code the assessment column
        for i in range(len(table_data)):
            assessment = table_data[i][-1]
            if assessment == 'Excellent':
                table[(i+1, 5)].set_facecolor('#90EE90')  # Light green
            elif assessment == 'Good':
                table[(i+1, 5)].set_facecolor('#FFE4B5')  # Light orange
            else:
                table[(i+1, 5)].set_facecolor('#FFB6C1')  # Light pink
        
        # Header styling
        for j in range(len(headers)):
            table[(0, j)].set_facecolor('#4CAF50')
            table[(0, j)].set_text_props(weight='bold', color='white')
        
        # Add session info as subtitle
        subtitle = f'Session: {self.session_id} | Analysis: {self.analysis_timestamp} | Records: {len(self.df)}'
        plt.figtext(0.5, 0.02, subtitle, ha='center', fontsize=10, style='italic')
        
        plt.title('FSM Sandbox Performance Summary', fontsize=16, fontweight='bold', pad=20)
        
        # Save table plot
        table_filename = os.path.join(self.analysis_dir, f'performance_summary_{self.analysis_timestamp}.png')
        plt.savefig(table_filename, dpi=300, bbox_inches='tight')
        print(f"Performance summary table saved: {table_filename}")
        
        return fig
    
    def create_analysis_index(self):
        """Create an index file listing all generated outputs"""
        index_content = f"""# FSM Sandbox Timing Analysis Results
        
## Analysis Information
- **Analysis Timestamp**: {self.analysis_timestamp}
- **Session ID**: {self.session_id}
- **Source Data File**: {os.path.basename(self.csv_file)}
- **Total Records Analyzed**: {len(self.df)}
- **Analysis Date**: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}

## Generated Files

### Statistical Analysis
- `statistics_{self.analysis_timestamp}.json` - Detailed statistics in JSON format
- `analysis_report_{self.analysis_timestamp}.txt` - Human-readable analysis report
- `performance_assessment_{self.analysis_timestamp}.txt` - Performance criteria assessment

### Visualizations
- `latency_distribution_{self.analysis_timestamp}.png` - Comprehensive latency distribution plots
- `performance_summary_{self.analysis_timestamp}.png` - Performance summary table

### Key Performance Indicators
"""
        
        # Add key metrics to index
        pipeline_stats = self.stats_results['Total_Pipeline_Latency_ms']
        render_stats = self.stats_results['Render_Latency_ms']
        ndt_stats = self.stats_results['NDT_Processing_Latency_ms']
        
        index_content += f"""
- **Total Pipeline Latency P99**: {pipeline_stats['p99']:.2f}ms
- **Render Latency Mean**: {render_stats['mean']:.2f}ms
- **NDT Processing Latency P95**: {ndt_stats['p95']:.2f}ms
- **Pipeline Latency Standard Deviation**: {pipeline_stats['std']:.2f}ms
"""
        
        index_file = os.path.join(self.analysis_dir, "README.md")
        with open(index_file, 'w') as f:
            f.write(index_content)
        
        print(f"Analysis index created: {index_file}")
    
    def run_complete_analysis(self):
        """Run the complete analysis pipeline"""
        print("Starting FSM Sandbox Timing Analysis...")
        print(f"Output directory: {self.analysis_dir}")
        
        if not self.load_and_validate_data():
            return False
        
        self.calculate_statistics()
        self.print_statistics_table()
        self.assess_performance_criteria()
        
        print(f"\nGenerating visualizations...")
        self.create_latency_distribution_plot()
        self.create_performance_summary_table_plot()
        
        print(f"\nCreating analysis index...")
        self.create_analysis_index()
        
        print(f"\nAnalysis complete!")
        print(f"All results saved to: {self.analysis_dir}")
        return True


def main():
    parser = argparse.ArgumentParser(description='Analyze FSM Sandbox timing performance data')
    parser.add_argument('csv_file', help='Path to the timing log CSV file')
    parser.add_argument('--show-plots', action='store_true', help='Display plots after generation')
    
    args = parser.parse_args()
    
    if not os.path.exists(args.csv_file):
        print(f"Error: File {args.csv_file} not found")
        sys.exit(1)
    
    analyzer = FSMTimingAnalyzer(args.csv_file)
    success = analyzer.run_complete_analysis()
    
    if success and args.show_plots:
        plt.show()
    
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()