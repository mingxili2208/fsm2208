#!/usr/bin/env python3

import pandas as pd
import numpy as np
import argparse
import os
import csv
from datetime import datetime
from typing import List, Dict, Tuple

class V2RLatencyAnalyzer:
    """
    Analyzer for V2R (Virtual-to-Real) latency using RTT measurements from timing logs
    """
    
    def __init__(self, timing_log_path: str):
        self.timing_log_path = timing_log_path
        self.session_timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        
        # Load and process timing log
        self.load_timing_log()
        self.analyze_v2r_latency()

    def load_timing_log(self):
        """Load timing log CSV file"""
        try:
            self.timing_df = pd.read_csv(self.timing_log_path)
            print(f"Loaded timing log: {len(self.timing_df)} records")
            
            # Validate required columns
            required_columns = ['event_type', 'pc_time', 'steering_angle', 'speed']
            missing_columns = [col for col in required_columns if col not in self.timing_df.columns]
            if missing_columns:
                raise ValueError(f"Missing columns in timing log: {missing_columns}")
                
        except Exception as e:
            print(f"Error loading timing log: {e}")
            raise

    def analyze_v2r_latency(self):
        """Analyze V2R latency using RTT measurements"""
        # Filter relevant events
        cmd_sent_events = self.timing_df[self.timing_df['event_type'] == 'CMD_SENT']
        nrf_sent_events = self.timing_df[self.timing_df['event_type'] == 'CMD_SENT_BY_NRF']
        
        print(f"Found {len(cmd_sent_events)} CMD_SENT events")
        print(f"Found {len(nrf_sent_events)} CMD_SENT_BY_NRF events")
        
        # Match command pairs and calculate RTT
        self.v2r_measurements = []
        
        for _, cmd_event in cmd_sent_events.iterrows():
            cmd_time = cmd_event['pc_time']
            
            # Find corresponding NRF event within reasonable time window
            time_window = 1000  # 1 second in ms
            candidates = nrf_sent_events[
                (nrf_sent_events['pc_time'] >= cmd_time) &
                (nrf_sent_events['pc_time'] <= cmd_time + time_window)
            ]
            
            if len(candidates) > 0:
                # Take the closest match
                closest_nrf = candidates.iloc[0]
                rtt_ms = closest_nrf['pc_time'] - cmd_time
                
                # Estimate V2R latency as RTT/2
                v2r_latency_ms = rtt_ms / 2.0
                
                # Validate measurement
                if 0 < rtt_ms < 500:  # Reasonable RTT range
                    measurement = {
                        'cmd_time': cmd_time,
                        'nrf_time': closest_nrf['pc_time'],
                        'rtt_ms': rtt_ms,
                        'v2r_latency_ms': v2r_latency_ms,
                        'steering_angle': cmd_event['steering_angle'],
                        'speed': cmd_event['speed']
                    }
                    self.v2r_measurements.append(measurement)
        
        print(f"Successfully analyzed {len(self.v2r_measurements)} V2R latency measurements")

    def get_v2r_statistics(self) -> Dict:
        """Calculate V2R latency statistics"""
        if not self.v2r_measurements:
            return {}
        
        v2r_latencies = [m['v2r_latency_ms'] for m in self.v2r_measurements]
        rtts = [m['rtt_ms'] for m in self.v2r_measurements]
        
        stats = {
            'count': len(v2r_latencies),
            'v2r_mean_ms': np.mean(v2r_latencies),
            'v2r_median_ms': np.median(v2r_latencies),
            'v2r_std_ms': np.std(v2r_latencies),
            'v2r_min_ms': np.min(v2r_latencies),
            'v2r_max_ms': np.max(v2r_latencies),
            'v2r_p95_ms': np.percentile(v2r_latencies, 95),
            'v2r_p99_ms': np.percentile(v2r_latencies, 99),
            'rtt_mean_ms': np.mean(rtts),
            'rtt_std_ms': np.std(rtts)
        }
        
        return stats

    def save_v2r_data(self, output_dir: str = 'data'):
        """Save V2R analysis results to CSV"""
        os.makedirs(output_dir, exist_ok=True)
        
        output_file = os.path.join(output_dir, f'v2r_latency_{self.session_timestamp}.csv')
        
        with open(output_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'measurement_id', 'cmd_time', 'nrf_time', 'rtt_ms', 'v2r_latency_ms',
                'steering_angle', 'speed'
            ])
            
            for i, measurement in enumerate(self.v2r_measurements):
                writer.writerow([
                    f"v2r_{self.session_timestamp}_{i:06d}",
                    measurement['cmd_time'],
                    measurement['nrf_time'],
                    measurement['rtt_ms'],
                    measurement['v2r_latency_ms'],
                    measurement['steering_angle'],
                    measurement['speed']
                ])
        
        print(f"V2R data saved to: {output_file}")
        return output_file


def main():
    parser = argparse.ArgumentParser(description='Analyze V2R latency from timing logs')
    parser.add_argument('timing_log', help='Path to timing log CSV file')
    parser.add_argument('--output-dir', default='data', help='Output directory for results')
    
    args = parser.parse_args()
    
    if not os.path.exists(args.timing_log):
        print(f"Error: Timing log file not found: {args.timing_log}")
        return
    
    # Run analysis
    analyzer = V2RLatencyAnalyzer(args.timing_log)
    stats = analyzer.get_v2r_statistics()
    
    if stats:
        print("\nV2R Latency Statistics:")
        print(f"  Samples: {stats['count']}")
        print(f"  Mean: {stats['v2r_mean_ms']:.2f} ms")
        print(f"  Median: {stats['v2r_median_ms']:.2f} ms")
        print(f"  Std: {stats['v2r_std_ms']:.2f} ms")
        print(f"  Range: {stats['v2r_min_ms']:.2f} - {stats['v2r_max_ms']:.2f} ms")
        print(f"  P95: {stats['v2r_p95_ms']:.2f} ms")
        print(f"  P99: {stats['v2r_p99_ms']:.2f} ms")
        
        # Save results
        output_file = analyzer.save_v2r_data(args.output_dir)
    else:
        print("No valid V2R measurements found")


if __name__ == '__main__':
    main()