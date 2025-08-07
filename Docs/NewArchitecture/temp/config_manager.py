#!/usr/bin/env python3

import yaml
import os
from pathlib import Path
from typing import Dict, Any, Optional

class ConfigManager:
    """
    Centralized configuration management for FSM Sandbox experiments.
    Supports both A1 and A2 experiments with shared configuration.
    """
    
    def __init__(self, config_file: str = "config.yaml"):
        # Always look for config in the Validation root directory
        if not os.path.isabs(config_file):
            # Find the Validation root directory
            current_dir = os.path.dirname(os.path.abspath(__file__))
            while current_dir != '/' and not os.path.basename(current_dir) == 'Validation':
                parent = os.path.dirname(current_dir)
                if parent == current_dir:  # Reached filesystem root
                    break
                current_dir = parent
            
            if os.path.basename(current_dir) == 'Validation':
                config_file = os.path.join(current_dir, config_file)
            else:
                # Fallback to current directory
                config_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), config_file)
        
        self.config_file = config_file
        self.config = self._load_config()
        self._validate_config()
        
    def _load_config(self) -> Dict[str, Any]:
        """Load configuration from YAML file"""
        if os.path.exists(self.config_file):
            try:
                with open(self.config_file, 'r') as f:
                    config = yaml.safe_load(f)
                print(f"Configuration loaded from: {os.path.abspath(self.config_file)}")
                return config
            except Exception as e:
                print(f"Error loading config from {self.config_file}: {e}")
        
        # If no config file found, create default
        print(f"No configuration file found. Creating default config at {self.config_file}...")
        default_config = self._create_default_config()
        self._save_config(default_config, self.config_file)
        return default_config
    
    def _create_default_config(self) -> Dict[str, Any]:
        """Create default configuration"""
        return {
            'project': {
                'name': 'FSM Sandbox Validation',
                'version': '2.0',
                'description': 'Integrated Timing and Fidelity Analysis'
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
                },
                'quality_thresholds': {
                    'pipeline_p99_ms': 100,
                    'render_mean_ms': 50,
                    'pipeline_std_ms': 30,
                    'ndt_p95_ms': 20
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
                },
                'parameters': {
                    'alignment_tolerance_windows_ms': [10, 50, 100, 200],
                    'min_alignment_rate_percent': 50,
                    'outlier_removal_enabled': True,
                    'outlier_iqr_factor': 1.5
                },
                'quality_thresholds': {
                    'translation_mean_mm': 5,
                    'translation_p99_mm': 20,
                    'yaw_mean_deg': 0.1,
                    'yaw_p95_deg': 0.5,
                    'translation_std_mm': 10
                }
            },
            'file_management': {
                'base_directories': {
                    'data': 'data',
                    'logs': 'logs',
                    'results': 'results'
                },
                'file_naming': {
                    'timestamp_format': '%Y%m%d_%H%M%S',
                    'use_session_id': True
                }
            },
            'visualization': {
                'style': 'default',
                'figure_size': [16, 12],
                'font_size': 12,
                'dpi': 300,
                'color_palette': 'husl',
                'enable_grid': True,
                'grid_alpha': 0.3
            }
        }
    
    def _save_config(self, config: Dict[str, Any], filepath: str):
        """Save configuration to file"""
        try:
            # Ensure directory exists
            os.makedirs(os.path.dirname(filepath), exist_ok=True)
            
            with open(filepath, 'w') as f:
                yaml.dump(config, f, default_flow_style=False, indent=2)
            print(f"Configuration saved to: {os.path.abspath(filepath)}")
        except Exception as e:
            print(f"Error saving config: {e}")
    
    def _validate_config(self):
        """Validate configuration structure"""
        required_sections = ['experiment_a1', 'experiment_a2', 'file_management']
        for section in required_sections:
            if section not in self.config:
                raise ValueError(f"Missing required configuration section: {section}")
    
    def get(self, key_path: str, default: Any = None) -> Any:
        """Get configuration value using dot notation"""
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