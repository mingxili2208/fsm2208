根据您的问题，我来逐一回答并提供完整的解决方案：

## 1. 关于实验内容
是的，我提供的新版本包含了A1和A2两个实验的完整内容：
- **A1_sensor_latency**: 时效性验证实验
- **A2_visual_location_error**: 保真度与自洽性验证实验

## 2. 配置文件目录级别
建议在 `Validation` 根目录下放置配置文件，这样两个子实验都可以共享同一个配置。

## 3. 优化的目录结构和一键启动脚本

### 推荐的目录结构
```
Validation/
├── config.yaml                    # 共享配置文件
├── requirements.txt               # Python依赖
├── setup_env.sh                  # 环境设置脚本
├── start_experiments.sh          # 一键启动脚本
├── config_manager.py             # 配置管理器（共享）
├── A1_sensor_latency/
│   ├── log_recorder_node.py
│   ├── analyze_timing_data.py
│   ├── data/
│   ├── logs/
│   └── results/
├── A2_visual_location_error/
│   ├── ground_truth_recorder.py
│   ├── offline_ndt_recorder.py
│   ├── analyze_fidelity_data.py
│   ├── data/
│   ├── logs/
│   └── results/
└── B_latency/                    # 为将来的实验预留
```

## 4. 完整解决方案

### 环境设置脚本 (setup_env.sh)
```bash
#!/bin/bash

# FSM Sandbox Environment Setup Script
echo "Setting up FSM Sandbox Validation Environment..."

# Your FSM system parameters
export SIMULATOR_LOCAL_HOST="localhost"
export SIMULATOR_PORT="2000"
export TRAFFIC_MANAGER_PORT="8000"
export TEAM_AGENT=${OP_BRIDGE_ROOT}/op_bridge/fsm_lab_simulation/ego_vehicle_initializer2.py
export PYTHONPATH="${CARLA_ROOT}/PythonAPI/carla/":"${SCENARIO_RUNNER_ROOT}":"${OP_BRIDGE_ROOT}":${PYTHONPATH}
export AGENT_FRAME_RATE="20"

# Autonomous actor default role_name and type
export NPC_ROLE_NAME="vis_npc"
export NPC_MODEL_TYPE="vehicle.tesla.model3"

# Bridge mode configuration
export OP_BRIDGE_MODE="free"
export FREE_MAP_NAME="fsm_lab_sandbox_right_hand_driving_scene"

# ROS2 environment
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=46
source ${AUTOWARE_ROOT}/install/setup.bash
source ${OP_ROS_PLUGINS_ROOT}/install/setup.bash

# Python virtual environment for experiments
VENV_PATH="$(dirname "$0")/venv_fsm_validation"

# Create virtual environment if it doesn't exist
if [ ! -d "$VENV_PATH" ]; then
    echo "Creating Python virtual environment at $VENV_PATH..."
    python3 -m venv "$VENV_PATH"
fi

# Activate virtual environment
echo "Activating Python virtual environment..."
source "$VENV_PATH/bin/activate"

# Install dependencies if requirements.txt exists
if [ -f "$(dirname "$0")/requirements.txt" ]; then
    echo "Installing Python dependencies..."
    pip install -r "$(dirname "$0")/requirements.txt"
fi

# Add current directory to Python path for config_manager
export PYTHONPATH="$(dirname "$0"):${PYTHONPATH}"

echo "Environment setup complete!"
echo "Virtual environment: $VENV_PATH"
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
```

### 一键启动脚本 (start_experiments.sh)
```bash
#!/bin/bash

# FSM Sandbox Validation Experiments Launcher
# This script starts both A1 (timing) and A2 (fidelity) data collection simultaneously

set -e  # Exit on any error

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_FILE="$SCRIPT_DIR/config.yaml"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

print_colored() {
    echo -e "${1}${2}${NC}"
}

print_header() {
    echo
    print_colored $BLUE "=================================="
    print_colored $BLUE "$1"
    print_colored $BLUE "=================================="
}

# Function to check if a command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Function to check if ROS topic exists
wait_for_topic() {
    local topic=$1
    local timeout=${2:-30}
    local count=0
    
    print_colored $YELLOW "Waiting for topic: $topic"
    while [ $count -lt $timeout ]; do
        if ros2 topic list | grep -q "$topic"; then
            print_colored $GREEN "Topic $topic is available!"
            return 0
        fi
        sleep 1
        count=$((count + 1))
        echo -n "."
    done
    
    print_colored $RED "Timeout waiting for topic: $topic"
    return 1
}

# Function to cleanup processes
cleanup() {
    print_colored $YELLOW "\nCleaning up processes..."
    if [ ! -z "$A1_PID" ]; then
        kill $A1_PID 2>/dev/null || true
    fi
    if [ ! -z "$A2_PID" ]; then
        kill $A2_PID 2>/dev/null || true
    fi
    if [ ! -z "$BAG_PID" ]; then
        kill $BAG_PID 2>/dev/null || true
    fi
    wait 2>/dev/null || true
    print_colored $GREEN "Cleanup completed."
}

# Set up signal handlers
trap cleanup EXIT INT TERM

print_header "FSM Sandbox Validation Experiments"

# Setup environment
print_colored $BLUE "Setting up environment..."
source "$SCRIPT_DIR/setup_env.sh"

# Verify configuration file exists
if [ ! -f "$CONFIG_FILE" ]; then
    print_colored $RED "Configuration file not found: $CONFIG_FILE"
    print_colored $YELLOW "Creating default configuration..."
    # This will trigger config_manager to create default config
    cd "$SCRIPT_DIR"
    python3 -c "from config_manager import ConfigManager; ConfigManager('config.yaml')"
    print_colored $GREEN "Default configuration created. Please edit config.yaml and run again."
    exit 1
fi

# Verify ROS2 environment
if ! command_exists ros2; then
    print_colored $RED "ROS2 not found. Please ensure ROS2 is installed and sourced."
    exit 1
fi

print_colored $GREEN "Environment setup completed."

# Check if FSM system is running by looking for expected topics
print_colored $BLUE "Checking FSM system status..."

# Read topics from config
cd "$SCRIPT_DIR"
TIMING_TOPIC=$(python3 -c "from config_manager import ConfigManager; cm = ConfigManager('config.yaml'); print(cm.get('experiment_a1.topics.timing_sync', '/fsm_sandbox/timing/t1_t2'))")
LIDAR_TOPIC=$(python3 -c "from config_manager import ConfigManager; cm = ConfigManager('config.yaml'); print(cm.get('experiment_a1.topics.lidar_input', '/carla/follow_adtruck/carla_pointcloud'))")
GT_TOPIC=$(python3 -c "from config_manager import ConfigManager; cm = ConfigManager('config.yaml'); print(cm.get('experiment_a2.topics.ground_truth', '/real_world/follow_adtruck/transformed_with_covariance'))")

print_colored $YELLOW "Expected topics:"
echo "  - Timing sync: $TIMING_TOPIC"
echo "  - LiDAR: $LIDAR_TOPIC" 
echo "  - Ground truth: $GT_TOPIC"

# Wait for critical topics (with timeout)
if ! wait_for_topic "$LIDAR_TOPIC" 60; then
    print_colored $RED "Critical topic $LIDAR_TOPIC not available. Is FSM system running?"
    exit 1
fi

if ! wait_for_topic "$GT_TOPIC" 30; then
    print_colored $YELLOW "Warning: Ground truth topic $GT_TOPIC not available."
    print_colored $YELLOW "A2 experiment may not collect data properly."
fi

print_header "Starting Data Collection"

# Create session timestamp
SESSION_TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
print_colored $GREEN "Session timestamp: $SESSION_TIMESTAMP"

# Start A1 experiment (timing analysis)
print_colored $BLUE "Starting A1 Timing Analysis..."
cd "$SCRIPT_DIR/A1_sensor_latency"
python3 log_recorder_node.py --config "$CONFIG_FILE" &
A1_PID=$!
print_colored $GREEN "A1 recorder started (PID: $A1_PID)"

# Start A2 experiment (fidelity analysis)
print_colored $BLUE "Starting A2 Fidelity Analysis..."
cd "$SCRIPT_DIR/A2_visual_location_error"
python3 ground_truth_recorder.py &
A2_PID=$!
print_colored $GREEN "A2 ground truth recorder started (PID: $A2_PID)"

# Start ROS bag recording for LiDAR data
BAG_DIR="$SCRIPT_DIR/A2_visual_location_error/data/lidar_bag_$SESSION_TIMESTAMP"
print_colored $BLUE "Starting ROS bag recording..."
ros2 bag record -o "$BAG_DIR" "$LIDAR_TOPIC" &
BAG_PID=$!
print_colored $GREEN "ROS bag recording started (PID: $BAG_PID)"
print_colored $YELLOW "Recording to: $BAG_DIR"

print_header "Data Collection Active"
print_colored $GREEN "Both experiments are now collecting data!"
print_colored $YELLOW "Recommendations:"
echo "  - Let the system run for 3-5 minutes to collect sufficient data"
echo "  - Drive the vehicle through various scenarios"
echo "  - Monitor the terminal outputs for data collection status"
echo "  - Press Ctrl+C to stop all recordings when ready"

# Wait for user interrupt or process completion
print_colored $BLUE "\nPress Ctrl+C to stop data collection..."

# Monitor processes
while true; do
    # Check if processes are still running
    if ! kill -0 $A1_PID 2>/dev/null; then
        print_colored $RED "A1 recorder stopped unexpectedly!"
        break
    fi
    if ! kill -0 $A2_PID 2>/dev/null; then
        print_colored $RED "A2 recorder stopped unexpectedly!"
        break
    fi
    if ! kill -0 $BAG_PID 2>/dev/null; then
        print_colored $RED "ROS bag recording stopped unexpectedly!"
        break
    fi
    
    sleep 5
    
    # Show brief status every 30 seconds
    if [ $(($(date +%s) % 30)) -eq 0 ]; then
        print_colored $GREEN "Data collection active... (Session: $SESSION_TIMESTAMP)"
    fi
done

print_header "Data Collection Completed"
print_colored $GREEN "Session $SESSION_TIMESTAMP data collection finished."
print_colored $BLUE "Next steps:"
echo "  1. For A1 analysis: cd A1_sensor_latency && python3 analyze_timing_data.py data/fsm_timing_data_$SESSION_TIMESTAMP.csv"
echo "  2. For A2 offline processing: Use offline_ndt_recorder.py with the recorded bag"
echo "  3. Check logs/ directories for detailed session information"
```

### 共享配置文件 (Validation/config.yaml)
```yaml
# FSM Sandbox Validation Configuration
project:
  name: "FSM Sandbox Validation"
  version: "2.0"
  description: "Integrated Timing and Fidelity Analysis"

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
    ground_truth_type: "PoseWithCovarianceStamped"
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
```

### 依赖文件 (requirements.txt)
```txt
# FSM Sandbox Validation Dependencies
pandas>=1.3.0
numpy>=1.20.0
matplotlib>=3.3.0
seaborn>=0.11.0
transforms3d>=0.3.1
pyyaml>=5.4.0
scipy>=1.7.0
```

### 更新的配置管理器 (config_manager.py)
```python
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
```

## 5. 使用指南

### 初始设置
```bash
cd Validation
chmod +x setup_env.sh start_experiments.sh
./setup_env.sh  # 这会创建独立的虚拟环境，不影响主环境
```

### 运行实验
```bash
cd Validation
./start_experiments.sh
```

### 手动分析（实验完成后）
```bash
# A1分析
cd A1_sensor_latency
python3 analyze_timing_data.py data/fsm_timing_data_YYYYMMDD_HHMMSS.csv

# A2离线处理和分析
cd A2_visual_location_error
python3 offline_ndt_recorder.py  # 在独立环境中运行
ros2 bag play data/lidar_bag_YYYYMMDD_HHMMSS
python3 analyze_fidelity_data.py data/ground_truth_poses_YYYYMMDD_HHMMSS.csv data/offline_ndt_poses_YYYYMMDD_HHMMSS.csv
```

这个解决方案提供了：
1. **完全隔离的Python环境** - 使用虚拟环境，不影响主系统
2. **统一的配置管理** - 在Validation根目录的config.yaml
3. **一键启动功能** - start_experiments.sh同时启动两个实验
4. **环境变量集成** - setup_env.sh包含您的FSM系统参数
5. **智能话题检测** - 自动检测FSM系统是否运行
6. **详细的状态监控** - 实时显示数据收集状态