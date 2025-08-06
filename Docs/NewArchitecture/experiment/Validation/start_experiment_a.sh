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