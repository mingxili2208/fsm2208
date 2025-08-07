#!/bin/bash

# Experiment B Launcher Script
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_FILE="$SCRIPT_DIR/../config.yaml"

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

# Cleanup function
cleanup() {
    print_colored $YELLOW "\nCleaning up processes..."
    if [ ! -z "$LATENCY_PID" ]; then
        kill $LATENCY_PID 2>/dev/null || true
    fi
    if [ ! -z "$ERROR_PID" ]; then
        kill $ERROR_PID 2>/dev/null || true
    fi
    wait 2>/dev/null || true
    print_colored $GREEN "Cleanup completed."
}

trap cleanup EXIT INT TERM

print_header "Experiment B: Latency and Baseline Tracking Error Analysis"

# Setup environment
print_colored $BLUE "Setting up environment..."
source "$(dirname "$SCRIPT_DIR")/setup_env.sh"

# Verify configuration
if [ ! -f "$CONFIG_FILE" ]; then
    print_colored $RED "Configuration file not found: $CONFIG_FILE"
    exit 1
fi

print_colored $GREEN "Environment setup completed."

# Check required topics
print_colored $BLUE "Checking system readiness..."

PERCEPTION_TOPIC=$(python3 -c "import sys; sys.path.append('$(dirname "$SCRIPT_DIR")'); from config_manager import ConfigManager; cm = ConfigManager('$CONFIG_FILE'); print(cm.get('experiment_b.topics.perception_input', '/localization/kinematic_state'))")
PLANNING_TOPIC=$(python3 -c "import sys; sys.path.append('$(dirname "$SCRIPT_DIR")'); from config_manager import ConfigManager; cm = ConfigManager('$CONFIG_FILE'); print(cm.get('experiment_b.topics.planning_output', '/planning/trajectory'))")
GT_TOPIC=$(python3 -c "import sys; sys.path.append('$(dirname "$SCRIPT_DIR")'); from config_manager import ConfigManager; cm = ConfigManager('$CONFIG_FILE'); print(cm.get('experiment_b.topics.ground_truth', '/real_world/follow_adtruck/transformed_with_covariance'))")

print_colored $YELLOW "Expected topics:"
echo "  - Perception: $PERCEPTION_TOPIC"
echo "  - Planning: $PLANNING_TOPIC"
echo "  - Ground Truth: $GT_TOPIC"

# Wait for critical topics
if ! wait_for_topic "$PERCEPTION_TOPIC" 60; then
    print_colored $RED "Critical topic $PERCEPTION_TOPIC not available. Is the ADS running?"
    exit 1
fi

if ! wait_for_topic "$GT_TOPIC" 30; then
    print_colored $YELLOW "Warning: Ground truth topic $GT_TOPIC not available."
fi

print_header "Starting Experiment B Data Collection"

SESSION_TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
print_colored $GREEN "Session timestamp: $SESSION_TIMESTAMP"

# Start latency impact logger
print_colored $BLUE "Starting Latency Impact Logger..."
cd "$SCRIPT_DIR"
python3 latency_impact_logger.py --config "$CONFIG_FILE" &
LATENCY_PID=$!
print_colored $GREEN "Latency logger started (PID: $LATENCY_PID)"

# Start tracking error analyzer
print_colored $BLUE "Starting Tracking Error Analyzer..."
python3 tracking_error_analyzer.py --config "$CONFIG_FILE" &
ERROR_PID=$!
print_colored $GREEN "Error analyzer started (PID: $ERROR_PID)"

print_header "Experiment B Data Collection Active"
print_colored $GREEN "Both loggers are now collecting data!"
print_colored $YELLOW "Instructions:"
echo "  1. First, drive normally for 3-5 minutes to collect latency data"
echo "  2. Then, position the vehicle at the start of the test path"
echo "  3. Drive straight at constant velocity (0.5 m/s) for error baseline test"
echo "  4. The error analyzer will automatically detect when you're at the start"
echo "  5. Press Ctrl+C when both tests are complete"

print_colored $BLUE "\nPress Ctrl+C to stop data collection..."

# Monitor processes
while true; do
    if ! kill -0 $LATENCY_PID 2>/dev/null; then
        print_colored $RED "Latency logger stopped unexpectedly!"
        break
    fi
    if ! kill -0 $ERROR_PID 2>/dev/null; then
        print_colored $RED "Error analyzer stopped unexpectedly!"
        break
    fi
    
    sleep 5
done

print_header "Experiment B Data Collection Completed"
print_colored $GREEN "Session $SESSION_TIMESTAMP data collection finished."
print_colored $BLUE "Next steps:"
echo "  1. Analyze the data: python3 analyze_experiment_b.py data/exp_b_latency_data_$SESSION_TIMESTAMP.csv data/exp_b_tracking_error_$SESSION_TIMESTAMP.csv"
echo "  2. Check results/ directory for comprehensive analysis outputs"
echo "  3. Review logs/ directory for session details"