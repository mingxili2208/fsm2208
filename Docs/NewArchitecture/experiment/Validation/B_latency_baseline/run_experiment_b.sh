#!/bin/bash

# Experiment B Execution Script
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

print_colored() {
    echo -e "${1}${2}${NC}"
}

print_header() {
    echo
    print_colored $BLUE "=========================================="
    print_colored $BLUE "$1"
    print_colored $BLUE "=========================================="
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

# Check dependencies
command -v ros2 >/dev/null 2>&1 || { echo "ROS2 not found. Please source ROS2 setup."; exit 1; }
command -v python3 >/dev/null 2>&1 || { echo "Python3 not found."; exit 1; }

# Setup environment
cd "$SCRIPT_DIR"
source /opt/ros/humble/setup.bash  # Adjust for your ROS2 distribution

print_colored $GREEN "Environment setup completed."

# Check required topics
print_colored $BLUE "Checking system readiness..."

# Wait for critical topics
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

# Check critical topics
if ! wait_for_topic "/localization/kinematic_state" 60; then
    print_colored $RED "Critical topic not available. Is the ADS running?"
    exit 1
fi

if ! wait_for_topic "/planning/trajectory" 30; then
    print_colored $YELLOW "Warning: Planning topic not available."
fi

print_header "Starting Experiment B Data Collection"

SESSION_TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
print_colored $GREEN "Session timestamp: $SESSION_TIMESTAMP"

# Start latency chain analyzer
print_colored $BLUE "Starting Latency Chain Analyzer..."
python3 latency_chain_analyzer.py &
LATENCY_PID=$!
print_colored $GREEN "Latency chain analyzer started (PID: $LATENCY_PID)"

# Start tracking error measurer
print_colored $BLUE "Starting Tracking Error Measurer..."
python3 tracking_error_measurer.py &
ERROR_PID=$!
print_colored $GREEN "Tracking error measurer started (PID: $ERROR_PID)"

print_header "Experiment B Data Collection Active"
print_colored $GREEN "Data collection is now active!"
print_colored $YELLOW "Instructions:"
echo "  1. First phase (5-10 minutes): Drive normally to collect latency data"
echo "  2. Second phase: Position vehicle at start of straight test path"
echo "  3. Drive straight at constant 0.5 m/s velocity for baseline test"
echo "  4. The system will automatically detect test phases"
echo "  5. Press Ctrl+C when both phases are complete"

print_colored $BLUE "\nPress Ctrl+C to stop data collection..."

# Monitor processes
while true; do
    if ! kill -0 $LATENCY_PID 2>/dev/null; then
        print_colored $RED "Latency analyzer stopped unexpectedly!"
        break
    fi
    if ! kill -0 $ERROR_PID 2>/dev/null; then
        print_colored $RED "Error measurer stopped unexpectedly!"
        break
    fi
    
    sleep 5
done

print_header "Experiment B Data Collection Completed"
print_colored $GREEN "Session $SESSION_TIMESTAMP data collection finished."

# Check for timing log from your PC controller
TIMING_LOG_PATTERN="logs/timing_log_*.csv"
TIMING_LOG=$(ls $TIMING_LOG_PATTERN 2>/dev/null | tail -1)

if [ -f "$TIMING_LOG" ]; then
    print_colored $GREEN "Found timing log: $TIMING_LOG"
    print_colored $BLUE "Analyzing V2R latency from timing log..."
    python3 v2r_latency_analyzer.py "$TIMING_LOG"
    V2R_CSV=$(ls data/v2r_latency_*.csv 2>/dev/null | tail -1)
else
    print_colored $YELLOW "Warning: No timing log found. V2R analysis will be skipped."
    V2R_CSV=""
fi

# Find generated data files
LATENCY_CHAIN_CSV=$(ls data/latency_chain_*.csv 2>/dev/null | tail -1)
TRACKING_ERROR_CSV=$(ls data/tracking_error_*.csv 2>/dev/null | tail -1)

print_colored $BLUE "Next steps:"
if [ -f "$LATENCY_CHAIN_CSV" ] && [ -f "$TRACKING_ERROR_CSV" ] && [ -f "$V2R_CSV" ]; then
    echo "  1. Run analysis: python3 experiment_b_analyzer.py $LATENCY_CHAIN_CSV $V2R_CSV $TRACKING_ERROR_CSV"
    echo "  2. Check results/ directory for comprehensive analysis"
    echo "  3. Review logs/ directory for session details"
    
    # Ask if user wants to run analysis now
    print_colored $YELLOW "Run analysis now? (y/n): "
    read -r response
    if [[ "$response" =~ ^[Yy]$ ]]; then
        print_colored $BLUE "Running analysis..."
        python3 experiment_b_analyzer.py "$LATENCY_CHAIN_CSV" "$V2R_CSV" "$TRACKING_ERROR_CSV" --show-plots
    fi
else
    print_colored $RED "Error: Some data files missing. Please check data collection."
    echo "  Expected files:"
    echo "    - Latency chain: $LATENCY_CHAIN_CSV"
    echo "    - Tracking error: $TRACKING_ERROR_CSV"
    echo "    - V2R data: $V2R_CSV"
fi