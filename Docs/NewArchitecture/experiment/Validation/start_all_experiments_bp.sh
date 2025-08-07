#!/bin/bash

# Complete FSM Sandbox Validation Suite Launcher
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
    print_colored $BLUE "=================================="
    print_colored $BLUE "$1"
    print_colored $BLUE "=================================="
}

# Cleanup function
cleanup() {
    print_colored $YELLOW "\nCleaning up all processes..."
    
    # Kill all experiment processes
    for pid in $CARLA_PID $FSM_PID $AUTOWARE_PID $A1_PID $A2_PID $B1_PID $B2_PID; do
        if [ ! -z "$pid" ]; then
            kill $pid 2>/dev/null || true
        fi
    done
    
    # Wait for processes to terminate
    sleep 3
    
    # Force kill if necessary
    pkill -f "CarlaUE4" 2>/dev/null || true
    pkill -f "log_recorder_node" 2>/dev/null || true
    pkill -f "latency_impact_logger" 2>/dev/null || true
    
    print_colored $GREEN "Cleanup completed."
}

trap cleanup EXIT INT TERM

print_header "FSM Sandbox Complete Validation Suite"

# Check if this is a dry run
DRY_RUN=false
EXPERIMENTS_ONLY=false

while [[ $# -gt 0 ]]; do
    case $1 in
        --dry-run)
            DRY_RUN=true
            shift
            ;;
        --experiments-only)
            EXPERIMENTS_ONLY=true
            shift
            ;;
        --help)
            echo "Usage: $0 [--dry-run] [--experiments-only] [--help]"
            echo "  --dry-run         : Check environment without starting processes"
            echo "  --experiments-only: Start only experiment nodes (assumes system is running)"
            echo "  --help           : Show this help message"
            exit 0
            ;;
        *)
            echo "Unknown option $1"
            exit 1
            ;;
    esac
done

# Setup environment
print_colored $BLUE "Setting up environment..."
source "$SCRIPT_DIR/setup_env.sh"

if [ "$DRY_RUN" = true ]; then
    print_colored $GREEN "Dry run mode - environment check completed successfully"
    exit 0
fi

if [ "$EXPERIMENTS_ONLY" = false ]; then
    print_header "Starting Core Systems"
    
    # Start CARLA
    print_colored $BLUE "Starting CARLA simulator..."
    cd $CARLA_ROOT
    ./CarlaUE4.sh -RenderOffScreen &
    CARLA_PID=$!
    print_colored $GREEN "CARLA started (PID: $CARLA_PID)"
    
    # Wait for CARLA to be ready
    print_colored $YELLOW "Waiting for CARLA to be ready..."
    sleep 15
    
    # Start FSM Sandbox system
    print_colored $BLUE "Starting FSM Sandbox system..."
    cd $OP_BRIDGE_ROOT
    # Replace this with your actual FSM launch command
    ros2 launch your_fsm_package fsm_sandbox.launch.py &
    FSM_PID=$!
    print_colored $GREEN "FSM Sandbox started (PID: $FSM_PID)"
    
    # Wait for FSM to initialize
    sleep 10
    
    # Start Autoware
    print_colored $BLUE "Starting Autoware system..."
    cd $AUTOWARE_ROOT
    # Replace this with your actual Autoware launch command  
    ros2 launch autoware_launch planning_simulator.launch.xml &
    AUTOWARE_PID=$!
    print_colored $GREEN "Autoware started (PID: $AUTOWARE_PID)"
    
    # Wait for Autoware to be ready
    print_colored $YELLOW "Waiting for Autoware to be ready..."
    sleep 20
else
    print_colored $YELLOW "Experiments-only mode - assuming core systems are running"
fi

print_header "Starting Validation Experiments"

SESSION_TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
print_colored $GREEN "Session timestamp: $SESSION_TIMESTAMP"

# Start Experiment A1
print_colored $BLUE "Starting Experiment A1 (Timing Analysis)..."
cd "$SCRIPT_DIR/A1_sensor_latency"
python3 log_recorder_node.py --config ../config.yaml &
A1_PID=$!
print_colored $GREEN "Experiment A1 started (PID: $A1_PID)"

# Start Experiment A2
print_colored $BLUE "Starting Experiment A2 (Fidelity Analysis)..."
cd "$SCRIPT_DIR/A2_visual_location_error"
python3 ground_truth_recorder.py &
A2_PID=$!
print_colored $GREEN "Experiment A2 started (PID: $A2_PID)"

# Start Experiment B - Latency Logger
print_colored $BLUE "Starting Experiment B1 (Latency Impact Logger)..."
cd "$SCRIPT_DIR/B_latency_baseline"
python3 latency_impact_logger.py --config ../config.yaml &
B1_PID=$!
print_colored $GREEN "Experiment B1 started (PID: $B1_PID)"

# Start Experiment B - Error Analyzer
print_colored $BLUE "Starting Experiment B2 (Tracking Error Analyzer)..."
python3 tracking_error_analyzer.py --config ../config.yaml &
B2_PID=$!
print_colored $GREEN "Experiment B2 started (PID: $B2_PID)"

print_header "All Validation Experiments Active"
print_colored $GREEN "Complete validation suite is now collecting data!"
print_colored $YELLOW "Test Protocol:"
echo "  Phase 1 (0-5 min): Normal driving for general data collection"
echo "  Phase 2 (5-8 min): Position vehicle at start of straight test path"
echo "  Phase 3 (8-13 min): Drive straight at constant 0.5 m/s for baseline test"
echo "  Phase 4: Press Ctrl+C to stop all experiments"

print_colored $BLUE "\nMonitoring all processes... Press Ctrl+C to stop"

# Monitor all processes
while true; do
    # Check if any critical process died
    for pid in $A1_PID $A2_PID $B1_PID $B2_PID; do
        if ! kill -0 $pid 2>/dev/null; then
            print_colored $RED "Experiment process $pid stopped unexpectedly!"
            exit 1
        fi
    done
    
    # Show status every 30 seconds
    if [ $(($(date +%s) % 30)) -eq 0 ]; then
        print_colored $GREEN "All experiments active... (Session: $SESSION_TIMESTAMP)"
    fi
    
    sleep 5
done