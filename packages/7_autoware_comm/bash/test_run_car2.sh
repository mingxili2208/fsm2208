#!/bin/bash

# Configuration file - move settings to a separate file that can be easily modified
CONFIG_FILE="${OP_BRIDGE_ROOT}/config/launch_config.sh"
if [ -f "$CONFIG_FILE" ]; then
    source "$CONFIG_FILE"
else
    # Default configuration
    export SIMULATOR_LOCAL_HOST="localhost"
    export SIMULATOR_PORT="2000"
    export TRAFFIC_MANAGER_PORT="8000"
    export TEAM_AGENT="/home/cityu-fsm-lab-carla/lmx/packages/7_autoware_comm/scripts/test/test_ego_initializer2.py"
    export PYTHONPATH="${CARLA_ROOT}/PythonAPI/carla/":"${SCENARIO_RUNNER_ROOT}":"${OP_BRIDGE_ROOT}":${PYTHONPATH}
    export AGENT_FRAME_RATE="30"
    export REMOTE_CONNECTION="False"
    export AGENT_ROLE_NAME="pygame_adtruck"
    export AGENT_MODEL_TYPE="vehicle.dodge.charger_police"
    export OP_BRIDGE_MODE="free"
    export FREE_MAP_NAME="fsm_lab_sandbox_right_hand_driving_scene"
    export FREE_AGENT_POSE="0"
    export CONTROL_MODE="autoware"
    export RUNNING_MODE="normal"
    export ROS_DOMAIN_ID=46
fi

# Setup logging
LOG_DIR="${OP_BRIDGE_ROOT}/logs"
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/autoware_launch_$(date +%Y%m%d_%H%M%S).log"
exec &> >(tee -a "$LOG_FILE")

# Define process management
PROCESS_IDS=()

# Enhanced cleanup function
function cleanup {
    echo "$(date): Terminating all processes..."
    for pid in "${PROCESS_IDS[@]}"; do
        if ps -p $pid > /dev/null; then
            echo "Killing process $pid"
            kill $pid 2>/dev/null || kill -9 $pid 2>/dev/null
        fi
    done
    
    # Additional cleanup specific to your system if needed
    echo "$(date): Cleanup completed"
}

# Error handling function
function handle_error {
    echo "$(date): ERROR: $1"
    cleanup
    exit 1
}

# Function to launch a process and track its PID
function launch_process {
    local cmd="$1"
    local name="$2"
    
    echo "$(date): Starting $name..."
    if [ "$3" == "background" ]; then
        eval "$cmd" &
        local pid=$!
        PROCESS_IDS+=($pid)
        echo "$(date): $name started with PID $pid"
    else
        eval "$cmd"
        if [ $? -ne 0 ]; then
            handle_error "$name failed to start"
        fi
    fi
}

# Register signals for cleanup
trap cleanup EXIT SIGINT SIGTERM

# Source ROS
echo "$(date): Sourcing ROS Humble..."
source /opt/ros/humble/setup.bash || handle_error "Failed to source ROS Humble"

# Source Autoware if needed
if [ "${CONTROL_MODE}" = "autoware" ]; then
    echo "$(date): Sourcing Autoware..."
    source ${AUTOWARE_ROOT}/install/setup.bash || handle_error "Failed to source Autoware"
    
    # Source appropriate plugins based on map
    if [[ ${FREE_MAP_NAME} == *fsm_lab* ]]; then
        echo "$(date): Applying official map in FSM Lab"
        source ${OP_ROS_PLUGINS_ROOT}/install/setup.bash || handle_error "Failed to source FSM Lab plugins"
    else
        echo "$(date): Applying official map in Carla Simulator"
    fi
    
    # Launch RViz2 in background
    launch_process "ros2 run rviz2 rviz2 -d ${OP_AGENT_ROOT}/rviz/carla_autoware.rviz -s ${OP_AGENT_ROOT}/rviz/image/autoware.png" "RViz2" "background"
    
    # Launch world launcher
    launch_process "python3 ${OP_BRIDGE_ROOT}/op_bridge/fsm_lab_simulation/world_launcher.py" "World Launcher"
    
    # Wait for world to initialize
    echo "$(date): Waiting for world initialization..."
    sleep 5
    
    # Launch vehicle
    launch_process "python3 /home/cityu-fsm-lab-carla/lmx/packages/7_autoware_comm/scripts/test/test_vehicle_launcher2.py" "Vehicle Launcher"
else
    # Non-autoware launch sequence
    launch_process "python3 ${OP_BRIDGE_ROOT}/op_bridge/fsm_lab_simulation/world_launcher.py" "World Launcher"
    launch_process "python3 ${OP_BRIDGE_ROOT}/op_bridge/fsm_lab_simulation/vehicle_launcher.py" "Vehicle Launcher"
fi

# Wait for all background processes
echo "$(date): All launchers started, waiting for completion..."
wait

echo "$(date): All processes completed"
exit 0