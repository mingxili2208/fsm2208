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