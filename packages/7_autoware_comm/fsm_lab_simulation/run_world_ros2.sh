#############################
# Launch the World
# Load the map specified with "FREE_MAP_NAME"
#############################

export SIMULATOR_LOCAL_HOST="localhost"
export SIMULATOR_PORT="2000"
export PYTHONPATH="${CARLA_ROOT}/PythonAPI/carla/":"${SCENARIO_RUNNER_ROOT}":"${OP_BRIDGE_ROOT}":${PYTHONPATH}
export AGENT_FRAME_RATE="20"

# modes are
#   * "leaderboard" : when runner the leaderboard (route based) scenario collections
#   * "srunner" : when scenario is loaded using scenario runner, and only agent is attached
#   * "free" : when loading empty map only , either carla town or any OpenDRIVE map
export OP_BRIDGE_MODE="free"

# CARLA town name or custom OpenDRIVE map name, when BRIDGE_MODE is free
export FREE_MAP_NAME="Town03"

source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=46
python3 ${OP_BRIDGE_ROOT}/op_bridge/fsm_lab_simulation/world_launcher.py
