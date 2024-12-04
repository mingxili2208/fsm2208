export SIMULATOR_LOCAL_HOST="localhost"
export SIMULATOR_PORT="2000"
export TRAFFIC_MANAGER_PORT="8000"
export TEAM_AGENT=${OP_BRIDGE_ROOT}/op_bridge/fsm_lab_simulation/ego_vehicle_initializer.py
export PYTHONPATH="${CARLA_ROOT}/PythonAPI/carla/":"${SCENARIO_RUNNER_ROOT}":"${OP_BRIDGE_ROOT}":${PYTHONPATH}
export AGENT_FRAME_RATE="20"
# export REMOTE_CONNECTION="False"
# Autonomous actor default role_name and type
export NPC_ROLE_NAME="vis_npc"
export NPC_MODEL_TYPE="vehicle.tesla.model3"

# modes are 
#   * "leaderboard" : when runner the leaderboard (route based) scenario collections 
#   * "srunner" : when scenario is loaded using scenario runner, and only agent is attached
#   * "free" : when loading empty map only , either carla town or any OpenDRIVE map 
export OP_BRIDGE_MODE="free"

# CARLA town name or custom OpenDRIVE absolute path, when BRIDGE_MODE is free 
export FREE_MAP_NAME="fsm_lab_sandbox_right_hand_driving_scene"

# Spawn point for the autonomous agent, when BRIDGE_MODE is free 
# "x,y,z,roll,pitch,yaw"
# Empty string means random starting position
# export FREE_AGENT_POSE="175.4,195.14,0,0,0,180" 
# export FREE_AGENT_POSE="88.6,-226,0,0,0,0" 
# export FREE_AGENT_POSE="0"

# # Set controller. ["autoware", "pygame", "follow", "teleop"]
# #export CONTROL_MODE="pygame"
# export CONTROL_MODE="autoware" 

# # Set the running mode. ["normal", "record"]
# export RUNNING_MODE="normal"

source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=46

source ${AUTOWARE_ROOT}/install/setup.bash

source ${OP_ROS_PLUGINS_ROOT}/install/setup.bash