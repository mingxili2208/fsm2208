#!/bin/bash

# 定义清理函数，在脚本退出时终止所有后台进程
function cleanup {
    echo "Terminating all background processes..."
    kill $(jobs -p)
}

# 捕获 SIGINT (Ctrl+C) 信号并调用 cleanup 函数
trap cleanup SIGINT

#############################
# Launch the Carla Ego-Vehicle
#############################

export SIMULATOR_LOCAL_HOST="localhost"
export SIMULATOR_PORT="2000"
export TRAFFIC_MANAGER_PORT="8000"
#export TEAM_AGENT=${OP_BRIDGE_ROOT}/op_bridge/fsm_lab_simulation/test_ego_vehicle_initializer.py
export TEAM_AGENT=/home/cityu-fsm-lab-carla/lmx/packages/7_autoware_comm/scripts/test/test_ego_initializer.py
export PYTHONPATH="${CARLA_ROOT}/PythonAPI/carla/":"${SCENARIO_RUNNER_ROOT}":"${OP_BRIDGE_ROOT}":${PYTHONPATH}
export AGENT_FRAME_RATE="30"
export REMOTE_CONNECTION="False"
# Autonomous actor default role_name and type
export AGENT_ROLE_NAME="pygame_adtruck" #############################################pygame-adtruck
export AGENT_MODEL_TYPE="vehicle.dodge.charger_police" #"vehicle.mitsubishi.fusorosa" #"vehicle.audi.etron" #"vehicle.dodge.charger_police" #"vehicle.mitsubishi.fusorosa" #"vehicle.tesla.cybertruck" #"vehicle.bmw.grandtourer"  #"vehicle.audi.etron" #"vehicle.dodge.charger_police" #"vehicle.carlamotors.carlacola" #"vehicle.dodge.charger_police" #"vehicle.tesla.cybertruck" #  #"vehicle.tesla.cybertruck" #"vehicle.mitsubishi.fusorosa" #"vehicle.dodge.charger_police"   #"vehicle.tesla.model3"

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
export FREE_AGENT_POSE="0"

# Set controller. ["autoware", "pygame", "follow", "teleop"]
#export CONTROL_MODE="pygame"
export CONTROL_MODE="autoware"

# Set the running mode. ["normal", "record"]
export RUNNING_MODE="normal"

source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=46
# 获取当前脚本的目录（即bash文件夹的路径）
source ${AUTOWARE_ROOT}/install/setup.bash

source ${OP_ROS_PLUGINS_ROOT}/install/setup.bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# 导航到Python脚本所在的目录
PYTHON_SCRIPT_DIR="$SCRIPT_DIR/../scripts/test"
PYTHON_SCRIPT="$PYTHON_SCRIPT_DIR/scenario_test5.py"

# Configuration file for CARLA traffic generation
CONFIG_FILE="$SCRIPT_DIR/traffic_config.json"

# Default values
VEHICLES=5
WALKERS=0
HOST="127.0.0.1"
PORT=2000
SEED=$(date +%s)
SAFE_MODE=true
BATCH_SPAWN=false

# Parse command line arguments
while getopts "v:w:h:p:c:s:b" opt; do
  case $opt in
    v) VEHICLES=$OPTARG ;;
    w) WALKERS=$OPTARG ;;
    h) HOST=$OPTARG ;;
    p) PORT=$OPTARG ;;
    c) CONFIG_PATH=$OPTARG ;;
    s) SEED=$OPTARG ;;
    b) BATCH_SPAWN=true ;;
    *) echo "Usage: $0 [-v vehicles] [-w walkers] [-h host] [-p port] [-c config_path] [-s seed] [-b batch_spawn]" >&2
       exit 1 ;;
  esac
done

# 创建配置文件 - 包含完整设置
cat > $CONFIG_FILE << EOF
{
  "number-of-vehicles": $VEHICLES,
  "number-of-walkers": $WALKERS,
  "host": "$HOST",
  "port": $PORT,
  "safe": $SAFE_MODE,
  "filterv": "vehicle.*",
  "generationv": "All",
  "filterw": "walker.pedestrian.*",
  "generationw": "2",
  "asynch": false,
  "seed": $SEED
}
EOF

echo "配置文件已创建: $CONFIG_FILE"
echo "Starting CARLA traffic simulation with $VEHICLES vehicles and $WALKERS pedestrians"
echo "Python script path: $PYTHON_SCRIPT"
echo "Host: $HOST, Port: $PORT, Seed: $SEED"

# 注意捕获Ctrl+C信号，确保清理
cleanup() {
    echo "接收到中断信号，正在清理..."
    rm -f "$CONFIG_FILE"
    # 给Python脚本时间进行清理
    sleep 1
    exit 1
}

trap cleanup SIGINT SIGTERM

# 设置批处理模式参数
BATCH_ARG=""
if [ "$BATCH_SPAWN" = true ]; then
    BATCH_ARG="--batch-spawn"
    echo "使用批量生成模式"
fi

# 运行Python脚本 - 使用配置文件或命令行参数
if [ -f "$CONFIG_FILE" ]; then
    echo "通过配置文件运行: $CONFIG_FILE"
    python3 "$PYTHON_SCRIPT" --config "$CONFIG_FILE" $BATCH_ARG
else
    echo "通过命令行参数运行"
    python3 "$PYTHON_SCRIPT" --host "$HOST" --port "$PORT" --seed "$SEED" --num-vehicles "$VEHICLES" $BATCH_ARG
fi

# 检查Python脚本退出状态
SCRIPT_STATUS=$?
if [ $SCRIPT_STATUS -ne 0 ]; then
    echo "Python脚本异常退出，状态码: $SCRIPT_STATUS"
fi

# 清理配置文件
if [ -f "$CONFIG_FILE" ]; then
    rm "$CONFIG_FILE"
    echo "已删除配置文件: $CONFIG_FILE"
fi

echo "脚本执行完成"
exit $SCRIPT_STATUS