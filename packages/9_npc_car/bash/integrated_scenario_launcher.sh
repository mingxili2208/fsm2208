#!/bin/bash
# integrated_scenario_launcher.sh - 启动集成测试脚本的便捷工具

# 设置颜色输出
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m' # 无颜色

# 检查Python和ROS2环境
echo -e "${BLUE}===== 检查环境 =====${NC}"

# 检查Python是否安装
if ! command -v python3 &> /dev/null; then
    echo -e "${RED}错误: 未找到Python3. 请安装Python3后再试.${NC}"
    exit 1
fi

# 获取Python版本信息
PYTHON_VERSION=$(python3 --version)
echo -e "${GREEN}已找到: $PYTHON_VERSION${NC}"

# # 检查keyboard模块是否安装
# python3 -c "import keyboard" 2>/dev/null
# if [ $? -ne 0 ]; then
#     echo -e "${YELLOW}警告: Python keyboard模块未安装. 正在尝试安装...${NC}"
#     pip3 install keyboard
#     if [ $? -ne 0 ]; then
#         echo -e "${RED}错误: 无法安装keyboard模块. 可能需要使用sudo pip3 install keyboard${NC}"
#     fi
# fi

# 在启动脚本中将检查 keyboard 模块替换为 pynput
python3 -c "import pynput" 2>/dev/null
if [ $? -ne 0 ]; then
    echo -e "${YELLOW}警告: Python pynput模块未安装. 正在尝试安装...${NC}"
    pip3 install pynput
    if [ $? -ne 0 ]; then
        echo -e "${RED}错误: 无法安装pynput模块. 可能需要使用 pip3 install pynput --user${NC}"
    fi
fi

# 创建日志目录
LOGS_DIR="logs"
if [ ! -d "$LOGS_DIR" ]; then
    echo -e "${YELLOW}创建日志目录: $LOGS_DIR${NC}"
    mkdir -p "$LOGS_DIR"
fi

# 环境变量设置
export SIMULATOR_LOCAL_HOST="localhost"
export SIMULATOR_PORT="2000"
export TRAFFIC_MANAGER_PORT="8000"
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

# 设置ROS环境
source /opt/ros/humble/setup.bash
source ${AUTOWARE_ROOT}/install/setup.bash
source ${OP_ROS_PLUGINS_ROOT}/install/setup.bash

# 生成时间戳用于日志文件名
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
LOG_FILE="$LOGS_DIR/scenario_run_$TIMESTAMP.log"

# 默认参数
HOST="127.0.0.1"
PORT=2000
TM_PORT=8100
NUM_VEHICLES=5
SEED=42
SAFE_MODE="--safe-mode"
BATCH_SPAWN="--batch-spawn"

# 可配置的时间和距离参数
WAIT_BEFORE_ENGAGE=2.0
WAIT_AT_WAYPOINT=5.0
POSITION_CHECK_INTERVAL=0.5
MAX_POSITION_WAIT_TIME=30.0
GOAL_DISTANCE_THRESHOLD=8.0
START_POINT_THRESHOLD=10.0

# 显示脚本信息
echo -e "${BLUE}===== 集成场景测试启动器 =====${NC}"
echo -e "${GREEN}启动时间: $(date)${NC}"
echo -e "${GREEN}日志文件: $LOG_FILE${NC}"
echo -e "${YELLOW}基本参数:${NC}"
echo -e "  主机: $HOST"
echo -e "  端口: $PORT"
echo -e "  TM端口: $TM_PORT"
echo -e "  NPC车辆数量: $NUM_VEHICLES"
echo -e "  随机种子: $SEED"

echo -e "${YELLOW}时间和距离参数:${NC}"
echo -e "  发送engage前等待时间: $WAIT_BEFORE_ENGAGE 秒"
echo -e "  在路点处等待时间: $WAIT_AT_WAYPOINT 秒"
echo -e "  位置检查间隔: $POSITION_CHECK_INTERVAL 秒"
echo -e "  最大位置等待时间: $MAX_POSITION_WAIT_TIME 秒"
echo -e "  目标检测距离阈值: $GOAL_DISTANCE_THRESHOLD 米"
echo -e "  起点检测距离阈值: $START_POINT_THRESHOLD 米"

# 显示键盘控制说明
echo -e "${YELLOW}键盘控制:${NC}"
echo -e "  在NPC测试期间按下 'T': 触发碰撞恢复模式（移除NPC车辆并恢复导航）"
echo -e "  在圆形循环期间按下 'R': 重新启动NPC测试"

# 确认是否继续
echo -e "${YELLOW}按回车键继续...${NC}"
read

# 显示运行信息
echo -e "${BLUE}===== 开始执行集成测试 =====${NC}"
echo -e "${YELLOW}运行集成测试脚本...${NC}"
echo -e "${YELLOW}按 Ctrl+C 停止测试${NC}"

# 运行集成测试脚本，包含新增的参数
python3 /home/cityu-fsm-lab-carla/lmx/packages/9_npc_car/scripts/test/integrated_scenario_test2.py \
    --host $HOST \
    --port $PORT \
    --tm-port $TM_PORT \
    --num-vehicles $NUM_VEHICLES \
    --seed $SEED \
    $SAFE_MODE \
    $BATCH_SPAWN \
    --log-file $LOG_FILE \
    --wait-before-engage $WAIT_BEFORE_ENGAGE \
    --wait-at-waypoint $WAIT_AT_WAYPOINT \
    --position-check-interval $POSITION_CHECK_INTERVAL \
    --max-position-wait-time $MAX_POSITION_WAIT_TIME \
    --goal-distance-threshold $GOAL_DISTANCE_THRESHOLD \
    --start-point-threshold $START_POINT_THRESHOLD

# 捕获结束状态
EXIT_STATUS=$?

echo -e "${BLUE}===== 测试执行完成 =====${NC}"
if [ $EXIT_STATUS -eq 0 ]; then
    echo -e "${GREEN}测试脚本正常结束${NC}"
else
    echo -e "${RED}测试脚本以状态码 $EXIT_STATUS 结束${NC}"
fi

echo -e "${GREEN}日志已保存到: $LOG_FILE${NC}"
echo -e "${BLUE}结束时间: $(date)${NC}"

# 显示最近日志内容
echo -e "${YELLOW}最近日志内容:${NC}"
tail -n 15 $LOG_FILE

# 根据退出状态显示适当的消息
if grep -q "NPC collision reported by admin" "$LOG_FILE"; then
    echo -e "${YELLOW}检测到管理员报告的NPC碰撞事件${NC}"
    echo -e "${GREEN}成功执行了恢复流程${NC}"
fi

if grep -q "Restart requested by admin" "$LOG_FILE"; then
    echo -e "${YELLOW}检测到管理员请求重启NPC测试${NC}"
    echo -e "${GREEN}成功执行了重启流程${NC}"
fi

echo -e "${BLUE}===== 启动器结束 =====${NC}"