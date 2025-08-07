#!/bin/bash
# enhanced_integrated_scenario_launcher.sh - 启动增强版集成测试脚本的便捷工具
# 版本: v2.0
# 作者: James LI
# 日期: 2025-01-XX

# ============================================================================
# 脚本配置和环境检查
# ============================================================================

# 设置颜色输出
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
PURPLE='\033[0;35m'
NC='\033[0m' # 无颜色

# 脚本信息
SCRIPT_VERSION="2.0"
SCRIPT_NAME="Enhanced Integrated Scenario Test Launcher"
SCRIPT_AUTHOR="James LI"

# 显示欢迎信息
echo -e "${PURPLE}============================================================================${NC}"
echo -e "${CYAN}    ${SCRIPT_NAME} v${SCRIPT_VERSION}${NC}"
echo -e "${CYAN}    作者: ${SCRIPT_AUTHOR}${NC}"
echo -e "${CYAN}    启动时间: $(date '+%Y-%m-%d %H:%M:%S')${NC}"
echo -e "${PURPLE}============================================================================${NC}"

# 获取脚本所在目录
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

# 检查Python和ROS2环境
echo -e "${BLUE}===== 环境检查 =====${NC}"

# 检查Python是否安装
if ! command -v python3 &> /dev/null; then
    echo -e "${RED}错误: 未找到Python3. 请安装Python3后再试.${NC}"
    exit 1
fi

# 获取Python版本信息
PYTHON_VERSION=$(python3 --version)
echo -e "${GREEN}✓ Python环境: $PYTHON_VERSION${NC}"

# 检查ROS2环境
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${YELLOW}警告: 未检测到ROS2环境变量${NC}"
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        echo -e "${YELLOW}尝试加载ROS2 Humble环境...${NC}"
        source /opt/ros/humble/setup.bash
    else
        echo -e "${RED}错误: 未找到ROS2安装${NC}"
        exit 1
    fi
fi
echo -e "${GREEN}✓ ROS2环境: $ROS_DISTRO${NC}"

# 检查必要的Python模块
echo -e "${BLUE}===== Python依赖检查 =====${NC}"

# 检查pynput模块
python3 -c "import pynput" 2>/dev/null
if [ $? -ne 0 ]; then
    echo -e "${YELLOW}警告: Python pynput模块未安装. 正在尝试安装...${NC}"
    pip3 install pynput --user
    if [ $? -ne 0 ]; then
        echo -e "${RED}错误: 无法安装pynput模块${NC}"
        echo -e "${YELLOW}请手动执行: pip3 install pynput --user${NC}"
        exit 1
    fi
fi
echo -e "${GREEN}✓ pynput模块可用${NC}"

# 检查其他必要模块
REQUIRED_MODULES=("numpy" "transforms3d" "setproctitle")
for module in "${REQUIRED_MODULES[@]}"; do
    python3 -c "import $module" 2>/dev/null
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✓ $module 模块可用${NC}"
    else
        echo -e "${YELLOW}警告: $module 模块未找到，尝试安装...${NC}"
        pip3 install $module --user
        if [ $? -ne 0 ]; then
            echo -e "${RED}错误: 无法安装 $module 模块${NC}"
        fi
    fi
done

# 检查ROS2 Python模块
python3 -c "import rclpy" 2>/dev/null
if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ ROS2 Python模块可用${NC}"
else
    echo -e "${YELLOW}警告: ROS2 Python模块未找到${NC}"
fi

# ============================================================================
# 目录和日志设置
# ============================================================================

echo -e "${BLUE}===== 目录设置 =====${NC}"

# 创建必要的目录
LOGS_DIR="$PROJECT_ROOT/logs"
CONFIG_DIR="$PROJECT_ROOT/config"

for dir in "$LOGS_DIR" "$CONFIG_DIR"; do
    if [ ! -d "$dir" ]; then
        echo -e "${YELLOW}创建目录: $dir${NC}"
        mkdir -p "$dir"
    else
        echo -e "${GREEN}✓ 目录存在: $dir${NC}"
    fi
done

# ============================================================================
# 环境变量配置
# ============================================================================

echo -e "${BLUE}===== 环境变量配置 =====${NC}"

# CARLA相关环境变量
export SIMULATOR_LOCAL_HOST="${SIMULATOR_LOCAL_HOST:-localhost}"
export SIMULATOR_PORT="${SIMULATOR_PORT:-2000}"
export TRAFFIC_MANAGER_PORT="${TRAFFIC_MANAGER_PORT:-8000}"

# Autoware相关环境变量
export AGENT_FRAME_RATE="${AGENT_FRAME_RATE:-30}"
export REMOTE_CONNECTION="${REMOTE_CONNECTION:-False}"
export AGENT_ROLE_NAME="${AGENT_ROLE_NAME:-pygame_adtruck}"
export AGENT_MODEL_TYPE="${AGENT_MODEL_TYPE:-vehicle.dodge.charger_police}"

# OpenPlanner Bridge相关环境变量
export OP_BRIDGE_MODE="${OP_BRIDGE_MODE:-free}"
export FREE_MAP_NAME="${FREE_MAP_NAME:-fsm_lab_sandbox_right_hand_driving_scene}"
export FREE_AGENT_POSE="${FREE_AGENT_POSE:-0}"
export CONTROL_MODE="${CONTROL_MODE:-autoware}"
export RUNNING_MODE="${RUNNING_MODE:-normal}"

# ROS相关环境变量
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-46}"

# Python路径设置
if [ -n "$CARLA_ROOT" ]; then
    export PYTHONPATH="${CARLA_ROOT}/PythonAPI/carla/":${PYTHONPATH}
    echo -e "${GREEN}✓ CARLA Python路径已设置${NC}"
fi

if [ -n "$SCENARIO_RUNNER_ROOT" ]; then
    export PYTHONPATH="${SCENARIO_RUNNER_ROOT}":${PYTHONPATH}
    echo -e "${GREEN}✓ Scenario Runner Python路径已设置${NC}"
fi

if [ -n "$OP_BRIDGE_ROOT" ]; then
    export PYTHONPATH="${OP_BRIDGE_ROOT}":${PYTHONPATH}
    echo -e "${GREEN}✓ OpenPlanner Bridge Python路径已设置${NC}"
fi

# 设置ROS环境
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    echo -e "${GREEN}✓ ROS2基础环境已加载${NC}"
fi

if [ -n "$AUTOWARE_ROOT" ] && [ -f "${AUTOWARE_ROOT}/install/setup.bash" ]; then
    source ${AUTOWARE_ROOT}/install/setup.bash
    echo -e "${GREEN}✓ Autoware环境已加载${NC}"
fi

if [ -n "$OP_ROS_PLUGINS_ROOT" ] && [ -f "${OP_ROS_PLUGINS_ROOT}/install/setup.bash" ]; then
    source ${OP_ROS_PLUGINS_ROOT}/install/setup.bash
    echo -e "${GREEN}✓ OpenPlanner ROS插件环境已加载${NC}"
fi

# ============================================================================
# 参数配置
# ============================================================================

echo -e "${BLUE}===== 参数配置 =====${NC}"

# 生成时间戳用于日志文件名
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
LOG_FILE="$LOGS_DIR/enhanced_scenario_run_$TIMESTAMP.log"

# 默认CARLA连接参数
HOST="${HOST:-127.0.0.1}"
PORT="${PORT:-2000}"
TM_PORT="${TM_PORT:-8100}"
ALT_TM_PORT="${ALT_TM_PORT:-8200}"

# 车辆生成参数
NUM_VEHICLES="${NUM_VEHICLES:-5}"
SEED="${SEED:-42}"
SAFE_MODE_FLAG="--safe-mode"
BATCH_SPAWN_FLAG="--batch-spawn"

# 时间和距离参数
WAIT_BEFORE_ENGAGE="${WAIT_BEFORE_ENGAGE:-2.0}"
WAIT_AT_WAYPOINT="${WAIT_AT_WAYPOINT:-5.0}"
POSITION_CHECK_INTERVAL="${POSITION_CHECK_INTERVAL:-0.5}"
MAX_POSITION_WAIT_TIME="${MAX_POSITION_WAIT_TIME:-30.0}"
GOAL_DISTANCE_THRESHOLD="${GOAL_DISTANCE_THRESHOLD:-8.0}"
START_POINT_THRESHOLD="${START_POINT_THRESHOLD:-10.0}"

# 检查配置文件
CONFIG_FILE="$CONFIG_DIR/test_config.json"
if [ -f "$CONFIG_FILE" ]; then
    echo -e "${GREEN}✓ 找到配置文件: $CONFIG_FILE${NC}"
    CONFIG_ARG="--config $CONFIG_FILE"
else
    echo -e "${YELLOW}! 未找到配置文件，使用默认参数${NC}"
    CONFIG_ARG=""
fi

# ============================================================================
# 显示配置信息
# ============================================================================

echo -e "${PURPLE}===== 配置信息总览 =====${NC}"
echo -e "${CYAN}CARLA连接配置:${NC}"
echo -e "  主机地址: ${HOST}"
echo -e "  端口: ${PORT}"
echo -e "  Traffic Manager端口: ${TM_PORT}"
echo -e "  备用TM端口: ${ALT_TM_PORT}"

echo -e "${CYAN}车辆配置:${NC}"
echo -e "  NPC车辆数量: ${NUM_VEHICLES}"
echo -e "  随机种子: ${SEED}"
echo -e "  安全模式: 启用"
echo -e "  批量生成: 启用"

echo -e "${CYAN}导航参数:${NC}"
echo -e "  engage前等待时间: ${WAIT_BEFORE_ENGAGE} 秒"
echo -e "  路点等待时间: ${WAIT_AT_WAYPOINT} 秒"
echo -e "  位置检查间隔: ${POSITION_CHECK_INTERVAL} 秒"
echo -e "  最大等待时间: ${MAX_POSITION_WAIT_TIME} 秒"
echo -e "  目标距离阈值: ${GOAL_DISTANCE_THRESHOLD} 米"
echo -e "  起点距离阈值: ${START_POINT_THRESHOLD} 米"

echo -e "${CYAN}日志配置:${NC}"
echo -e "  日志文件: ${LOG_FILE}"

# ============================================================================
# 键盘控制说明
# ============================================================================

echo -e "${PURPLE}===== 键盘控制说明 =====${NC}"
echo -e "${YELLOW}测试期间可用的键盘控制:${NC}"
echo -e "  ${GREEN}T键${NC}: 触发NPC碰撞恢复模式（立即移除NPC车辆并恢复导航）"
echo -e "  ${GREEN}R键${NC}: 重新启动NPC测试序列"
echo -e "  ${GREEN}P键${NC}: 暂停当前回环测试"
echo -e "  ${GREEN}S键${NC}: 停止/退出当前回环测试"
echo -e "  ${GREEN}Ctrl+C${NC}: 紧急停止整个程序"

# ============================================================================
# 功能模块说明
# ============================================================================

echo -e "${PURPLE}===== 功能模块说明 =====${NC}"
echo -e "${YELLOW}主要测试模式:${NC}"
echo -e "  ${GREEN}1${NC}. 到达任务起点 - 导航到NPC测试起始位置"
echo -e "  ${GREEN}2${NC}. NPC测试 - 完整的NPC环境导航测试"
echo -e "  ${GREEN}3${NC}. 回环测试 - 单次循环路径测试"
echo -e "  ${GREEN}4${NC}. NPC导航(无车) - 执行NPC路径但不生成车辆"
echo -e "  ${GREEN}5${NC}. 点对点导航 - 直接导航到指定目标点"
echo -e "  ${GREEN}6${NC}. 3次回环 - 执行3次完整回环测试"
echo -e "  ${GREEN}7${NC}. N次回环 - 执行指定次数或无限回环测试"
echo -e "  ${GREEN}8${NC}. 导航到回环起点 - 从任意位置导航到回环测试起点"
echo -e "  ${GREEN}9${NC}. 导航到NPC终点 - 导航到NPC测试的最终位置"

# ============================================================================
# 安全检查和确认
# ============================================================================

echo -e "${PURPLE}===== 安全检查 =====${NC}"

# 检查CARLA服务器连接
echo -e "${YELLOW}正在检查CARLA服务器连接...${NC}"
python3 -c "
import carla
import sys
try:
    client = carla.Client('$HOST', $PORT)
    client.set_timeout(5.0)
    world = client.get_world()
    print('✓ CARLA服务器连接成功')
    print(f'  地图: {world.get_map().name}')
    print(f'  同步模式: {world.get_settings().synchronous_mode}')
except Exception as e:
    print(f'✗ CARLA服务器连接失败: {e}')
    sys.exit(1)
" 2>/dev/null
if [ $? -eq 0 ]; then
    echo -e "${GREEN}CARLA连接验证通过${NC}"
else
    echo -e "${RED}错误: 无法连接到CARLA服务器 ($HOST:$PORT)${NC}"
    echo -e "${YELLOW}请确保:${NC}"
    echo -e "  1. CARLA服务器已启动"
    echo -e "  2. 服务器地址和端口正确"
    echo -e "  3. 网络连接正常"
    
    read -p "是否忽略此错误并继续? (y/N): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo -e "${RED}用户取消执行${NC}"
        exit 1
    fi
fi

# 最终确认
echo -e "${PURPLE}===== 启动确认 =====${NC}"
echo -e "${YELLOW}即将启动增强版自动驾驶测试系统${NC}"
echo -e "${YELLOW}所有参数已配置完成，环境检查通过${NC}"
echo
read -p "按回车键开始测试，或按Ctrl+C取消: "

# ============================================================================
# 启动主程序
# ============================================================================

echo -e "${PURPLE}============================================================================${NC}"
echo -e "${CYAN}    开始执行增强版集成测试${NC}"
echo -e "${CYAN}    测试脚本正在启动...${NC}"
echo -e "${PURPLE}============================================================================${NC}"

# 构建Python脚本路径
PYTHON_SCRIPT="$SCRIPT_DIR/enhanced_integrated_scenario_test.py"

# 检查Python脚本是否存在
if [ ! -f "$PYTHON_SCRIPT" ]; then
    echo -e "${RED}错误: 找不到Python脚本文件: $PYTHON_SCRIPT${NC}"
    exit 1
fi

# 运行增强版集成测试脚本
python3 "$PYTHON_SCRIPT" \
    --host "$HOST" \
    --port "$PORT" \
    --tm-port "$TM_PORT" \
    --alt-tm-port "$ALT_TM_PORT" \
    --num-vehicles "$NUM_VEHICLES" \
    --seed "$SEED" \
    $SAFE_MODE_FLAG \
    $BATCH_SPAWN_FLAG \
    --log-file "$LOG_FILE" \
    --wait-before-engage "$WAIT_BEFORE_ENGAGE" \
    --wait-at-waypoint "$WAIT_AT_WAYPOINT" \
    --position-check-interval "$POSITION_CHECK_INTERVAL" \
    --max-position-wait-time "$MAX_POSITION_WAIT_TIME" \
    --goal-distance-threshold "$GOAL_DISTANCE_THRESHOLD" \
    --start-point-threshold "$START_POINT_THRESHOLD" \
    $CONFIG_ARG

# 捕获程序退出状态
EXIT_STATUS=$?

# ============================================================================
# 结果报告
# ============================================================================

echo -e "${PURPLE}============================================================================${NC}"
echo -e "${CYAN}    测试执行完成${NC}"
echo -e "${PURPLE}============================================================================${NC}"

if [ $EXIT_STATUS -eq 0 ]; then
    echo -e "${GREEN}✓ 测试脚本正常结束${NC}"
    echo -e "${GREEN}  退出状态码: $EXIT_STATUS${NC}"
else
    echo -e "${RED}✗ 测试脚本异常结束${NC}"
    echo -e "${RED}  退出状态码: $EXIT_STATUS${NC}"
fi

echo -e "${CYAN}执行信息:${NC}"
echo -e "  开始时间: $(date '+%Y-%m-%d %H:%M:%S')"
echo -e "  结束时间: $(date '+%Y-%m-%d %H:%M:%S')"
echo -e "  日志文件: ${LOG_FILE}"

# 显示日志摘要
if [ -f "$LOG_FILE" ]; then
    echo -e "${CYAN}日志摘要 (最后15行):${NC}"
    echo -e "${YELLOW}$(tail -n 15 "$LOG_FILE")${NC}"
    
    # 统计日志中的关键信息
    echo -e "${CYAN}测试统计:${NC}"
    if grep -q "NPC collision reported by admin" "$LOG_FILE"; then
        echo -e "  ${YELLOW}! 检测到管理员报告的NPC碰撞事件${NC}"
    fi
    
    if grep -q "Restart requested by admin" "$LOG_FILE"; then
        echo -e "  ${YELLOW}! 检测到管理员请求重启NPC测试${NC}"
    fi
    
    if grep -q "回环测试完成" "$LOG_FILE"; then
        LOOP_COUNT=$(grep -c "回环测试完成" "$LOG_FILE")
        echo -e "  ${GREEN}✓ 完成 $LOOP_COUNT 次回环测试${NC}"
    fi
    
    if grep -q "NPC测试完成" "$LOG_FILE"; then
        echo -e "  ${GREEN}✓ NPC测试成功完成${NC}"
    fi
else
    echo -e "${RED}警告: 未找到日志文件${NC}"
fi

# 清理和建议
echo -e "${CYAN}后续建议:${NC}"
echo -e "  1. 查看完整日志: ${LOG_FILE}"
echo -e "  2. 如需重新测试，可直接重新运行此脚本"
echo -e "  3. 如遇问题，请检查CARLA和Autoware服务状态"

echo -e "${PURPLE}============================================================================${NC}"
echo -e "${CYAN}    感谢使用增强版自动驾驶测试系统${NC}"
echo -e "${CYAN}    脚本执行完毕${NC}"
echo -e "${PURPLE}============================================================================${NC}"

exit $EXIT_STATUS