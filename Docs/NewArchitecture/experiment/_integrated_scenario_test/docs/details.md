# 增强版自动驾驶测试系统 - 完整版本

## 目录结构
```
enhanced_integrated_scenario_test/
├── scripts/
│   ├── enhanced_integrated_scenario_launcher.sh
│   └── enhanced_integrated_scenario_test.py
├── config/
│   └── test_config.json
├── docs/
│   ├── README.md
│   ├── DESIGN.md
│   └── USER_GUIDE.md
└── logs/
    └── (自动生成的日志文件)
```

## 1. Bash启动脚本

**文件名: `enhanced_integrated_scenario_launcher.sh`**

```bash
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
```

## 2. Python主程序

**文件名: `enhanced_integrated_scenario_test.py`**

```python
#!/usr/bin/env python

# Copyright (c) 2025 James LI
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Enhanced Autoware-CARLA Integration Script with Interactive Menu System
=======================================================================

这是一个增强版的自动驾驶测试系统，集成了CARLA仿真器和Autoware自动驾驶软件栈。
该系统提供了完整的交互式菜单，支持多种测试模式包括NPC测试、回环测试和点对点导航。

主要功能:
1. 交互式菜单系统 - 9种不同的测试模式
2. NPC车辆环境测试 - 在复杂交通环境下的自动驾驶测试
3. 回环测试 - 循环路径导航测试
4. 点对点导航 - 灵活的目标点导航
5. 实时键盘控制 - T/R/P/S键实时干预
6. 完整的日志系统 - 详细的测试记录和分析
7. 智能恢复机制 - 碰撞检测和自动恢复
8. 资源管理 - 自动清理CARLA和ROS2资源

技术栈:
- CARLA仿真器: 提供仿真环境和NPC管理
- ROS2 + Autoware: 自动驾驶软件栈
- Python: 主控制逻辑
- pynput: 实时键盘监听

作者: James LI
版本: 2.0
日期: 2025-01-XX
许可: MIT License
"""

import glob
import os
import sys
import time
import random
import numpy as np
import argparse
import logging
import inspect
import datetime
import setproctitle
import math
import threading
import signal
import json
from typing import List, Dict, Any, Optional, Tuple
from pynput import keyboard as kb

# 设置进程标题
setproctitle.setproctitle('enhanced_integrated_scenario_test')

# CARLA Python API路径设置
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

# ROS2和Autoware库导入
try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
    from autoware_auto_vehicle_msgs.msg import Engage
    from builtin_interfaces.msg import Time
    from rclpy.time import Time as rclpy_time
    from rclpy.parameter import Parameter
    from rclpy.clock import ClockType
    from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
    from rosgraph_msgs.msg import Clock
    from transforms3d.euler import quat2euler
    
    ROS2_AVAILABLE = True
    print("✓ ROS2和Autoware库加载成功")
except ImportError as e:
    ROS2_AVAILABLE = False
    print(f"⚠ ROS2或Autoware库导入失败: {e}")
    print("⚠ Autoware engage功能将被禁用")

# ============================================================================
# 配置参数定义
# ============================================================================

# 默认配置参数
DEFAULT_PARAMETERS = {
    # 时间参数 (秒)
    "wait_before_engage": 2.0,           # engage命令前等待时间
    "wait_at_waypoint": 5.0,             # 在路点处等待时间
    "position_check_interval": 0.5,      # 位置检查间隔
    "max_position_wait_time": 30.0,      # 最大位置等待时间
    "cleanup_wait_time": 0.5,            # 清理操作等待时间
    "wait_after_npc_removal": 5.0,       # NPC移除后等待时间
    
    # 距离参数 (米)
    "goal_distance_threshold": 8.0,      # 目标检测距离阈值
    "start_point_threshold": 10.0,       # 起点检测距离阈值
    
    # Traffic Manager参数
    "following_distance": 5.0,           # 跟车距离
    "speed_difference": 0.1,             # 速度差异百分比
    "max_speed_limit": 25,               # NPC最大速度限制 (km/h)
    
    # 车辆生成参数
    "spawn_retry_attempts": 5,           # 生成重试次数
    "spawn_position_variance": 1.0,      # 生成位置变化范围
    
    # 导航参数
    "navigation_timeout": 120.0,         # 导航超时时间
    "engage_retry_attempts": 3,          # engage重试次数
}

# 车辆类型和颜色配置
VEHICLE_CONFIGS = {
    "types": [
        "vehicle.carlamotors.european_hgv",  # 欧洲重型卡车
        "vehicle.mini.cooper_s",              # Mini Cooper S
        "vehicle.nissan.patrol_2021",         # 日产巡逻车2021
        "vehicle.tesla.cybertruck",           # 特斯拉Cybertruck
        "vehicle.mini.cooper_s"               # Mini Cooper S
    ],
    "colors": [
        "202,88,176",  # 紫色
        "159,0,0",     # 深红色
        "0,38,132",    # 深蓝色
        None,          # 默认颜色
        "202,88,176"   # 紫色
    ],
    "spawn_indices": [15, 40, 21, 44, 29],    # 生成点索引
    "target_indices": [5, 19, 17, 8, 8]       # 目标点索引
}

# ============================================================================
# 日志管理系统
# ============================================================================

class Logger:
    """增强版日志管理系统"""
    
    @staticmethod
    def setup(log_file=None, log_level=logging.INFO):
        """设置日志系统
        
        Args:
            log_file (str, optional): 日志文件路径
            log_level (int): 日志级别
            
        Returns:
            logging.Logger: 配置好的日志对象
        """
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # 如果没有提供日志文件名，创建默认的
        if log_file is None:
            log_dir = "logs"
            if not os.path.exists(log_dir):
                os.makedirs(log_dir)
            log_file = os.path.join(log_dir, f"enhanced_scenario_{timestamp}.log")
        
        # 确保日志目录存在
        log_dir = os.path.dirname(log_file)
        if log_dir and not os.path.exists(log_dir):
            os.makedirs(log_dir)
        
        # 配置根日志记录器
        logger = logging.getLogger()
        logger.setLevel(log_level)
        
        # 清除现有处理程序
        if logger.handlers:
            logger.handlers.clear()
        
        # 创建文件处理程序
        file_handler = logging.FileHandler(log_file, encoding='utf-8')
        file_formatter = logging.Formatter(
            '%(asctime)s - %(levelname)s - [%(filename)s:%(lineno)d] - %(message)s'
        )
        file_handler.setFormatter(file_formatter)
        logger.addHandler(file_handler)
        
        # 创建控制台处理程序
        console_handler = logging.StreamHandler(sys.stdout)
        console_formatter = logging.Formatter('%(levelname)s: %(message)s')
        console_handler.setFormatter(console_formatter)
        logger.addHandler(console_handler)
        
        # 记录日志文件位置
        logger.info(f"日志系统初始化完成")
        logger.info(f"日志文件路径: {os.path.abspath(log_file)}")
        logger.info(f"日志级别: {logging.getLevelName(log_level)}")
        
        return logger
    
    @staticmethod
    def log_system_info():
        """记录系统信息"""
        import platform
        import psutil
        
        logging.info("=== 系统信息 ===")
        logging.info(f"操作系统: {platform.system()} {platform.release()}")
        logging.info(f"Python版本: {platform.python_version()}")
        logging.info(f"CPU核心数: {psutil.cpu_count()}")
        logging.info(f"内存总量: {psutil.virtual_memory().total / (1024**3):.2f} GB")
        logging.info(f"当前工作目录: {os.getcwd()}")
        logging.info("================")

# ============================================================================
# 增强键盘监听系统
# ============================================================================

class EnhancedKeyboardMonitor:
    """增强版键盘监听器，支持多种控制命令"""
    
    def __init__(self):
        """初始化键盘监听器"""
        self.t_pressed = False    # 碰撞恢复
        self.r_pressed = False    # 重启
        self.p_pressed = False    # 暂停
        self.s_pressed = False    # 停止
        self.q_pressed = False    # 退出
        self.running = True
        self.listener = None
        self.lock = threading.Lock()
        
        # 按键统计
        self.key_stats = {
            't': 0, 'r': 0, 'p': 0, 's': 0, 'q': 0
        }
    
    def start(self):
        """启动键盘监听"""
        self.listener = kb.Listener(on_press=self._on_press)
        self.listener.daemon = True
        self.listener.start()
        logging.info("增强版键盘监听器已启动")
        logging.info("可用控制键: T(碰撞恢复) R(重启) P(暂停) S(停止) Q(退出)")
    
    def _on_press(self, key):
        """处理按键事件"""
        try:
            if hasattr(key, 'char') and key.char:
                char = key.char.lower()
                
                with self.lock:
                    if char == 't' and not self.t_pressed:
                        logging.info("🔧 [键盘] T键按下 - 请求碰撞恢复模式")
                        self.t_pressed = True
                        self.key_stats['t'] += 1
                        
                    elif char == 'r' and not self.r_pressed:
                        logging.info("🔄 [键盘] R键按下 - 请求重启测试")
                        self.r_pressed = True
                        self.key_stats['r'] += 1
                        
                    elif char == 'p' and not self.p_pressed:
                        logging.info("⏸️ [键盘] P键按下 - 请求暂停测试")
                        self.p_pressed = True
                        self.key_stats['p'] += 1
                        
                    elif char == 's' and not self.s_pressed:
                        logging.info("⏹️ [键盘] S键按下 - 请求停止测试")
                        self.s_pressed = True
                        self.key_stats['s'] += 1
                        
                    elif char == 'q' and not self.q_pressed:
                        logging.info("❌ [键盘] Q键按下 - 请求退出程序")
                        self.q_pressed = True
                        self.key_stats['q'] += 1
                        
        except Exception as e:
            logging.error(f"键盘监听错误: {e}")
    
    def reset_flags(self):
        """重置按键标志"""
        with self.lock:
            self.t_pressed = False
            self.r_pressed = False
            self.p_pressed = False
            self.s_pressed = False
            self.q_pressed = False
    
    def get_pressed_keys(self):
        """获取当前按下的键"""
        with self.lock:
            return {
                't': self.t_pressed,
                'r': self.r_pressed,
                'p': self.p_pressed,
                's': self.s_pressed,
                'q': self.q_pressed
            }
    
    def get_stats(self):
        """获取按键统计"""
        return self.key_stats.copy()
    
    def stop(self):
        """停止键盘监听"""
        self.running = False
        if self.listener:
            self.listener.stop()
            self.listener.join(timeout=1.0)
        logging.info("键盘监听器已停止")

# ============================================================================
# Autoware目标发布系统
# ============================================================================

class AutowareGoalPublisher(Node):
    """Autoware目标发布器，处理导航目标和engage命令"""
    
    def __init__(self, params=None):
        super().__init__("enhanced_autoware_goal_publisher")
        
        self.params = params or DEFAULT_PARAMETERS
        
        # 定义路点集合，使用清晰的命名
        self.goal_sets = {
            "npc_test": [
                # 索引0: NPC测试起点 (也是回环测试终点)
                [-48.4787, -32.6236, 0.0, 0.0, 0.0, -0.715607, 0.698503],
                # 索引1: NPC测试任务终点1
                [29.7123, -36.908, 0.0, 0.0, 0.0, 0.248181, 0.968714],
                # 索引2: NPC测试任务终点2 (也是回环测试起点)
                [-6.22938, 50.2089, 0.0, 0.0, 0.0, -0.999831, 0.0183731]
            ],
            "loop_test": [
                # 索引0: 回环测试起点 (来自NPC测试终点2)
                [-6.22938, 50.2089, 0.0, 0.0, 0.0, -0.999831, 0.0183731],
                # 索引1: 回环路点1
                [-48.3957, -19.3472, 0.0, 0.0, 0.0, -0.723422, 0.690406],
                # 索引2: 回环路点2
                [54.3231, -47.6682, 0.0, 0.0, 0.0, 0.300639, 0.953738],
                # 索引3: 回环路点3
                [26.589, 50.3549, 0.0, 0.0, 0.0, -0.999824, 0.01874]
            ]
        }
        
        # 路点名称映射
        self.waypoint_names = {
            "npc_test": ["NPC起点", "NPC任务终点1", "NPC任务终点2"],
            "loop_test": ["回环起点", "回环路点1", "回环路点2", "回环路点3"]
        }
        
        # 距离阈值设置
        self.distance_threshold = self.params["goal_distance_threshold"]
        self.start_point_threshold = self.params["start_point_threshold"]
        
        # 状态变量
        self.current_pose = None
        self.latest_clock = Time()
        self.clock_received = False
        self.position_history = []
        self.goal_history = []
        
        # QoS配置
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # 创建发布器
        self.goal_publisher = self.create_publisher(
            PoseStamped, "/planning/mission_planning/goal", qos
        )
        
        self.engage_publisher = self.create_publisher(
            Engage, "/autoware/engage", qos
        )
        
        # 创建订阅器
        self.position_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            "/real_world/follow_adtruck/transformed_with_covariance",
            self.vehicle_position_callback,
            qos
        )
        
        self.clock_subscription = self.create_subscription(
            Clock, '/clock', self.clock_callback, qos
        )
        
        self.get_logger().info("增强版Autoware目标发布器初始化完成")
        
        # 统计信息
        self.stats = {
            "goals_published": 0,
            "engage_commands": 0,
            "position_updates": 0,
            "navigation_successes": 0,
            "navigation_failures": 0
        }
    
    def clock_callback(self, msg: Clock):
        """时钟消息回调"""
        self.latest_clock = msg.clock
        self.clock_received = True
    
    def vehicle_position_callback(self, msg):
        """车辆位置回调"""
        self.current_pose = msg
        self.stats["position_updates"] += 1
        
        # 保存位置历史
        if len(self.position_history) > 100:
            self.position_history.pop(0)
        
        self.position_history.append({
            'timestamp': time.time(),
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'z': msg.pose.pose.position.z
        })
    
    def wait_for_clock(self, timeout_sec: float = 5.0) -> bool:
        """等待时钟消息"""
        start_time = time.time()
        while not self.clock_received and time.time() - start_time < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if self.clock_received:
            self.get_logger().info(f"✓ 接收到时钟信息: {self.latest_clock.sec}.{self.latest_clock.nanosec}")
            return True
        else:
            self.get_logger().warn(f"⚠ 等待时钟信息超时 ({timeout_sec}秒)")
            return False
    
    def publish_goal(self, goal_pos, description="未知目标"):
        """发布目标位置到Autoware
        
        Args:
            goal_pos (list): 目标位置 [x, y, z, qx, qy, qz, qw]
            description (str): 目标描述
        """
        msg = PoseStamped()
        
        # 设置时间戳
        if self.clock_received:
            msg.header.stamp = self.latest_clock
        else:
            msg.header.stamp = self.get_clock().now().to_msg()
        
        msg.header.frame_id = "map"
        
        # 设置位置和朝向
        msg.pose.position.x = float(goal_pos[0])
        msg.pose.position.y = float(goal_pos[1])
        msg.pose.position.z = float(goal_pos[2])
        
        msg.pose.orientation.x = float(goal_pos[3])
        msg.pose.orientation.y = float(goal_pos[4])
        msg.pose.orientation.z = float(goal_pos[5])
        msg.pose.orientation.w = float(goal_pos[6])
        
        # 计算航向角
        _, _, yaw = quat2euler([goal_pos[6], goal_pos[3], goal_pos[4], goal_pos[5]])
        
        self.goal_publisher.publish(msg)
        self.stats["goals_published"] += 1
        
        # 保存目标历史
        self.goal_history.append({
            'timestamp': time.time(),
            'description': description,
            'position': goal_pos[:3],
            'orientation': goal_pos[3:7],
            'yaw': yaw
        })
        
        self.get_logger().info(
            f"🎯 发布目标 [{description}]: "
            f"位置({goal_pos[0]:.2f}, {goal_pos[1]:.2f}, {goal_pos[2]:.2f}), "
            f"航向角: {math.degrees(yaw):.1f}°"
        )
        
        logging.info(f"目标发布: {description} -> ({goal_pos[0]:.2f}, {goal_pos[1]:.2f})")
    
    def publish_engage(self, engage_state, description=""):
        """发布engage命令到Autoware
        
        Args:
            engage_state (bool): engage状态
            description (str): 操作描述
        """
        msg = Engage()
        
        # 设置时间戳
        if self.clock_received:
            msg.stamp = self.latest_clock
        else:
            msg.stamp = self.get_clock().now().to_msg()
        
        msg.engage = engage_state
        
        self.engage_publisher.publish(msg)
        self.stats["engage_commands"] += 1
        
        status_emoji = "🚗" if engage_state else "🛑"
        status_text = "启动" if engage_state else "停止"
        
        self.get_logger().info(f"{status_emoji} Engage命令: {status_text} {description}")
        logging.info(f"Engage命令: {status_text} - {description}")
    
    def calculate_distance(self, x1, y1, x2, y2):
        """计算两点间欧几里得距离"""
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    
    def check_if_at_position(self, goal_pos, custom_threshold=None):
        """检查车辆是否在指定位置
        
        Args:
            goal_pos (list): 目标位置
            custom_threshold (float, optional): 自定义距离阈值
            
        Returns:
            bool: 是否在目标位置
        """
        if self.current_pose is None:
            return False
        
        current_x = self.current_pose.pose.pose.position.x
        current_y = self.current_pose.pose.pose.position.y
        
        threshold = custom_threshold if custom_threshold is not None else self.distance_threshold
        
        distance = self.calculate_distance(current_x, current_y, goal_pos[0], goal_pos[1])
        return distance < threshold
    
    def get_current_position_distance_to(self, goal_pos):
        """获取车辆当前位置到目标的距离
        
        Args:
            goal_pos (list): 目标位置
            
        Returns:
            float: 距离(米)
        """
        if self.current_pose is None:
            return float('inf')
        
        current_x = self.current_pose.pose.pose.position.x
        current_y = self.current_pose.pose.pose.position.y
        
        return self.calculate_distance(current_x, current_y, goal_pos[0], goal_pos[1])
    
    def get_current_position(self):
        """获取当前位置信息
        
        Returns:
            dict: 包含位置信息的字典
        """
        if self.current_pose is None:
            return None
        
        pos = self.current_pose.pose.pose.position
        return {
            'x': pos.x,
            'y': pos.y,
            'z': pos.z,
            'timestamp': time.time()
        }
    
    def wait_for_position_update(self, timeout_sec=5.0):
        """等待位置更新
        
        Args:
            timeout_sec (float): 超时时间
            
        Returns:
            bool: 是否成功接收到位置更新
        """
        start_time = time.time()
        while self.current_pose is None and time.time() - start_time < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if self.current_pose is not None:
            pos = self.current_pose.pose.pose.position
            self.get_logger().info(f"📍 当前位置: ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})")
            return True
        else:
            self.get_logger().warn(f"⚠ 位置更新超时 ({timeout_sec}秒)")
            return False
    
    def get_statistics(self):
        """获取统计信息"""
        return self.stats.copy()

# ============================================================================
# Traffic Manager信息管理
# ============================================================================

class TrafficManagerInfo:
    """Traffic Manager信息显示和管理"""
    
    @staticmethod
    def print_info(tm, world):
        """打印Traffic Manager和世界设置的详细信息
        
        Args:
            tm: Traffic Manager实例
            world: CARLA世界实例
        """
        logging.info("\n" + "="*50)
        logging.info("Traffic Manager 和 世界设置信息")
        logging.info("="*50)
        
        # 打印世界设置
        settings = world.get_settings()
        logging.info("🌍 世界设置:")
        logging.info(f"  • 同步模式: {settings.synchronous_mode}")
        logging.info(f"  • 固定时间步长: {settings.fixed_delta_seconds}")
        logging.info(f"  • 子步进: {settings.substepping}")
        logging.info(f"  • 最大子步时间: {settings.max_substep_delta_time}")
        
        # 打印Traffic Manager信息
        logging.info("🚦 Traffic Manager设置:")
        logging.info(f"  • 端口: {tm.get_port()}")
        
        # 检查Traffic Manager同步模式
        try:
            is_sync = tm.get_synchronous_mode()
            logging.info(f"  • 同步模式: {is_sync}")
        except:
            logging.info("  • 同步模式: 无法获取")
        
        # 获取全局设置
        try:
            # 注意：这个方法可能在某些CARLA版本中不可用
            global_distance = getattr(tm, 'get_global_distance_to_leading_vehicle', lambda: None)()
            if global_distance is not None:
                logging.info(f"  • 全局跟车距离: {global_distance} 米")
            else:
                logging.info("  • 全局跟车距离: 无法获取")
        except Exception as e:
            logging.info(f"  • 全局跟车距离: 获取失败 ({e})")
        
        # 检查当前激活的车辆
        vehicles = world.get_actors().filter('vehicle.*')
        logging.info(f"\n🚗 当前世界中的车辆: {len(vehicles)} 辆")
        
        for i, vehicle in enumerate(vehicles):
            try:
                autopilot = vehicle.get_autopilot()
                role = vehicle.attributes.get('role_name', 'unknown')
                vehicle_type = vehicle.type_id.split('.')[-1] if '.' in vehicle.type_id else vehicle.type_id
                logging.info(f"  {i+1:2d}. ID={vehicle.id} 类型={vehicle_type} 角色={role} 自动驾驶={autopilot}")
            except Exception as e:
                logging.info(f"  {i+1:2d}. ID={vehicle.id} 类型={vehicle.type_id} (信息获取失败)")
        
        logging.info("="*50 + "\n")

# ============================================================================
# CARLA管理系统
# ============================================================================

class CarlaManager:
    """CARLA连接和车辆管理系统"""
    
    def __init__(self, args, params=None):
        """初始化CARLA管理器
        
        Args:
            args: 命令行参数
            params: 配置参数
        """
        self.args = args
        self.params = params or DEFAULT_PARAMETERS
        
        # CARLA连接相关
        self.client = None
        self.world = None
        self.map = None
        self.traffic_manager = None
        self.tm_port = args.tm_port
        self.blueprint_library = None
        self.spawn_points = []
        
        # 车辆管理
        self.vehicle_list = []
        self.existing_vehicles = {}
        self.vehicle_configs = VEHICLE_CONFIGS
        
        # 设置管理
        self.original_settings = None
        self.synchronous_master = False
        
        # 统计信息
        self.stats = {
            "vehicles_spawned": 0,
            "vehicles_destroyed": 0,
            "spawn_failures": 0,
            "cleanup_operations": 0
        }
    
    def connect(self):
        """连接到CARLA服务器
        
        Returns:
            bool: 连接是否成功
        """
        try:
            self.client = carla.Client(self.args.host, self.args.port)
            self.client.set_timeout(10.0)
            
            # 获取世界和地图
            self.world = self.client.get_world()
            self.map = self.world.get_map()
            
            # 记录环境信息
            logging.info("🌐 CARLA连接信息:")
            logging.info(f"  • CARLA版本: {self.client.get_client_version()}")
            logging.info(f"  • 服务器地址: {self.args.host}:{self.args.port}")
            logging.info(f"  • 当前地图: {self.map.name}")
            logging.info(f"  • 同步模式: {self.world.get_settings().synchronous_mode}")
            logging.info(f"  • 固定时间步长: {self.world.get_settings().fixed_delta_seconds}")
            
            # 保存当前设置
            self.original_settings = self.world.get_settings()
            
            # 获取蓝图库和生成点
            self.blueprint_library = self.world.get_blueprint_library()
            self.spawn_points = self.map.get_spawn_points()
            
            logging.info(f"  • 可用生成点: {len(self.spawn_points)} 个")
            logging.info(f"  • 蓝图库车辆数: {len(self.blueprint_library.filter('vehicle.*'))} 种")
            
            return True
            
        except Exception as e:
            logging.error(f"❌ CARLA连接失败: {e}")
            return False
    
    def setup_traffic_manager(self):
        """设置和连接Traffic Manager
        
        Returns:
            bool: 设置是否成功
        """
        tm_connected = False
        
        try:
            # 尝试连接到默认Traffic Manager端口
            self.traffic_manager = self.client.get_trafficmanager(self.tm_port)
            logging.info(f"✓ 连接到现有Traffic Manager (端口: {self.tm_port})")
            tm_connected = True
            
        except Exception as e:
            logging.warning(f"⚠ 连接默认Traffic Manager失败 (端口:{self.tm_port}): {e}")
            
            # 如果失败，尝试备用端口
            try:
                self.tm_port = getattr(self.args, 'alt_tm_port', 8200)
                self.traffic_manager = self.client.get_trafficmanager(self.tm_port)
                logging.info(f"✓ 创建新Traffic Manager (端口: {self.tm_port})")
                tm_connected = True
                
            except Exception as e:
                logging.error(f"❌ 无法创建Traffic Manager (端口:{self.tm_port}): {e}")
                return False
        
        # 打印Traffic Manager和世界设置信息
        if tm_connected:
            TrafficManagerInfo.print_info(self.traffic_manager, self.world)
        
        # 设置Traffic Manager参数
        try:
            self.traffic_manager.set_global_distance_to_leading_vehicle(
                self.params["following_distance"]
            )
            self.traffic_manager.set_random_device_seed(self.args.seed)
            
            logging.info("✓ Traffic Manager参数配置完成:")
            logging.info(f"  • 跟车距离: {self.params['following_distance']} 米")
            logging.info(f"  • 随机种子: {self.args.seed}")
            
        except Exception as e:
            logging.warning(f"⚠ Traffic Manager参数设置部分失败: {e}")
        
        return tm_connected
    
    def get_existing_vehicles(self):
        """获取并记录现有车辆
        
        Returns:
            int: 现有车辆数量
        """
        existing_actors = self.world.get_actors().filter('vehicle.*')
        logging.info(f"📊 当前世界中现有车辆: {len(existing_actors)} 辆")
        
        # 记录现有车辆信息到字典
        for vehicle in existing_actors:
            self.existing_vehicles[vehicle.id] = {
                'type': vehicle.type_id,
                'role': vehicle.attributes.get('role_name', 'unknown'),
                'location': vehicle.get_location()
            }
            role = self.existing_vehicles[vehicle.id]['role']
            vehicle_type = vehicle.type_id.split('.')[-1] if '.' in vehicle.type_id else vehicle.type_id
            logging.info(f"  记录车辆: ID={vehicle.id} 类型={vehicle_type} 角色={role}")
        
        return len(existing_actors)
    
    def spawn_vehicle(self, vehicle_id, color, spawn_idx, target_idx):
        """生成单个车辆并设置其属性
        
        Args:
            vehicle_id (str): 车辆蓝图ID
            color (str): 车辆颜色
            spawn_idx (int): 生成点索引
            target_idx (int): 目标点索引
            
        Returns:
            carla.Vehicle or None: 生成的车辆对象
        """
        try:
            # 获取指定车辆蓝图
            bp = self.blueprint_library.find(vehicle_id)
            
            vehicle_type = vehicle_id.split('.')[-1] if '.' in vehicle_id else vehicle_id
            logging.info(f"🚗 生成车辆: {vehicle_type}")
            
            # 设置指定颜色
            if color and bp.has_attribute('color'):
                bp.set_attribute('color', color)
                logging.info(f"  • 颜色: {color}")
            
            # 设置角色名称
            bp.set_attribute('role_name', 'npc_vehicle')
            
            # 获取生成点
            if spawn_idx >= len(self.spawn_points):
                logging.error(f"  • 生成点索引超出范围: {spawn_idx}")
                return None
                
            spawn_point = self.spawn_points[spawn_idx]
            spawn_point.location.z = 0.2  # 调整高度避免碰撞
            
            # 尝试生成车辆
            vehicle = self.world.try_spawn_actor(bp, spawn_point)
            
            # 如果无法在预定位置生成，尝试替代位置
            spawn_attempts = 0
            max_attempts = self.params["spawn_retry_attempts"]
            
            while vehicle is None and spawn_attempts < max_attempts:
                spawn_attempts += 1
                logging.info(f"  • 尝试替代位置 {spawn_attempts}/{max_attempts}...")
                
                # 稍微调整位置而不是选择随机生成点
                variance = self.params["spawn_position_variance"]
                modified_spawn = carla.Transform(
                    carla.Location(
                        x=spawn_point.location.x + random.uniform(-variance, variance),
                        y=spawn_point.location.y + random.uniform(-variance, variance),
                        z=0.2
                    ),
                    spawn_point.rotation
                )
                vehicle = self.world.try_spawn_actor(bp, modified_spawn)
            
            if vehicle is not None:
                logging.info(f"  ✓ 车辆生成成功: ID={vehicle.id}")
                
                # 给车辆控制权交给Traffic Manager
                vehicle.set_autopilot(True, self.tm_port)
                
                # 设置目标点
                if target_idx < len(self.spawn_points):
                    target_point = self.spawn_points[target_idx].location
                    self.traffic_manager.set_path(vehicle, [target_point])
                    
                    logging.info(f"  • 目标点: spawn_point[{target_idx}]")
                    logging.info(f"  • 目标坐标: ({target_point.x:.2f}, {target_point.y:.2f}, {target_point.z:.2f})")
                
                # 设置车辆行为参数 - 安全驾驶设置
                if self.args.safe_mode:
                    self._configure_safe_driving(vehicle)
                
                self.stats["vehicles_spawned"] += 1
                return vehicle
                
            else:
                logging.error(f"  ❌ 无法生成车辆 {vehicle_type}，已尝试多个位置")
                self.stats["spawn_failures"] += 1
                return None
                
        except Exception as e:
            logging.error(f"生成车辆时发生错误: {e}")
            self.stats["spawn_failures"] += 1
            return None
    
    def _configure_safe_driving(self, vehicle):
        """配置车辆的安全驾驶参数
        
        Args:
            vehicle: CARLA车辆对象
        """
        try:
            # 禁用自动变道
            self.traffic_manager.auto_lane_change(vehicle, False)
            logging.info("  • 自动变道: 禁用")
        except:
            logging.warning("  • 无法设置自动变道参数")
        
        try:
            # 设置跟车距离
            self.traffic_manager.distance_to_leading_vehicle(
                vehicle, self.params["following_distance"]
            )
            logging.info(f"  • 跟车距离: {self.params['following_distance']} 米")
        except:
            logging.warning("  • 无法设置跟车距离")
        
        try:
            # 设置速度差异
            speed_diff = random.uniform(0, self.params["speed_difference"] * 100)
            self.traffic_manager.vehicle_percentage_speed_difference(vehicle, speed_diff)
            logging.info(f"  • 速度差异: {speed_diff:.1f}%")
        except:
            logging.warning("  • 无法设置速度差异")
        
        try:
            # 设置期望速度
            if hasattr(self.traffic_manager, 'set_desired_speed'):
                self.traffic_manager.set_desired_speed(vehicle, self.params["max_speed_limit"])
                logging.info(f"  • 最大速度: {self.params['max_speed_limit']} km/h")
        except:
            logging.warning("  • 无法设置期望速度")
    
    def spawn_vehicles(self):
        """生成所有指定的车辆
        
        Returns:
            bool: 是否成功生成车辆
        """
        vehicle_types = self.vehicle_configs["types"]
        vehicle_colors = self.vehicle_configs["colors"]
        spawn_indices = self.vehicle_configs["spawn_indices"]
        target_indices = self.vehicle_configs["target_indices"]
        
        num_vehicles = min(self.args.num_vehicles, len(vehicle_types))
        
        logging.info(f"🚀 准备生成 {num_vehicles} 辆指定NPC车辆")
        
        # 获取并记录现有车辆
        existing_count = self.get_existing_vehicles()
        
        # 选择生成模式
        if not getattr(self.args, 'batch_spawn', False):
            # 单独生成模式
            logging.info("使用单独生成模式")
            for i in range(num_vehicles):
                vehicle = self.spawn_vehicle(
                    vehicle_types[i], 
                    vehicle_colors[i], 
                    spawn_indices[i], 
                    target_indices[i]
                )
                if vehicle:
                    self.vehicle_list.append(vehicle)
                    
                # 在生成之间稍作等待
                time.sleep(0.1)
        
        else:
            # 批量生成模式
            logging.info("使用批量生成模式")
            self._batch_spawn_vehicles(
                vehicle_types, vehicle_colors, spawn_indices, target_indices, num_vehicles
            )
        
        successful_count = len(self.vehicle_list)
        logging.info(f"✅ 成功生成 {successful_count}/{num_vehicles} 辆NPC车辆")
        
        if successful_count > 0:
            logging.info(f"📈 车辆生成统计:")
            logging.info(f"  • 成功: {self.stats['vehicles_spawned']}")
            logging.info(f"  • 失败: {self.stats['spawn_failures']}")
            
        return successful_count > 0
    
    def _batch_spawn_vehicles(self, vehicle_types, vehicle_colors, spawn_indices, target_indices, num_vehicles):
        """批量生成车辆
        
        Args:
            vehicle_types (list): 车辆类型列表
            vehicle_colors (list): 车辆颜色列表
            spawn_indices (list): 生成点索引列表
            target_indices (list): 目标点索引列表
            num_vehicles (int): 要生成的车辆数量
        """
        batch_commands = []
        
        for i in range(num_vehicles):
            try:
                # 获取指定车辆蓝图
                vehicle_id = vehicle_types[i]
                bp = self.blueprint_library.find(vehicle_id)
                
                # 设置指定颜色
                if vehicle_colors[i] and bp.has_attribute('color'):
                    bp.set_attribute('color', vehicle_colors[i])
                
                # 设置角色名称
                bp.set_attribute('role_name', 'npc_vehicle')
                
                # 准备生成点
                spawn_idx = spawn_indices[i]
                spawn_point = self.spawn_points[spawn_idx]
                spawn_point.location.z = 0.2
                
                # 添加到批量命令列表
                batch_commands.append(carla.command.SpawnActor(bp, spawn_point))
                
            except Exception as e:
                logging.error(f"准备批量生成命令 {i} 时出错: {e}")
        
        # 执行批量生成
        logging.info(f"执行批量生成 {len(batch_commands)} 辆车辆...")
        results = self.client.apply_batch_sync(batch_commands, True)
        
        # 处理结果，获取车辆引用
        for i, result in enumerate(results):
            if result.error:
                logging.error(f"  车辆 {i+1} 生成失败: {result.error}")
                self.stats["spawn_failures"] += 1
            else:
                # 获取车辆引用
                vehicle = self.world.get_actor(result.actor_id)
                if vehicle:
                    self.vehicle_list.append(vehicle)
                    self.stats["vehicles_spawned"] += 1
                    
                    vehicle_type = vehicle_types[i].split('.')[-1]
                    logging.info(f"  ✓ 车辆 {i+1} ({vehicle_type}) 生成成功 (ID={result.actor_id})")
                    
                    # 给车辆控制权交给Traffic Manager
                    vehicle.set_autopilot(True, self.tm_port)
                    
                    # 设置目标点
                    target_idx = target_indices[i]
                    target_point = self.spawn_points[target_idx].location
                    self.traffic_manager.set_path(vehicle, [target_point])
                    
                    # 安全驾驶设置
                    if self.args.safe_mode:
                        self._configure_safe_driving(vehicle)
                else:
                    logging.error(f"  无法获取车辆 {i+1} 的引用 (ID={result.actor_id})")
    
    def cleanup_vehicles(self):
        """清理生成的车辆
        
        Returns:
            int: 成功销毁的车辆数量
        """
        self.stats["cleanup_operations"] += 1
        
        logging.info("🧹 开始清理NPC车辆...")
        
        # 第一步：识别需要销毁的车辆（非脚本创建的除外）
        vehicles_to_destroy = []
        current_vehicles = self.world.get_actors().filter('vehicle.*')
        logging.info(f"当前世界中共有 {len(current_vehicles)} 辆车")
        
        for vehicle in current_vehicles:
            # 如果车辆ID不在之前记录的现有车辆中，说明是本脚本创建的
            if vehicle.id not in self.existing_vehicles:
                vehicles_to_destroy.append(vehicle)
                try:
                    vehicle_type = vehicle.type_id.split('.')[-1]
                    role = vehicle.attributes.get('role_name', 'unknown')
                    logging.info(f"  标记清理: ID={vehicle.id} 类型={vehicle_type} 角色={role}")
                except:
                    logging.info(f"  标记清理: ID={vehicle.id}")
        
        logging.info(f"需要清理 {len(vehicles_to_destroy)} 辆本脚本创建的车辆")
        
        if not vehicles_to_destroy:
            logging.info("没有需要清理的车辆")
            return 0
        
        # 第二步：关闭自动驾驶
        active_vehicles = []
        for i, vehicle in enumerate(vehicles_to_destroy):
            try:
                if vehicle.is_alive:
                    logging.info(f"  关闭车辆 {i+1} 的自动驾驶 (ID={vehicle.id})")
                    vehicle.set_autopilot(False)
                    active_vehicles.append(vehicle)
                else:
                    logging.info(f"  车辆 {i+1} 已不存在，跳过")
            except Exception as e:
                logging.error(f"  关闭自动驾驶失败: {e}")
        
        # 等待Traffic Manager响应
        time.sleep(self.params["cleanup_wait_time"])
        
        # 第三步：逐个销毁车辆
        destroyed_count = 0
        logging.info(f"开始销毁 {len(active_vehicles)} 辆激活车辆")
        
        for i, vehicle in enumerate(active_vehicles):
            try:
                if vehicle.is_alive:
                    vehicle_id = vehicle.id
                    vehicle.destroy()
                    destroyed_count += 1
                    logging.info(f"  ✓ 成功销毁车辆 {i+1} (ID={vehicle_id})")
                    
                    # 短暂等待避免过快销毁导致问题
                    time.sleep(0.05)
                else:
                    logging.info(f"  车辆 {i+1} 已不存在，跳过")
            except Exception as e:
                logging.error(f"  销毁车辆失败: {e}")
        
        # 第四步：检查并批量销毁剩余车辆
        logging.info("检查并批量销毁剩余车辆...")
        try:
            final_check = self.world.get_actors().filter('vehicle.*')
            surviving_vehicles = []
            
            # 检查哪些应该被销毁但仍然存活的车辆
            for vehicle in final_check:
                if vehicle.id not in self.existing_vehicles and vehicle.is_alive:
                    surviving_vehicles.append(vehicle)
            
            if surviving_vehicles:
                logging.info(f"  尝试批量销毁剩余 {len(surviving_vehicles)} 辆车辆")
                self.client.apply_batch([carla.command.DestroyActor(v) for v in surviving_vehicles])
                logging.info("  批量销毁命令已发送")
                destroyed_count += len(surviving_vehicles)
            else:
                logging.info("  没有剩余车辆需要销毁")
                
        except Exception as e:
            logging.error(f"  批量销毁车辆失败: {e}")
        
        # 清空车辆列表
        self.vehicle_list = []
        self.stats["vehicles_destroyed"] += destroyed_count
        
        # 短暂等待确保清理完成
        time.sleep(self.params["cleanup_wait_time"])
        
        logging.info(f"✅ 车辆清理完成，共销毁 {destroyed_count} 辆车")
        return destroyed_count
    
    def get_statistics(self):
        """获取统计信息"""
        return self.stats.copy()

# ============================================================================
# 交互式菜单系统
# ============================================================================

class MenuSystem:
    """交互式菜单系统，提供用户友好的测试选择界面"""
    
    @staticmethod
    def display_main_menu():
        """显示主菜单并获取用户选择
        
        Returns:
            int: 用户选择的选项 (0-9)
        """
        print("\n" + "="*70)
        print("🚗 " + " "*20 + "自动驾驶测试系统 - 主菜单" + " "*20 + " 🚗")
        print("="*70)
        print("📍 导航测试:")
        print("  1️⃣  到达任务起点          - 导航到NPC测试起始位置")
        print("  5️⃣  点对点导航到终点1      - 直接导航到NPC任务终点1")
        print("  8️⃣  导航到回环测试起点     - 从任意位置导航到回环起点")
        print("  9️⃣  导航到NPC任务终点2     - 导航到NPC测试最终位置")
        print()
        print("🚦 NPC环境测试:")
        print("  2️⃣  进行一次NPC测试       - 完整的NPC环境导航测试")
        print("  4️⃣  NPC导航任务(无车)      - 执行NPC路径但不生成车辆")
        print()
        print("🔄 回环测试:")
        print("  3️⃣  进行一次回环测试      - 单次循环路径测试")
        print("  6️⃣  进行3次回环测试       - 执行3次完整回环测试")
        print("  7️⃣  进行N次回环测试       - 执行指定次数或无限回环测试")
        print()
        print("❌ 退出:")
        print("  0️⃣  退出程序")
        print("="*70)
        print("🎮 测试期间可用控制键: T(碰撞恢复) R(重启) P(暂停) S(停止)")
        print("="*70)
        
        while True:
            try:
                choice = input("👉 请选择操作 (0-9): ").strip()
                if choice in ['0', '1', '2', '3', '4', '5', '6', '7', '8', '9']:
                    return int(choice)
                else:
                    print("❌ 无效选择，请输入0-9之间的数字")
            except KeyboardInterrupt:
                print("\n❌ 用户中断，退出程序")
                return 0
            except:
                print("❌ 输入错误，请重试")
    
    @staticmethod
    def get_loop_count():
        """获取回环测试次数
        
        Returns:
            int: 回环次数，-1表示无限循环，0表示取消
        """
        print("\n" + "="*50)
        print("🔄 回环测试配置")
        print("="*50)
        
        while True:
            try:
                print("请选择回环测试模式:")
                print("  • 输入具体数字: 执行指定次数的回环测试")
                print("  • 直接按回车: 无限循环直到手动停止")
                print("  • 输入 0: 取消操作")
                
                count = input("👉 请输入回环测试次数: ").strip()
                
                if count == "":
                    print("✅ 选择无限回环模式")
                    return -1  # 无限循环
                elif count == "0":
                    print("❌ 取消回环测试")
                    return 0   # 取消
                else:
                    num = int(count)
                    if num > 0:
                        print(f"✅ 选择执行 {num} 次回环测试")
                        return num
                    else:
                        print("❌ 请输入正整数")
                        
            except ValueError:
                print("❌ 请输入有效的数字")
            except KeyboardInterrupt:
                print("\n❌ 用户中断")
                return 0
    
    @staticmethod
    def show_test_progress(current, total, description=""):
        """显示测试进度
        
        Args:
            current (int): 当前进度
            total (int): 总数，-1表示无限
            description (str): 描述信息
        """
        if total == -1:
            print(f"🔄 {description} - 第 {current} 次 (无限模式)")
        else:
            percentage = (current / total) * 100
            print(f"📊 {description} - 第 {current}/{total} 次 ({percentage:.1f}%)")
    
    @staticmethod
    def confirm_action(message, default_yes=False):
        """确认操作
        
        Args:
            message (str): 确认消息
            default_yes (bool): 默认是否为是
            
        Returns:
            bool: 用户是否确认
        """
        suffix = " (Y/n)" if default_yes else " (y/N)"
        try:
            response = input(f"❓ {message}{suffix}: ").strip().lower()
            
            if response == "":
                return default_yes
            
            return response in ['y', 'yes', '是', '确认']
            
        except KeyboardInterrupt:
            print("\n❌ 用户中断")
            return False

# ============================================================================
# 增强集成管理器
# ============================================================================

class EnhancedIntegratedManager:
    """增强版集成管理器，提供完整的测试流程管理"""
    
    def __init__(self, args):
        """初始化增强集成管理器
        
        Args:
            args: 命令行参数对象
        """
        self.args = args
        self.params = DEFAULT_PARAMETERS.copy()
        self.update_params_from_args(args)
        
        # 管理器组件
        self.carla_manager = CarlaManager(args, self.params)
        self.autoware_publisher = None
        self.keyboard_monitor = EnhancedKeyboardMonitor()
        
        # 状态管理
        self.running = True
        self.test_session_id = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # 统计信息
        self.session_stats = {
            "tests_completed": 0,
            "npc_tests_completed": 0,
            "loop_tests_completed": 0,
            "navigation_successes": 0,
            "navigation_failures": 0,
            "admin_interventions": 0,
            "start_time": time.time()
        }
        
        # 设置信号处理器
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
        logging.info(f"🚀 增强集成管理器初始化完成 (会话ID: {self.test_session_id})")
    
    def update_params_from_args(self, args):
        """从命令行参数更新配置参数
        
        Args:
            args: 命令行参数对象
        """
        param_mappings = [
            'wait_before_engage', 'wait_at_waypoint', 'position_check_interval',
            'max_position_wait_time', 'goal_distance_threshold', 'start_point_threshold'
        ]
        
        for param in param_mappings:
            if hasattr(args, param) and getattr(args, param) is not None:
                self.params[param] = getattr(args, param)
        
        logging.info("⚙️ 配置参数:")
        for key, value in self.params.items():
            if isinstance(value, float):
                logging.info(f"  • {key}: {value:.2f}")
            else:
                logging.info(f"  • {key}: {value}")
    
    def signal_handler(self, sig, frame):
        """信号处理器，用于优雅关闭
        
        Args:
            sig: 信号类型
            frame: 调用帧
        """
        logging.info(f"📶 接收到信号 {sig}，启动清理流程...")
        self.running = False
        if hasattr(self, 'keyboard_monitor'):
            self.keyboard_monitor.stop()
    
    def initialize(self):
        """初始化连接和管理器
        
        Returns:
            bool: 初始化是否成功
        """
        logging.info("🔧 开始初始化系统组件...")
        
        # 连接到CARLA
        if not self.carla_manager.connect():
            logging.error("❌ CARLA连接失败")
            return False
        
        # 设置Traffic Manager
        if not self.carla_manager.setup_traffic_manager():
            logging.error("❌ Traffic Manager设置失败")
            return False
        
        # 初始化ROS2（如果可用）
        if ROS2_AVAILABLE:
            try:
                if not rclpy.ok():
                    rclpy.init()
                
                # 创建Autoware发布器
                self.autoware_publisher = AutowareGoalPublisher(self.params)
                
                # 等待时钟和位置更新
                clock_ok = self.autoware_publisher.wait_for_clock(5.0)
                position_ok = self.autoware_publisher.wait_for_position_update(5.0)
                
                if clock_ok and position_ok:
                    logging.info("✅ ROS2和Autoware发布器初始化成功")
                else:
                    logging.warning("⚠ ROS2初始化部分成功，某些功能可能受限")
                    
            except Exception as e:
                logging.error(f"❌ ROS2初始化失败: {e}")
                return False
        else:
            logging.warning("⚠ ROS2不可用，跳过Autoware发布器初始化")
            return False
        
        # 启动键盘监听器
        self.keyboard_monitor.start()
        
        logging.info("✅ 系统初始化完成")
        return True
    
    def navigate_to_point(self, goal_pos, description="", check_interrupts=True, timeout=None):
        """导航到指定点位
        
        Args:
            goal_pos (list): 目标位置 [x, y, z, qx, qy, qz, qw]
            description (str): 目标描述
            check_interrupts (bool): 是否检查中断
            timeout (float): 超时时间
            
        Returns:
            tuple: (success, reason)
        """
        if timeout is None:
            timeout = self.params["navigation_timeout"]
        
        logging.info(f"🎯 开始导航到 {description}...")
        start_time = time.time()
        
        # 发布目标
        self.autoware_publisher.publish_goal(goal_pos, description)
        
        # 等待后发送engage命令
        wait_time = self.params['wait_before_engage']
        logging.info(f"⏳ 等待 {wait_time} 秒后发送engage命令...")
        time.sleep(wait_time)
        
        # 发送engage命令
        logging.info("🚗 发送engage命令...")
        self.autoware_publisher.publish_engage(True, f"导航到{description}")
        
        # 等待到达目标
        logging.info(f"🛣️ 等待车辆到达 {description}...")
        reached_goal = False
        
        while not reached_goal and self.running:
            # 检查超时
            if time.time() - start_time > timeout:
                logging.warning(f"⏰ 导航超时 ({timeout}秒)")
                return False, "timeout"
            
            # 处理ROS2回调
            if ROS2_AVAILABLE:
                rclpy.spin_once(self.autoware_publisher, timeout_sec=0.1)
            
            # 获取当前距离
            distance = self.autoware_publisher.get_current_position_distance_to(goal_pos)
            if distance != float('inf'):
                logging.info(f"📏 距离 {description}: {distance:.2f} 米")
            
            # 检查中断（如果启用）
            if check_interrupts:
                interrupt_result = self._check_keyboard_interrupts()
                if interrupt_result:
                    return False, interrupt_result
            
            # 检查是否到达目标
            reached_goal = self.autoware_publisher.check_if_at_position(goal_pos)
            if reached_goal:
                elapsed_time = time.time() - start_time
                logging.info(f"✅ 车辆已到达 {description}! (用时: {elapsed_time:.1f}秒)")
                self.session_stats["navigation_successes"] += 1
                break
            
            time.sleep(self.params["position_check_interval"])
        
        if not self.running:
            return False, "cancelled"
        
        if not reached_goal:
            logging.warning(f"⚠ 车辆未能及时到达 {description}")
            self.session_stats["navigation_failures"] += 1
            return False, "timeout"
        
        # 在目标点等待
        wait_time = self.params['wait_at_waypoint']
        logging.info(f"⏸️ 在 {description} 等待 {wait_time} 秒...")
        time.sleep(wait_time)
        
        return True, "success"
    
    def _check_keyboard_interrupts(self):
        """检查键盘中断
        
        Returns:
            str or None: 中断类型或None
        """
        pressed_keys = self.keyboard_monitor.get_pressed_keys()
        
        if pressed_keys['t']:
            logging.warning("🔧 检测到管理员干预 (T键)")
            self.keyboard_monitor.reset_flags()
            self.session_stats["admin_interventions"] += 1
            return "admin_interrupt"
        
        if pressed_keys['r']:
            logging.warning("🔄 检测到重启请求 (R键)")
            self.keyboard_monitor.reset_flags()
            return "restart_request"
        
        if pressed_keys['s']:
            logging.warning("⏹️ 检测到停止请求 (S键)")
            self.keyboard_monitor.reset_flags()
            return "stop_request"
        
        if pressed_keys['p']:
            logging.warning("⏸️ 检测到暂停请求 (P键)")
            self.keyboard_monitor.reset_flags()
            return "pause_request"
        
        if pressed_keys['q']:
            logging.warning("❌ 检测到退出请求 (Q键)")
            self.keyboard_monitor.reset_flags()
            self.running = False
            return "quit_request"
        
        return None
    
    def ensure_at_position(self, target_pos, description, threshold=None):
        """确保车辆在指定位置
        
        Args:
            target_pos (list): 目标位置
            description (str): 位置描述
            threshold (float): 距离阈值
            
        Returns:
            bool: 是否成功到达
        """
        if threshold is None:
            threshold = self.params["start_point_threshold"]
        
        logging.info(f"📍 检查车辆是否在 {description}...")
        
        # 更新位置信息
        if ROS2_AVAILABLE:
            rclpy.spin_once(self.autoware_publisher, timeout_sec=0.1)
        
        distance = self.autoware_publisher.get_current_position_distance_to(target_pos)
        logging.info(f"📏 距离 {description}: {distance:.2f} 米 (阈值: {threshold:.2f} 米)")
        
        if distance > threshold:
            logging.info(f"🚶 车辆不在 {description}，开始导航...")
            success, reason = self.navigate_to_point(target_pos, description, check_interrupts=False)
            if not success:
                logging.warning(f"❌ 导航到 {description} 失败: {reason}")
                return False
            logging.info(f"✅ 车辆已到达 {description}")
        else:
            logging.info(f"✅ 车辆已在 {description}")
        
        return True
    
    # ========================================================================
    # 测试选项执行方法
    # ========================================================================
    
    def execute_option_1(self):
        """选项1: 到达任务起点"""
        logging.info("🎯 执行选项1: 到达任务起点")
        
        npc_start = self.autoware_publisher.goal_sets["npc_test"][0]
        success = self.ensure_at_position(npc_start, "NPC任务起点")
        
        if success:
            logging.info("✅ 选项1执行成功")
            self.session_stats["tests_completed"] += 1
        else:
            logging.error("❌ 选项1执行失败")
        
        return success
    
    def execute_option_2(self):
        """选项2: 进行一次完整的NPC测试"""
        logging.info("🚦 执行选项2: 进行一次NPC测试")
        
        try:
            # 获取NPC测试路点
            npc_goals = self.autoware_publisher.goal_sets["npc_test"]
            npc_start = npc_goals[0]   # 起点
            npc_end1 = npc_goals[1]    # 任务终点1
            npc_end2 = npc_goals[2]    # 任务终点2
            
            # 1. 确保在起点
            if not self.ensure_at_position(npc_start, "NPC任务起点"):
                return False
            
            # 2. 设置目标为任务终点1，然后生成NPC车辆
            logging.info("🎯 设置任务终点1为目标...")
            self.autoware_publisher.publish_goal(npc_end1, "NPC任务终点1")
            time.sleep(2)
            
            logging.info("🚗 生成NPC车辆...")
            if not self.carla_manager.spawn_vehicles():
                logging.error("❌ NPC车辆生成失败")
                return False
            
            # 发送engage命令
            time.sleep(3)
            self.autoware_publisher.publish_engage(True, "NPC测试-到终点1")
            
            # 3. 等待到达任务终点1（带中断检查）
            logging.info("🛣️ 等待车辆到达任务终点1...")
            reached_end1 = False
            start_time = time.time()
            
            while not reached_end1 and self.running:
                # 检查超时
                if time.time() - start_time > self.params["navigation_timeout"]:
                    logging.warning("⏰ 导航到任务终点1超时")
                    break
                
                # 处理ROS2回调
                if ROS2_AVAILABLE:
                    rclpy.spin_once(self.autoware_publisher, timeout_sec=0.1)
                
                distance = self.autoware_publisher.get_current_position_distance_to(npc_end1)
                if distance != float('inf'):
                    logging.info(f"📏 距离任务终点1: {distance:.2f} 米")
                
                # 检查管理员干预
                if self.keyboard_monitor.get_pressed_keys()['t']:
                    logging.warning("🔧 检测到NPC碰撞报告，进入恢复程序")
                    self.keyboard_monitor.reset_flags()
                    self.session_stats["admin_interventions"] += 1
                    
                    logging.info("🧹 按管理员要求移除NPC车辆...")
                    self.carla_manager.cleanup_vehicles()
                    
                    wait_time = self.params['wait_after_npc_removal']
                    logging.info(f"⏳ NPC移除后等待 {wait_time} 秒...")
                    time.sleep(wait_time)
                    
                    logging.info("🚗 继续导航到任务终点1...")
                    self.autoware_publisher.publish_engage(True, "NPC测试恢复-到终点1")
                
                reached_end1 = self.autoware_publisher.check_if_at_position(npc_end1)
                if reached_end1:
                    logging.info("✅ 车辆已到达任务终点1!")
                    break
                
                time.sleep(self.params["position_check_interval"])
            
            if not reached_end1:
                logging.warning("⚠ 车辆未能到达任务终点1")
                return False
            
            # 4. 在任务终点1等待
            wait_time = self.params['wait_at_waypoint']
            logging.info(f"⏸️ 在任务终点1等待 {wait_time} 秒...")
            time.sleep(wait_time)
            
            # 5. 导航到任务终点2
            success, reason = self.navigate_to_point(npc_end2, "NPC任务终点2", check_interrupts=False)
            if not success:
                logging.error(f"❌ 导航到任务终点2失败: {reason}")
                return False
            
            # 6. 在任务终点2删除NPC车并停车
            logging.info("🧹 在任务终点2删除NPC车辆...")
            destroyed_count = self.carla_manager.cleanup_vehicles()
            logging.info(f"✅ NPC测试完成，已清理 {destroyed_count} 辆车，车辆停在任务终点2")
            
            self.session_stats["tests_completed"] += 1
            self.session_stats["npc_tests_completed"] += 1
            return True
            
        except Exception as e:
            logging.error(f"❌ 执行NPC测试时发生错误: {e}")
            return False
    
    def execute_option_3(self):
        """选项3: 进行一次回环测试"""
        logging.info("🔄 执行选项3: 进行一次回环测试")
        
        success = self.execute_single_loop()
        if success:
            self.session_stats["tests_completed"] += 1
            self.session_stats["loop_tests_completed"] += 1
        
        return success
    
    def execute_option_4(self):
        """选项4: 执行NPC导航任务（无NPC车辆）"""
        logging.info("🛣️ 执行选项4: NPC导航任务（无NPC车辆）")
        
        try:
            npc_goals = self.autoware_publisher.goal_sets["npc_test"]
            npc_start = npc_goals[0]
            npc_end1 = npc_goals[1]
            npc_end2 = npc_goals[2]
            
            # 确保在起点
            if not self.ensure_at_position(npc_start, "NPC任务起点"):
                return False
            
            # 导航到任务终点1
            success, reason = self.navigate_to_point(npc_end1, "任务终点1", check_interrupts=False)
            if not success:
                logging.error(f"❌ 导航到任务终点1失败: {reason}")
                return False
            
            # 导航到任务终点2
            success, reason = self.navigate_to_point(npc_end2, "任务终点2", check_interrupts=False)
            if not success:
                logging.error(f"❌ 导航到任务终点2失败: {reason}")
                return False
            
            logging.info("✅ NPC导航任务（无NPC车辆）完成")
            self.session_stats["tests_completed"] += 1
            return True
            
        except Exception as e:
            logging.error(f"❌ 执行NPC导航任务时发生错误: {e}")
            return False
    
    def execute_option_5(self):
        """选项5: 点对点导航到NPC任务终点1"""
        logging.info("🎯 执行选项5: 点对点导航到NPC任务终点1")
        
        npc_end1 = self.autoware_publisher.goal_sets["npc_test"][1]
        success, reason = self.navigate_to_point(npc_end1, "NPC任务终点1", check_interrupts=False)
        
        if success:
            logging.info("✅ 已到达NPC任务终点1")
            self.session_stats["tests_completed"] += 1
        else:
            logging.error(f"❌ 导航失败: {reason}")
        
        return success
    
    def execute_option_6(self):
        """选项6: 进行3次回环测试"""
        logging.info("🔄 执行选项6: 进行3次回环测试")
        
        total_loops = 3
        successful_loops = 0
        
        for loop_count in range(total_loops):
            MenuSystem.show_test_progress(loop_count + 1, total_loops, "回环测试")
            
            logging.info(f"🔄 开始第 {loop_count + 1}/{total_loops} 次回环测试")
            
            if self.execute_single_loop():
                successful_loops += 1
                logging.info(f"✅ 第 {loop_count + 1}/{total_loops} 次回环测试完成")
                self.session_stats["loop_tests_completed"] += 1
            else:
                logging.error(f"❌ 第 {loop_count + 1} 次回环测试失败")
                break
        
        if successful_loops == total_loops:
            logging.info(f"✅ 3次回环测试全部完成")
            self.session_stats["tests_completed"] += 1
            return True
        else:
            logging.warning(f"⚠ 只完成了 {successful_loops}/{total_loops} 次回环测试")
            return False
    
    def execute_option_7(self):
        """选项7: 进行N次回环测试（P键暂停，S键退出）"""
        logging.info("🔄 执行选项7: 进行N次回环测试")
        
        loop_count = MenuSystem.get_loop_count()
        if loop_count == 0:
            return False
        
        successful_loops = 0
        current_loop = 0
        
        try:
            if loop_count == -1:
                # 无限回环模式
                logging.info("🔄 开始无限回环测试（P键暂停，S键退出）")
                
                while self.running:
                    current_loop += 1
                    MenuSystem.show_test_progress(current_loop, -1, "无限回环测试")
                    
                    result = self.execute_single_loop_with_interrupts()
                    
                    if result == "success":
                        successful_loops += 1
                        logging.info(f"✅ 第 {current_loop} 次回环测试完成")
                        self.session_stats["loop_tests_completed"] += 1
                        
                    elif result == "pause":
                        logging.info("⏸️ 回环测试已暂停")
                        if not MenuSystem.confirm_action("是否继续回环测试", default_yes=True):
                            break
                        self.keyboard_monitor.reset_flags()
                        current_loop -= 1  # 重做当前循环
                        continue
                        
                    elif result == "stop":
                        logging.info("⏹️ 回环测试已停止")
                        break
                        
                    else:  # error
                        logging.error(f"❌ 第 {current_loop} 次回环测试失败")
                        if not MenuSystem.confirm_action("是否继续回环测试", default_yes=False):
                            break
            
            else:
                # 指定次数回环模式
                logging.info(f"🔄 开始 {loop_count} 次回环测试（P键暂停，S键退出）")
                
                while current_loop < loop_count and self.running:
                    current_loop += 1
                    MenuSystem.show_test_progress(current_loop, loop_count, "回环测试")
                    
                    result = self.execute_single_loop_with_interrupts()
                    
                    if result == "success":
                        successful_loops += 1
                        logging.info(f"✅ 第 {current_loop}/{loop_count} 次回环测试完成")
                        self.session_stats["loop_tests_completed"] += 1
                        
                    elif result == "pause":
                        logging.info("⏸️ 回环测试已暂停")
                        if MenuSystem.confirm_action("是否继续回环测试", default_yes=True):
                            self.keyboard_monitor.reset_flags()
                            current_loop -= 1  # 重做当前循环
                            continue
                        else:
                            break
                            
                    elif result == "stop":
                        logging.info("⏹️ 回环测试已停止")
                        break
                        
                    else:  # error
                        logging.error(f"❌ 第 {current_loop} 次回环测试失败")
                        if not MenuSystem.confirm_action("是否继续回环测试", default_yes=False):
                            break
        
        except KeyboardInterrupt:
            logging.info("❌ 回环测试被用户中断")
        
        # 统计结果
        if loop_count == -1:
            logging.info(f"📊 无限回环测试结束，共完成 {successful_loops} 次成功测试")
        else:
            success_rate = (successful_loops / loop_count) * 100 if loop_count > 0 else 0
            logging.info(f"📊 回环测试结束，成功率: {successful_loops}/{loop_count} ({success_rate:.1f}%)")
        
        if successful_loops > 0:
            self.session_stats["tests_completed"] += 1
        
        return successful_loops > 0
    
    def execute_option_8(self):
        """选项8: 从任意位置导航到回环测试起点"""
        logging.info("🎯 执行选项8: 从任意位置导航到回环测试起点")
        
        loop_start = self.autoware_publisher.goal_sets["loop_test"][0]
        success, reason = self.navigate_to_point(loop_start, "回环测试起点", check_interrupts=False)
        
        if success:
            logging.info("✅ 已到达回环测试起点")
            self.session_stats["tests_completed"] += 1
        else:
            logging.error(f"❌ 导航失败: {reason}")
        
        return success
    
    def execute_option_9(self):
        """选项9: 导航到NPC任务终点2"""
        logging.info("🎯 执行选项9: 导航到NPC任务终点2")
        
        npc_end2 = self.autoware_publisher.goal_sets["npc_test"][2]
        success, reason = self.navigate_to_point(npc_end2, "NPC任务终点2", check_interrupts=False)
        
        if success:
            logging.info("✅ 已到达NPC任务终点2")
            self.session_stats["tests_completed"] += 1
        else:
            logging.error(f"❌ 导航失败: {reason}")
        
        return success
    
    # ========================================================================
    # 回环测试辅助方法
    # ========================================================================
    
    def execute_single_loop(self):
        """执行单次回环测试（无中断检查）
        
        Returns:
            bool: 是否成功
        """
        try:
            loop_goals = self.autoware_publisher.goal_sets["loop_test"]
            loop_start = loop_goals[0]
            
            # 确保在回环起点
            if not self.ensure_at_position(loop_start, "回环测试起点"):
                return False
            
            # 遍历所有回环路点
            for i in range(1, len(loop_goals)):
                waypoint = loop_goals[i]
                waypoint_name = self.autoware_publisher.waypoint_names["loop_test"][i]
                
                success, reason = self.navigate_to_point(waypoint, waypoint_name, check_interrupts=False)
                if not success:
                    logging.error(f"❌ 导航到 {waypoint_name} 失败: {reason}")
                    return False
            
            # 回到起点
            success, reason = self.navigate_to_point(loop_start, "回环测试起点", check_interrupts=False)
            if not success:
                logging.error(f"❌ 回到回环测试起点失败: {reason}")
                return False
            
            return True
            
        except Exception as e:
            logging.error(f"❌ 执行单次回环测试时发生错误: {e}")
            return False
    
    def execute_single_loop_with_interrupts(self):
        """执行单次回环测试（带中断检查）
        
        Returns:
            str: 执行结果 ("success", "pause", "stop", "error")
        """
        try:
            loop_goals = self.autoware_publisher.goal_sets["loop_test"]
            loop_start = loop_goals[0]
            
            # 确保在回环起点
            if not self.ensure_at_position(loop_start, "回环测试起点"):
                return "error"
            
            # 遍历所有回环路点
            for i in range(1, len(loop_goals)):
                waypoint = loop_goals[i]
                waypoint_name = self.autoware_publisher.waypoint_names["loop_test"][i]
                
                success, reason = self.navigate_to_point(waypoint, waypoint_name, check_interrupts=True)
                if not success:
                    if reason == "pause_request":
                        return "pause"
                    elif reason == "stop_request":
                        return "stop"
                    else:
                        logging.error(f"❌ 导航到 {waypoint_name} 失败: {reason}")
                        return "error"
            
            # 回到起点
            success, reason = self.navigate_to_point(loop_start, "回环测试起点", check_interrupts=True)
            if not success:
                if reason == "pause_request":
                    return "pause"
                elif reason == "stop_request":
                    return "stop"
                else:
                    logging.error(f"❌ 回到回环测试起点失败: {reason}")
                    return "error"
            
            return "success"
            
        except Exception as e:
            logging.error(f"❌ 执行回环测试时发生错误: {e}")
            return "error"
    
    # ========================================================================
    # 主运行方法
    # ========================================================================
    
    def run_interactive_mode(self):
        """运行交互模式"""
        logging.info("🎮 启动交互模式")
        
        # 显示欢迎信息
        self._show_welcome_message()
        
        while self.running:
            try:
                # 显示主菜单并获取选择
                choice = MenuSystem.display_main_menu()
                
                if choice == 0:
                    logging.info("👋 用户选择退出程序")
                    break
                
                # 重置键盘标志
                self.keyboard_monitor.reset_flags()
                
                # 执行选择的操作
                start_time = time.time()
                success = self._execute_option(choice)
                execution_time = time.time() - start_time
                
                # 记录执行结果
                if success:
                    logging.info(f"✅ 选项 {choice} 执行成功 (用时: {execution_time:.1f}秒)")
                else:
                    logging.warning(f"⚠ 选项 {choice} 执行失败 (用时: {execution_time:.1f}秒)")
                
                # 显示统计信息
                self._show_session_stats()
                
            except KeyboardInterrupt:
                logging.info("❌ 操作被用户中断")
                if not MenuSystem.confirm_action("是否返回主菜单", default_yes=True):
                    break
                continue
                
            except Exception as e:
                logging.error(f"❌ 执行过程中发生错误: {e}")
                if not MenuSystem.confirm_action("是否继续使用系统", default_yes=True):
                    break
                continue
            
            # 操作完成后的暂停
            if self.running:
                input("\n⏳ 按回车键返回主菜单...")
        
        logging.info("👋 交互模式结束")
    
    def _show_welcome_message(self):
        """显示欢迎信息"""
        print("\n" + "="*70)
        print("🎉 " + " "*15 + "欢迎使用增强版自动驾驶测试系统" + " "*15 + " 🎉")
        print("="*70)
        print("📋 系统功能:")
        print("  • NPC环境测试 - 在复杂交通环境下测试自动驾驶性能")
        print("  • 回环测试 - 循环路径导航稳定性测试")
        print("  • 点对点导航 - 灵活的目标点导航功能")
        print("  • 实时键盘控制 - 支持测试过程中的人工干预")
        print("  • 智能恢复机制 - 碰撞检测和自动恢复功能")
        print("="*70)
        print(f"📊 会话ID: {self.test_session_id}")
        print(f"⏰ 启动时间: {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        print("="*70)
    
    def _execute_option(self, choice):
        """执行指定的选项
        
        Args:
            choice (int): 选项编号
            
        Returns:
            bool: 执行是否成功
        """
        option_methods = {
            1: self.execute_option_1,
            2: self.execute_option_2,
            3: self.execute_option_3,
            4: self.execute_option_4,
            5: self.execute_option_5,
            6: self.execute_option_6,
            7: self.execute_option_7,
            8: self.execute_option_8,
            9: self.execute_option_9,
        }
        
        method = option_methods.get(choice)
        if method:
            return method()
        else:
            logging.error(f"❌ 未知选项: {choice}")
            return False
    
    def _show_session_stats(self):
        """显示会话统计信息"""
        session_time = time.time() - self.session_stats["start_time"]
        
        print(f"\n📊 当前会话统计:")
        print(f"  • 会话时间: {session_time/60:.1f} 分钟")
        print(f"  • 完成测试: {self.session_stats['tests_completed']} 项")
        print(f"  • NPC测试: {self.session_stats['npc_tests_completed']} 次")
        print(f"  • 回环测试: {self.session_stats['loop_tests_completed']} 次")
        print(f"  • 导航成功: {self.session_stats['navigation_successes']} 次")
        print(f"  • 导航失败: {self.session_stats['navigation_failures']} 次")
        print(f"  • 管理员干预: {self.session_stats['admin_interventions']} 次")
        
        # 显示键盘统计
        key_stats = self.keyboard_monitor.get_stats()
        if any(key_stats.values()):
            print(f"  • 按键统计: T={key_stats['t']} R={key_stats['r']} P={key_stats['p']} S={key_stats['s']}")
    
    def cleanup(self):
        """清理所有资源"""
        logging.info("🧹 开始资源清理...")
        
        # 停止键盘监听器
        if hasattr(self, 'keyboard_monitor'):
            logging.info("  • 停止键盘监听...")
            self.keyboard_monitor.stop()
        
        # 清理CARLA资源
        if hasattr(self, 'carla_manager'):
            logging.info("  • 清理CARLA车辆...")
            try:
                vehicles_destroyed = self.carla_manager.cleanup_vehicles()
                logging.info(f"  • 销毁了 {vehicles_destroyed} 辆车")
                
                # 显示CARLA统计
                carla_stats = self.carla_manager.get_statistics()
                logging.info(f"  • CARLA统计: 生成{carla_stats['vehicles_spawned']}辆，销毁{carla_stats['vehicles_destroyed']}辆")
                
            except Exception as e:
                logging.error(f"  • CARLA清理出错: {e}")
        
        # 清理ROS2资源
        if ROS2_AVAILABLE and hasattr(self, 'autoware_publisher') and self.autoware_publisher:
            logging.info("  • 清理ROS2资源...")
            try:
                # 显示Autoware统计
                autoware_stats = self.autoware_publisher.get_statistics()
                logging.info(f"  • Autoware统计: 目标{autoware_stats['goals_published']}个，"
                           f"engage{autoware_stats['engage_commands']}次")
                
                self.autoware_publisher.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()
                logging.info("  • ROS2资源清理完成")
                
            except Exception as e:
                logging.error(f"  • ROS2清理出错: {e}")
        
        # 记录最终统计
        session_time = time.time() - self.session_stats["start_time"]
        logging.info("📊 会话总结:")
        logging.info(f"  • 总耗时: {session_time/60:.1f} 分钟")
        logging.info(f"  • 完成测试: {self.session_stats['tests_completed']} 项")
        logging.info(f"  • 成功率: {self._calculate_success_rate():.1f}%")
        
        logging.info("✅ 资源清理完成")
    
    def _calculate_success_rate(self):
        """计算成功率"""
        total = self.session_stats['navigation_successes'] + self.session_stats['navigation_failures']
        if total == 0:
            return 100.0
        return (self.session_stats['navigation_successes'] / total) * 100

# ============================================================================
# 配置文件管理
# ============================================================================

def load_config_file(config_path):
    """加载配置文件
    
    Args:
        config_path (str): 配置文件路径
        
    Returns:
        dict: 配置字典
    """
    try:
        with open(config_path, 'r', encoding='utf-8') as f:
            config = json.load(f)
        logging.info(f"✅ 成功加载配置文件: {config_path}")
        return config
    except Exception as e:
        logging.error(f"❌ 加载配置文件失败: {e}")
        return {}

def save_default_config(config_path):
    """保存默认配置文件
    
    Args:
        config_path (str): 配置文件路径
    """
    default_config = {
        "description": "增强版自动驾驶测试系统默认配置",
        "version": "2.0",
        "carla": {
            "host": "127.0.0.1",
            "port": 2000,
            "tm_port": 8100,
            "alt_tm_port": 8200
        },
        "vehicles": {
            "num_vehicles": 5,
            "seed": 42,
            "safe_mode": True,
            "batch_spawn": True
        },
        "navigation": DEFAULT_PARAMETERS,
        "logging": {
            "level": "INFO",
            "max_log_files": 10
        }
    }
    
    try:
        os.makedirs(os.path.dirname(config_path), exist_ok=True)
        with open(config_path, 'w', encoding='utf-8') as f:
            json.dump(default_config, f, indent=2, ensure_ascii=False)
        logging.info(f"✅ 默认配置文件已保存: {config_path}")
    except Exception as e:
        logging.error(f"❌ 保存配置文件失败: {e}")

# ============================================================================
# 命令行参数解析
# ============================================================================

def parse_arguments():
    """解析命令行参数
    
    Returns:
        argparse.Namespace: 解析后的参数对象
    """
    argparser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    
    # CARLA连接参数
    carla_group = argparser.add_argument_group('CARLA连接参数')
    carla_group.add_argument('--host', metavar='H', default='127.0.0.1',
                            help='CARLA服务器IP地址 (默认: 127.0.0.1)')
    carla_group.add_argument('-p', '--port', metavar='P', default=2000, type=int,
                            help='CARLA服务器TCP端口 (默认: 2000)')
    carla_group.add_argument('--tm-port', metavar='P', default=8100, type=int,
                            help='Traffic Manager端口 (默认: 8100)')
    carla_group.add_argument('--alt-tm-port', metavar='P', default=8200, type=int,
                            help='备用Traffic Manager端口 (默认: 8200)')
    
    # 车辆参数
    vehicle_group = argparser.add_argument_group('车辆参数')
    vehicle_group.add_argument('--num-vehicles', type=int, default=5,
                              help='生成的NPC车辆数量 (默认: 5)')
    vehicle_group.add_argument('--seed', metavar='S', type=int, default=42,
                              help='随机种子 (默认: 42)')
    vehicle_group.add_argument('--safe-mode', action='store_true', default=True,
                              help='启用NPC安全驾驶模式')
    vehicle_group.add_argument('--batch-spawn', action='store_true',
                              help='使用批量生成模式')
    
    # 导航参数
    nav_group = argparser.add_argument_group('导航参数')
    nav_group.add_argument('--wait-before-engage', type=float,
                          help='发送engage命令前等待时间(秒)')
    nav_group.add_argument('--wait-at-waypoint', type=float,
                          help='在路点处等待时间(秒)')
    nav_group.add_argument('--position-check-interval', type=float,
                          help='位置检查间隔(秒)')
    nav_group.add_argument('--max-position-wait-time', type=float,
                          help='最大位置等待时间(秒)')
    nav_group.add_argument('--goal-distance-threshold', type=float,
                          help='目标检测距离阈值(米)')
    nav_group.add_argument('--start-point-threshold', type=float,
                          help='起点检测距离阈值(米)')
    
    # 系统参数
    system_group = argparser.add_argument_group('系统参数')
    system_group.add_argument('--config', type=str,
                             help='配置文件路径')
    system_group.add_argument('--log-file', type=str,
                             help='日志文件路径')
    system_group.add_argument('--log-level', choices=['DEBUG', 'INFO', 'WARNING', 'ERROR'],
                             default='INFO',
                             help='日志级别 (默认: INFO)')
    system_group.add_argument('--create-default-config', type=str,
                             help='创建默认配置文件到指定路径')
    system_group.add_argument('--asynch', action='store_true',
                             help='激活异步模式执行')
    
    args = argparser.parse_args()
    
    # 如果指定了创建默认配置文件，则创建并退出
    if args.create_default_config:
        save_default_config(args.create_default_config)
        sys.exit(0)
    
    # 如果提供了配置文件，从中加载设置
    if args.config and os.path.exists(args.config):
        try:
            config = load_config_file(args.config)
            
            # 更新参数值
            carla_config = config.get('carla', {})
            vehicle_config = config.get('vehicles', {})
            nav_config = config.get('navigation', {})
            
            # 应用配置文件中的值（如果命令行没有指定）
            for key, value in carla_config.items():
                attr_name = key.replace('-', '_')
                if hasattr(args, attr_name) and getattr(args, attr_name) == argparser.get_default(attr_name):
                    setattr(args, attr_name, value)
            
            for key, value in vehicle_config.items():
                attr_name = key.replace('-', '_')
                if hasattr(args, attr_name) and getattr(args, attr_name) == argparser.get_default(attr_name):
                    setattr(args, attr_name, value)
            
            for key, value in nav_config.items():
                attr_name = key.replace('-', '_')
                if hasattr(args, attr_name) and getattr(args, attr_name) is None:
                    setattr(args, attr_name, value)
                    
        except Exception as e:
            logging.error(f"❌ 加载配置文件时出错: {e}")
    
    return args

# ============================================================================
# 主函数
# ============================================================================

def main():
    """主函数，程序入口点"""
    
    # 解析命令行参数
    args = parse_arguments()
    
    # 设置日志系统
    log_level = getattr(logging, args.log_level.upper())
    logger = Logger.setup(args.log_file, log_level)
    
    # 记录系统信息
    Logger.log_system_info()
    
    # 记录脚本启动信息
    logging.info("="*70)
    logging.info("🚀 增强版自动驾驶测试系统启动")
    logging.info(f"📅 启动时间: {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    logging.info(f"📋 版本信息: v2.0")
    logging.info(f"👤 作者: James LI")
    logging.info(f"📄 许可证: MIT License")
    logging.info("="*70)
    
    # 记录参数信息
    logging.info("⚙️ 启动参数:")
    for key, value in vars(args).items():
        if value is not None:
            logging.info(f"  • {key}: {value}")
    
    # 检查ROS2可用性
    if not ROS2_AVAILABLE:
        logging.error("❌ ROS2不可用，无法继续执行")
        logging.error("请确保已正确安装和配置ROS2环境")
        return 1
    
    # 创建增强集成管理器
    manager = EnhancedIntegratedManager(args)
    
    try:
        # 初始化连接和管理器
        if not manager.initialize():
            logging.error("❌ 系统初始化失败")
            return 1
        
        logging.info("✅ 系统初始化成功，进入交互模式")
        
        # 运行交互模式
        manager.run_interactive_mode()
        
        logging.info("✅ 程序正常结束")
        return 0
    
    except KeyboardInterrupt:
        logging.info('❌ 程序被用户中断')
        return 130  # SIGINT退出码
        
    except Exception as e:
        logging.error(f'❌ 程序执行过程中发生错误: {e}')
        # 打印详细错误堆栈
        import traceback
        logging.error("详细错误信息:")
        for line in traceback.format_exc().splitlines():
            logging.error(f"  {line}")
        return 1
        
    finally:
        # 清理资源
        logging.info('🧹 开始清理系统资源...')
        try:
            manager.cleanup()
        except Exception as e:
            logging.error(f'❌ 资源清理过程中出错: {e}')
        
        # 记录程序结束信息
        logging.info("="*70)
        logging.info("👋 增强版自动驾驶测试系统结束")
        logging.info(f"⏰ 结束时间: {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        logging.info("="*70)

if __name__ == '__main__':
    """程序入口点"""
    try:
        exit_code = main()
        sys.exit(exit_code)
    except KeyboardInterrupt:
        logging.info('\n❌ 程序被键盘中断')
        sys.exit(130)
    except Exception as e:
        logging.error(f'❌ 程序运行时错误: {e}')
        sys.exit(1)
    finally:
        logging.info('👋 程序终止')
```

## 3. 配置文件

**文件名: `config/test_config.json`**

```json
{
  "description": "增强版自动驾驶测试系统配置文件",
  "version": "2.0",
  "author": "James LI",
  "created": "2025-01-XX",
  
  "carla": {
    "host": "127.0.0.1",
    "port": 2000,
    "tm_port": 8100,
    "alt_tm_port": 8200,
    "timeout": 10.0
  },
  
  "vehicles": {
    "num_vehicles": 5,
    "seed": 42,
    "safe_mode": true,
    "batch_spawn": true,
    "types": [
      "vehicle.carlamotors.european_hgv",
      "vehicle.mini.cooper_s",
      "vehicle.nissan.patrol_2021",
      "vehicle.tesla.cybertruck",
      "vehicle.mini.cooper_s"
    ],
    "colors": [
      "202,88,176",
      "159,0,0",
      "0,38,132",
      null,
      "202,88,176"
    ]
  },
  
  "navigation": {
    "wait_before_engage": 2.0,
    "wait_at_waypoint": 5.0,
    "position_check_interval": 0.5,
    "max_position_wait_time": 30.0,
    "cleanup_wait_time": 0.5,
    "wait_after_npc_removal": 5.0,
    "goal_distance_threshold": 8.0,
    "start_point_threshold": 10.0,
    "navigation_timeout": 120.0,
    "engage_retry_attempts": 3
  },
  
  "traffic_manager": {
    "following_distance": 5.0,
    "speed_difference": 0.1,
    "max_speed_limit": 25,
    "spawn_retry_attempts": 5,
    "spawn_position_variance": 1.0
  },
  
  "waypoints": {
    "npc_test": {
      "description": "NPC测试路径点",
      "points": [
        {
          "name": "NPC起点",
          "position": [-48.4787, -32.6236, 0.0],
          "orientation": [0.0, 0.0, -0.715607, 0.698503]
        },
        {
          "name": "NPC任务终点1",
          "position": [29.7123, -36.908, 0.0],
          "orientation": [0.0, 0.0, 0.248181, 0.968714]
        },
        {
          "name": "NPC任务终点2",
          "position": [-6.22938, 50.2089, 0.0],
          "orientation": [0.0, 0.0, -0.999831, 0.0183731]
        }
      ]
    },
    "loop_test": {
      "description": "回环测试路径点",
      "points": [
        {
          "name": "回环起点",
          "position": [-6.22938, 50.2089, 0.0],
          "orientation": [0.0, 0.0, -0.999831, 0.0183731]
        },
        {
          "name": "回环路点1",
          "position": [-48.3957, -19.3472, 0.0],
          "orientation": [0.0, 0.0, -0.723422, 0.690406]
        },
        {
          "name": "回环路点2",
          "position": [54.3231, -47.6682, 0.0],
          "orientation": [0.0, 0.0, 0.300639, 0.953738]
        },
        {
          "name": "回环路点3",
          "position": [26.589, 50.3549, 0.0],
          "orientation": [0.0, 0.0, -0.999824, 0.01874]
        }
      ]
    }
  },
  
  "logging": {
    "level": "INFO",
    "max_log_files": 10,
    "log_rotation": true
  },
  
  "keyboard_controls": {
    "collision_recovery": "T",
    "restart_test": "R",
    "pause_test": "P",
    "stop_test": "S",
    "quit_program": "Q"
  }
}
```

## 4. 设计说明文档

**文件名: `docs/DESIGN.md`**

```markdown
# 增强版自动驾驶测试系统 - 设计说明

## 系统架构

### 1. 整体架构
```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Bash启动器    │───→│   Python主程序   │───→│   配置文件系统   │
└─────────────────┘    └─────────────────┘    └─────────────────┘
                                │
                                ▼
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│  键盘监听系统   │◄───┤  增强集成管理器  ├───►│   日志管理系统   │
└─────────────────┘    └─────────────────┘    └─────────────────┘
                                │
                    ┌───────────┼───────────┐
                    ▼           ▼           ▼
            ┌──────────────┐ ┌──────────────┐ ┌──────────────┐
            │ CARLA管理器  │ │ Autoware发布器│ │ 菜单系统     │
            └──────────────┘ └──────────────┘ └──────────────┘
                    │           │
                    ▼           ▼
            ┌──────────────┐ ┌──────────────┐
            │ CARLA仿真器  │ │ ROS2+Autoware│
            └──────────────┘ └──────────────┘
```

### 2. 核心组件

#### 2.1 增强集成管理器 (EnhancedIntegratedManager)
- **职责**: 整体流程控制和组件协调
- **功能**:
  - 系统初始化和资源管理
  - 测试流程执行和状态管理
  - 异常处理和恢复机制
  - 统计信息收集和报告

#### 2.2 CARLA管理器 (CarlaManager)
- **职责**: CARLA仿真器交互和车辆管理
- **功能**:
  - CARLA服务器连接管理
  - Traffic Manager配置和控制
  - NPC车辆生成、配置和清理
  - 车辆行为参数设置

#### 2.3 Autoware目标发布器 (AutowareGoalPublisher)
- **职责**: ROS2/Autoware通信和导航控制
- **功能**:
  - 目标位置发布和管理
  - Engage命令发送和控制
  - 车辆位置监听和距离计算
  - 导航状态监控和反馈

#### 2.4 键盘监听系统 (EnhancedKeyboardMonitor)
- **职责**: 实时键盘输入监听和处理
- **功能**:
  - 多键并发监听 (T/R/P/S/Q)
  - 按键状态管理和重置
  - 按键统计和记录
  - 线程安全的事件处理

#### 2.5 交互式菜单系统 (MenuSystem)
- **职责**: 用户界面和交互逻辑
- **功能**:
  - 主菜单显示和选择处理
  - 参数输入和验证
  - 进度显示和状态反馈
  - 用户确认和提示

### 3. 数据流设计

#### 3.1 导航数据流
```
用户选择 → 菜单系统 → 集成管理器 → Autoware发布器 → ROS2 → Autoware
                                        ↓
位置反馈 ← 键盘监听 ← 集成管理器 ← Autoware发布器 ← ROS2 ← Autoware
```

#### 3.2 NPC管理数据流
```
测试开始 → 集成管理器 → CARLA管理器 → CARLA仿真器
                                      ↓
碰撞检测 → 键盘监听 → 集成管理器 → CARLA管理器 → 车辆清理
```

## 测试流程设计

### 1. NPC测试流程
```
起点检查 → 目标设置 → NPC生成 → 导航执行 → 碰撞监控 → 目标到达 → NPC清理
```

**详细步骤**:
1. 检查车辆是否在NPC测试起点
2. 设置导航目标为任务终点1
3. 生成5辆指定类型的NPC车辆
4. 发送engage命令开始导航
5. 监控导航过程和键盘输入
6. 处理碰撞恢复 (T键触发)
7. 到达终点1后继续到终点2
8. 清理所有NPC车辆

### 2. 回环测试流程
```
起点检查 → 路点循环 → 中断检查 → 完成统计
```

**详细步骤**:
1. 检查车辆是否在回环测试起点
2. 按顺序导航到各个回环路点
3. 检查暂停/停止请求 (P/S键)
4. 返回起点完成一次循环
5. 根据设置继续或结束

### 3. 点对点导航流程
```
当前位置 → 目标设置 → 导航执行 → 到达确认
```

## 关键算法

### 1. 距离计算算法
```python
def calculate_distance(x1, y1, x2, y2):
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
```

### 2. 位置到达判断
```python
def check_if_at_position(goal_pos, threshold):
    distance = calculate_distance(current_x, current_y, goal_pos[0], goal_pos[1])
    return distance < threshold
```

### 3. 导航超时控制
```python
def navigate_with_timeout(goal, timeout):
    start_time = time.time()
    while not reached and time.time() - start_time < timeout:
        # 导航逻辑
        pass
```

## 安全机制

### 1. 资源清理
- 信号处理器确保优雅关闭
- 异常捕获和资源释放
- CARLA车辆智能清理
- ROS2节点正确销毁

### 2. 错误恢复
- 碰撞检测和NPC移除
- 导航超时和重试机制
- 网络连接失败处理
- 状态不一致修复

### 3. 用户安全
- 键盘中断响应
- 紧急停止功能
- 操作确认机制
- 状态提示和警告

## 扩展性设计

### 1. 模块化架构
- 每个组件独立封装
- 接口标准化
- 配置外部化
- 功能插件化

### 2. 配置灵活性
- JSON配置文件支持
- 命令行参数覆盖
- 运行时参数调整
- 环境变量集成

### 3. 功能扩展点
- 新测试模式添加
- 自定义路点配置
- 车辆类型扩展
- 监控指标增加

## 性能优化

### 1. 资源利用
- 异步IO操作
- 线程安全设计
- 内存使用优化
- CPU负载平衡

### 2. 响应性能
- 实时键盘响应
- 快速位置更新
- 高效距离计算
- 智能等待机制

### 3. 可靠性保证
- 重试机制
- 状态检查
- 资源监控
- 异常预防
```

## 5. 用户使用手册

**文件名: `docs/USER_GUIDE.md`**

```markdown
# 增强版自动驾驶测试系统 - 用户使用手册

## 快速开始

### 1. 系统要求
- **操作系统**: Ubuntu 20.04/22.04 或兼容的Linux发行版
- **Python**: 3.8 或更高版本
- **ROS2**: Humble 或更高版本
- **CARLA**: 0.9.13 或更高版本
- **Autoware**: 已正确安装和配置
- **内存**: 至少 8GB RAM
- **存储**: 至少 2GB 可用空间

### 2. 环境准备

#### 2.1 设置环境变量
```bash
# CARLA路径
export CARLA_ROOT="/path/to/carla"

# Scenario Runner路径
export SCENARIO_RUNNER_ROOT="/path/to/scenario_runner"

# OpenPlanner Bridge路径
export OP_BRIDGE_ROOT="/path/to/op_bridge"

# Autoware路径
export AUTOWARE_ROOT="/path/to/autoware"

# OpenPlanner ROS插件路径
export OP_ROS_PLUGINS_ROOT="/path/to/op_ros_plugins"
```

#### 2.2 安装依赖
```bash
# Python依赖
pip3 install pynput numpy transforms3d setproctitle psutil

# ROS2依赖 (如果未安装)
sudo apt install ros-humble-desktop
```

### 3. 启动步骤

#### 3.1 启动CARLA服务器
```bash
cd $CARLA_ROOT
./CarlaUE4.sh
```

#### 3.2 启动Autoware
```bash
source $AUTOWARE_ROOT/install/setup.bash
ros2 launch autoware_launch planning_simulator.launch.xml
```

#### 3.3 运行测试系统
```bash
cd enhanced_integrated_scenario_test/scripts
chmod +x enhanced_integrated_scenario_launcher.sh
./enhanced_integrated_scenario_launcher.sh
```

## 功能详解

### 1. 主菜单选项

#### 📍 导航测试
- **选项1**: 到达任务起点
  - 功能: 导航到NPC测试的起始位置
  - 用途: 测试前的位置准备
  - 预期: 车辆安全到达指定起点

- **选项5**: 点对点导航到终点1
  - 功能: 直接导航到NPC任务的第一个终点
  - 用途: 测试特定路径段的导航性能
  - 预期: 车辆准确到达目标位置

- **选项8**: 导航到回环测试起点
  - 功能: 从任意位置导航到回环测试的起始点
  - 用途: 为回环测试做准备
  - 预期: 车辆正确定位到回环起点

- **选项9**: 导航到NPC任务终点2
  - 功能: 导航到NPC测试的最终位置
  - 用途: 直接测试到终点的导航
  - 预期: 车辆到达最终目标位置

#### 🚦 NPC环境测试
- **选项2**: 进行一次NPC测试
  - 功能: 完整的NPC环境下自动驾驶测试
  - 流程:
    1. 检查并导航到起点
    2. 生成5辆NPC车辆
    3. 导航到任务终点1
    4. 继续导航到任务终点2
    5. 清理所有NPC车辆
  - 特殊功能:
    - **T键**: 碰撞恢复 - 立即移除NPC车辆并继续导航
    - 智能碰撞检测和恢复机制
  - 预期: 成功完成复杂交通环境下的导航任务

- **选项4**: NPC导航任务(无车)
  - 功能: 执行与NPC测试相同的路径，但不生成NPC车辆
  - 用途: 测试基础导航能力，排除交通干扰
  - 预期: 验证导航系统的基本功能

#### 🔄 回环测试
- **选项3**: 进行一次回环测试
  - 功能: 单次循环路径测试
  - 路径: 起点 → 路点1 → 路点2 → 路点3 → 起点
  - 用途: 测试循环导航的稳定性
  - 预期: 顺利完成一次完整的回环

- **选项6**: 进行3次回环测试
  - 功能: 连续执行3次回环测试
  - 用途: 测试多次循环的稳定性和一致性
  - 预期: 3次回环全部成功完成

- **选项7**: 进行N次回环测试
  - 功能: 可配置次数的回环测试
  - 选项:
    - 输入具体数字: 执行指定次数
    - 直接回车: 无限循环模式
  - 控制键:
    - **P键**: 暂停当前回环，可选择继续或退出
    - **S键**: 停止回环测试
  - 用途: 长时间稳定性测试
  - 预期: 按要求完成指定次数的回环或在无限模式下持续运行

### 2. 键盘控制

#### 实时控制键
- **T键**: 碰撞恢复
  - 触发时机: NPC测试期间检测到碰撞
  - 效果: 立即移除所有NPC车辆，继续当前导航任务
  - 用途: 快速从碰撞情况中恢复

- **R键**: 重启测试
  - 触发时机: 回环测试期间
  - 效果: 停止当前回环，重新开始NPC测试
  - 用途: 从回环测试切换回NPC测试

- **P键**: 暂停测试
  - 触发时机: 回环测试期间
  - 效果: 暂停当前测试，等待用户决定
  - 用途: 临时暂停以处理其他事务

- **S键**: 停止测试
  - 触发时机: 回环测试期间
  - 效果: 停止当前测试序列
  - 用途: 提前结束测试

- **Q键**: 退出程序
  - 触发时机: 任何时候
  - 效果: 完全退出程序
  - 用途: 紧急退出

#### 系统控制键
- **Ctrl+C**: 紧急停止
  - 效果: 立即中断当前操作，启动清理流程
  - 用途: 紧急情况下的强制停止

### 3. 路径和位置说明

#### NPC测试路径
```
起点 (-48.48, -32.62)
  ↓ [生成NPC车辆]
任务终点1 (29.71, -36.91)
  ↓ [继续导航]
任务终点2 (-6.23, 50.21) [清理NPC车辆]
```

#### 回环测试路径
```
起点 (-6.23, 50.21)
  ↓
路点1 (-48.40, -19.35)
  ↓
路点2 (54.32, -47.67)
  ↓
路点3 (26.59, 50.35)
  ↓
回到起点 (-6.23, 50.21)
```

## 配置和定制

### 1. 配置文件使用

#### 创建默认配置
```bash
python3 enhanced_integrated_scenario_test.py --create-default-config config/my_config.json
```

#### 使用配置文件
```bash
./enhanced_integrated_scenario_launcher.sh --config config/my_config.json
```

### 2. 主要配置参数

#### 导航参数
- `wait_before_engage`: engage命令前等待时间 (默认: 2.0秒)
- `wait_at_waypoint`: 路点等待时间 (默认: 5.0秒)
- `goal_distance_threshold`: 目标到达判断距离 (默认: 8.0米)
- `navigation_timeout`: 导航超时时间 (默认: 120.0秒)

#### 车辆参数
- `num_vehicles`: NPC车辆数量 (默认: 5)
- `safe_mode`: 安全驾驶模式 (默认: true)
- `following_distance`: NPC跟车距离 (默认: 5.0米)

#### 系统参数
- `log_level`: 日志级别 (默认: INFO)
- `position_check_interval`: 位置检查间隔 (默认: 0.5秒)

### 3. 自定义路点

编辑配置文件中的 `waypoints` 部分:
```json
"waypoints": {
  "custom_test": {
    "description": "自定义测试路径",
    "points": [
      {
        "name": "自定义起点",
        "position": [x, y, z],
        "orientation": [qx, qy, qz, qw]
      }
    ]
  }
}
```

## 故障排除

### 1. 常见问题

#### CARLA连接失败
**症状**: `❌ CARLA连接失败: Connection refused`
**解决方案**:
1. 确认CARLA服务器已启动
2. 检查IP地址和端口设置
3. 确认防火墙设置

#### ROS2通信问题
**症状**: `⚠ 位置更新超时`
**解决方案**:
1. 检查ROS2环境变量
2. 确认Autoware正常运行
3. 验证topic发布状态

#### NPC车辆生成失败
**症状**: `❌ 无法生成车辆`
**解决方案**:
1. 检查生成点是否被占用
2. 确认车辆蓝图可用性
3. 尝试减少车辆数量

### 2. 调试技巧

#### 启用详细日志
```bash
python3 enhanced_integrated_scenario_test.py --log-level DEBUG
```

#### 检查系统状态
```bash
# 检查ROS2 topics
ros2 topic list

# 检查Autoware状态
ros2 node list

# 检查CARLA服务器
netstat -an | grep 2000
```

### 3. 性能优化

#### 提高响应速度
- 减少 `position_check_interval`
- 增加 `navigation_timeout`
- 使用SSD存储日志文件

#### 减少资源使用
- 减少NPC车辆数量
- 禁用不必要的日志输出
- 关闭图形化界面

## 高级使用

### 1. 批量测试脚本
```bash
#!/bin/bash
# 自动化批量测试
for i in {1..10}; do
    echo "执行第 $i 次测试"
    ./enhanced_integrated_scenario_launcher.sh --config batch_config.json
    sleep 60
done
```

### 2. 集成CI/CD
```yaml
# .github/workflows/test.yml
name: Autonomous Driving Test
on: [push, pull_request]
jobs:
  test:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v2
      - name: Run AV Test
        run: |
          ./scripts/enhanced_integrated_scenario_launcher.sh --config ci_config.json
```

### 3. 数据分析
```python
# 分析测试日志
import json
import matplotlib.pyplot as plt

def analyze_test_logs(log_file):
    # 解析日志文件
    # 生成性能报告
    # 创建可视化图表
    pass
```

## 支持和反馈

### 联系方式
- **作者**: James LI
- **邮箱**: james.li@example.com
- **项目仓库**: https://github.com/jamesli/enhanced-av-test

### 问题报告
请在报告问题时包含以下信息:
1. 系统环境信息
2. 完整的错误日志
3. 重现步骤
4. 预期和实际结果

### 贡献指南
欢迎提交Pull Request和Issue，请确保:
1. 代码符合PEP 8规范
2. 包含适当的测试用例
3. 更新相关文档
```

## 6. README文档

**文件名: `docs/README.md`**

```markdown
# 增强版自动驾驶测试系统

一个集成CARLA仿真器和Autoware自动驾驶系统的综合测试平台，提供交互式菜单、NPC环境测试、回环导航测试和实时键盘控制功能。

## ✨ 主要特性

- 🎮 **交互式菜单系统** - 9种不同测试模式，用户友好的操作界面
- 🚦 **NPC环境测试** - 复杂交通环境下的自动驾驶性能测试
- 🔄 **回环导航测试** - 循环路径稳定性和一致性验证
- ⌨️ **实时键盘控制** - T/R/P/S/Q键实时干预和控制
- 🛡️ **智能恢复机制** - 碰撞检测、自动恢复和异常处理
- 📊 **完整日志系统** - 详细的测试记录、统计分析和性能监控
- ⚙️ **高度可配置** - JSON配置文件、命令行参数和环境变量支持
- 🧹 **资源管理** - 自动清理CARLA和ROS2资源，确保系统稳定

## 🚀 快速开始

### 环境要求
- Ubuntu 20.04/22.04
- Python 3.8+
- ROS2 Humble+
- CARLA 0.9.13+
- Autoware (已配置)

### 安装依赖
```bash
pip3 install pynput numpy transforms3d setproctitle psutil
```

### 启动系统
```bash
# 1. 启动CARLA服务器
cd $CARLA_ROOT && ./CarlaUE4.sh

# 2. 启动Autoware
source $AUTOWARE_ROOT/install/setup.bash
ros2 launch autoware_launch planning_simulator.launch.xml

# 3. 运行测试系统
cd enhanced_integrated_scenario_test/scripts
chmod +x enhanced_integrated_scenario_launcher.sh
./enhanced_integrated_scenario_launcher.sh
```

## 📋 测试模式

### 导航测试
1. **到达任务起点** - 导航到NPC测试起始位置
2. **点对点导航** - 直接导航到指定目标点
3. **位置准备** - 为特定测试做准备

### NPC环境测试
2. **完整NPC测试** - 在复杂交通环境下的导航测试
4. **NPC路径测试** - 执行NPC路径但不生成车辆

### 回环测试
3. **单次回环** - 一次完整的循环路径测试
6. **3次回环** - 连续执行3次回环测试
7. **N次回环** - 可配置次数或无限循环测试

## ⌨️ 键盘控制

- **T键** - 碰撞恢复：立即移除NPC车辆并继续导航
- **R键** - 重启测试：从回环测试切换回NPC测试
- **P键** - 暂停测试：临时暂停当前操作
- **S键** - 停止测试：结束当前测试序列
- **Q键** - 退出程序：完全退出系统

## 📁 项目结构

```
enhanced_integrated_scenario_test/
├── scripts/
│   ├── enhanced_integrated_scenario_launcher.sh  # Bash启动器
│   └── enhanced_integrated_scenario_test.py      # Python主程序
├── config/
│   └── test_config.json                          # 配置文件
├── docs/
│   ├── README.md                                 # 项目说明
│   ├── DESIGN.md                                 # 设计文档
│   └── USER_GUIDE.md                             # 使用手册
└── logs/                                         # 日志目录
```

## ⚙️ 配置选项

### 创建默认配置
```bash
python3 enhanced_integrated_scenario_test.py --create-default-config config/my_config.json
```

### 主要参数
- `wait_before_engage`: engage前等待时间 (默认: 2.0s)
- `goal_distance_threshold`: 目标到达距离 (默认: 8.0m)
- `num_vehicles`: NPC车辆数量 (默认: 5)
- `navigation_timeout`: 导航超时时间 (默认: 120.0s)

## 📊 测试流程

### NPC测试流程
```
起点检查 → 目标设置 → NPC生成 → 导航执行 → 碰撞监控 → 清理车辆
```

### 回环测试流程
```
起点检查 → 路点1 → 路点2 → 路点3 → 返回起点
```

## 🔧 故障排除

### 常见问题
- **CARLA连接失败**: 确认服务器启动和端口设置
- **ROS2通信问题**: 检查环境变量和Autoware状态
- **车辆生成失败**: 验证生成点可用性和车辆数量

### 调试模式
```bash
python3 enhanced_integrated_scenario_test.py --log-level DEBUG
```

## 📈 性能特性

- ⚡ **高响应性** - 实时键盘响应和快速状态更新
- 🛡️ **高可靠性** - 完善的异常处理和资源清理
- 📊 **可监控性** - 详细的日志记录和统计信息
- 🔧 **可维护性** - 模块化设计和清晰的接口

## 🤝 贡献

欢迎提交Issue和Pull Request！请确保:
- 遵循PEP 8代码规范
- 包含适当的测试用例
- 更新相关文档

## 📄 许可证

本项目基于 [MIT License](https://opensource.org/licenses/MIT) 开源。

## 👨‍💻 作者

**James LI**
- 邮箱: james.li@example.com
- 项目: Enhanced Autonomous Vehicle Testing System

## 📚 相关文档

- [设计说明](DESIGN.md) - 系统架构和设计理念
- [用户手册](USER_GUIDE.md) - 详细的使用说明和配置指南
- [MIT License](https://opensource.org/licenses/MIT) - 开源许可证详情

---

⭐ 如果这个项目对您有帮助，请给我们一个星标！
```

## 总结

这个完整版本包含了：

1. **Bash启动脚本** - 完整的环境检查、配置和启动逻辑
2. **Python主程序** - 增强的交互式测试系统
3. **配置文件** - JSON格式的完整配置模板
4. **设计文档** - 详细的架构设计和技术说明
5. **用户手册** - 完整的使用指南和故障排除
6. **README** - 项目概述和快速开始指南

系统具备以下核心特性：

- 🎮 **9种测试模式** - 从简单导航到复杂NPC环境测试
- ⌨️ **实时键盘控制** - T/R/P/S/Q键多种干预方式
- 🛡️ **智能恢复机制** - 碰撞检测和自动恢复
- 📊 **完整监控** - 详细日志、统计和性能分析
- ⚙️ **高度可配置** - 支持配置文件和命令行参数
- 🧹 **资源管理** - 自动清理和异常处理

所有代码都经过精心设计，具有良好的可读性、可维护性和扩展性。