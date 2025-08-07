#!/bin/bash

# FSM Sandbox 验证实验启动器 - 精简修正版
# 基于"修复和复用"原则，统一配置，最小改动

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 颜色定义
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

# 清理函数
cleanup() {
    print_colored $YELLOW "\n清理验证实验进程..."
    
    # 停止数据收集进程
    for pid in $A1_PID $A2_PID $B_ERROR_PID; do
        if [ ! -z "$pid" ]; then
            kill $pid 2>/dev/null || true
        fi
    done
    
    sleep 2
    
    # 停止特定节点
    pkill -f "log_recorder_node" 2>/dev/null || true
    pkill -f "ground_truth_recorder" 2>/dev/null || true
    pkill -f "tracking_error_measurer" 2>/dev/null || true
    
    print_colored $GREEN "实验清理完成"

    # 自动化分析
    print_header "自动化实验分析"

    # 1. 分析V2R延迟 (来自PC控制器日志)
    PC_CONTROLLER_LOG_DIR="$SCRIPT_DIR/../_Sys/log"
    TIMING_LOG=$(ls -t "$PC_CONTROLLER_LOG_DIR/timing_log_"*.csv 2>/dev/null | head -1)

    if [ -z "$TIMING_LOG" ]; then
        print_colored $YELLOW "警告: 未找到PC控制器时序日志，跳过V2R分析"
        V2R_CSV=""
    else
        print_colored $GREEN "找到V2R时序日志: $TIMING_LOG"
        print_colored $BLUE "分析V2R延迟..."
        python3 "$SCRIPT_DIR/v2r_latency_analyzer.py" "$TIMING_LOG"
        V2R_CSV=$(ls -t "$SCRIPT_DIR/data/v2r_latency_"*.csv 2>/dev/null | head -1)
    fi

    # 2. 查找数据文件
    R2V_TIMING_CSV=$(ls -t "$SCRIPT_DIR/data/r2v_timing_data_"*.csv 2>/dev/null | head -1)
    TRACKING_ERROR_CSV=$(ls -t "$SCRIPT_DIR/data/tracking_error_"*.csv 2>/dev/null | head -1)

    print_colored $BLUE "数据文件检查:"
    print_colored $GREEN "  R2V延迟数据 (A1): ${R2V_TIMING_CSV:-未找到}"
    print_colored $GREEN "  V2R延迟数据: ${V2R_CSV:-未找到}"
    print_colored $GREEN "  跟踪误差数据: ${TRACKING_ERROR_CSV:-未找到}"

    # 3. 执行A1分析
    if [ -f "$R2V_TIMING_CSV" ]; then
        print_colored $BLUE "运行A1时效性分析..."
        python3 "$SCRIPT_DIR/analyze_timing_data.py" "$R2V_TIMING_CSV"
        print_colored $GREEN "A1分析完成"
    fi

    # 4. 执行B实验综合分析
    if [ -f "$R2V_TIMING_CSV" ] && [ -f "$TRACKING_ERROR_CSV" ] && [ -f "$V2R_CSV" ]; then
        print_colored $BLUE "运行B实验综合分析..."
        python3 "$SCRIPT_DIR/experiment_b_analyzer.py" \
            "$R2V_TIMING_CSV" \
            "$V2R_CSV" \
            "$TRACKING_ERROR_CSV"
        
        ANALYSIS_DIR=$(ls -td "$SCRIPT_DIR/results/experiment_b_"* 2>/dev/null | head -1)
        print_colored $GREEN "B实验分析完成! 结果位置: $ANALYSIS_DIR"
        
    elif [ -f "$R2V_TIMING_CSV" ] && [ -f "$TRACKING_ERROR_CSV" ]; then
        print_colored $YELLOW "部分B实验分析 (缺少V2R数据)..."
        # 可以创建一个简化版本的分析
        print_colored $YELLOW "建议: 检查PC控制器日志配置"
        
    else
        print_colored $RED "错误: B实验数据不完整，无法进行综合分析"
        echo "  需要文件:"
        echo "    - R2V延迟数据: ${R2V_TIMING_CSV:-缺失}"
        echo "    - 跟踪误差数据: ${TRACKING_ERROR_CSV:-缺失}"
        echo "    - V2R延迟数据: ${V2R_CSV:-缺失}"
    fi

    # 5. A2实验提示
    GROUND_TRUTH_CSV=$(ls -t "$SCRIPT_DIR/data/ground_truth_poses_"*.csv 2>/dev/null | head -1)
    if [ -f "$GROUND_TRUTH_CSV" ]; then
        print_colored $BLUE "A2保真度分析指南:"
        echo "  1. 停止当前系统"
        echo "  2. 使用录制的LiDAR数据进行离线NDT处理"
        echo "  3. 运行: python3 analyze_fidelity_data.py $GROUND_TRUTH_CSV ndt_offline_results.csv"
    fi
}

trap cleanup EXIT INT TERM

print_header "FSM Sandbox 验证实验套件启动器 (精简版)"

# 检查配置文件
CONFIG_FILE="$SCRIPT_DIR/config.yaml"
if [ ! -f "$CONFIG_FILE" ]; then
    print_colored $RED "错误: 未找到配置文件 $CONFIG_FILE"
    exit 1
fi

print_colored $GREEN "使用配置文件: $CONFIG_FILE"

# 检查环境
source /opt/ros/humble/setup.bash 2>/dev/null || true

if ! command -v ros2 &> /dev/null; then
    print_colored $RED "错误: 未找到ROS2环境"
    exit 1
fi

# 环境设置
print_header "环境设置"
source "$SCRIPT_DIR/setup_env.sh" --no-install
print_colored $GREEN "环境设置完成"

# print_header "启动验证实验数据收集"
# SESSION_TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
# print_colored $GREEN "会话时间戳: $SESSION_TIMESTAMP"

# # 启动A1实验 (时效性验证)
# print_colored $BLUE "启动A1实验 (时效性验证)..."
# (cd "$SCRIPT_DIR" && python3 log_recorder_node.py --config "$CONFIG_FILE") &
# A1_PID=$!
# print_colored $GREEN "  A1数据收集器已启动 (PID: $A1_PID)"

# # 启动A2实验 (保真度验证)
# print_colored $BLUE "启动A2实验 (保真度验证)..."
# (cd "$SCRIPT_DIR" && python3 ground_truth_recorder.py --config "$CONFIG_FILE") &
# A2_PID=$!
# print_colored $GREEN "  A2数据收集器已启动 (PID: $A2_PID)"

# # 启动B实验 (跟踪误差测量)
# print_colored $BLUE "启动B实验 (跟踪误差测量)..."
# (cd "$SCRIPT_DIR" && python3 tracking_error_measurer.py --config "$CONFIG_FILE") &
# B_ERROR_PID=$!
# print_colored $GREEN "  B实验数据收集器已启动 (PID: $B_ERROR_PID)"

# print_header "数据收集监控和用户指南"
# print_colored $GREEN "所有验证实验节点正在运行!"
# print_colored $YELLOW "测试协议:"
# echo "  阶段1 (0-5分钟): 使用VIL系统进行正常导航测试"
# echo "  阶段2 (5-8分钟): 将车辆定位到直线测试路径起点"
# echo "  阶段3 (8-13分钟): 以恒定0.5 m/s速度直线行驶 (基线误差测试)"
# echo "  阶段4: 在此终端按Ctrl+C停止实验并触发自动分析"

# print_colored $BLUE "\n监控实验进程... 按Ctrl+C停止"
print_header "启动智能验证实验数据收集"
SESSION_TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
print_colored $GREEN "会话时间戳: $SESSION_TIMESTAMP"

# 启动A1实验 (时效性验证)
print_colored $BLUE "启动A1实验 (时效性验证)..."
(cd "$SCRIPT_DIR" && python3 log_recorder_node.py --config "$CONFIG_FILE") &
A1_PID=$!
print_colored $GREEN "  A1数据收集器已启动 (PID: $A1_PID)"

# 启动A2实验 (保真度验证)
print_colored $BLUE "启动A2实验 (保真度验证)..."
(cd "$SCRIPT_DIR" && python3 ground_truth_recorder.py --config "$CONFIG_FILE") &
A2_PID=$!
print_colored $GREEN "  A2数据收集器已启动 (PID: $A2_PID)"

# 启动B实验 (智能跟踪误差测量)
print_colored $BLUE "启动B实验 (智能跟踪误差测量)..."
(cd "$SCRIPT_DIR" && python3 intelligent_tracking_error_measurer.py --config "$CONFIG_FILE") &
B_ERROR_PID=$!
print_colored $GREEN "  B实验智能数据收集器已启动 (PID: $B_ERROR_PID)"

print_header "智能数据收集监控"
print_colored $GREEN "所有验证实验节点正在运行!"
print_colored $YELLOW "智能测试协议:"
echo "  • 系统将自动检测您的导航过程中的直线行驶段"
echo "  • 当检测到速度稳定的直线行驶时，自动记录基线误差数据"
echo "  • 无需手动控制测试阶段，只需正常使用VIL系统进行导航"
echo "  • 系统会智能识别适合基线分析的路段"
echo "  • 在此终端按Ctrl+C停止实验并触发自动分析"

print_colored $BLUE "\n监控智能实验进程... 按Ctrl+C停止"

# 监控进程
while true; do
    for pid in $A1_PID $A2_PID $B_ERROR_PID; do
        if ! kill -0 $pid 2>/dev/null; then
            print_colored $RED "实验进程 (PID $pid) 意外停止! 检查日志: $LOG_FILE"
            exit 1
        fi
    done
    sleep 5
done