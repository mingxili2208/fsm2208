#!/bin/bash

# =========================================
# 自动化启动 CARLA 及相关服务的 Bash 脚本
# =========================================

# 日志函数，带时间戳
log() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1"
}

# 等待服务启动的函数（简单等待，可根据需要优化）
wait_for_service() {
    SERVICE_NAME=$1
    WAIT_TIME=$2
    log "等待 $SERVICE_NAME 启动..."
    sleep $WAIT_TIME
    log "$SERVICE_NAME 应该已经启动。"
}

# =========================================
# 步骤1：启动 CARLA
# =========================================
start_carla() {
    CARLA_ROOT="/path/to/your/carla_root"  # 请替换为您的 CARLA 根目录路径
    log "启动 CARLA 模拟器..."
    cd "$CARLA_ROOT" || { log "无法进入 CARLA 根目录。"; exit 1; }
    make launch > "$CARLA_ROOT/carla_launch.log" 2>&1 &
    log "CARLA 正在后台启动。日志位于 $CARLA_ROOT/carla_launch.log。"

    # 等待 CARLA 初始化（根据实际情况调整等待时间）
    wait_for_service "CARLA" 15

    # 自动点击播放按钮（需要安装 xdotool）
    if command -v xdotool &> /dev/null; then
        log "尝试自动点击 CARLA 播放按钮..."
        sleep 10  # 等待 CARLA 窗口完全加载
        WINDOW_ID=$(xdotool search --name "Carla" | head -1)
        if [ -n "$WINDOW_ID" ]; then
            # 根据实际情况调整鼠标坐标
            xdotool mousemove --window "$WINDOW_ID" 500 500 click 1
            log "已在 CARLA 窗口点击播放按钮。"
        else
            log "未找到 CARLA 窗口，请手动点击播放按钮。"
        fi
    else
        log "未安装 xdotool，请手动点击 CARLA 播放按钮。"
    fi
}

# =========================================
# 步骤2：启动 SteamVR
# =========================================
start_steamvr() {
    log "启动 SteamVR..."
    steamvr > /tmp/steamvr.log 2>&1 &
    log "SteamVR 正在后台启动。日志位于 /tmp/steamvr.log。"

    # 等待 SteamVR 初始化
    wait_for_service "SteamVR" 20
}

# =========================================
# 步骤3：启动 VR Pose Transmit
# =========================================
start_vr_pose_transmit() {
    log "启动 VR Pose Transmit..."
    VR_POSE_DIR="/home/cityu-fsm-lab-carla/lmx/Docs/packages/autoware_comm/bash"
    INITIAL_CONFIG="$VR_POSE_DIR/initial_config.bash"
    DEBUG_EKF="$VR_POSE_DIR/../scripts/debug/debug_ekf.py"

    # 加载初始配置
    source "$INITIAL_CONFIG" || { log "无法加载 $INITIAL_CONFIG。"; exit 1; }

    # 运行 debug_ekf.py 脚本
    python3 "$DEBUG_EKF" > /home/cityu-fsm-lab-carla/lmx/Docs/packages/autoware_comm/scripts/debug/debug_ekf.log 2>&1 &
    log "VR Pose Transmit 已在后台启动。日志位于 debug_ekf.log。"
}

# =========================================
# 步骤4：启动 Serial Transmit
# =========================================
start_serial_transmit() {
    log "启动 Serial Transmit..."

    SERIAL_PORT="/dev/ttyUSB0"
    if [ ! -e "$SERIAL_PORT" ]; then
        log "串口 $SERIAL_PORT 未找到，请连接设备。"
        exit 1
    fi

    # 设置串口权限
    sudo chmod 777 "$SERIAL_PORT" || { log "无法设置 $SERIAL_PORT 的权限。"; exit 1; }

    SERIAL_TRANSMIT_SCRIPT="/home/cityu-fsm-lab-carla/lmx/Docs/packages/autoware_comm/scripts/debug/debug_ros2_serial.py"
    cd "/home/cityu-fsm-lab-carla/lmx/Docs/packages/autoware_comm/bash" || { log "无法进入 bash 目录。"; exit 1; }
    source initial_config.bash || { log "无法加载 initial_config.bash。"; exit 1; }

    # 运行 debug_ros2_serial.py 脚本
    python3 "$SERIAL_TRANSMIT_SCRIPT" > /home/cityu-fsm-lab-carla/lmx/Docs/packages/autoware_comm/scripts/debug/debug_ros2_serial.log 2>&1 &
    log "Serial Transmit 已在后台启动。日志位于 debug_ros2_serial.log。"
}

# =========================================
# 步骤5：启动 Autoware Vehicle Agent
# =========================================
start_autoware_vehicle_agent() {
    log "启动 Autoware Vehicle Agent..."
    AUTOWARE_SCRIPT_DIR="/home/cityu-fsm-lab-carla/Workspace/Carla/op_carla/op_bridge/op_scripts/fsm_lab_simulation/"
    AUTOWARE_SCRIPT="$AUTOWARE_SCRIPT_DIR/debug_run_vehicle_ros2.sh"

    if [ ! -x "$AUTOWARE_SCRIPT" ]; then
        log "Autoware 脚本 $AUTOWARE_SCRIPT 未找到或不可执行。"
        exit 1
    fi

    "$AUTOWARE_SCRIPT" > "$AUTOWARE_SCRIPT_DIR/autoware_vehicle_agent.log" 2>&1 &
    log "Autoware Vehicle Agent 已在后台启动。日志位于 autoware_vehicle_agent.log。"

    # 等待 Autoware 初始化
    wait_for_service "Autoware Vehicle Agent" 15
}

# =========================================
# 步骤6：启动 Car Update
# =========================================
start_car_update() {
    log "启动 Car Update..."
    CAR_UPDATE_DIR="/home/cityu-fsm-lab-carla/lmx/Docs/packages/autoware_comm/bash"
    INITIAL_CONFIG="$CAR_UPDATE_DIR/initial_config.bash"
    CAR_UPDATE_SCRIPT="/home/cityu-fsm-lab-carla/lmx/Docs/packages/autoware_comm/scripts/debug/debug_update_vehicle.py"

    # 加载初始配置
    source "$INITIAL_CONFIG" || { log "无法加载 $INITIAL_CONFIG。"; exit 1; }

    # 运行 debug_update_vehicle.py 脚本
    python3 "$CAR_UPDATE_SCRIPT" > /home/cityu-fsm-lab-carla/lmx/Docs/packages/autoware_comm/scripts/debug/debug_update_vehicle.log 2>&1 &
    log "Car Update 已在后台启动。日志位于 debug_update_vehicle.log。"
}

# =========================================
# 主执行流程
# =========================================
main() {
    log "开始执行自动化启动脚本..."

    # 启动各个服务，适当加入延时以确保依赖服务有足够时间启动
    start_carla
    sleep 5
    start_steamvr
    sleep 5
    start_vr_pose_transmit
    sleep 5
    start_serial_transmit
    sleep 5
    start_autoware_vehicle_agent
    sleep 5
    start_car_update

    log "所有服务已启动完成。"
    log "正在监控服务运行。按 Ctrl+C 终止脚本。"

    # 保持脚本运行，以维持后台进程
    wait
}

# 执行主函数
main