#!/bin/bash

# 脚本名称: record_control_cmd.sh
# 描述: 录制ROS2 control_cmd话题并输出到日志文件，在接收到中断信号时安全终止

# 设置bag文件保存路径和文件名（基于当前时间）
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
BAG_DIR="$FSM_ROOT/ros2_bags"
BAG_NAME="control_cmd_${TIMESTAMP}"
BAG_PATH="${BAG_DIR}/${BAG_NAME}"

# 设置日志文件保存目录
LOG_DIR="../cmd_logs"
LOG_FILE="${LOG_DIR}/control_cmd_${TIMESTAMP}.log"

# 确保保存目录存在
mkdir -p "${BAG_DIR}"
mkdir -p "${LOG_DIR}"

# 要录制的话题
TOPIC="/control/command/control_cmd"

# 创建安全退出函数
cleanup() {
  echo -e "\n正在安全终止录制和日志记录..."
  
  # 向ros2 bag进程发送终止信号
  if [ -n "$ROS2_BAG_PID" ]; then
    kill -SIGINT $ROS2_BAG_PID
    wait $ROS2_BAG_PID 2>/dev/null
  fi
  
  # 终止echo进程
  if [ -n "$ECHO_PID" ]; then
    kill -SIGTERM $ECHO_PID
    wait $ECHO_PID 2>/dev/null
  fi
  
  echo "录制已完成"
  echo "bag文件保存在: ${BAG_PATH}"
  echo "日志文件保存在: ${LOG_FILE}"
  
  exit 0
}

# 设置信号处理器
trap cleanup SIGINT SIGTERM

# 显示录制信息
echo "开始录制话题: ${TOPIC}"
echo "bag文件保存在: ${BAG_PATH}"
echo "日志文件保存在: ${LOG_FILE}"
echo "按 Ctrl+C 停止录制"

# 启动话题echo进程
echo "启动记录: ${TOPIC} > ${LOG_FILE}"
ros2 topic echo "${TOPIC}" > "${LOG_FILE}" &
ECHO_PID=$!

# 开始录制bag
ros2 bag record -o "${BAG_PATH}" "${TOPIC}" &
ROS2_BAG_PID=$!

# 输出一些进程信息
echo "Bag录制进程PID: $ROS2_BAG_PID"
echo "话题输出进程PID: $ECHO_PID"

# 等待ros2 bag进程完成(通常是通过Ctrl+C触发cleanup函数)
wait $ROS2_BAG_PID

# 如果进程正常结束（而不是被信号终止）
cleanup