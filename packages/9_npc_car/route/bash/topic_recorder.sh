#!/bin/bash

# 脚本名称: record_ros2_topics.sh
# 描述: 录制指定的ROS2话题，输出话题内容到日志文件，并在接收到中断信号时安全终止

# 设置bag文件保存路径和文件名（基于当前时间）
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
BAG_DIR="$FSM_ROOT/ros2_bags"
BAG_NAME="autoware_topics_${TIMESTAMP}"
BAG_PATH="${BAG_DIR}/${BAG_NAME}"

# 设置日志文件保存目录
LOG_DIR="../topics_logs"
mkdir -p "${LOG_DIR}"

# 确保保存目录存在
mkdir -p "${BAG_DIR}"

# 要录制的话题列表和对应的日志文件
declare -A TOPICS_AND_LOGS=(
  ["/planning/scenario_planning/lane_driving/behavior_planning/path"]="${LOG_DIR}/path_${TIMESTAMP}.log"
  ["/planning/scenario_planning/trajectory"]="${LOG_DIR}/trajectory_${TIMESTAMP}.log"
  ["/planning/path_candidate/start_planner"]="${LOG_DIR}/start_planner_${TIMESTAMP}.log"
  ["/real_world/follow_adtruck/transformed_with_covariance"]="${LOG_DIR}/transformed_pose_${TIMESTAMP}.log"
)

# 话题数组（用于ros2 bag record）
TOPICS=("${!TOPICS_AND_LOGS[@]}")

# 创建安全退出函数
cleanup() {
  echo -e "\n正在安全终止录制和日志记录..."
  
  # 向ros2 bag进程发送终止信号
  if [ -n "$ROS2_BAG_PID" ]; then
    kill -SIGINT $ROS2_BAG_PID
    wait $ROS2_BAG_PID 2>/dev/null
  fi
  
  # 终止所有echo进程
  for pid in "${ECHO_PIDS[@]}"; do
    if kill -0 $pid 2>/dev/null; then
      kill -SIGTERM $pid
      wait $pid 2>/dev/null
    fi
  done
  
  echo "录制已完成"
  echo "bag文件保存在: ${BAG_PATH}"
  echo "日志文件保存在:"
  for log_file in "${TOPICS_AND_LOGS[@]}"; do
    echo "- $log_file"
  done
  
  exit 0
}

# 设置信号处理器
trap cleanup SIGINT SIGTERM

# 显示录制信息
echo "开始录制以下话题:"
echo "1. ${TOPICS[0]} (autoware_auto_planning_msgs/msg/Path)"
echo "2. ${TOPICS[1]} (autoware_auto_planning_msgs/msg/Trajectory)"
echo "3. ${TOPICS[2]} (autoware_auto_planning_msgs/msg/Path)"
echo "4. ${TOPICS[3]} (geometry_msgs/msg/PoseWithCovarianceStamped)"
echo "保存路径: ${BAG_PATH}"
echo "日志文件将保存到: ${LOG_DIR}"
echo "按 Ctrl+C 停止录制"

# 启动每个话题的echo进程
ECHO_PIDS=()
for topic in "${!TOPICS_AND_LOGS[@]}"; do
  log_file="${TOPICS_AND_LOGS[$topic]}"
  echo "启动记录: $topic > $log_file"
  ros2 topic echo "$topic" > "$log_file" &
  ECHO_PIDS+=($!)
done

# 开始录制bag
ros2 bag record -o "${BAG_PATH}" "${TOPICS[@]}" &
ROS2_BAG_PID=$!

# 等待ros2 bag进程完成(通常是通过Ctrl+C触发cleanup函数)
wait $ROS2_BAG_PID

# 如果进程正常结束（而不是被信号终止）
cleanup