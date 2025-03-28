LOG_FILE="ros2_detailed_topic_info.log"
echo "ROS 2 Topic Info Log - $(date)" > "$LOG_FILE"
echo "==========================================" >> "$LOG_FILE"

ros2 topic list | while read topic; do
  echo "Topic: $topic" >> "$LOG_FILE"
  ros2 topic info "$topic" --verbose >> "$LOG_FILE"
  echo "------------------------------------------" >> "$LOG_FILE"
done