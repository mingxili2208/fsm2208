#! /bin/bash -e

echo "--------------------------------------"
echo "carla gnss record start"
echo "--------------------------------------"

source /opt/ros/humble/setup.bash
source ${OP_ROS_PLUGINS_ROOT}/install/setup.bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}

ros2 launch gnss_poser gnss_poser.launch.xml
