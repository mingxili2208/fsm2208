#! /bin/bash -e
role_name=$1
map_name=$2
explore_mode=$3
route_topic=$4

echo "carla_release"
echo "-------------------------------"
echo "Starting OpenPlanner .. "
echo "-------------------------------"
echo "Agent Name: " $role_name
echo "Exploration Mode: " $explore_mode
# echo $route_topic
echo "Map Path: " ${OP_AGENT_ROOT}/autoware-contents/maps/$map_name
echo "CARLA Path: " ${CARLA_ROOT}
echo "-------------------------------"
echo ""

source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}
export FREE_MAP_NAME=${FREE_MAP_NAME}
# ros2 launch /home/hatem/carla/op_agent/autoware_carla_launch/carla_autoware_sensors_interface.xml

if [[ ${FREE_MAP_NAME} == *fsm_lab* ]]
then
    source ${OP_ROS_PLUGINS_ROOT}/install/setup.bash
    ros2 launch ${OP_AGENT_ROOT}/autoware_carla_launch/carla_simulator_fsm_lab.launch.xml map_path:=${OP_AGENT_ROOT}/autoware-contents/maps/fsm_lab_maps/$map_name vehicle_model:=sample_vehicle sensor_model:=carla_sensor_kit > /home/cityu-fsm-lab-carla/lmx/Docs/packages/8_positioning_compensation/log/autoware_log.log >&1
else
    source ${AUTOWARE_ROOT}/install/setup.bash
    ros2 launch ${OP_AGENT_ROOT}/autoware_carla_launch/carla_simulator_carla.launch.xml map_path:=${OP_AGENT_ROOT}/autoware-contents/maps/$map_name vehicle_model:=sample_vehicle sensor_model:=carla_sensor_kit 
fi
# $SHELL

