#############################
# Launch the Carla Ego-Vehicle
#############################

if [ ${CONTROL_MODE} = "autoware" ]
then
    source ${AUTOWARE_ROOT}/install/setup.bash
    if [[ ${FREE_MAP_NAME} == *fsm_lab* ]]
    then
        source ${OP_ROS_PLUGINS_ROOT}/install/setup.bash
        echo "Apply the official map in FSM Lab."
    else
        echo "Apply the official map in Carla Simulator."
    fi
    ros2 run rviz2 rviz2 -d ${OP_AGENT_ROOT}/rviz/carla_autoware.rviz -s ${OP_AGENT_ROOT}/rviz/image/autoware.png 
    gnome-terminal -- bash -c "source ./initial_config.sh; python3 ${OP_BRIDGE_ROOT}/op_bridge/fsm_lab_simulation/world_launcher.py; exec bash" & 
    #python3 ${OP_BRIDGE_ROOT}/op_bridge/fsm_lab_simulation/carla_vehicle_follow_RC/launch/launch_vehicle.py &
    gnome-terminal -- bash -c "source ./initial_config.sh; python3 ~/lmx/source_code/key_board_control/vis_pygame_tel_test.py; exec bash" &
    #python3 ${OP_BRIDGE_ROOT}/op_bridge/fsm_lab_simulation/carla_vehicle_follow_RC/transformed_world_coor_publisher.py &
    #python3 ${OP_BRIDGE_ROOT}/op_bridge/fsm_lab_simulation/carla_vehicle_follow_RC/vehicle_follow_handler.py &
else
    #python3 ${OP_BRIDGE_ROOT}/op_bridge/fsm_lab_simulation/world_launcher.py &
    gnome-terminal -- bash -c "source ./initial_config.sh; python3 ${OP_BRIDGE_ROOT}/op_bridge/fsm_lab_simulation/world_launcher.py; exec bash" & 
    #python3 ${OP_BRIDGE_ROOT}/op_bridge/fsm_lab_simulation/carla_vehicle_follow_RC/launch/launch_vehicle.py &
    gnome-terminal -- bash -c "source ./initial_config.sh; python3 ~/lmx/source_code/key_board_control/vis_pygame_tel_test.py; exec bash" &
    #python3 ~/lmx/source_code/key_board_control/vis_pygame_tel_test.py
    #python3 ${OP_BRIDGE_ROOT}/op_bridge/fsm_lab_simulation/carla_vehicle_follow_RC/transformed_world_coor_publisher.py &
    #python3 ${OP_BRIDGE_ROOT}/op_bridge/fsm_lab_simulation/carla_vehicle_follow_RC/vehicle_follow_handler.py &
fi
