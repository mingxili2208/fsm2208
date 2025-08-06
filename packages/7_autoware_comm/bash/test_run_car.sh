#!/bin/bash

# 定义清理函数，在脚本退出时终止所有后台进程
function cleanup {
    echo "Terminating all background processes..."
    kill $(jobs -p)
}

# 捕获 SIGINT (Ctrl+C) 信号并调用 cleanup 函数
trap cleanup SIGINT

#############################
# Launch the Carla Ego-Vehicle
#############################

export SIMULATOR_LOCAL_HOST="localhost"
export SIMULATOR_PORT="2000"
export TRAFFIC_MANAGER_PORT="8000"
#export TEAM_AGENT=${OP_BRIDGE_ROOT}/op_bridge/fsm_lab_simulation/test_ego_vehicle_initializer.py
export TEAM_AGENT=/home/cityu-fsm-lab-carla/lmx/packages/7_autoware_comm/scripts/test/test_ego_vehicle_initializer2.py
export PYTHONPATH="${CARLA_ROOT}/PythonAPI/carla/":"${SCENARIO_RUNNER_ROOT}":"${OP_BRIDGE_ROOT}":${PYTHONPATH}
export AGENT_FRAME_RATE="30"
export REMOTE_CONNECTION="False"
# Autonomous actor default role_name and type
export AGENT_ROLE_NAME="pygame_adtruck" #############################################pygame-adtruck
export AGENT_MODEL_TYPE="vehicle.dodge.charger_police" #"vehicle.mitsubishi.fusorosa" #"vehicle.audi.etron" #"vehicle.dodge.charger_police" #"vehicle.mitsubishi.fusorosa" #"vehicle.tesla.cybertruck" #"vehicle.bmw.grandtourer"  #"vehicle.audi.etron" #"vehicle.dodge.charger_police" #"vehicle.carlamotors.carlacola" #"vehicle.dodge.charger_police" #"vehicle.tesla.cybertruck" #  #"vehicle.tesla.cybertruck" #"vehicle.mitsubishi.fusorosa" #"vehicle.dodge.charger_police"   #"vehicle.tesla.model3"

# modes are 
#   * "leaderboard" : when runner the leaderboard (route based) scenario collections 
#   * "srunner" : when scenario is loaded using scenario runner, and only agent is attached
#   * "free" : when loading empty map only , either carla town or any OpenDRIVE map 
export OP_BRIDGE_MODE="free"

# CARLA town name or custom OpenDRIVE absolute path, when BRIDGE_MODE is free 
export FREE_MAP_NAME="fsm_lab_sandbox_right_hand_driving_scene"

# Spawn point for the autonomous agent, when BRIDGE_MODE is free 
# "x,y,z,roll,pitch,yaw"
# Empty string means random starting position
# export FREE_AGENT_POSE="175.4,195.14,0,0,0,180" 
# export FREE_AGENT_POSE="88.6,-226,0,0,0,0" 
export FREE_AGENT_POSE="0"

# Set controller. ["autoware", "pygame", "follow", "teleop"]
#export CONTROL_MODE="pygame"
export CONTROL_MODE="autoware"

# Set the running mode. ["normal", "record"]
export RUNNING_MODE="normal"

source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=46

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
    
    # 将 rviz2 作为后台进程启动
    ros2 run rviz2 rviz2 -d ${OP_AGENT_ROOT}/rviz/carla_autoware.rviz -s ${OP_AGENT_ROOT}/rviz/image/autoware.png &

    # 启动其他脚本
    python3 ${OP_BRIDGE_ROOT}/op_bridge/fsm_lab_simulation/world_launcher.py
    sleep 5
    python3 /home/cityu-fsm-lab-carla/lmx/packages/7_autoware_comm/scripts/test/test_vehicle_launcher.py
    
    # # 启动视觉 UI
    # echo "启动车辆可视化界面..."
    # python3 /home/cityu-fsm-lab-carla/lmx/packages/7_autoware_comm/scripts/test/vehicle_monitor.py --host 127.0.0.1 --port 2000 --role-name pygame_adtruck
    #python3 ${OP_BRIDGE_ROOT}/op_bridge/fsm_lab_simulation/vehicle_follow.py
    #python3 /home/cityu-fsm-lab-carla/lmx/source_code/tracker2carla/transformed_world_coor_publisher.py 
    
    #python3 ${OP_BRIDGE_ROOT}/op_bridge/fsm_lab_simulation/vehicle_launcher_follow.py
    #python3 ${OP_BRIDGE_ROOT}/op_bridge/fsm_lab_simulation/testing_scripts/launch_vehicle.py
else
    python3 ${OP_BRIDGE_ROOT}/op_bridge/fsm_lab_simulation/world_launcher.py
    python3 ${OP_BRIDGE_ROOT}/op_bridge/fsm_lab_simulation/vehicle_launcher.py
    
    #python3 ${OP_BRIDGE_ROOT}/op_bridge/fsm_lab_simulation/vehicle_launcher_follow.py
    #python3 ${OP_BRIDGE_ROOT}/op_bridge/fsm_lab_simulation/testing_scripts/launch_vehicle.py
fi

# 等待所有后台进程完成
wait