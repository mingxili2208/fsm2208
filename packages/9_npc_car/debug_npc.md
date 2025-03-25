# description of how to launch the auto_test

## init resources monitor

cd /home/cityu-fsm-lab-carla/lmx/packages/7_autoware_comm/scripts

python3 ./monitor/test_sys_monitor.py

## init car

1. start carla

    ```sh
    cd $CARLA_ROOT
    make launch
    ```

    then have to click the button play

## init npc

1. init terminal

    cd  /home/cityu-fsm-lab-carla/lmx/packages/9_npc_car/bash

    source ./initial_npc_param.bash

2. init npc world(only if not init any world)

    cd  /home/cityu-fsm-lab-carla/lmx/packages/9_npc_car/bash

    bash ./npc_world_init.bash

3. init npc_car

    cd  /home/cityu-fsm-lab-carla/lmx/packages/9_npc_car/bash

    bash ./npc_car_init.bash

## init navigation

1. if in ps2

2. if in waypoints(spawn_points)

## init sandbox_car

1. start steam vr
2. start vr_pose transmit

    cd /home/cityu-fsm-lab-carla/lmx/packages/7_autoware_comm/bash

    source initial_config.bash

    cd /home/cityu-fsm-lab-carla/lmx/packages/8_positioning_compensation

    python3 test3_fix_ekf.py

    cd /home/cityu-fsm-lab-carla/lmx/packages/7_autoware_comm/scripts/ekf

    python3 test_fix_ekf.py

3. start serial transmit

    check if the serial is not online

    ```bash
    ls /dev/ttyUSB0

    sudo chmod 777 /dev/ttyUSB0

    ```

    start transmit

    ```bash

    cd /home/cityu-fsm-lab-carla/lmx/packages/7_autoware_comm/bash

    source initial_config.bash 

    cd /home/cityu-fsm-lab-carla/lmx/packages/7_autoware_comm/scripts/debug

    python3 debug5_serial.py
    
    ```

4. start autoware vehicle agent

    ```bash
    cd /home/cityu-fsm-lab-carla/Workspace/Carla/op_carla/op_bridge/op_scripts/fsm_lab_simulation/

    ./debug_run_vehicle_ros2.sh

    ```

5. start car update

    ```bash
    cd /home/cityu-fsm-lab-carla/lmx/packages/7_autoware_comm/bash

    source initial_config.bash 

    cd /home/cityu-fsm-lab-carla/lmx/packages/7_autoware_comm/scripts/debug

    python3 debug_update_vehicle_2.py

    ```

## debug

ros2 topic info /localization/pose_estimator/pose_with_covariance is published by ndt and subscribed by ekf

this is the echo of topic /localization/pose_twist_fusion_filter/biased_pose_with_covariance
![1](img/2024-09-23-16-07-51.png)

[transformed_SandBox_coordinate_publisher_node]:
 Transformed Published with covariance! [-33.79422926148789, 62.72506004639747, 0.0016000000400000006, 0.9080939452713015, 0.0, 0.0, -0.41876650601690013]

the same as the send msg

```py

    return carla.Transform(
        ros_point_to_carla_location(ros_pose.position),
        ros_quaternion_to_carla_rotation(ros_pose.orientation),
    )

def ros_point_to_carla_location(ros_point):
    ##########for tracker
    return carla.Location(ros_point.x, -ros_point.z, ros_point.y)
    #########################################################################################
    #return carla.Location(ros_point.x, -ros_point.y, ros_point.z)   

```

the set_frame of carla does not fellow the ekf

![1](img/2024-09-30-17-09-07.png) control launch

## for debuging

``` bash

sudo fallocate -l 8G /swapfile   # 创建8GB的交换文件
sudo chmod 600 /swapfile         # 设置正确的权限
sudo mkswap /swapfile            # 设置交换文件
sudo swapon /swapfile            # 启用交换文件

```

```bash
cat /proc/sys/fs/inotify/max_user_watches

sudo sysctl fs.inotify.max_user_watches=524288
sudo sysctl fs.inotify.max_user_instances=1024

sudo sysctl -p
```

```bash

dmesg | grep -i 'killed process'

```

```bash
sudo fallocate -l 8G /swapfile

sudo chmod 600 /swapfile

sudo mkswap /swapfile

sudo swapon /swapfile

#将这个 swap 设置永久生效（重启后仍可用），编辑 /etc/fstab 文件并添加以下内容:

/swapfile swap swap defaults 0 0

```

![2](img/2024-10-04-17-43-59.png)

watch -n 1 nvidia-smi

nvidia-settings
