# this is the potential topic which is about route

## global path 

/planning/scenario_planning/lane_driving/behavior_planning/path
type: autoware_auto_planning_msgs/msg/Path

## local path

/planning/scenario_planning/trajectory
    type: autoware_auto_planning_msgs/msg/Trajectory

    /home/cityu-fsm-lab-carla/Workspace/autoware/install/autoware_auto_planning_msgs/include/autoware_auto_planning_msgs/autoware_auto_planning_msgs/msg/trajectory.h

/planning/path_candidate/start_planner
    type: autoware_auto_planning_msgs/msg/Path

    /home/cityu-fsm-lab-carla/Workspace/autoware/install/autoware_auto_planning_msgs/include/autoware_auto_planning_msgs/autoware_auto_planning_msgs/msg/path.h

/real_world/follow_adtruck/transformed_with_covariance
    type: geometry_msgs/msg/PoseWithCovarianceStamped


```cpp

#include "autoware_auto_planning_msgs/msg/path.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"

```

```bash
~/Workspace$ find . -type d -name "autoware_auto_planning_msgs"

./autoware/install/autoware_auto_planning_msgs
./autoware/install/autoware_auto_planning_msgs/include/autoware_auto_planning_msgs
./autoware/install/autoware_auto_planning_msgs/include/autoware_auto_planning_msgs/autoware_auto_planning_msgs
./autoware/install/autoware_auto_planning_msgs/share/autoware_auto_planning_msgs
./autoware/install/autoware_auto_planning_msgs/local/lib/python3.10/dist-packages/autoware_auto_planning_msgs
./autoware/src/core/external/autoware_auto_msgs/autoware_auto_planning_msgs
./autoware/log/build_2024-12-11_16-11-41/autoware_auto_planning_msgs
./autoware/log/build_2024-12-11_16-06-50/autoware_auto_planning_msgs
./autoware/log/build_2024-12-11_16-29-43/autoware_auto_planning_msgs
./autoware/log/build_2024-12-11_17-45-38/autoware_auto_planning_msgs
./autoware/log/build_2024-12-17_16-06-03/autoware_auto_planning_msgs
./autoware/log/build_2024-05-31_13-12-51/autoware_auto_planning_msgs

```


0.02347555673045143, y : 0.0, z : -1.5808621201177706, yaw is -178.281394931699

x : 0.07367357080606096, y : 0.0, z : -1.5304785781495163, yaw is -178.305494931687
