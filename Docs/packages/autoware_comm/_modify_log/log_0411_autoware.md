# log

/home/cityu-fsm-lab-carla/Workspace/autoware/src/universe/autoware.universe/planning/behavior_path_planner/config/behavior_path_planner.param.yaml

```yaml
    lane_following:
      drivable_area_right_bound_offset: 0.713
      drivable_area_left_bound_offset: 0.713
      drivable_area_types_to_skip: [road_border]
```

/home/cityu-fsm-lab-carla/Workspace/autoware/src/universe/autoware.universe/control/vehicle_cmd_gate/config/vehicle_cmd_gate.param.yaml

```yaml
/**:
  ros__parameters:
    update_rate: 10.0
    system_emergency_heartbeat_timeout: 0.5
    use_emergency_handling: false
    check_external_emergency_heartbeat: false
    use_start_request: false
    enable_cmd_limit_filter: true
    external_emergency_stop_heartbeat_timeout: 0.0
    stop_hold_acceleration: -1.5
    emergency_acceleration: -2.4
    moderate_stop_service_acceleration: -1.5
    stopped_state_entry_duration_time: 0.1
    stop_check_duration: 1.0
    nominal:
      vel_lim: 25.0
      reference_speed_points: [20.0, 30.0]
      steer_lim: [1.0, 0.8]
      steer_rate_lim: [1.0, 0.8]
      lon_acc_lim: [5.0, 4.0]
      lon_jerk_lim: [5.0, 4.0]
      lat_acc_lim: [5.0, 4.0]
      lat_jerk_lim: [7.0, 6.0]
      actual_steer_diff_lim: [1.0, 0.8]
    on_transition:
      vel_lim: 50.0
      reference_speed_points: [20.0, 30.0]
      steer_lim: [1.0, 0.8]
      steer_rate_lim: [1.0, 0.8]
      lon_acc_lim: [1.0, 0.9]
      lon_jerk_lim: [0.5, 0.4]
      lat_acc_lim: [2.0, 1.8]
      lat_jerk_lim: [7.0, 6.0]
      actual_steer_diff_lim: [1.0, 0.8]


```

```yaml
/**:
  ros__parameters:
    wheel_radius: 0.39
    wheel_width: 0.42
    wheel_base: 2.74 # between front wheel center and rear wheel center   142
    wheel_tread: 1.63 # between left wheel center and right wheel center
    front_overhang: 1.0 # between front wheel center and vehicle front
    rear_overhang: 1.03 # between rear wheel center and vehicle rear
    left_overhang: 0.1 # between left wheel center and vehicle left
    right_overhang: 0.1 # between right wheel center and vehicle right
    vehicle_height: 2.5
    max_steer_angle: 0.70 # [rad]


```

```shell

this is orginal pose: x: 0.28334152858299966, y: -0.7929726805410002, z: 0.0016,yaw: -0.8060164893060717


error message:

[INFO] [1731392017.015929398] [transformed_SandBox_coordinate_publisher_node]: this is orginal pose: x: 0.5884465218329997, y: -0.30214417219500067, z: 0.0016,yaw: 79.01638352734433,

[INFO] [1731392103.729495487] [transformed_SandBox_coordinate_publisher_node]: this is orginal pose: x: 0.30794050636399994, y: -0.31195007053000046, z: 0.0016,yaw: 105.12088349550496,


[INFO] [1731392129.852527822] [transformed_SandBox_coordinate_publisher_node]: this is orginal pose: x: 0.46157619249599957, y: -0.2941030524320003, z: 0.0016,yaw: 90.31518349092273,
[INFO] [1731392129.852933887] [transformed_SandBox_coordinate_publisher_node]: ____transformed___pose x: -38.72229587793303, y: -62.71514515812645, z:0.0016000000400000006,yaw: -0.3151834909227347,


[INFO] [1731392190.720718401] [transformed_SandBox_coordinate_publisher_node]: this is orginal pose: x: 0.8221694748179997, y: -0.38359453453400016, z: 0.0016,yaw: 68.70328352451922,
[INFO] [1731392190.720972984] [transformed_SandBox_coordinate_publisher_node]: ____transformed___pose x: -27.185204881843074, y: -60.18848514965954, z:0.0016000000400000006,yaw: 21.29671647548078,




[INFO] [1731392315.740968740] [transformed_SandBox_coordinate_publisher_node]: this is orginal pose: x: 0.2498663989679999, y: -1.2145123186610003, z: 0.0016,yaw: -13.044016491454226,
[INFO] [1731392315.741239040] [transformed_SandBox_coordinate_publisher_node]: ____transformed___pose x: -45.76641622744761, y: -33.52928312961133, z:0.0016000000400000006,yaw: 103.04401649145423,


[INFO] [1731394634.834019375] [transformed_SandBox_coordinate_publisher_node]: this is orginal pose: x: 0.271747696032, y: -2.059356429767, z: 0.0016,yaw: 2.3397835112436205,
[INFO] [1731394634.834948963] [transformed_SandBox_coordinate_publisher_node]: ____transformed___pose x: -39.9975320221597, y: -5.906098811324558, z:0.0016000000400000006,yaw: 87.66021648875638,


[INFO] [1731394693.747832657] [transformed_SandBox_coordinate_publisher_node]: this is orginal pose: x: 0.2863793067029996, y: -2.755915234295, z: 0.0016,yaw: 0.25148351087870363,
[INFO] [1731394693.748888700] [transformed_SandBox_coordinate_publisher_node]: ____transformed___pose x: -38.83748561456424, y: 17.021790565126686, z:0.0016000000400000006,yaw: 89.7485164891213,

[INFO] [1731394790.758239733] [transformed_SandBox_coordinate_publisher_node]: this is orginal pose: x: 0.28738442110899953, y: -3.0098053483930003, z: 0.0016,yaw: -0.6515164892790771,
[INFO] [1731394790.759202967] [transformed_SandBox_coordinate_publisher_node]: ____transformed___pose x: -38.557388987251926, y: 25.391171115641477, z:0.0016000000400000006,yaw: 90.65151648927908,

[INFO] [1731394828.130556544] [transformed_SandBox_coordinate_publisher_node]: this is orginal pose: x: 0.274309823621, y: -1.9181773392740002, z: 0.0016,yaw: 0.9657835110035207,
[INFO] [1731394828.131596394] [transformed_SandBox_coordinate_publisher_node]: ____transformed___pose x: -40.04568759840433, y: -10.45584126444685, z:0.0016000000400000006,yaw: 89.03421648899648,


```

[INFO] [1731399310.123644539] [transformed_SandBox_coordinate_publisher_node]: this is orginal pose: x: 0.7197051236339997, y: -0.3908517229830002, z: 0.0016,yaw: 78.3469835271488,
[INFO] [1731399310.123931290] [transformed_SandBox_coordinate_publisher_node]: ____transformed___pose x: -30.20548063569314, y: -60.77787253596383, z:0.0016000000400000006,yaw: 11.653016472851206,
![1](img/2024-11-12-16-15-40.png)

[INFO] [1731399379.264073582] [transformed_SandBox_coordinate_publisher_node]: this is orginal pose: x: 1.475994034318, y: -0.5454380622000001, z: 0.0016,yaw: 96.40788349293562,
[INFO] [1731399379.264653997] [transformed_SandBox_coordinate_publisher_node]: ____transformed___pose x: -5.275999964155503, y: -56.44509545679973, z:0.0016000000400000006,yaw: -6.407883492935625,
![2](img/2024-11-12-16-16-33.png)

[INFO] [1731399426.183767801] [transformed_SandBox_coordinate_publisher_node]: this is orginal pose: x: 2.7786898233859993, y: -0.539390019287, z: 0.0016,yaw: 57.23118352175811,
[INFO] [1731399426.184188791] [transformed_SandBox_coordinate_publisher_node]: ____transformed___pose x: 37.40097662171005, y: -57.9045399581916, z:0.0016000000400000006,yaw: 32.76881647824189,
![3](img/2024-11-12-16-17-24.png)

[INFO] [1731399476.223802164] [transformed_SandBox_coordinate_publisher_node]: this is orginal pose: x: 3.263137860962, y: -0.8443970689950002, z: 0.0016,yaw: 55.10648352128115,
[INFO] [1731399476.224084795] [transformed_SandBox_coordinate_publisher_node]: ____transformed___pose x: 53.56922860012342, y: -48.380029262619026, z:0.0016000000400000006,yaw: 34.89351647871885,
![4](img/2024-11-12-16-18-11.png)

[INFO] [1731399554.823637911] [transformed_SandBox_coordinate_publisher_node]: this is orginal pose: x: 3.905932441948, y: -2.115823254756, z: 0.0016,yaw: -9.843816490889429,
[INFO] [1731399554.823970589] [transformed_SandBox_coordinate_publisher_node]: ____transformed___pose x: 75.86140030051891, y: -7.344104372628021, z:0.0016000000400000006,yaw: 99.84381649088942,
![5](img/2024-11-12-16-19-27.png)

[INFO] [1731399639.943756883] [transformed_SandBox_coordinate_publisher_node]: this is orginal pose: x: 3.096207453352, y: -3.8076815995240003, z: 0.0016,yaw: -91.58031646968811,
[INFO] [1731399639.944142358] [transformed_SandBox_coordinate_publisher_node]: ____transformed___pose x: 50.968828666581956, y: 48.87362268354772, z:0.0016000000400000006,yaw: 181.5803164696881,
![6](img/2024-11-12-16-20-56.png)

[INFO] [1731399728.543766246] [transformed_SandBox_coordinate_publisher_node]: this is orginal pose: x: 0.08239862717899982, y: -1.7681294037710003, z: 0.0016,yaw: -175.9283164884535,
[INFO] [1731399728.544063915] [transformed_SandBox_coordinate_publisher_node]: ____transformed___pose x: -49.7533392204199, y: -15.034319045065839, z:0.0016000000400000006,yaw: 265.9283164884535,
![7](img/2024-11-12-16-22-21.png)


[INFO] [1731400393.954713518] [rviz2]: Setting goal pose: Frame:map, Position(-15.625, -11.9509, 0), Orientation(0, 0, 0.687054, 0.726606) = Angle: 1.51485 
