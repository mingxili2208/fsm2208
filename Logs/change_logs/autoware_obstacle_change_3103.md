# this file record the changes of obstacle 

## obstacle_collision_checker.param.yaml

/home/cityu-fsm-lab-carla/Workspace/autoware/install/autoware_launch/share/autoware_launch/config/control/obstacle_collision_checker/obstacle_collision_checker.param.yaml

/**:
  ros__parameters:
    # Node
    update_rate: 10.0

    # Core
    delay_time: 0.03 # delay time of vehicle [s]
    footprint_margin: 1.0 #0.0 # margin for footprint [m]
    max_deceleration: 1.5 # max deceleration [m/ss]
    resample_interval: 0.15 #0.3 # interval distance to resample point cloud [m]
    search_radius: 10.0 #5.0 # search distance from trajectory to point cloud [m]

## predicted_path_checker.param.yaml

/home/cityu-fsm-lab-carla/Workspace/autoware/install/autoware_launch/share/autoware_launch/config/control/predicted_path_checker/predicted_path_checker.param.yaml

/**:
  ros__parameters:
    # Node
    update_rate: 10.0
    delay_time: 0.17
    max_deceleration: 1.5
    resample_interval: 0.5
    stop_margin: 0.5 # [m]
    ego_nearest_dist_threshold: 3.0 # [m]
    ego_nearest_yaw_threshold: 1.046 # [rad] = 60 [deg]
    min_trajectory_check_length: 1.5 # [m]
    trajectory_check_time: 3.0
    distinct_point_distance_threshold: 0.15 #0.3
    distinct_point_yaw_threshold: 5.0 # [deg]
    filtering_distance_threshold: 1.5 # [m]
    use_object_prediction: true

    collision_checker_params:
      width_margin: 1.0 #0.2
      chattering_threshold: 0.2
      z_axis_filtering_buffer: 0.3
      enable_z_axis_obstacle_filtering: false


## obstacle_stop_planner.param.yaml

/home/cityu-fsm-lab-carla/Workspace/autoware/install/autoware_launch/share/autoware_launch/config/planning/scenario_planning/lane_driving/motion_planning/obstacle_stop_planner/obstacle_stop_planner.param.yaml

/**:
  ros__parameters:
    chattering_threshold: 0.5                # even if the obstacle disappears, the stop judgment continues for chattering_threshold [s]
    lowpass_gain: 0.9                        # gain parameter for low pass filter [-]
    max_velocity: 20.0                       # max velocity [m/s]
    enable_slow_down: False                  # whether to use slow down planner [-]
    enable_z_axis_obstacle_filtering: False #True   # filter obstacles in z axis (height) [-]
    z_axis_filtering_buffer: 0.0             # additional buffer for z axis filtering [m]
    voxel_grid_x: 0.05                       # voxel grid x parameter for filtering pointcloud [m]
    voxel_grid_y: 0.05                       # voxel grid y parameter for filtering pointcloud [m]
    voxel_grid_z: 100000.0                   # voxel grid z parameter for filtering pointcloud [m]
    use_predicted_objects: False            # whether to use predicted objects [-]
    publish_obstacle_polygon: True #False          # whether to publish obstacle polygon [-]
    predicted_object_filtering_threshold: 1.5 # threshold for filtering predicted objects (valid only publish_obstacle_polygon true) [m]

    stop_planner:
      # params for stop position
      stop_position:
        max_longitudinal_margin: 5.0             # stop margin distance from obstacle on the path [m]
        max_longitudinal_margin_behind_goal: 3.0 # stop margin distance from obstacle behind goal on the path [m]
        min_longitudinal_margin: 5.0             # stop margin distance when any other stop point is inserted in stop margin [m]
        hold_stop_margin_distance: 0.0           # the ego keeps stopping if the ego is in this margin [m]

      # params for detection area
      detection_area:
        lateral_margin: 0.0                  # margin [m]
        vehicle_lateral_margin: 0.0          # margin of vehicle footprint [m]
        pedestrian_lateral_margin: 0.0       # margin of pedestrian footprint [m]
        unknown_lateral_margin: 0.0          # margin of unknown footprint [m]
        step_length: 1.0                     # step length for pointcloud search range [m]
        enable_stop_behind_goal_for_obstacle: True # enable extend trajectory after goal lane for obstacle detection


    slow_down_planner:
      # params for slow down section
      slow_down_section:
        longitudinal_forward_margin: 5.0     # margin distance from slow down point to vehicle front [m]
        longitudinal_backward_margin: 0.0    # margin distance from slow down point to vehicle rear [m]
        longitudinal_margin_span: -0.1       # fineness param for relaxing slow down margin (use this param if consider_constraints is True) [m/s]
        min_longitudinal_forward_margin: 1.0 # min margin for relaxing slow down margin (use this param if consider_constraints is True) [m/s]

      # params for detection area
      detection_area:
        lateral_margin: 1.0                  # offset from vehicle side edge for expanding the search area of the surrounding point cloud [m]
        vehicle_lateral_margin: 1.0          # offset from vehicle side edge for expanding the search area of the surrounding point cloud [m]
        pedestrian_lateral_margin: 1.0       # offset from pedestrian side edge for expanding the search area of the surrounding point cloud [m]
        unknown_lateral_margin: 1.0          # offset from unknown side edge for expanding the search area of the surrounding point cloud [m]

      # params for velocity
      target_velocity:
        max_slow_down_velocity: 1.38         # max slow down velocity (use this param if consider_constraints is False)[m/s]
        min_slow_down_velocity: 0.28         # min slow down velocity (use this param if consider_constraints is False)[m/s]
        slow_down_velocity: 1.38             # target slow down velocity (use this param if consider_constraints is True)[m/s]

      # params for deceleration constraints (use this param if consider_constraints is True)
      constraints:
        jerk_min_slow_down: -0.6             # min slow down jerk constraint [m/sss]
        jerk_span: -0.01                     # fineness param for planning deceleration jerk [m/sss]
        jerk_start: -0.1                     # init jerk used for deceleration planning [m/sss]

      # others
      consider_constraints: False                 # set "True", if no decel plan found under jerk/dec constrains, relax target slow down vel [-]
      velocity_threshold_decel_complete: 0.2      # use for judge whether the ego velocity converges the target slow down velocity [m/s]
      acceleration_threshold_decel_complete: 0.1  # use for judge whether the ego velocity converges the target slow down velocity [m/ss]


## surround_obstacle_checker.param.yaml

/home/cityu-fsm-lab-carla/Workspace/autoware/install/autoware_launch/share/autoware_launch/config/planning/scenario_planning/lane_driving/motion_planning/surround_obstacle_checker/surround_obstacle_checker.param.yaml

/**:
  ros__parameters:
    # obstacle check
    # surround_check_*_distance: if objects exist in this distance, transit to "exist-surrounding-obstacle" status [m]
    # surround_check_hysteresis_distance: if no object exists in this hysteresis distance added to the above distance, transit to "non-surrounding-obstacle" status [m]
    pointcloud:
      enable_check: true #false
      surround_check_front_distance: 0.5
      surround_check_side_distance: 0.5
      surround_check_back_distance: 0.5
    unknown:
      enable_check: true
      surround_check_front_distance: 0.5
      surround_check_side_distance: 0.5
      surround_check_back_distance: 0.5
    car:
      enable_check: true
      surround_check_front_distance: 0.5
      surround_check_side_distance: 0.0
      surround_check_back_distance: 0.5
    truck:
      enable_check: true
      surround_check_front_distance: 0.5
      surround_check_side_distance: 0.0
      surround_check_back_distance: 0.5
    bus:
      enable_check: true
      surround_check_front_distance: 0.5
      surround_check_side_distance: 0.0
      surround_check_back_distance: 0.5
    trailer:
      enable_check: true
      surround_check_front_distance: 0.5
      surround_check_side_distance: 0.0
      surround_check_back_distance: 0.5
    motorcycle:
      enable_check: true
      surround_check_front_distance: 0.5
      surround_check_side_distance: 0.0
      surround_check_back_distance: 0.5
    bicycle:
      enable_check: true
      surround_check_front_distance: 0.5
      surround_check_side_distance: 0.5
      surround_check_back_distance: 0.5
    pedestrian:
      enable_check: true
      surround_check_front_distance: 0.5
      surround_check_side_distance: 0.5
      surround_check_back_distance: 0.5

    surround_check_hysteresis_distance: 0.3

    state_clear_time: 2.0

    # ego stop state
    stop_state_ego_speed: 0.1 #[m/s]

    # debug
    publish_debug_footprints: true # publish vehicle footprint & footprints with surround_check_distance and surround_check_recover_distance offsets
    debug_footprint_label: "car"
