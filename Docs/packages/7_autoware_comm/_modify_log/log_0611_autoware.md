# autoware

all autoware fire are in path /home/cityu-fsm-lab-carla/Workspace/autoware/install

/home/cityu-fsm-lab-carla/Workspace/autoware/install/autoware_launch/share/autoware_launch/launch/autoware.launch.xml

/home/cityu-fsm-lab-carla/Workspace/autoware/install/tier4_control_launch/share/tier4_control_launch/launch/control.launch.py


## in file autoware.launch.xml

```xml
  <!-- Planning -->
  <group if="$(var launch_planning)">
    <include file="$(find-pkg-share autoware_launch)/launch/components/tier4_planning_component.launch.xml"/>
  </group>

  <!-- Control -->
  <group if="$(var launch_control)">
    <include file="$(find-pkg-share autoware_launch)/launch/components/tier4_control_component.launch.xml"/>
  </group>
```

### in tier4_planning_component


### in tier4_control_component

```xml
 <include file="$(find-pkg-share tier4_control_launch)/launch/control.launch.py">
```

```xml
  <include file="$(find-pkg-share tier4_control_launch)/launch/control.launch.py">
    <!-- option -->
    <arg name="vehicle_param_file" value="$(find-pkg-share $(var vehicle_model)_description)/config/vehicle_info.param.yaml"/>
    <arg name="vehicle_id" value="$(var vehicle_id)"/>
    <arg name="enable_obstacle_collision_checker" value="false"/>
    <arg name="trajectory_follower_node_param_path" value="$(find-pkg-share autoware_launch)/config/control/trajectory_follower/trajectory_follower_node.param.yaml"/>
    <arg
      name="lat_controller_param_path"
      value="$(find-pkg-share autoware_launch)/config/control/trajectory_follower/$(var latlon_controller_param_path_dir)/lateral/$(var lateral_controller_mode).param.yaml"
    />
    <ar  name="lon_controller_param_path"
      value="$(find-pkg-share autoware_launch)/config/control/trajectory_follower/$(var latlon_controller_param_path_dir)/longitudinal/$(var longitudinal_controller_mode).param.yaml"
    />
    <arg name="lateral_controller_mode" value="$(var lateral_controller_mode)"/>
    <arg name="longitudinal_controller_mode" value="$(var longitudinal_controller_mode)"/>
    <arg name="check_external_emergency_heartbeat" value="$(var check_external_emergency_heartbeat)"/>

    <!-- common param path -->
    <arg name="nearest_search_param_path" value="$(find-pkg-share autoware_launch)/config/control/common/nearest_search.param.yaml"/>

    <!-- package param path -->
    <arg name="lat_controller_param_path" value="$(var lat_controller_param_path)"/>
    <arg name="lon_controller_param_path" value="$(var lon_controller_param_path)"/>
    <arg name="vehicle_cmd_gate_param_path" value="$(find-pkg-share autoware_launch)/config/control/vehicle_cmd_gate/vehicle_cmd_gate.param.yaml"/>
    <arg name="lane_departure_checker_param_path" value="$(find-pkg-share autoware_launch)/config/control/lane_departure_checker/lane_departure_checker.param.yaml"/>
    <arg name="control_validator_param_path" value="$(find-pkg-share autoware_launch)/config/control/control_validator/control_validator.param.yaml"/>
    <arg name="operation_mode_transition_manager_param_path" value="$(find-pkg-share autoware_launch)/config/control/operation_mode_transition_manager/operation_mode_transition_manager.param.yaml"/>
    <arg name="shift_decider_param_path" value="$(find-pkg-share autoware_launch)/config/control/shift_decider/shift_decider.param.yaml"/>
    <arg name="obstacle_collision_checker_param_path" value="$(find-pkg-share autoware_launch)/config/control/obstacle_collision_checker/obstacle_collision_checker.param.yaml"/>
    <arg name="external_cmd_selector_param_path" value="$(find-pkg-share autoware_launch)/config/control/external_cmd_selector/external_cmd_selector.param.yaml"/>
    <arg name="aeb_param_path" value="$(find-pkg-share autoware_launch)/config/control/autonomous_emergency_braking/autonomous_emergency_braking.param.yaml"/>
    <arg name="enable_autonomous_emergency_braking" value="$(var enable_autonomous_emergency_braking)"/>
    <arg name="enable_predicted_path_checker" value="$(var enable_predicted_path_checker)"/>
    <arg name="predicted_path_checker_param_path" value="$(find-pkg-share autoware_launch)/config/control/predicted_path_checker/predicted_path_checker.param.yaml"/>
  </include>
```