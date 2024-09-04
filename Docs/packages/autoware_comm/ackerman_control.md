# this is the framework design for the autoware_commn

## AckermannControlCommand

from autoware_auto_control_msgs.msg import AckermannControlCommand

![0](img/img-2024-09-02-16-56-17.png)

<https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-interfaces/components/control/>

![0-1](img/img-2024-09-02-16-57-24.png)

![0-2](img/img-2024-09-02-16-58-56.png)

perhaps need Instruction Adjustment Mechanism

## source_code_framework

this is the framework of launching the autoware

![1](img/img-2024-09-02-16-41-57.png)

## ego_vehicle_initializer.py

* fun_setup()  this is the initializer of the whole agent
* Load a series of startup files
* run_step()
* init_subscribers()
* call_backfunctions()
* init_publishers()
* publisher_handlers()
* other_handlers()

## start_ros2.sh

this bash include the following two files

### carla_simulation_fsm.launch

(find-pkg-share autoware_launch)/launch/autoware.launch.xml"

### carla_simulation_carla.launch

autoware_launch

## run_carla_gnss_ros2.sh

gnss.launch.xml

## run_lidar_slam_ros2.sh

lidarslam.launch.py

## **autoware_launch.xml**


## localization node list

```bash

/localization/localization_error_monitor
/localization/pose_estimator/ndt_scan_matcher
/localization/pose_estimator/transform_listener_impl_57e936088da8
/localization/pose_twist_fusion_filter/ekf_localizer
/localization/pose_twist_fusion_filter/stop_filter
/localization/pose_twist_fusion_filter/twist2accel
/localization/twist_estimator/gyro_odometer
/localization/twist_estimator/transform_listener_impl_5990160b90a0
/localization/util/crop_box_filter_measurement_range
/localization/util/default_ad_api/helpers/automatic_pose_initializer
/localization/util/pose_initializer_node
/localization/util/random_downsample_filter
/localization/util/transform_listener_impl_559561216108
/localization/util/voxel_grid_downsample_filter

```

## localization topic list

```bash

/localization/acceleration
/localization/debug/ellipse_marker
/localization/initialization_state

        /localization/kinematic_state

/localization/pose_estimator/debug/loaded_pointcloud_map
/localization/pose_estimator/exe_time_ms

/localization/pose_estimator/initial_pose_with_covariance
/localization/pose_estimator/initial_to_result_distance
/localization/pose_estimator/initial_to_result_distance_new
/localization/pose_estimator/initial_to_result_distance_old
/localization/pose_estimator/iteration_num

/localization/pose_estimator/monte_carlo_initial_pose_marker
/localization/pose_estimator/ndt_marker
/localization/pose_estimator/nearest_voxel_transformation_likelihood
/localization/pose_estimator/no_ground_nearest_voxel_transformation_likelihood
/localization/pose_estimator/no_ground_transform_probability
/localization/pose_estimator/points_aligned
/localization/pose_estimator/points_aligned_no_ground
        /localization/pose_estimator/pose
        /localization/pose_estimator/pose_with_covariance
/localization/pose_estimator/transform_probability
/localization/pose_twist_fusion_filter/biased_pose
/localization/pose_twist_fusion_filter/biased_pose_with_covariance
/localization/pose_twist_fusion_filter/debug
/localization/pose_twist_fusion_filter/debug/measured_pose
/localization/pose_twist_fusion_filter/debug/stop_flag
/localization/pose_twist_fusion_filter/estimated_yaw_bias
/localization/pose_twist_fusion_filter/kinematic_state
        /localization/pose_twist_fusion_filter/pose
        /localization/pose_twist_fusion_filter/twist
/localization/pose_twist_fusion_filter/twist_with_covariance
        /localization/pose_with_covariance
/localization/twist_estimator/gyro_twist
/localization/twist_estimator/gyro_twist_raw
        /localization/twist_estimator/twist_with_covariance
/localization/twist_estimator/twist_with_covariance_raw
/localization/util/crop_box_filter/debug/cyclic_time_ms
/localization/util/crop_box_filter/debug/processing_time_ms
/localization/util/crop_box_filter_measurement_range/crop_box_polygon
/localization/util/downsample/pointcloud
/localization/util/measurement_range/pointcloud
/localization/util/voxel_grid_downsample/pointcloud

```
