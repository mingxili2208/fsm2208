# recording the topics of control autoware

/planning/mission_planning/goal this is for set goals

---
header:
  stamp:
    sec: 1742471016
    nanosec: 263420553
  frame_id: map
pose:
  position:
    x: -48.14820861816406
    y: -24.610034942626953
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: -0.7245607855925108
    w: 0.6892109023960398
---

/autoware/engage       this is for start autoware

---
stamp:
  sec: 1223
  nanosec: 900018237
engage: true
---



/api/autoware/get/emergency
/api/autoware/get/engage
/api/autoware/get/map/info/hash
/api/external/get/command/selected/control
/api/external/get/rtc_status
/api/external/set/command/local/control
/api/external/set/command/local/heartbeat
/api/external/set/command/local/shift
/api/external/set/command/local/turn_signal
/api/external/set/command/remote/control
/api/external/set/command/remote/heartbeat
/api/external/set/command/remote/shift
/api/external/set/command/remote/turn_signal
/api/fail_safe/mrm_state
/api/localization/initialization_state
/api/motion/state
/api/operation_mode/state
/api/perception/objects
/api/planning/steering_factors
/api/planning/velocity_factors
/api/routing/route
/api/routing/state
/api/vehicle/kinematics
/api/vehicle/status
/autoware/engage
/autoware/state
