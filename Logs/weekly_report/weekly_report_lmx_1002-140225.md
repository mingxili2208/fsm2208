# LIMINGXI Weekly Report for 2025/02/10--2025/02/14

## Planned

1. Recalibrate the alignment between the sand table and VR using a high-precision laser sensor.  
2. Measure delays in various system components, including tracker refresh rate, control system response time, and data processing delays.  
3. Research and test Carla and Autoware running in synchronous mode.  
4. Update the car model to match the correct scale.  

## Plan Completion Progress (Working)

1. Progress made in debugging the regional location compensation algorithm on the sand table map.  
2. Preliminary delay measurement experiments conducted for tracker refresh rate and control system response time. Further refinement required.  

## Finished

1. Measured tracker refresh rate using SteamVR, confirming it operates at 30Hz in the current configuration. Identified potential improvements with a fixed_delta_seconds update interval of 1/30 in Carla.  
2. Selected a suitable chassis for the scaled car model.  
3. Conducted research on Carla's synchronous mode operation.  

## Next Work Plan

1. Collect sand table data points and recalibrate the positioning alignment.  
2. Develop and test methods for measuring control system response time, tracker delay, and overall data processing delay.  
3. Complete the synchronization settings between Carla and Autoware and calibrate the time flow speed.  
4. Reshoot the small car's vehicle avoidance behavior in Carla.