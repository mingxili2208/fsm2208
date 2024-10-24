# this file is the log of the debug of ue4 break

## problems

1. with test the memory do will out when we initialize the ./debug```
2. tody we need to start the program one by one to log the memory changes

    ``` bash

        # 0. start world
        python3 ${OP_BRIDGE_ROOT}/op_bridge/fsm_lab_simulation/world_launcher.py
        # with error message 1

        # 1.0 start car
        # 1.1 original version
        python3 ${OP_BRIDGE_ROOT}/op_bridge/fsm_lab_simulation/vehicle_launcher.py
        # 1.2 test version
        python3 ${OP_BRIDGE_ROOT}/op_bridge/fsm_lab_simulation/test_vehicle_launcher.py
        
    ```


    ```cpp
        //message 1
        LoginId:2af007ef88cc4c94bbab9a0a654bfe7e-000003e8
        EpicAccountId:

        Fatal error: [File:/home/cityu-fsm-lab-carla/Workspace/UnrealEngine/Engine/Source/Runtime/VulkanRHI/Private/VulkanUtil.cpp] [Line: 803] Result failed, VkResult=-4 at /home/cityu-fsm-lab-carla/Workspace/UnrealEngine/Engine/Source/Runtime/VulkanRHI/Private/VulkanMemory.cpp:4743 with error VK_ERROR_DEVICE_LOST << callstack too long >>

        libUE4Editor-Core.so!FGenericPlatformMisc::RaiseException(unsigned int) [/home/cityu-fsm-lab-carla/Workspace/UnrealEngine/Engine/Source/Runtime/Core/Private/GenericPlatform/GenericPlatformMisc.cpp:472]
        libUE4Editor-Core.so!FOutputDevice::LogfImpl(char16_t const*, ...) [/home/cityu-fsm-lab-carla/Workspace/UnrealEngine/Engine/Source/Runtime/Core/Private/Misc/OutputDevice.cpp:61]
        libUE4Editor-VulkanRHI.so!VulkanRHI::VerifyVulkanResult(VkResult, char const*, char const*, unsigned int) [/home/cityu-fsm-lab-carla/Workspace/UnrealEngine/Engine/Source/Runtime/VulkanRHI/Private/VulkanUtil.cpp:802]
        libUE4Editor-VulkanRHI.so!VulkanRHI::FFenceManager::WaitForFence(VulkanRHI::FFence*, unsigned long long) [/home/cityu-fsm-lab-carla/Workspace/UnrealEngine/Engine/Source/Runtime/VulkanRHI/Private/VulkanMemory.cpp:4743]
        libUE4Editor-VulkanRHI.so!FVulkanCommandBufferManager::WaitForCmdBuffer(FVulkanCmdBuffer*, float) [/home/cityu-fsm-lab-carla/Workspace/UnrealEngine/Engine/Source/Runtime/VulkanRHI/Private/VulkanCommandBuffer.cpp:457]
        libUE4Editor-VulkanRHI.so!FVulkanDynamicRHI::RHIGetRenderQueryResult(FRHIRenderQuery*, unsigned long long&, bool, unsigned int) [/home/cityu-fsm-lab-carla/Workspace/UnrealEngine/Engine/Source/Runtime/VulkanRHI/Private/VulkanQuery.cpp:513]
        libUE4Editor-RenderCore.so!FRealtimeGPUProfilerEvent::GatherQueryResults(FRHICommandListImmediate&) [/home/cityu-fsm-lab-carla/Workspace/UnrealEngine/Engine/Source/Runtime/RenderCore/Private/ProfilingDebugging/RealtimeGPUProfiler.cpp:171]
        libUE4Editor-RenderCore.so!FRealtimeGPUProfilerFrame::UpdateStats(FRHICommandListImmediate&) [/home/cityu-fsm-lab-carla/Workspace/UnrealEngine/Engine/Source/Runtime/RenderCore/Private/ProfilingDebugging/RealtimeGPUProfiler.cpp:416]
        libUE4Editor-RenderCore.so!FRealtimeGPUProfiler::EndFrame(FRHICommandListImmediate&) [/home/cityu-fsm-lab-carla/Workspace/UnrealEngine/Engine/Source/Runtime/RenderCore/Private/ProfilingDebugging/RealtimeGPUProfiler.cpp:839]
        UE4Editor!FEngineLoop::Tick()::$_83::operator()(FRHICommandListImmediate&) const [/home/cityu-fsm-lab-carla/Workspace/UnrealEngine/Engine/Source/Runtime/Launch/Private/LaunchEngineLoop.cpp:5071]
        UE4Editor!TEnqueueUniqueRenderCommandType<FEngineLoop::Tick()::EndFrameName, FEngineLoop::Tick()::$_83>::DoTask(ENamedThreads::Type, TRefCountPtr<FGraphEvent> const&) [/home/cityu-fsm-lab-carla/Workspace/UnrealEngine/Engine/Source/Runtime/RenderCore/Public/RenderingThread.h:183]
        UE4Editor!TGraphTask<TEnqueueUniqueRenderCommandType<FEngineLoop::Tick()::EndFrameName, FEngineLoop::Tick()::$_83> >::ExecuteTask(TArray<FBaseGraphTask*, TSizedDefaultAllocator<32> >&, ENamedThreads::Type) [/home/cityu-fsm-lab-carla/Workspace/UnrealEngine/Engine/Source/Runtime/Core/Public/Async/TaskGraphInterfaces.h:886]
        libUE4Editor-Core.so!FNamedTaskThread::ProcessTasksNamedThread(int, bool) [/home/cityu-fsm-lab-carla/Workspace/UnrealEngine/Engine/Source/Runtime/Core/Private/Async/TaskGraph.cpp:709]
        libUE4Editor-Core.so!FNamedTaskThread::ProcessTasksUntilQuit(int) [/home/cityu-fsm-lab-carla/Workspace/UnrealEngine/Engine/Source/Runtime/Core/Private/Async/TaskGraph.cpp:600]
        libUE4Editor-RenderCore.so!RenderingThreadMain(FEvent*) [/home/cityu-fsm-lab-carla/Workspace/UnrealEngine/Engine/Source/Runtime/RenderCore/Private/RenderingThread.cpp:372]
        libUE4Editor-RenderCore.so!FRenderingThread::Run() [/home/cityu-fsm-lab-carla/Workspace/UnrealEngine/Engine/Source/Runtime/RenderCore/Private/RenderingThread.cpp:526]
        libUE4Editor-Core.so!FRunnableThreadPThread::Run() [/home/cityu-fsm-lab-carla/Workspace/UnrealEngine/Engine/Source/Runtime/Core/Private/HAL/PThreadRunnableThread.cpp:25]
        libUE4Editor-Core.so!FRunnableThreadPThread::_ThreadProc(void*) [/home/cityu-fsm-lab-carla/Workspace/UnrealEngine/Engine/Source/Runtime/Core/Private/HAL/PThreadRunnableThread.h:185]
        libc.so.6!UnknownFunction(0x94ac2)
        libc.so.6!UnknownFunction(0x12684f)
    ```

    log info while load world

    ```log
    WARNING: cannot parse georeference: ''. Using default values. 
    [2024.10.17-06.51.23:987][300]LogCarla: There are 47 SpawnPoints in the map
    [2024.10.17-06.51.23:988][300]LogWorld: Bringing up level for play took: 0.087469
    [2024.10.17-06.51.23:988][300]LogGameMode: FindPlayerStart: PATHS NOT DEFINED or NO PLAYERSTART with positive rating
    [2024.10.17-06.51.23:988][300]LogBlueprintUserMessages: [BP_Sky_2] Altitude: 40.0
    [2024.10.17-06.51.23:988][300]LogBlueprintUserMessages: [BP_Sky_2] Fog: 10.0
    [2024.10.17-06.51.23:988][300]LogBlueprintUserMessages: [BP_Sky_2] Level: 
    [2024.10.17-06.51.23:995][300]LogCarlaServer: New episode 'fsm_lab_sandbox_right_hand_driving_scene' started
    [2024.10.17-06.51.23:996][300]LogBlueprintUserMessages: [BP_Sky_2] Altitude: 75.0
    [2024.10.17-06.51.23:996][300]LogBlueprintUserMessages: [BP_Sky_2] Fog: 0.0
    [2024.10.17-06.51.23:996][300]LogBlueprintUserMessages: [BP_Sky_2] Level: 
    [2024.10.17-06.51.23:997][300]LogBlueprintUserMessages: [BP_Weather_C_0] fsm_lab_sandbox_right_hand_driving_scene default weather not found
    [2024.10.17-06.51.23:997][300]LogBlueprintUserMessages: [BP_Sky_2] Altitude: 45.0
    [2024.10.17-06.51.23:998][300]LogBlueprintUserMessages: [BP_Sky_2] Fog: 2.0
    [2024.10.17-06.51.23:998][300]LogBlueprintUserMessages: [BP_Sky_2] Level: 
    [2024.10.17-06.51.23:998][300]LogLoad: Took 0.667752 seconds to LoadMap(/Game/fsm_lab_map_package/Maps/fsm_lab_sandbox_right_hand_driving_scene/fsm_lab_sandbox_right_hand_driving_scene)
    [2024.10.17-06.51.24:068][302]LogStaticMesh: Allocated 1024x1024x256 distance field atlas = 512.0Mb, with 1081 objects containing 474.6Mb backing data
    [2024.10.17-06.51.24:786][307]LogTemp: Loaded OpenDrive file '/home/cityu-fsm-lab-carla/Workspace/Carla/carla-0.9.15/Unreal/CarlaUE4/Content/fsm_lab_map_package/Maps/fsm_lab_sandbox_right_hand_driving_scene/OpenDrive/fsm_lab_sandbox_right_hand_driving_scene.xodr'

    ```
