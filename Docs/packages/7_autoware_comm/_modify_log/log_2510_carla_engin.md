# this is the record of changes of carla_engine

```ini
[/Script/Engine.Engine]
bSmoothFrameRate=false
SmoothedFrameRateRange=(LowerBound=(Type="ERangeBoundTypes::Inclusive",Value=22),UpperBound=(Type="ERangeBoundTypes::Exclusive",Value=120))

[/Script/HardwareTargeting.HardwareTargetingSettings]
TargetedHardwareClass=Desktop
AppliedTargetedHardwareClass=Desktop
DefaultGraphicsPerformance=Maximum
AppliedDefaultGraphicsPerformance=Maximum

[/Script/EngineSettings.GameMapsSettings]
EditorStartupMap=/Game/Carla/Maps/Town10HD_Opt.Town10HD_Opt
GameDefaultMap=/Game/Carla/Maps/Town10HD_Opt.Town10HD_Opt
ServerDefaultMap=/Game/Carla/Maps/Town10HD_Opt.Town10HD_Opt
GlobalDefaultGameMode=/Game/Carla/Blueprints/Game/CarlaGameMode.CarlaGameMode_C
GameInstanceClass=/Script/Carla.CarlaGameInstance
TransitionMap=/Game/Carla/Maps/Town10HD_Opt.Town10HD_Opt
GlobalDefaultServerGameMode=/Game/Carla/Blueprints/Game/CarlaGameMode.CarlaGameMode_C

[/Script/Engine.RendererSettings]
r.DefaultFeature.MotionBlur=True
r.BasePassOutputsVelocity=True
r.BasePassForceOutputsVelocity=False
r.AllowStaticLighting=True
r.DiscardUnusedQuality=True
r.DefaultFeature.Bloom=False
r.DefaultFeature.AmbientOcclusion=False
r.DefaultFeature.AmbientOcclusionStaticFraction=False
r.DefaultFeature.AutoExposure=False
r.CustomDepth=3
r.Streaming.PoolSize=4000
r.TextureStreaming=True
r.GenerateMeshDistanceFields=True
r.DistanceFieldBuild.EightBit=False
r.DistanceFieldBuild.Compress=False
r.DistanceFields.AtlasSizeXY=1024
r.DistanceFields.AtlasSizeZ=2048
r.DefaultFeature.AutoExposure.ExtendDefaultLuminanceRange=True
r.DefaultFeature.AntiAliasing=2
r.VirtualTextures=True

[/Script/AIModule.AISense_Sight]
bAutoRegisterAllPawnsAsSources=False
bAutoRegisterNewPawnsAsSources=False

[/Script/NavigationSystem.RecastNavMesh]
RuntimeGeneration=Static

[/Script/AIModule.CrowdManager]
MaxAgents=1000

[/Script/LinuxTargetPlatform.LinuxTargetSettings]
SpatializationPlugin=
ReverbPlugin=
OcclusionPlugin=
-TargetedRHIs=SF_VULKAN_SM5
-TargetedRHIs=GLSL_430
+TargetedRHIs=SF_VULKAN_SM5
+TargetedRHIs=GLSL_430

[/Script/Engine.PhysicsSettings]
DefaultGravityZ=-980.000000
DefaultTerminalVelocity=4000.000000
DefaultFluidFriction=0.300000
SimulateScratchMemorySize=262144
RagdollAggregateThreshold=4
TriangleMeshTriangleMinAreaThreshold=5.000000
bEnableShapeSharing=False
bEnablePCM=False
bEnableStabilization=False
bWarnMissingLocks=True
bEnable2DPhysics=False
PhysicErrorCorrection=(PingExtrapolation=0.100000,PingLimit=100.000000,ErrorPerLinearDifference=1.000000,ErrorPerAngularDifference=1.000000,MaxRestoredStateError=1.000000,MaxLinearHardSnapDistance=400.000000,PositionLerp=0.000000,AngleLerp=0.400000,LinearVelocityCoefficient=100.000000,AngularVelocityCoefficient=10.000000,ErrorAccumulationSeconds=0.500000,ErrorAccumulationDistanceSq=15.000000,ErrorAccumulationSimilarity=100.000000)
LockedAxis=Invalid
DefaultDegreesOfFreedom=Full3D
BounceThresholdVelocity=200.000000
FrictionCombineMode=Average
RestitutionCombineMode=Average
MaxAngularVelocity=3600.000000
MaxDepenetrationVelocity=0.000000
ContactOffsetMultiplier=0.010000
MinContactOffset=0.000100
MaxContactOffset=1.000000
bSimulateSkeletalMeshOnDedicatedServer=True
DefaultShapeComplexity=CTF_UseSimpleAndComplex
bDefaultHasComplexCollision=True
bSuppressFaceRemapTable=False
bSupportUVFromHitResults=False
bDisableActiveActors=False
bDisableKinematicStaticPairs=False
bDisableKinematicKinematicPairs=False
bDisableCCD=False
bEnableEnhancedDeterminism=True
MaxPhysicsDeltaTime=0.333330
bSubstepping=True
bSubsteppingAsync=False
MaxSubstepDeltaTime=0.010000
MaxSubsteps=10
SyncSceneSmoothingFactor=0.000000
InitialAverageFrameRate=0.016667
PhysXTreeRebuildRate=10
DefaultBroadphaseSettings=(bUseMBPOnClient=False,bUseMBPOnServer=False,MBPBounds=(Min=(X=0.000000,Y=0.000000,Z=0.000000),Max=(X=0.000000,Y=0.000000,Z=0.000000),IsValid=0),MBPNumSubdivs=2)

[/Script/WindowsTargetPlatform.WindowsTargetSettings]
Compiler=Default
-TargetedRHIs=PCD3D_SM5
+TargetedRHIs=PCD3D_SM5
+TargetedRHIs=SF_VULKAN_SM5
+TargetedRHIs=PCD3D_ES31
DefaultGraphicsRHI=DefaultGraphicsRHI_DX12
MinimumOSVersion=MSOS_Vista
bTarget32Bit=False
AudioSampleRate=48000
AudioCallbackBufferFrameSize=1024
AudioNumBuffersToEnqueue=1
AudioMaxChannels=0
AudioNumSourceWorkers=4
SpatializationPlugin=
ReverbPlugin=
OcclusionPlugin=
CompressionOverrides=(bOverrideCompressionTimes=False,DurationThreshold=5.000000,MaxNumRandomBranches=0,SoundCueQualityIndex=0)
bUseAudioStreamCaching=False
CacheSizeKB=0
MaxChunkSizeOverrideKB=0
bResampleForDevice=False
MaxSampleRate=0.000000
HighSampleRate=0.000000
MedSampleRate=0.000000
LowSampleRate=0.000000
MinSampleRate=0.000000
CompressionQualityModifier=0.000000
AutoStreamingThreshold=0.000000
SoundCueCookQualityIndex=-1

[/Script/Engine.CollisionProfile]
+Profiles=(Name="CustomSensorCollision",CollisionEnabled=QueryOnly,bCanModify=True,ObjectTypeName="SensorObject",CustomResponses=((Channel="WorldStatic",Response=ECR_Ignore),(Channel="WorldDynamic",Response=ECR_Ignore),(Channel="Pawn",Response=ECR_Ignore),(Channel="Visibility",Response=ECR_Ignore),(Channel="Camera",Response=ECR_Ignore),(Channel="PhysicsBody",Response=ECR_Ignore),(Channel="Vehicle",Response=ECR_Ignore),(Channel="Destructible",Response=ECR_Ignore),(Channel="SensorObject"),(Channel="SensorTrace")),HelpMessage="Used for custom collision meshes for objects that has very complex meshes but we want them to appear in raycast based sensors")
+DefaultChannelResponses=(Channel=ECC_GameTraceChannel1,DefaultResponse=ECR_Ignore,bTraceType=False,bStaticObject=False,Name="SensorObject")
+DefaultChannelResponses=(Channel=ECC_GameTraceChannel2,DefaultResponse=ECR_Ignore,bTraceType=True,bStaticObject=False,Name="SensorTrace")
+DefaultChannelResponses=(Channel=ECC_GameTraceChannel3,DefaultResponse=ECR_Overlap,bTraceType=True,bStaticObject=False,Name="OverlapChannel")
+EditProfiles=(Name="BlockAll",CustomResponses=((Channel="SensorObject"),(Channel="SensorTrace")))
+EditProfiles=(Name="OverlapAll",CustomResponses=((Channel="SensorObject",Response=ECR_Overlap),(Channel="SensorTrace",Response=ECR_Overlap)))

```

您提供的配置文件涉及 **Unreal Engine**（UE）在运行 **CARLA** 仿真器过程中的多个设置，尤其是与渲染、物理、AI 和系统性能相关的参数。以下是对每个配置部分的详细解读及优化建议，以帮助您降低渲染负荷，减少因高渲染设置导致的崩溃风险。

### **1. [/Script/Engine.Engine]**

```ini
bSmoothFrameRate=false
SmoothedFrameRateRange=(LowerBound=(Type="ERangeBoundTypes::Inclusive",Value=22),UpperBound=(Type="ERangeBoundTypes::Exclusive",Value=120))
```

- **参数解释**：
  - `bSmoothFrameRate=false`：
    - **含义**：禁用帧率平滑。帧率平滑通常用于减少帧率波动，提高视觉体验。
    - **优化建议**：在高负荷情况下，禁用帧率平滑可以减少GPU的额外计算负担，有助于提升性能稳定性。
  - `SmoothedFrameRateRange`：
    - **含义**：定义帧率平滑的范围。下限为22 FPS，上限为120 FPS。
    - **优化建议**：由于`bSmoothFrameRate`已禁用，此设置不会生效。

### **2. [/Script/HardwareTargeting.HardwareTargetingSettings]**

```ini
TargetedHardwareClass=Desktop
AppliedTargetedHardwareClass=Desktop
DefaultGraphicsPerformance=Maximum
AppliedDefaultGraphicsPerformance=Maximum
```

- **参数解释**：
  - `TargetedHardwareClass=Desktop` 和 `AppliedTargetedHardwareClass=Desktop`：
    - **含义**：目标硬件类别为桌面，表示优化针对桌面级别的硬件。
  - `DefaultGraphicsPerformance=Maximum` 和 `AppliedDefaultGraphicsPerformance=Maximum`：
    - **含义**：默认和应用的图形性能设置为最高（Maximum）。
    - **优化建议**：将 `DefaultGraphicsPerformance` 调整为较低级别（如 `Low` 或 `Medium`），以减少渲染负荷。例如：
      ```ini
      DefaultGraphicsPerformance=Low
      AppliedDefaultGraphicsPerformance=Low
      ```

### **3. [/Script/EngineSettings.GameMapsSettings]**

```ini
EditorStartupMap=/Game/Carla/Maps/Town10HD_Opt.Town10HD_Opt
GameDefaultMap=/Game/Carla/Maps/Town10HD_Opt.Town10HD_Opt
ServerDefaultMap=/Game/Carla/Maps/Town10HD_Opt.Town10HD_Opt
GlobalDefaultGameMode=/Game/Carla/Blueprints/Game/CarlaGameMode.CarlaGameMode_C
GameInstanceClass=/Script/Carla.CarlaGameInstance
TransitionMap=/Game/Carla/Maps/Town10HD_Opt.Town10HD_Opt
GlobalDefaultServerGameMode=/Game/Carla/Blueprints/Game/CarlaGameMode.CarlaGameMode_C
```

- **参数解释**：
  - 这些设置定义了编辑器启动、游戏默认和服务器默认的地图，以及全局默认的游戏模式和游戏实例类。
  - **优化建议**：主要影响仿真器启动和地图加载行为，与渲染负荷关系不大。确保所选地图适合您的硬件配置，避免加载过于复杂的地图。

### **4. [/Script/Engine.RendererSettings]**

```ini
r.DefaultFeature.MotionBlur=True
r.BasePassOutputsVelocity=True
r.BasePassForceOutputsVelocity=False
r.AllowStaticLighting=True
r.DiscardUnusedQuality=True
r.DefaultFeature.Bloom=False
r.DefaultFeature.AmbientOcclusion=False
r.DefaultFeature.AutoExposure=False
r.CustomDepth=3
r.Streaming.PoolSize=4000
r.TextureStreaming=True
r.GenerateMeshDistanceFields=True
r.DistanceFieldBuild.EightBit=False
r.DistanceFieldBuild.Compress=False
r.DistanceFields.AtlasSizeXY=1024
r.DistanceFields.AtlasSizeZ=2048
r.DefaultFeature.AutoExposure.ExtendDefaultLuminanceRange=True
r.DefaultFeature.AntiAliasing=2
r.VirtualTextures=True
```

- **参数解释**：
  - `r.DefaultFeature.MotionBlur=True`：
    - **含义**：启用运动模糊效果。
    - **优化建议**：禁用运动模糊以减少GPU计算负担。
      ```ini
      r.DefaultFeature.MotionBlur=False
      ```
  
  - `r.DefaultFeature.Bloom=False`：
    - **含义**：禁用Bloom（辉光）效果。
    - **优化建议**：已经禁用，无需调整。
  
  - `r.DefaultFeature.AmbientOcclusion=False`：
    - **含义**：禁用环境光遮蔽效果。
    - **优化建议**：已经禁用，无需调整。
  
  - `r.DefaultFeature.AutoExposure=False`：
    - **含义**：禁用自动曝光。
    - **优化建议**：已经禁用，无需调整。
  
  - `r.AntiAliasing=2`：
    - **含义**：抗锯齿质量设为2（目前设定具体含义需参考Unreal Engine文档）。
    - **优化建议**：将抗锯齿质量降低，或完全禁用，以减少GPU负荷。例如：
      ```ini
      r.AntiAliasing=0
      ```
  
  - `r.Streaming.PoolSize=4000`：
    - **含义**：设置纹理流池大小为4000 MB。
    - **优化建议**：根据显存情况适当调整，过大的纹理流池可能占用过多显存。可以适当降低，例如：
      ```ini
      r.Streaming.PoolSize=2000
      ```
  
  - `r.GenerateMeshDistanceFields=True`：
    - **含义**：启用网格距离场的生成，用于全局光照和SSAO等效果。
    - **优化建议**：禁用以减少计算负担。
      ```ini
      r.GenerateMeshDistanceFields=False
      ```
  
  - `r.DistanceFields.AtlasSizeXY=1024` 和 `r.DistanceFields.AtlasSizeZ=2048`：
    - **含义**：设置距离场的图集尺寸。
    - **优化建议**：如果需要保留距离场，可以适当降低尺寸，以减少内存占用：
      ```ini
      r.DistanceFields.AtlasSizeXY=512
      r.DistanceFields.AtlasSizeZ=1024
      ```
  
  - `r.VirtualTextures=True`：
    - **含义**：启用虚拟纹理。
    - **优化建议**：禁用虚拟纹理以减少显存占用和渲染负担。
      ```ini
      r.VirtualTextures=False
      ```

### **5. [/Script/AIModule.AISense_Sight]**

```ini
bAutoRegisterAllPawnsAsSources=False
bAutoRegisterNewPawnsAsSources=False
```

- **参数解释**：
  - 这些设置控制AI系统是否自动将所有Pawn（角色）注册为感知源。
  - **优化建议**：根据AI角色数量和需求调整。如果不需要所有Pawn作为感知源，可以保持禁用，以减少AI计算开销。

### **6. [/Script/NavigationSystem.RecastNavMesh]**

```ini
RuntimeGeneration=Static
```

- **参数解释**：
  - `RuntimeGeneration=Static`：
    - **含义**：导航网格在运行时为静态生成，不会动态更新。
    - **优化建议**：如果环境不动态变化，保持静态模式可以减少运行时计算负担。无需调整。

### **7. [/Script/AIModule.CrowdManager]**

```ini
MaxAgents=1000
```

- **参数解释**：
  - `MaxAgents=1000`：
    - **含义**：设置最大AI代理数量为1000。
    - **优化建议**：根据实际需求和硬件性能，可能需要降低此值以减少AI计算负担。例如：
      ```ini
      MaxAgents=500
      ```

### **8. [/Script/LinuxTargetPlatform.LinuxTargetSettings]**

```ini
SpatializationPlugin=
ReverbPlugin=
OcclusionPlugin=
-TargetedRHIs=SF_VULKAN_SM5
-TargetedRHIs=GLSL_430
+TargetedRHIs=SF_VULKAN_SM5
+TargetedRHIs=GLSL_430
```

- **参数解释**：
  - `SpatializationPlugin`、`ReverbPlugin`、`OcclusionPlugin`为空，表明未启用特定的音频插件。
  - `TargetedRHIs`：
    - **含义**：指定目标渲染后端接口（RHI），包括Vulkan 和 GLSL 4.30。
    - **优化建议**：确保仅启用必要的RHI，以避免不必要的资源占用。如果主要使用Vulkan，可以确保其他RHI配置正确无误。

### **9. [/Script/Engine.PhysicsSettings]**

```ini
... （省略部分参数）
bSimulateSkeletalMeshOnDedicatedServer=True
...
bEnableEnhancedDeterminism=True
...
bSubstepping=True
...
```

- **参数解释**：
  - **物理设置** 涉及模拟物理世界的各项参数，包括重力、速度限制、碰撞设置等。
  - `bSubstepping=True`：
    - **含义**：启用物理子步进，提高物理模拟精度。
    - **优化建议**：启用子步进会增加CPU负荷，特别是在有大量物理对象时。可以适当降低 `MaxSubsteps` 或禁用子步进：
      ```ini
      bSubstepping=False
      ```
  
  - `bEnableEnhancedDeterminism=True`：
    - **含义**：启用增强的确定性，确保物理模拟的一致性。
    - **优化建议**：根据需求决定是否关闭。如果关闭，可以减少物理计算开销，但可能影响模拟精度。
      ```ini
      bEnableEnhancedDeterminism=False
      ```
  
  - **其他参数**：
    - `MaxPhysicsDeltaTime=0.333330`、`MaxSubstepDeltaTime=0.010000`、`MaxSubsteps=10` 等：
      - **含义**：控制物理引擎的时间步长和子步数量。
      - **优化建议**：适当降低 `MaxSubsteps` 或增大 `MaxPhysicsDeltaTime` 以减少计算频率：
        ```ini
        MaxSubsteps=5
        MaxPhysicsDeltaTime=0.5
        ```

### **10. [/Script/WindowsTargetPlatform.WindowsTargetSettings]**

```ini
Compiler=Default
-TargetedRHIs=PCD3D_SM5
+TargetedRHIs=PCD3D_SM5
+TargetedRHIs=SF_VULKAN_SM5
+TargetedRHIs=PCD3D_ES31
DefaultGraphicsRHI=DefaultGraphicsRHI_DX12
MinimumOSVersion=MSOS_Vista
...
```

- **参数解释**：
  - `TargetedRHIs`：
    - **含义**：指定目标渲染后端接口，包括 DirectX 12 (SM5)、Vulkan 和 OpenGL ES 3.1。
    - **优化建议**：如果主要使用Vulkan，可以减少其他RHI的支持，以优化资源利用。例如，仅保留Vulkan：
      ```ini
      TargetedRHIs=SF_VULKAN_SM5
      ```
    - 确保 `DefaultGraphicsRHI` 设置为合适的渲染后端：
      ```ini
      DefaultGraphicsRHI=Vulkan
      ```

### **11. [/Script/Engine.CollisionProfile]**

```ini
+Profiles=(Name="CustomSensorCollision",CollisionEnabled=QueryOnly,bCanModify=True,ObjectTypeName="SensorObject",CustomResponses=((Channel="WorldStatic",Response=ECR_Ignore),(Channel="WorldDynamic",Response=ECR_Ignore),(Channel="Pawn",Response=ECR_Ignore),(Channel="Visibility",Response=ECR_Ignore),(Channel="Camera",Response=ECR_Ignore),(Channel="PhysicsBody",Response=ECR_Ignore),(Channel="Vehicle",Response=ECR_Ignore),(Channel="Destructible",Response=ECR_Ignore),(Channel="SensorObject"),(Channel="SensorTrace")),HelpMessage="Used for custom collision meshes for objects that has very complex meshes but we want them to appear in raycast based sensors")
+DefaultChannelResponses=(Channel=ECC_GameTraceChannel1,DefaultResponse=ECR_Ignore,bTraceType=False,bStaticObject=False,Name="SensorObject")
+DefaultChannelResponses=(Channel=ECC_GameTraceChannel2,DefaultResponse=ECR_Ignore,bTraceType=True,bStaticObject=False,Name="SensorTrace")
+DefaultChannelResponses=(Channel=ECC_GameTraceChannel3,DefaultResponse=ECR_Overlap,bTraceType=True,bStaticObject=False,Name="OverlapChannel")
+EditProfiles=(Name="BlockAll",CustomResponses=((Channel="SensorObject"),(Channel="SensorTrace")))
+EditProfiles=(Name="OverlapAll",CustomResponses=((Channel="SensorObject",Response=ECR_Overlap),(Channel="SensorTrace",Response=ECR_Overlap)))
```

- **参数解释**：
  - **自定义碰撞配置**：
    - `CustomSensorCollision`：
      - **含义**：定义了一个名为“CustomSensorCollision”的碰撞配置，仅进行查询（QueryOnly），忽略多种碰撞通道。
      - **优化建议**：确保此配置仅应用于必要的传感器对象，以减少不必要的碰撞检测开销。
  
  - **默认通道响应**：
    - `ECR_Ignore` 和 `ECR_Overlap`：
      - **含义**：设置不同碰撞通道的默认响应行为，如忽略、重叠等。
      - **优化建议**：根据实际需求调整碰撞响应，避免过多的碰撞检测操作。例如，仅启用必要的通道响应。

### **优化总结与建议**

基于上述配置解析，以下是针对降低渲染负荷和提升稳定性的具体优化建议：

1. **降低图形性能设置**：
   - 将 `DefaultGraphicsPerformance` 从 `Maximum` 调整为 `Low` 或 `Medium`。
     ```ini
     DefaultGraphicsPerformance=Low
     AppliedDefaultGraphicsPerformance=Low
     ```
   
2. **禁用高负荷渲染效果**：
   - 关闭运动模糊、自动曝光和抗锯齿，减少GPU计算负担。
     ```ini
     r.DefaultFeature.MotionBlur=False
     r.DefaultFeature.AutoExposure=False
     r.AntiAliasing=0
     ```
   
3. **调整纹理和距离场设置**：
   - 降低纹理流池大小，禁用或优化距离场生成，以减少显存和计算资源的占用。
     ```ini
     r.Streaming.PoolSize=2000
     r.GenerateMeshDistanceFields=False
     r.VirtualTextures=False
     ```
   
4. **优化AI和物理设置**：
   - 减少AI代理数量，降低物理模拟复杂度，关闭不必要的物理功能。
     ```ini
     MaxAgents=500
     bSubstepping=False
     bEnableEnhancedDeterminism=False
     ```
   
5. **调整渲染后端接口（RHI）**：
   - 根据实际需求，仅启用必要的渲染后端接口，避免不必要的资源占用。
     ```ini
     TargetedRHIs=SF_VULKAN_SM5
     DefaultGraphicsRHI=Vulkan
     ```
   
6. **优化碰撞配置**：
   - 确保碰撞配置仅针对必要的对象，避免过多的碰撞检测操作，减少CPU开销。
     ```ini
     CollisionProfiles=(CustomSensorCollision)
     ```

7. **验证并调整物理设置**：
   - 适当调整 `PhysX` 相关参数，平衡物理模拟精度与性能。
     ```ini
     MaxSubsteps=5
     MaxPhysicsDeltaTime=0.5
     ```

8. **优化系统资源分配**：
   - 确保系统有足够的物理内存和显存，避免因资源不足导致的不稳定性。
   - 关闭其他占用大量资源的应用程序，释放更多资源给CARLA和Unreal Engine使用。

### **额外建议**

- **更新GPU驱动程序**：
  - 确保您的GPU驱动程序是最新版本，以获得最佳的Vulkan支持和性能优化。
  
- **监控系统状态**：
  - 使用工具如 `nvidia-smi` 实时监控GPU的负载和温度，确保硬件运行在安全范围内。

- **逐步调整与测试**：
  - 在进行配置修改后，逐步启动CARLA并测试其稳定性。记录每次修改的效果，以找到最佳的平衡点。

- **参考官方文档**：
  - 查阅Unreal Engine和CARLA的官方文档，了解各项配置参数的详细说明和最佳实践。

通过以上优化，您应该能够有效降低渲染负荷，提升CARLA仿真器的运行稳定性，减少因高渲染设置导致的崩溃风险。如果在实施过程中遇到任何问题，欢迎随时提问！