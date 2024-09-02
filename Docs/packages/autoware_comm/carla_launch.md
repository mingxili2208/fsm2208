# this file is to describe how to launch the carla

```sh
if ${LAUNCH_UE4_EDITOR} ; then

  log "Launching UE4Editor..."
  ${GDB} ${UE4_ROOT}/Engine/Binaries/Linux/UE4Editor "${PWD}/CarlaUE4.uproject" ${RHI} ${EDITOR_FLAGS}

else

  log "Success!"
fi
```

```sh
BuildCarlaUE4.sh: Launching UE4Editor...
Increasing per-process limit of core file size to infinity.
LogInit: LLM is enabled
LogInit: LLM CsvWriter: off TraceWriter: off
LogInit: Display: Running engine for game: CarlaUE4
LogPlatformFile: Not using cached read wrapper
LogInit: NumberOfWorkerThreadsToSpawn:
LogInit:  - Number of physical cores available for the process: 12
LogInit:  - Number of logical cores available for the process: 20
LogInit:  - Worker number by default: 11 (you can change this number with the command line parameter '-workersthreadpool X', for a max of 26 threads)
LogTaskGraph: Started task graph with 5 named threads and 38 total threads with 3 sets of task threads.
LogStats: Stats thread started at 0.068915
LogInit: NumberOfWorkerThreadsToSpawn:
LogInit:  - Number of physical cores available for the process: 12
LogInit:  - Number of logical cores available for the process: 20
LogInit:  - Worker number by default: 11 (you can change this number with the command line parameter '-workersthreadpool X', for a max of 26 threads)
LogICUInternationalization: ICU TimeZone Detection - Raw Offset: +8:00, Platform Override: ''
LogPluginManager: Mounting plugin XGEController
LogPluginManager: Mounting plugin MeshPainting
LogPluginManager: Mounting plugin ChaosNiagara
LogPluginManager: Mounting plugin ChaosCloth
LogPluginManager: Mounting plugin GeometryCache
LogPluginManager: Mounting plugin ChaosSolverPlugin
LogPluginManager: Mounting plugin GeometryProcessing
LogPluginManager: Mounting plugin SkeletalReduction
LogPluginManager: Mounting plugin GeometryCollectionPlugin
LogPluginManager: Mounting plugin BackChannel
LogPluginManager: Mounting plugin ProxyLODPlugin
LogPluginManager: Mounting plugin AlembicImporter
LogPluginManager: Mounting plugin ChaosEditor
LogPluginManager: Mounting plugin PythonScriptPlugin
LogPluginManager: Mounting plugin PlatformCrypto
LogPluginManager: Mounting plugin CharacterAI
LogPluginManager: Mounting plugin AutomationUtils
LogPluginManager: Mounting plugin MotoSynth
LogPluginManager: Mounting plugin PlanarCut
LogPluginManager: Mounting plugin ChaosClothEditor
LogPluginManager: Mounting plugin ImgMedia
LogPluginManager: Mounting plugin MediaPlayerEditor
LogPluginManager: Mounting plugin WmfMedia
LogPluginManager: Mounting plugin WebMMedia
LogPluginManager: Mounting plugin AvfMedia
LogPluginManager: Mounting plugin MediaCompositing
LogPluginManager: Mounting plugin WindowsMoviePlayer
LogPluginManager: Mounting plugin ActorLayerUtilities
LogPluginManager: Mounting plugin EditableMesh
LogPluginManager: Mounting plugin RuntimePhysXCooking
LogPluginManager: Mounting plugin WebMMoviePlayer
LogPluginManager: Mounting plugin AudioSynesthesia
LogPluginManager: Mounting plugin PropertyAccessEditor
LogPluginManager: Mounting plugin Synthesis
LogPluginManager: Mounting plugin ChunkDownloader
LogPluginManager: Mounting plugin CableComponent
LogPluginManager: Mounting plugin LinuxDeviceProfileSelector
LogPluginManager: Mounting plugin PostSplashScreen
LogPluginManager: Mounting plugin PhysXVehicles
LogPluginManager: Mounting plugin AssetTags
LogPluginManager: Mounting plugin ProceduralMeshComponent
LogPluginManager: Mounting plugin AppleImageUtils
LogPluginManager: Mounting plugin CustomMeshComponent
LogPluginManager: Mounting plugin GooglePAD
LogPluginManager: Mounting plugin ArchVisCharacter
LogPluginManager: Mounting plugin SoundFields
LogPluginManager: Mounting plugin AudioCapture
LogPluginManager: Mounting plugin SignificanceManager
LogPluginManager: Mounting plugin HairStrands
LogPluginManager: Mounting plugin LauncherChunkInstaller
LogPluginManager: Mounting plugin TemplateSequence
LogPluginManager: Mounting plugin ScreenshotTools
LogPluginManager: Mounting plugin AlembicHairImporter
LogPluginManager: Mounting plugin OnlineSubsystemNull
LogPluginManager: Mounting plugin OnlineSubsystem
LogPluginManager: Mounting plugin OnlineSubsystemUtils
LogPluginManager: Mounting plugin DatasmithContent
LogPluginManager: Mounting plugin ActorSequence
LogPluginManager: Mounting plugin LevelSequenceEditor
LogPluginManager: Mounting plugin PerformanceMonitor
LogPluginManager: Mounting plugin VariantManagerContent
LogPluginManager: Mounting plugin LightPropagationVolume
LogPluginManager: Mounting plugin CameraShakePreviewer
LogPluginManager: Mounting plugin MacGraphicsSwitching
LogPluginManager: Mounting plugin GeometryMode
LogPluginManager: Mounting plugin AssetManagerEditor
LogPluginManager: Mounting plugin DataValidation
LogPluginManager: Mounting plugin FacialAnimation
LogPluginManager: Mounting plugin CryptoKeys
LogPluginManager: Mounting plugin MagicLeap
LogPluginManager: Mounting plugin MagicLeapMedia
LogPluginManager: Mounting plugin MLSDK
LogPluginManager: Mounting plugin MagicLeapLightEstimation
LogPluginManager: Mounting plugin MagicLeapPassableWorld
LogPluginManager: Mounting plugin LuminPlatformFeatures
LogPluginManager: Mounting plugin EnvironmentQueryEditor
LogPluginManager: Mounting plugin MatineeToLevelSequence
LogPluginManager: Mounting plugin AISupport
LogPluginManager: Mounting plugin Paper2D
LogPluginManager: Mounting plugin GameplayTagsEditor
LogPluginManager: Mounting plugin PropertyAccessNode
LogPluginManager: Mounting plugin MaterialAnalyzer
LogPluginManager: Mounting plugin SubversionSourceControl
LogPluginManager: Mounting plugin SpeedTreeImporter
LogPluginManager: Mounting plugin VisualStudioCodeSourceCodeAccess
LogPluginManager: Mounting plugin PluginBrowser
LogPluginManager: Mounting plugin VisualStudioSourceCodeAccess
LogPluginManager: Mounting plugin TcpMessaging
LogPluginManager: Mounting plugin CurveEditorTools
LogPluginManager: Mounting plugin CodeLiteSourceCodeAccess
LogPluginManager: Mounting plugin PlasticSourceControl
LogPluginManager: Mounting plugin RiderSourceCodeAccess
LogPluginManager: Mounting plugin RenderDocPlugin
LogPluginManager: Mounting plugin UdpMessaging
LogPluginManager: Mounting plugin AnimationSharing
LogPluginManager: Mounting plugin KDevelopSourceCodeAccess
LogPluginManager: Mounting plugin CLionSourceCodeAccess
LogPluginManager: Mounting plugin PerforceSourceControl
LogPluginManager: Mounting plugin GitSourceControl
LogPluginManager: Mounting plugin EditorScriptingUtilities
LogPluginManager: Mounting plugin XCodeSourceCodeAccess
LogPluginManager: Mounting plugin NullSourceCodeAccess
LogPluginManager: Mounting plugin Niagara
LogPluginManager: Mounting plugin PluginUtils
LogPluginManager: Mounting plugin OnlineSubsystemIOS
LogPluginManager: Mounting plugin ContentBrowserAssetDataSource
LogPluginManager: Mounting plugin ContentBrowserFileDataSource
LogPluginManager: Mounting plugin ContentBrowserClassDataSource
LogPluginManager: Mounting plugin CarlaExporter
LogPluginManager: Mounting plugin StreetMap
LogPluginManager: Mounting plugin Carla
LogPluginManager: Mounting plugin CarlaTools
LogInit: Using libcurl 7.65.3-DEV
LogInit:  - built for x86_64-unknown-linux-gnu
LogInit:  - supports SSL with OpenSSL/1.1.1c
LogInit:  - supports HTTP deflate (compression) using libz 1.2.8
LogInit:  - other features:
LogInit:      CURL_VERSION_SSL
LogInit:      CURL_VERSION_LIBZ
LogInit:      CURL_VERSION_IPV6
LogInit:      CURL_VERSION_ASYNCHDNS
LogInit:      CURL_VERSION_LARGEFILE
LogInit:      CURL_VERSION_TLSAUTH_SRP
LogInit:  CurlRequestOptions (configurable via config and command line):
LogInit:  - bVerifyPeer = true  - Libcurl will verify peer certificate
LogInit:  - bUseHttpProxy = false  - Libcurl will NOT use HTTP proxy
LogInit:  - bDontReuseConnections = false  - Libcurl will reuse connections
LogInit:  - MaxHostConnections = 16  - Libcurl will limit the number of connections to a host
LogInit:  - LocalHostAddr = Default
LogInit:  - BufferSize = 65536
LogOnline: OSS: Creating online subsystem instance for: NULL
LogOnline: OSS: TryLoadSubsystemAndSetDefault: Loaded subsystem for module [NULL]
LogInit: Build: ++UE4+Release-4.26-CL-0
LogInit: Engine Version: 4.26.2-0+++UE4+Release-4.26
LogInit: Compatible Engine Version: 4.26.0-0+++UE4+Release-4.26
LogInit: Net CL: 0
LogInit: OS: GenericOSVersionLabel (GenericOSSubVersionLabel), CPU: 12th Gen Intel(R) Core(TM) i7-12700, GPU: GenericGPUBrand
LogInit: Compiled (64-bit): Apr 19 2024 00:52:31
LogInit: Compiled with Clang: 10.0.1 
LogInit: Build Configuration: Development
LogInit: Branch Name: ++UE4+Release-4.26
LogInit: Command Line:  -vulkan
LogInit: Base Directory: /home/cityu-fsm-lab-carla/Workspace/UnrealEngine/Engine/Binaries/Linux/
LogInit: Allocator: binned2
LogInit: Installed Engine Build: 0
LogDevObjectVersion: Number of dev versions registered: 29

```

```sh

carla gnss record start

```

```sh

carla_release
-------------------------------
Starting OpenPlanner .. 
-------------------------------
Agent Name:  pygame_adtruck
Exploration Mode:  true
Map Path:  /home/cityu-fsm-lab-carla/Workspace/Carla/op_carla/op_agent/autoware-contents/maps/fsm_lab_sandbox_right_hand_driving_scene
CARLA Path:  /home/cityu-fsm-lab-carla/Workspace/Carla/carla-0.9.15

```

## path

```sh

${OP_ROS_PLUGINS_ROOT}=/home/cityu-fsm-lab-carla/Workspace/Carla/op_carla/op_ros_plugins

export TEAM_AGENT=${OP_BRIDGE_ROOT}/op_bridge/fsm_lab_simulation/ego_vehicle_initializer.py

${OP_BRIDGE_ROOT}=/home/cityu-fsm-lab-carla/Workspace/Carla/op_carla/op_bridge

${OP_AGENT_ROOT}=/home/cityu-fsm-lab-carla/Workspace/Carla/op_carla/op_agent


```

## this is the description of the carla_vechile launch

```sh

source ${OP_ROS_PLUGINS_ROOT}/install/setup.bash
ros2 launch ${OP_AGENT_ROOT}/autoware_carla_launch/carla_simulator_fsm_lab.launch.xml map_path:=${OP_AGENT_ROOT}/autoware-contents/maps/fsm_lab_maps/$map_name vehicle_model:=sample_vehicle sensor_model:=carla_sensor_kit

```
