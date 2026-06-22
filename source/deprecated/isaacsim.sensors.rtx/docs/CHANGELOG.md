# Changelog

## [15.17.1] - 2026-06-09
### Fixed
- Fix linter errors and missing or incomplete docstrings, and update `python_api.md`.

## [15.17.0] - 2026-05-07
### Changed
- Multitick now enabled by default. Scan accumulation controlled by omni:sensor:Core:accumulateOutputs on OmniLidar prim.
- OmniLidar omni:sensor:Core:skipDroppingInvalidPoints set to False by default when using command
- OmniLidar omni:Sensor:Core:accumulateOutputs set to True by default when using command

### Removed
- Remove CUDA-based post-processing paths for lidar scan accumulation
- Multitick nodes replace non-multitick nodes
- Multitick annotator tests replace non-multitick annotator tests

## [15.16.4] - 2026-05-06
### Added
- `IsaacSensorCreateRtxLidar`: `variant` parameter now accepts `dict[str, str]` for USDs with multiple variant sets (e.g. SICK family `Product` × `Profile`), matching `Lidar.create()` in `isaacsim.sensors.experimental.rtx`.

## [15.16.3] - 2026-05-04
### Fixed
- Annotator tests correctly set RTX Radar output level to BASIC instead of FULL.

## [15.16.2] - 2026-04-28
### Fixed
- Restored the deprecated `outputNormal` input on `IsaacCreateRTXLidarScanBufferNew` as an alias for `outputHitNormal`, emitting a deprecation warning when used. Previously the "New" node rejected `outputNormal=True` with a `SyntheticDataException` even though the non-`New` node still accepted it (NVBug 5985827).
- `IsaacCreateRTXLidarScanBuffer`/`IsaacCreateRTXLidarScanBufferNew`: insufficient `auxType` on the `OmniLidar` prim caused requested ID outputs (`materialId`, `objectId`, `emitterId`, etc.) to be silently disabled at init time. The non-`New` node logged this at `WARN` (often missed); the `New` node logged at `INFO` (filtered by default). Both now log at `ERROR` with explicit remediation naming the `omni:sensor:Core:auxOutputType` USD attribute and the level required for each output (NVBug 5985774).

## [15.16.1] - 2026-04-13
### Fixed
- Incorporate bug fixes from update to kit-kernel.

## [15.16.0] - 2026-04-07
### Added
- Add optional multitick support.  When enabled, switch to nodes that consume GenericModelOutput assuming it contains a full scan, eliminating lidar accumulation post-processing.

## [15.15.2] - 2026-04-03
### Changed
- Formally deprecate outputNormal input in favor of outputHitNormal input in IsaacCreateRTXLidarScanBuffer

## [15.15.1] - 2026-03-31
### Deprecated
- Extension deprecated in favor of the Experimental extension `isaacsim.sensors.experimental.rtx`

## [15.15.0] - 2026-03-25
### Changed
- Move `generic_model_output`, `sensor_checker` and supported lidar configurations to the experimental extension version

## [15.14.3] - 2026-03-22
### Added
- WAR for omni.replicator.core 1.13.5 - specify RTX Lidar and Radar auxOutputType as "channels" attribute on GenericModelOutput RenderVar

### Fixed
- Increase grace frame count for TestGenericModelOutput.test_radar_timestamp_alignment

## [15.14.2] - 2026-03-21
### Fixed
- Added temporary fix for missing GenericModelOutput annotator in omni.replicator.core

## [15.14.1] - 2026-03-09
### Fixed
- Fixed occasional segfault when running with many lidars doing full-scan processing at the same time by copying GenericModelOutput buffers into local copies rather than manipulating AOV

## [15.14.0] - 2026-03-04
### Changed
- Added Overview.md, python_api.md, and SETTINGS.md and updated docstrings

## [15.13.0] - 2026-03-02
### Changed
- Shifted RTX sensor scan accumulation and post-processing back to host by default to reduce GPU resource contention and improve frametime & frametime consistency. Post-processing-on-device still available as option by setting app.sensors.nv.[modality].outputBufferOnGPU=true.

## [15.12.3] - 2026-02-13
### Changed
- Re-enable object ID test in CI to track fix

## [15.12.2] - 2026-02-04
### Added
- Test for scan tearing when --/app/player/useFixedTimeStepping=True

## [15.12.1] - 2026-02-03
### Removed
- Tools for manipulating deprecated JSONs

### Fixed
- IsaacCreateRTXLidarScanBuffer.transform output no longer resets frame-to-frame

## [15.12.0] - 2026-01-30
### Added
- Use Hydra time (omni.timeline) in RTX Sensor models

### Changed
- Refactor Annotator tests to include soak tests and comparisons against upstream Annotator outputs
- TestSupportedLidarConfigs tests only prims without saving a new USD for each test

### Removed
- Test only specific lidar configs in Annotator tests, rather than all configs
- Unused Lidar USDA test

### Fixed
- Only reset outputs in IsaacCreateRTXLidarScanBuffer if enablePerFrameOutput == True, preventing output "flickering"
- Prevent USD path warnings when using IsaacSensorCreateRtxSensor commands

## [15.11.9] - 2026-01-28
### Changed
- IsaacSensorCreateRtxSensor commands accept usd_path argument to enable adding arbitrary RTX Sensor USDs to the stage
- IsaacSensorCreateRtxRadar command validates if user has enabled Motion BVH before creating prim

## [15.11.8] - 2026-01-23
### Changed
- Cleanup and make docstrings consistent and add missing docstrings, example strings and type hints

## [15.11.7] - 2026-01-09
### Changed
- Cleanup unused imports

## [15.11.6] - 2026-01-06
### Changed
- Migrate more events to Events 2.0.

## [15.11.5] - 2025-12-18
### Fixed
- IsaacSensorCreateRTXSensor commands resolve prim paths as expected when using Replicator APIs as a fallback

## [15.11.4] - 2025-12-12
### Changed
- RtxLidar.get_object_ids correctly handles GenericModelOutput.objId

## [15.11.3] - 2025-12-05
### Changed
- Migrate to Events 2.0.

## [15.11.2] - 2025-12-03
### Changed
- Remove TODOs.

## [15.11.1] - 2025-12-01
### Changed
- OgnIsaacCreateRTXLidarScanBuffer uses lambda function to initialize and allocate buffers

## [15.11.0] - 2025-11-26
### Added
- IsaacCreateRTXRadarPointCloud annotator to explicitly support RTX Radar

### Changed
- OgnIsaacCreateRTXLidarScanBuffer includes support for RTX Radar metadata

## [15.10.1] - 2025-11-25
### Changed
- Fix tests after updating to kit 109.0.1.

## [15.10.0] - 2025-11-20
### Changed
- Compute maximum points per Lidar scan from Lidar configuration

### Removed
- No longer dependent on RtxSensorMetadata AOV

## [15.9.0] - 2025-11-10
### Added
- Link sensor_checker utility as Python module
- Add sensor_checker to unit tests to verify supported Lidar configs

## [15.8.7] - 2025-11-07
### Changed
- Update to Kit 109 and Python 3.12

## [15.8.6] - 2025-10-27
### Changed
- Make omni.isaac.ml_archive an explicit test dependency

## [15.8.5] - 2025-10-22
### Changed
- Exclude Simple Example Solid State config from tests

## [15.8.4] - 2025-10-07
### Fixed
- LidarRtx warns if rational time is 0/0 and will not collect simulation time or Annotator data in that case

## [15.8.3] - 2025-09-26
### Added
- Kwargs to LidarRtx.attach_annotator and LidarRtx.attach_writer to specify keywords upon Annotator or Writer initialization

### Fixed
- Fixed docstring references to nonexistent annotators

## [15.8.2] - 2025-09-17
### Changed
- Specify rtx.materialDb.nonVisualMaterialSemantics.prefix as "omni:simready:nonvisual" to align with SimReady spec

## [15.8.1] - 2025-09-15
### Added
- GenericModelOutput explicitly added to valid arguments in LidarRtx.attach_annotator

### Fixed
- LidarRtx no longer resets prim orientation and position if called with existing prim path

## [15.8.0] - 2025-09-09
### Added
- New APIs to apply non-visual material attributes to Material prims, resolve non-visual material ID as base/coating/attribute.

### Removed
- No longer using CSV mapping from visual to non-visual materials as default behavior, favoring USD attribute specification instead

## [15.7.0] - 2025-09-05
### Added
- Per-frame output option to IsaacCreateRTXLidarScanBuffer

### Changed
- Use CUDA graphs, improved kernels, and cached device properties in IsaacCreateRTXLidarScanBuffer to improve perf
- IsaacExtractRTXSensorPointCloudNoAccumulator annotator now uses IsaacCreateRTXLidarScanBuffer node with per-frame output enabled

### Removed
- IsaacExtractRTXSensorPointCloud node no longer used
- Unused CUDA kernels removed

### Fixed
- IsaacCreateRTXLidarScanBuffer outputs correct timestamps

## [15.6.4] - 2025-08-29
### Changed
- Renamed CARB profiling zones to include [IsaacSim] prefix

## [15.6.3] - 2025-08-22
### Changed
- Use SimulationManager instead of deprecated CoreNodes APIs for time related APIs
- Add dependency on isaacsim.core.simulation_manager

## [15.6.2] - 2025-08-22
### Removed
- Motion BVH no longer enabled by default.

## [15.6.1] - 2025-08-22
### Fixed
- Corrected field name in test_generic_model_output

## [15.6.0] - 2025-08-08
### Added
- StableIdMap annotator available in LidarRtx class
- New static methods in LidarRTX class to decode StableIdMap output into map of stable IDs to prim paths, and resolve GenericModelOutput object IDs as 128-bit stable IDs.

## [15.5.2] - 2025-08-07
### Fixed
- Annotator test tolerance increased to 2% to account for FlatScan not interpolating distance based on azimuths
- Annotator tests combined where appropriate to reduce test time

## [15.5.1] - 2025-07-29
### Fixed
- Remove version locks for omni.sensors.nv.*

## [15.5.0] - 2025-07-25
### Added
- Restored IsaacCreateRTXLidarScanBuffer, switched to device-side processing with double buffers
- Support for additional SICK Lidars

### Removed
- LidarPointAccumulator usage removed from extension

### Changed
- Improvements to stream processing in IsaacExtractRTXSensorPointCloud
- IsaacComputeRTXLidarFlatScan gets inputs from IsaacCreateRTXLidarScanBuffer
- OmniLidar prims created with IsaacSensorCreateRtxLidar command have omni:sensor:Core:skipDroppingInvalidPoints set to True

### Fixed
- No longer run IsaacExtractRTXSensorPointCloud tests twice.

## [15.4.6] - 2025-07-15
### Fixed
- No longer run IsaacExtractRTXSensorPointCloud tests twice.

## [15.4.5] - 2025-07-09
### Fixed
- Explicitly lock the pre-release versions of omni.sensors.nv.* to ensure that the correct versions are loaded in all scenarios including ETM.

## [15.4.4] - 2025-07-07
### Fixed
- IsaacComputeRTXLidarFlatScan outputs azimuth range guaranted to be in [0, 360], with linear depth data appropriately sorted.

## [15.4.3] - 2025-07-07
### Changed
- Cleanup docstring for bindings

## [15.4.2] - 2025-07-07
### Fixed
- Correctly enable omni.kit.loop-isaac in test dependency (fixes issue from 15.4.1)

## [15.4.1] - 2025-07-03
### Changed
- Make omni.kit.loop-isaac an explicit test dependency

## [15.4.0] - 2025-06-27
### Changed
- Lidar and Radar data now on GPU by default
- IsaacExtractRTXSensorPointCloud CUDA operations fully rewritten
- Preallocating buffers in all nodes to avoid costly reallocation during compute

### Fixed
- IsaacComputeRTXLidarFlatScan synchronizes on CUDA stream before testing inputs to ensure data buffer is properly filled before processing.

## [15.3.3] - 2025-06-25
### Changed
- Add --reset-user to test args

## [15.3.2] - 2025-06-25
### Changed
- Removed nanoScan3 from supported lidar configs.

## [15.3.1] - 2025-06-13
### Changed
- Switched CUDA files to Apache license.

## [15.3.0] - 2025-06-12
### Removed
- omni.sensors.net from direct dependency list

## [15.2.1] - 2025-06-10
### Added
- Expanded support for specifying supported Lidars by config name.

## [15.2.0] - 2025-06-06
### Added
- SUPPORTED_LIDAR_CONFIGS maps official Lidar asset paths to variants

### Changed
- Autogenerate test structures, valid commands from SUPPORTED_LIDAR_CONFIGS

## [15.1.4] - 2025-06-04
### Changed
- Changed CUDA_CHECK in ScopedCudaDevice.h to indicate file and line for more verbose error logging

## [15.1.3] - 2025-06-03
### Changed
- Cleaned up and updated extension setting documentation

## [15.1.2] - 2025-05-31
### Changed
- Use default nucleus server for all tests

## [15.1.1] - 2025-05-30
### Changed
- Prim creation commands automatically return OmniSensor prim, even if adding USD as reference
- Downgrade some warnings into info logs

### Fixed
- Unique sendDataId input for each autogenerated LidarPointAccumulator node

## [15.1.0] - 2025-05-26
### Changed
- IsaacComputeRTXLidarFlatScan inputs include cudaStream
- IsaacExtractRTXSensorPointCloud inputs changed to dataPtr and cudaStream
- IsaacExtractRTXSensorPointCloud outputs include cudaStream
- Annotators updated to connect cudaStreams
- IsaacComputeRTXLidarFlatScan shifts compute work to enqueued host function in stream
- Automatically sets LidarPointAccumulator output to CPU for IsaacComputeRTXLidarFlatScan
- Automatically sets LidarPointAccumulator output to CPU or GPU based on carb setting for IsaacExtractRTXSensorPointCloud

### Removed
- IsaacTransformRTXSensorReturns node - functionality handled by omni.sensors.* carb settings.
- IsaacExtractRTXSensorPointCloud no longer outputs azimuth/elevation/range

## [15.0.8] - 2025-05-25
### Changed
- Update replicator api calls to use create.omni_lidar and create.omni_radar

## [15.0.7] - 2025-05-23
### Changed
- Add docstrings

## [15.0.6] - 2025-05-22
### Changed
- Update copyright and license to apache v2.0

## [15.0.5] - 2025-05-20
### Changed
- OmniSensor auxiliary data level now controlled by prim attributes, rather than carb setting

### Fixed
- LidarRtx no longer throws error if deprecated methods are called
- isaacsim.sensors.rtx.get_gmo_data guards against null input

## [15.0.4] - 2025-05-19
### Added
- Added Ouster OS2 to test suite.

## [15.0.3] - 2025-05-16
### Changed
- Make extension target a specific kit version

## [15.0.2] - 2025-05-15
### Changed
- UsdUtilities.h was updated

## [15.0.1] - 2025-05-12
### Added
- Added SICK TiM 481 and Slamtec RPLIDAR S2E to test suite.

## [15.0.0] - 2025-05-11
### Added
- Full support for OmniSensor prims
- IsaacSensorCreateRtxSensor command base class
- exts."isaacsim.sensors.rtx".supportedLidarConfigs specifies OmniLidar USDs (some including meshes) that can be added via sensor create commands.
- Sensor creation commands now have "force_camera_prim" bool argument to forcibly create Camera prims as RTX Sensors (see below).

### Changed
- Deprecated Camera prims as RTX Sensors
- LidarRTX no longer implicitly attaches annotators or writers; user must now explicitly attach annotators or writers using helper methods.
- LidarRTX get_current_frame() data dictionary exposes data from each attached annotator
- LidarRTX now supports all annotators in isaacsim.sensors.rtx extension
- IsaacSensorCreateRtxLidar, IsaacSensorCreateRtxRadar, IsaacSensorCreateRtxIDS, and IsaacSensorCreateRtxUltrasonic commands now subclass IsaacSensorCreateRtxSensor
- IsaacSensorCreateRtxLidar will now add a reference USD to stage if a matching config is provided
- IsaacSensorCreateRtxLidar and IsaacSensorCreateRtxRadar will now create OmniSensor prims on the stage by default
- OgnIsaacComputeRTXLidarFlatScan now only works with lidar configurations or OmniLidar prims specifying 2D lidars, with all emitters at 0 degrees elevation.

### Removed
- OgnIsaacComputeRTXLidarPointCloud node removed in favor of OgnIsaacExtractRTXSensorPointCloud
- OgnIsaacComputeRTXRadarPointCloud node removed in favor of OgnIsaacExtractRTXSensorPointCloud
- OgnIsaacCreateRTXLidarScanBuffer node removed in favor of OgnIsaacExtractRTXSensorPointCloud
- OgnIsaacReadRTXLidarData node removed in favor of standalone examples

### Fixed
- OgnIsaacComputeRTXLidarFlatScan properly outputs all returns from full scan of 2D lidar.

## [14.4.2] - 2025-05-10
### Changed
- Enable FSD in test settings

## [14.4.1] - 2025-05-10
### Changed
- Remove internal build time dependency

## [14.4.0] - 2025-05-09
### Changed
- Rtx lidar ["rendering_frame"] contains both the rational time numerator and denominator and not just the numerator

## [14.3.1] - 2025-04-30
### Changed
- Update event subscriptions to Event 2.0 system

## [14.3.0] - 2025-04-09
### Changed
- Drops dependency on internal sensor headers

## [14.2.4] - 2025-04-09
### Changed
- Update all test args to be consistent

## [14.2.3] - 2025-04-04
### Changed
- Version bump to fix extension publishing issues

## [14.2.2] - 2025-03-26
### Changed
- Cleanup and standardize extension.toml, update code formatting for all code

## [14.2.1] - 2025-03-24
### Changed
- Migrate to Events 2.0

## [14.2.0] - 2025-03-18
### Added
- Support for USD variant sets to RTX Sensor creation commands

### Removed
- OmniLidar USDAs (moved to /Isaac/Sensors)

## [14.1.1] - 2025-03-11
### Changed
- Switch asset root for tests to internal nucleus

## [14.1.0] - 2025-03-05
### Added
- OmniLidar USDAs corresponding to vendor-supplied lidar configurations
- Support for adding RTX Lidar, Radar, Ultrasonic as OmniSensor prims
- Tool to convert RTX Lidar configuration JSON to OmniLidar USDA

## [14.0.1] - 2025-03-05
### Changed
- Update extension codebase to adhere to isaac sim extension structure and file naming  guidelines

### Changed
- Deprecated APIs for adding RTX Sensors as Camera prim

## [14.0.0] - 2025-03-05
### Changed
- Updated build dependencies to latest kit

## [13.6.5] - 2025-02-21
### Changed
- Update style format and naming conventions in c++ code, add doxygen docstrings

## [13.6.4] - 2025-01-28
### Fixed
- Windows signing issue

## [13.6.3] - 2025-01-26
### Changed
- Update test settings

## [13.6.2] - 2025-01-22
### Fixed
- Test error by forcing test to run with vulkan

## [13.6.1] - 2025-01-21
### Changed
- Update extension description and add extension specific test settings

## [13.6.0] - 2024-12-13
### Removed
- Removed legacy sick lidar sensors

## [13.5.1] - 2024-12-12
### Fixed
- IsaacPrintRTXSensor now uses updated Python bindings to retrieve GMO buffer

## [13.5.0] - 2024-12-10
### Removed
- Menu components moved to isaacsim.sensors.camera.ui, isaacsim.sensors.physics.ui, isaacsim.sensors.physx.ui, isaacsim.sensors.rtx.ui

## [13.4.3] - 2024-12-09
### Fixed
- Menu icon glyph call

## [13.4.2] - 2024-12-05
### Changed
- Updated Nova carter path

## [13.4.1] - 2024-12-04
### Changed
- Glyph for the create menu

## [13.4.0] - 2024-11-26
### Added
- New settings to toggle auxiliary data output for lidar, radar

### Changed
- Updated omni.sensor extension dependencies to latest.
- Updated GMO struct layout

## [13.3.1] - 2024-11-25
### Changed
- Added logic to automatically add usd in given folders to menu

## [13.3.0] - 2024-11-25
### Changed
- Re-enabled per-view TLAS.

## [13.2.0] - 2024-11-01
### Changed
- Menu names and location

## [13.1.2] - 2024-10-29
### Changed
- Corrects output units for OgnIsaacComputeRTXLidarPointCloud, OgnIsaacComputeRTXRadarPointCloud, OgnIsaacReadRTXLidarData.

## [13.1.1] - 2024-10-24
### Changed
- Updated dependencies and imports after renaming

## [13.1.0] - 2024-10-10
### Changed
- Updated omni.sensor extension dependencies to latest kit.
- Updates default LidarRTX rotary lidar configuration to avoid deprecation warnings.

## [13.0.0] - 2024-09-24
### Changed
- Renamed extension to isaacsim.sensors.rtx

### Removed
- Moved Camera, CameraView APIs to isaacsim.sensors.camera
- Moved ContactSensor, ImuSensor, EffortSensor APIs to isaacsim.sensors.physics
- Moved PhysX Lidar, LightBeam APIs to isaacsim.sensors.physx

## [12.9.1] - 2024-09-02
### Fixed
- Point cloud error in data acquisition test for camera class

## [12.9.0] - 2024-08-29
### Changed
- CameraView class to use the tiled RTX sensor instead of the single raycast Tiled Sensor
- Changed default output annotators to rgb and depth

### Removed
- CameraView methods: set_frequency, get_frequency, set_dt, get_dt, get_data (Currently the CameraView class follows the global rendering_dt)

## [12.8.7] - 2024-08-27
### Fixed
- Add omni.sensors.tiled as a dependency

## [12.8.6] - 2024-08-21
### Fixed
- Corrects prim path for Sensing SG2-AR0233C-5200-G2A-H100F1A

## [12.8.5] - 2024-08-21
### Fixed
- Making auto-add contact report API when creating a sensor

## [12.8.4] - 2024-08-20
### Added
- Adds Owl to Create->Sensor menu.

### Fixed
- Corrects path manipulations when building menu.

## [12.8.3] - 2024-08-13
### Fixed
- Sensing RGBD sensor names in Create -> Isaac menu now match USD filenames.
- Create -> Isaac -> Sensor -> RTX Lidar options now spawn USD sample asset rather than bare prim, if one exists

## [12.8.2] - 2024-08-09
### Added
- Added Gemini 335, 335L to the menu

### Changed
- Updated Gemini 2 usd file path

## [12.8.1] - 2024-08-06
### Fixed
- Disables per-view TLAS so that individual RTX sensors return separate point clouds

## [12.8.0] - 2024-07-23
### Added
- New command "IsaacSensorCreateRtxIDS" to support occupancy sim extension

## [12.7.1] - 2024-07-16
### Changed
- OgnIsaacPrintRTXSensorInfo - adds check for auxType to avoid non-lidar data error messages in material mapping

## [12.7.0] - 2024-07-12
### Fixed
- OgnIsaacComputeRTXLidarFlatScan publishes timestamp of first beam in scan

## [12.6.1] - 2024-07-10
### Fixed
- Isaac Sim lidar profiles omit deprecated fields and include new default fields

## [12.6.0] - 2024-07-09
### Fixed
- Default sensor profiles omit deprecated fields and include new default fields
- Warns when user provides invalid sensor config paths

## [12.5.0] - 2024-07-05
### Added
- Use register_*_with_telemetry functions to simplify registration code
- Rename registered_template to registered_templates

## [12.4.2] - 2024-07-05
### Added
- Fixed bug with rigid body sleep threshold

## [12.4.1] - 2024-07-03
### Added
- Added error message for parents missing contact report API

## [12.4.0] - 2024-06-28
### Added
- Setting the rigid body parent sleep threshold to 0 to prevent physics to go to sleep.
- Setting the contact reporter threshold to 0 to make sure all contacts are reported.

### Fixed
- Performance issue due to CUDA sync in omni::sensors::nv::lidar::LidarRotary::batchEnd
- Missing omni::sensors:::IProfileReaderFactory v0.1 dependency

## [12.3.1] - 2024-06-25
### Changed
- OgnIsaacPrintRTXSensorInfo - corrects node name in warning text

## [12.3.0] - 2024-06-14
### Added
- OgnIsaacPrintRTXSensorInfo - uses Python bindings to decompose and print GMO struct

### Removed
- OgnIsaacPrintLidarInfo and OgnIsaacPrintRadarInfo - deprecated by OgnIsaacPrintRTXSensorInfo

## [12.2.0] - 2024-06-14
### Added
- Unit test for IsaacComputeRTXRadarPointCloud

## [12.1.0] - 2024-06-14
### Changed
- Upgraded to omni.sensors.nv.* v1.1.0
- Uses new common sensor profile reader

## [12.0.0] - 2024-06-12
### Added
- LightBeam Sensor class that uses raycasts to detect if a beam is broken
- Omnigraph nodes to read data from this light beam sensor

## [11.4.0] - 2024-06-11
### Added
- Added unit tests for IMU, Contact, and Effort sensor omnigraph nodes.

## [11.3.2] - 2024-06-11
### Added
- Unit tests for rotary and solid state lidars evaluating `RtxSensorCpuIsaacComputeRTXLidarPointCloud` and `RtxSensorCpuIsaacComputeRTXLidarFlatScan` annotators

### Fixed
- `LidarRTX` no longer causes error when `position` is set in constructor
- Corrects `OgnIsaacComputeRTXLidarFlatScan` `azimuthRange` output to span true min/max azimuth of scan

## [11.3.1] - 2024-06-06
### Fixed
- Crash when new stage was removed and sensors were not cleaned up properly

## [11.3.0] - 2024-05-15
### Added
- Added `get_rgb` and `get_depth` methods to `CameraView` class returning the data reshaped into camera batches
- Added `get_rgb_tiled` and `get_depth_tiled` methods to `CameraView` class returning the tiled data
- Added `test_camera_view_sensor.py` with golden images for testing `get_rgb`, `get_depth`, `get_rgb_tiled`, and `get_depth_tiled` methods

### Changed
- Removed replicator step function call inside `camera_view.py`
- Removed deleting stage camera prims in `camera_view.py`

## [11.2.5] - 2024-05-14
### Changed
- Downgraded IMU and Contact Sensor rigid body errors to warnings.

## [11.2.4] - 2024-05-08
### Fixed
- OgnROS2PublishLaserScan switches buffer index calculation to use monotonically-increasing integer, fixing bug where floating-point math would occasionally result in buffer never being filled

## [11.2.3] - 2024-05-08
### Added
- isaacsim.sensors.rtx.gmo_types module, containing `ctypes` structures for `omni.sensors` extension types

### Changed
- SICK lidar configs now include rangeOffset parameter

### Fixed
- OgnIsaacPrintRTXRadarInfo prints radar info correctly

## [11.2.2] - 2024-05-07
### Fixed
- OgnROS2PublishLaserScan publishes correct flat scan
    - Buffers are now sorted by azimuth, min -> max
    - Accumulates then publishes full scan, rather than partial scans
    - Updated output descriptions
    - Output angles now all in degrees, rather than mixed degrees/radians

## [11.2.1] - 2024-05-07
### Fixed
- Crash in IsaacComputeRTXRadarPointCloud node

### Changed
- Using wpm radar
- Get radar transform from camera

## [11.2.0] - 2024-05-07
### Added
- Added device argument with "cpu" or "cuda" for get_rgba/get_depth functions of `camera_view.py`
- Added get_data to `camera_view.py` returning the raw annotator data

## [11.1.0] - 2024-05-06
### Added
- Added additional Ouster lidar configs for OS0, OS1, REV6, REV7, and OS2 lidars

## [11.0.6] - 2024-05-03
### Fixed
- Incorrect rendervar in RtxSensorCpuIsaacRTXLidarOutput

## [11.0.5] - 2024-05-03
### Fixed
- get_rgba() and get_depth() for CameraView class now works for both rgb and depth
- Account for linear array with depth values being at end

## [11.0.4] - 2024-04-30
### Changed
- Updated Sensing camera names

## [11.0.3] - 2024-04-25
### Fixed
- Fixed rotating physX lidar python class initialization issue

## [11.0.2] - 2024-04-23
### Fixed
- Rational polynomial camera distortion coefficients will be stored as k1, k2, p1, p2, k3, k4, k5, k6 in the camera schema

## [11.0.1] - 2024-04-22
### Fixed
- Fixed invalid IMU sensor crash bug

## [11.0.0] - 2024-04-19
### Changed
- Removed visualization attributes
- Deprecated get raw data function

## [10.2.0] - 2024-04-19
### Added
- CameraView class for managing multiple cameras and rendering used tiled_sensor from replicator

## [10.1.1] - 2024-04-17
### Added
- Telemetry for writers and annotators

### Fixed
- Update IStageUpdate usage to fix deprecation error

## [10.1.0] - 2024-04-17
### Changed
- omni.sensors backend moved to v1.0.0
- Includes RTX nonvisual material support

### Removed
- Up/downElevationDeg and start/endAzimuthDeg because they are no longer used.

## [10.0.0] - 2024-04-09
### Changed
- Tensor API support for IMU sensor
- Changed physics based sensor to be created on play and destroyed on stop

### Removed
- Deprecated functions: get_sensor_readings, get_sensor_num_readings, get_sensor_sim_readings
- Sensor visualziation. (Please use omnigraph nodes to visualize the sensors)

## [9.15.0] - 2024-03-21
### Added
- Added Sick sensor config files for multiScan136, multiScan165, picoScan150

### Changed
- Replaced current sensor config files for tim781

## [9.14.4] - 2024-03-14
### Fixed
- IsaacComputeRTXLidarFlatScan works with CCW Solid State lidar configs
- Crash in PrintRTXLidarInfo node

### Added
- Test for IsaacComputeRTXLidarFlatScan Node

## [9.14.3] - 2024-03-11
### Fixed
- Fixed contact sensor threshold bug
- Multi GPU support

### Changed
- Changed XT-32 lidar name from PandarXT-32 to XT-32

## [9.14.2] - 2024-03-07
### Changed
- Removed the usage of the deprecated dynamic_control extension

## [9.14.1] - 2024-03-04
### Changed
- Updated omnigraph nodes to use per instance state instead of internal state

## [9.14.0] - 2024-03-01
### Added
- IsaacComputeRTXLidarFlatScan Node now works with Solid State lidar.
- Official version of SICK microscan3 config, and marked old one as legacy.

## [9.13.7] - 2024-02-26
### Fixed
- Added execOut trigger commands in Read IMU and Contact Sensor nodes to allow attached downstream nodes to tick

## [9.13.6] - 2024-02-24
### Changed
- Update source code to run a build clean of warnings

## [9.13.5] - 2024-02-22
### Changed
- Using externally built omni.sensors for rtx lidar and rtx radar
- Use intensityScalePercent in IsaacComputeRTXLidarPointCloud if present
- Location of default and temp lidar config files set to ${app}/../extsbuild/omni.sensors.nv.common/data/lidar/

### Fixed
- No longer crash when number of rtx lidar ticks are out of sync

## [9.13.1] - 2024-02-05
### Changed
- Replaced internalState with perInstanceState for the ogn nodes
- Updated path to the nucleus extension

## [9.13.0] - 2024-01-30
### Changed
- Converted read IMU and contact sensor nodes to C++
- Added read latest data input flag to the read contact sensor node (default to false)
- Renamed Isaac Read Contact Sensor to Isaac Read Contact Sensor Node

## [9.12.1] - 2024-01-26
### Fixed
- RTX Lidar config parameter nearRangeM < 0.4 was broken. Added minDistBetweenEchos, which also affects the near hits.

### Changed
- Changed get_assets_root_path to get_assets_root_path_async for the unit tests

## [9.12.0] - 2024-01-08
### Changed
- Moved header files to extension

## [9.11.1] - 2023-12-12
### Changed
- IsaacCreateRTXLidarScanBuffer transformPoints defaults to false
- RtxSensorCpuIsaacCreateRTXLidarScanBuffer doTransform defaults to true

## [9.11.0] - 2023-12-01
### Added
- ZED X sensor to menu item

## [9.10.1] - 2023-11-29
### Fixed
- Aperture setters and getters in Camera.py
- Camera Matrix calculation in set_matching_fisheye_polynomial_properties method in Camera.py
- Camera class to work with torch backend

## [9.10.0] - 2023-11-14
### Fixed
- Contact Sensor can now measure force correctly when the rigid body is not its direct parent

## [9.9.2] - 2023-11-13
### Fixed
- Updated documentation link

## [9.9.1] - 2023-10-09
### Fixed
- Removed extra frame transformation in the LidarRtx wrapper
- Changed physX lidar tests and samples to use Carter V1

## [9.9.0] - 2023-10-08
### Changed
- get_all_camera_objects() now ensures that camera names are unique

## [9.8.1] - 2023-10-06
### Fixed
- Realsense D455 menu

### Changed
- Updated assets to use carter v2.4

## [9.8.0] - 2023-10-04
### Changed
- Changed sensor type member names to adhere to naming conventions, inContact to in_contact

## [9.7.3] - 2023-10-03
### Changed
- Changed default prim path for get_all_camera_objects() to be "/" instead of "/World"

## [9.7.2] - 2023-10-03
### Fixed
- Fixed WriterReadRTXLidarData Synthetic Data writer so it sets the render_product_path correctly.

## [9.7.1] - 2023-09-29
### Fixed
- Fixed elevation output for ComputeRTXLidarPointCloud node.

## [9.7.0] - 2023-09-29
### Added
- numEchos, numChannels, and numTicks output to IsaacReadRTXLidarData

### Changed
- Changed IU names for CreateRTXLidarScanBuffer outputs to be more user friendly.
- Set default RXT sensor space to be the same as isaac-sim so rotations in IsaacSensorCreateRtxLidar make sense

### Fixed
- Fixed bug with ReadRTXLidarData node output when using keepOnlyPositiveDistance.

## [9.6.3] - 2023-09-29
### Fixed
- Add the allowedToken metadata for the cameraProjectionType attribute in cameras if it wasn't set already.

## [9.6.2] - 2023-09-27
### Changed
- Updated data acquisition callback for Camera objects to be on the next frame event
- Used frameTime annotator instead of the dispather node inputs for better accuracy in the data acquisition callback
- Moved initializing render products to the initialize method in Camera to reduce overhead and decoupling the usage of pose utils and render product related methods

## [9.6.1] - 2023-09-26
### Fixed
- Fixed bug with horizontal resolution divison
- Fixed contact sensor sample

## [9.6.0] - 2023-09-25
### Added
- depthRange output to IsaacReadRTXLidarData
- numBeams output to IsaacReadRTXLidarData

## [9.5.1] - 2023-09-20
### Fixed
- ComputeRTXLidarFlatScan now uses lidar config for more accurate output

### Changed
- RGBD menu updated to include manufacturer sub-menu
- Updated usd paths for Sensing assets
- Changed base prim type for sensors from Camera to Xform (real camera sensor should be inside of the default prim)

## [9.5.0] - 2023-09-19
### Added
- IsaacPrintRTXLidarInfo outputs largest velocity length
- IsaacCreateRTXLidarScanBuffer node outputs all possible lidar data based on output flags on node.

## [9.4.1] - 2023-09-19
### Changed
- IsaacComputeRTXLidarFlatScan now works with single emitter lidar configs like RPLIDAR_S2E

### Fixed
- IsaacComputeRTXLidarFlatScan range projected to 0 elevation
- IMU and contact sensor read omnigraph nodes can now support parents from multiple levels up.

## [9.4.0] - 2023-09-18
### Added
- Support, samples for OpenCV calibration models

### Changed
- Added Kannala Brandt and Rational Polynomial tests for the camera properties test

## [9.3.0] - 2023-09-06
### Changed
- Updated USD path for NVIDIA Hawk RGBD sensor
- The IMU now gets the transform from world directly.
- IMU can have intermediate parents that are non rigid body.
- IMU can measure or ignore gravitational acceleration via read_gravity flag

## [9.2.0] - 2023-09-05
### Added
- Sensing GMSL2 RGBD sensors

### Fixed
- Scale issues with Orbbec RGBD sensors

## [9.1.1] - 2023-09-01
### Fixed
- RtxLidarDebugDrawPointCloudBuffer writer to use correct default transform
- RTX Lidar Menu setting wrong config parameter on sensor prim

## [9.1.0] - 2023-08-30
### Added
- transformPoints setting on IsaacCreateRTXLidarScanBuffer to enable correct world transformed points.
- Velodyne and ZVISION RTX lidar config files.

### Changed
- The layout of the add sensors menu.

### Fixed
- Added force threshold unit test for the contact sensor, now contact forces smaller than the min threshold will be treated as no contact

## [9.0.0] - 2023-08-29
### Added
- Sick_TiM781 lidar config file.

### Changed
- On CreateRTXLidarScanBuffer returnsPerScan output to numReturnsPerScan

### Fixed
-  CreateRTXLidarScanBuffer works when input data wraps around the output buffer

## [8.1.2] - 2023-08-28
### Changed
- Added standard out fail pattern for the expected no prim found edge case for the ogn test

## [8.1.1] - 2023-08-22
### Fixed
-  CreateRTXLidarScanBuffer works on Solid State Lidar
- Improved unit test stability

### Changed
- Cleaned up IsaacComputeRTXLidarPointCloud, no intended functional changes.

## [8.1.0] - 2023-08-22
### Fixed
- keepOnlyPositiveDistance now works on CreateRTXLidarScanBuffer node

## [8.0.0] - 2023-08-17
### Added
- testMode to IsaacPrintRTXRadarInfo
- Writers for radar point cloud node

### Changed
- RtxSensorCpuIsaacComputeRTXRadarPointCloud template to Annotator
- Radar Point Cloud creator now gets transform from render_product
- Changed PrintRTX templates to Writers

### Removed
- RtxRadarGetPrimLocalToWorldTransform

## [7.5.1] - 2023-08-17
### Added
- Effort sensor standalone example
- Sensor buffer size test for effort sensor and IMU

### Changed
- Effort sensor omnigraph node update sensor params
- Unified input param names for get_sensor_reading across sensors

### Fixed
- Sensor buffer bug for effort sensor and IMU

## [7.5.0] - 2023-08-16
### Added
- Added menu items for more RTX Lidar sensor configs

## [7.4.1] - 2023-08-15
### Added
- Changed isaac sensor node prim from bundle to target type

### Fixed
- Vertical Aperture used from reading the horizonal aperture usd property and multiplying it by resolution ratio to conform to the square pixels assumption in place. (Camera class)

## [7.4.0] - 2023-08-15
### Changed
- RTX point cloud node returns width and height of buffer
- Convert RTX templates to annotators

### Fixed
- RTX lidar class not returning data

## [7.3.0] - 2023-08-11
### Added
- Add function to camera.py to scrap all Camera objects in the scene
- Added IsaacCreateRTXLidarScanBuffer Node
- Changed contact sensor and IMU node prim from bundle to target type

## [7.2.1] - 2023-08-10
### Added
- Docstrings to Camera class for functions adding annotators

## [7.2.0] - 2023-08-08
### Added
- Added support for ros and usd camera axes in get_world_pose, get_local_pose, set_world_pose, set_local_pose.
- Added Effort Sensor
- supported_annotators property
- Add unit tests for Camera class to test get_point_cloud(), get_depth(), get_rgb()

### Fixed
- Fixed divisible by zero error in IMU linear interpolation
- Removed reading pairs from the IMU sensor to use the buffer directly
- Error when removing an annotator that had not been added yet

### Changed
- Store and destroy internaly created renderproduct

## [7.1.0] - 2023-08-04
### Added
- Add following functions to Camera class
- get_point_cloud()
- get_depth()
- get_rgb()

### Changed
- get_current_frame() now accepts optional argument to return deepcopy of data

## [7.0.0] - 2023-08-03
### Added
- Added get_sensor_reading function

### Changed
- RTX Nodes updated to work with dataPtr/cudaDeviceIndex/bufferSize inputs and outputs.
- RTX Nodes updated to auto connect with synthetic data/replicator nodes.
- RTX Node and DebugDrawPointCloud templates and writers updated to auto connect with synthetic data/replicator nodes.
- Changed sensor reading behavior that prohibit interpolation when the sensor frequency is higher than physics frequency
- Uses data interpolated at previous sensor measurement period when it's lower than physics frequency for the IMU sensor and nearest physics step data for the contact sensor
- Deprecated get_sensor_readings, get_sensor_sim_reading, get_sensor_num_reading for IMU Sensor and Contact Sensor
- Changed orientation measurement to be the orientation of the IMU sensor rather than the parent prim
- Changed contact sensor logic to process sensor measurement every step instead of on call
- Updated ant sensor samples to use the new API
- Updated python wrapper and removed callbacks.

### Removed
- IsaacRenderVarToCpuPointer, use omni.syntheticdata.SdRenderVarPtr instead.

### Fixed
- Fixed index out of bound error for the read IMU and contact sensor nodes

## [6.1.0] - 2023-08-03
### Added
- Added RGBD sensors to Create > Isaac > Sensors menu

## [6.0.2] - 2023-07-31
### Changed
- Location of default and temp lidar config files set to ${app}/../data/sensors/lidar/

## [6.0.1] - 2023-07-28
### Changed
- IsaacPrintRTXLidarInfo works with changed rtx lidar data

## [6.0.0] - 2023-07-25
### Changed
- IsaacReadRTXLidarData outputs match changed rtx lidar data

### Added
- RTX Sensors to Windows
- WriterIsaacReadRTXLidarData Synthetic Data Writer
- RtxSensorCpuIsaacReadRTXLidarData Synthetic Data Template

## [5.11.0] - 2023-07-05
### Added
- Added filter width attributes to the imu sensor for adjusting noise
- Added unit test for imu sensor filter, and repeated imu sensor readings

### Fixed
- Imu frequency to downtime calculation has been fixed

## [5.10.0] - 2023-07-05
### Added
- get_render_product_path to camera class

## [5.9.0] - 2023-06-30
### Added
- An existing render product path can be specified for the camera helper class
- bounding_box_3d annotator to camera class

### Changed
- If /app/runLoops/main/rateLimitFrequency is not set Frequency goes to -1, and all frames are captured

## [5.8.2] - 2023-06-23
### Changed
- IsaacPrintRTXLidarInfo node now prints prim paths and return data for first named prim hits.

## [5.8.1] - 2023-06-22
### Fixed
- Bug causing the data frame to stop updating in the camera class over extended periods of time

## [5.8.0] - 2023-06-21
### Added
- Test Mode for PrintRTXLidarInfo node.
- Synthetic data templates for Noop, RtxSensorCpuIsaacPrintRTXLidarInfo, and TestIsaacPrintRTXLidarInfo

## [5.7.0] - 2023-06-12
### Added
- TemplateRtxLidarDebugDrawPointCloud Synthetic Data template that mirrors the RtxLidarDebugDrawPointCloud writer.

### Changed
- Update to kit 105.1
- RTX Lidar/Radar Nodes cleanup
- Location of default and temp lidar config files set to ${app}/../data/lidar/
- Removed pxr::Simulation Gate from Rtx[Lidar|Radar]DebugDrawPointCloud writers
- Renamed pxr::IsaacSensorSchemaIsaacBaseSensor to pxr::IsaacSensorIsaacBaseSensor
- Renamed pxr::IsaacSensorSchemaIsaacContactSensor to pxr::IsaacSensorIsaacContactSensor
- Renamed pxr::IsaacSensorSchemaIsaacImuSensor to pxr::IsaacSensorIsaacImuSensor

## [5.6.4] - 2023-05-09
### Fixed
- Missing connection for IsaacComputeRTXLidarPointCloud node

## [5.6.3] - 2023-03-13
### Fixed
- Fix issue where lidar flatscan node as accessing data before it was ready

## [5.6.2] - 2023-03-06
### Fixed
- Default physics scene gravity is not read correctly by IMU

## [5.6.1] - 2023-03-02
### Fixed
- IMU sensor was not reading physics scene gravity correctly

## [5.6.0] - 2023-03-01
### Added
- Unlabeled points can be ignored when enabling pointcloud

### Changed
- Removing an annotator detaches it
- Update rtx lidar on app update

### Fixed
- Occlusion could not be enabled
- RTX lidar not returning data

## [5.5.1] - 2023-02-20
### Fixed
- ComputeFlatscan disconnected upon activation

## [5.5.0] - 2023-02-14
### Fixed
- RTX point cloud publishers publishing twice per frame by removing extra simulation gate nodes
- Sensor classes should only subscribe to the type of stage event they need

### Changed
- Use SdRenderVarPtr node instead of IsaacRenderVarToCpuPointer

## [5.4.4] - 2023-02-05
### Fixed
- Test failures, extra test warnings

## [5.4.3] - 2023-02-01
### Fixed
- Test failures, disabled solid state lidar test due to crash

## [5.4.2] - 2023-01-25
### Fixed
- Remove un-needed cpp ogn files from extension

## [5.4.1] - 2023-01-19
### Fixed
- Crashes during testing

## [5.4.0] - 2023-01-17
### Added
- Normal at hit for rtx lidar

## [5.3.2] - 2023-01-06
### Fixed
- onclick_fn warning when creating UI

## [5.3.1] - 2022-12-14
### Fixed
- Crash when deleting
- test_rtx_lidar passes now

## [5.3.0] - 2022-12-11
### Changed
- Switch debug draw nodes to use replicator writer backend
- Hide rtx lidar menu from windows as rtx sensor is not supported
- Disable rtx sensor tests on windows

## [5.2.4] - 2022-12-11
### Fixed
- IMU sensor example not working
- Broken docs link for imu sensor example

## [5.2.3] - 2022-12-09
### Fixed
- Crash when deleting rtx_lidar, again.

### Changed
- RTX nodes pass reasonable defaults if sensor is not found.

## [5.2.2] - 2022-12-05
### Fixed
- Crash when deleting rtx_lidar

## [5.2.1] - 2022-12-01
### Fixed
- IsaacSensorCreateContactSensor, IsaacSensorCreateImuSensor, IsaacSensorCreateRtxLidar and IsaacSensorCreateRtxRadar commands .do() only returns the created prim and not a tuple

## [5.2.0] - 2022-11-29
### Added
- Added contact sensor and IMU sensor wrappers.

## [5.1.1] - 2022-11-28
### Fixed
- Crash with Solid State Lidar.

## [5.1.0] - 2022-11-22
### Added
- Added RTX lidar and Rotating physics lidar wrappers.

## [5.0.0] - 2022-11-21
### Added
- Camera class that provides many utilities to interact with a camera prim in stage

## [4.0.0] - 2022-11-16
### Added
- Node template for rtx_radar
- Nodes for rtx_radar: PrintRTXRadarInfo, ComputeRTXRadarPointCloud
- ReadRTXLidarData node for getting lidar data without computing point cloud
- Added profile support for Lidar Point Cloud creation
- IsaacSensorCreateRtxRadar command

### Changed
- Changed node template name for rtx_lidar
- Renamed ReadRTXLidar nodes to ComputeRTXLidar
- Nvlidar dep to nvsensor and updated version.

## [3.0.1] - 2022-11-14
### Fixed
- Removed extra copy of BaseResetNode and use the one from core_nodes

## [3.0.0] - 2022-11-01
### Added
- IsaacRenderVarToCpuPointer node to replace rtx_lidar need for SdRenderVarToRawArray

### Removed
- ReadRTXRaw node and moved pointer pass through functionality to IsaacRenderVarToCpuPointer

### Changed
- Inputs to ReadRTXLidar[PointCloud|FlatScan] nodes to use IsaacRenderVarToCpuPointer cpuPointer

## [2.1.0] - 2022-11-01
### Added
- ReadRTXRaw node
- PrintRTXLidarInfo node

## [2.0.0] - 2022-10-19
### Changed
- Extension name to isaacsim.sensors.rtx

## [1.6.2] - 2022-10-19
### Changed
- ReadRTXLidarPointCloud code doc and ignore 0 values.

### Fixed
- Accuracy error calculation in ReadRTXLidarPointCloud

## [1.6.1] - 2022-10-18
### Added
- ReadRTXLidarPointCloud has transform lidarToWorld output
- ReadRTXLidarPointCloud has output on demand for all possible attributes

### Changed
- ReadRTXLidarPointCloud outputs in lidar coords

## [1.6.0] - 2022-10-09
### Added
- IsaacRtxLidarSensorAPI applied schema to differential regular cameras from RTX lidar cameras

## [1.5.1] - 2022-10-07
### Changed
- Changed the backend contact api to use updated batched update instead of notifications

## [1.5.0] - 2022-10-06
### Added
- keepOnlyPositiveDistance flag to ReadRTXLidarPointCloud Node
- Intensity output to ReadRTXLidarPointCloud Node
- Accuracy error post process to ReadRTXLidarPointCloud Node
- Synthetic data template for DebugDrawPointCloud

### Fixed
- Positions of points in ReadRTXLidarPointCloud

## [1.4.0] - 2022-09-28
### Added
- ReadRTXLidarFlatScan Node

## [1.3.0] - 2022-09-27
### Changed
- Tests to use nucleus assets

### Removed
- Usd files local to extension

## [1.2.1] - 2022-09-07
### Fixed
- Fixes for kit 103.5

## [1.2.0] - 2022-09-02
### Changed
- Remove RTX tests from windows
- Disable failing contact sensor tests from windows
- Cleanup contact sensor tests
- Use xform utilities instead of XformPrim for commands

## [1.1.1] - 2022-09-01
### Changed
- Remove legacy viewport calls from tests

## [1.1.0] - 2022-08-24
### Added
- Lidar Config file location as data/lidar_configs

## [1.0.2] - 2022-08-09
### Changed
- Removed simple_articulation.usd, test_imu_sensor uses Nucleus asset

## [1.0.1] - 2022-07-29
### Changed
- Added an exec out on the ReadContact and ReadIMU nodes

### Fixed
- Removed extra print statement

## [1.0.0] - 2022-07-22
### Added
- ReadRTXLidarPointCloud Node

### Changed
- IsaacSensorCreateContactSensor, renamed offset to translation to be consistent with core
- IsaacSensorCreateImuSensor, renamed offset to translation to be consistent with core
- Use XformPrim to initialize sensors for consistency with core
- Make return values for commands consistent, they now return: command_status, (success, prim)

## [0.5.1] - 2022-07-15
### Changed
- Renamed BindingsContactSensorPython to BindingsIsaacSensorPython

## [0.5.0] - 2022-07-11
### Added
- Read contact sensor omnigraph node and tests
- Orientation reading to Imu sensor sample

### Changed
- Contact sensor resets on stop/start

## [0.4.0] - 2022-06-24
### Added
- Absolute orientation output to Imu sensor + tests
- Read Imu node

### Fixed
- Imu mRawBuffer resets upon stop/start

## [0.3.4] - 2022-05-24
### Fixed
- Property orientation loading bug

## [0.3.3] - 2022-04-22
### Changed
- Moved sensor data aquisition function from tick to onPhysicsStep

## [0.3.2] - 2022-04-14
### Fixed
- Fixed component visualization

## [0.3.1] - 2022-04-07
### Changed
- Draw function runs onUpdate instead of physics call back

### Fixed
- Fixed visualization error of the isaac sensors

## [0.3.0] - 2022-04-04
### Added
- Added Imu sensor

### Changed
- Extension name to omni.isaac.isaac_sensor
- Imu sensor getSensorReadings to output the readings from the last frame
- Updated index.rst documentation for contact sensor and imu sensors

## [0.2.1] - 2022-03-28
### Added
- Add UI element to create contact sensor

### Changed
- Converted contact sensor namespaces to isaac sensor namespaces
- Modified draw function to use USD util's global pose

## [0.2.0] - 2022-03-18
### Changed
- Converted contact sensors into usdSchemas

### Fixed
- Enable visualization of contact sensors in the stage

## [0.1.3] - 2022-03-16
### Fixed
- Bugfix for failing tests and missing updates

## [0.1.2] - 2022-01-26
### Changed
- Compatibility for sdk 103

## [0.1.1] - 2021-07-26
### Added
- New UI

## [0.1.0] - 2021-07-08
### Added
- Initial version of Isaac Sim Contact Sensor Extension
