# Changelog

## [1.22.9] - 2026-10-02
### Fixed
- `test_stereo_without_distortion`: wait for both ROS 2 CameraInfo publishers before checking message delivery.

## [1.22.8] - 2026-09-28
### Fixed
- Stabilize differential-drive reset and semantic-label clock checks in ROS 2 tests.
- Wait for RTX lidar output before starting the LaserScan publisher discovery timeout.

## [1.22.7] - 2026-08-27
### Fixed
- `SetPrimAttribute` now rejects invalid matrix JSON values without terminating Isaac Sim.

## [1.22.6] - 2026-08-18
### Fixed
- Prevent standard ROS 2 GPU image and point-cloud publishers from reading expired inputs or reusing active message buffers.
- Only signal ROS 2 service responses after receiving them.
- Preserve nested message arrays in ROS 2 service requests and responses.

## [1.22.5] - 2026-08-13
### Fixed
- Update differential-base tests to validate standard `world` -> `odom` -> `base_link` TF publishing without creating an `odom` USD prim.
- Disable unrelated Nova Carter camera and RTX sensor graph nodes in differential-base tests.
- Use an explicit render product for 2D and 3D bounding-box publishing coverage.

## [1.22.4] - 2026-08-12
### Fixed
- Update compressed RGB camera tests for the Nova Carter asset graph.

## [1.22.3] - 2026-08-06
### Fixed
- `create_ros2_joint_states_graph` now creates the migrated `IsaacReadJointState` publisher path.

## [1.22.2] - 2026-08-05
### Added
- Example interfaces packages to Windows internal ROS 2 libs.

### Fixed
- Compressed RGB golden-image tests now use the current Nova Carter left camera publisher path when retargeting image topics.

## [1.22.1] - 2026-08-03
### Added
- Test covering how quickly Nova Carter reaches a commanded angular velocity

## [1.22.0] - 2026-07-27
### Added
- `ROS2PublishTransformTree` and `ROS2PublishRawTransformTree`: aggregate TF submissions by ROS domain, topic, static/dynamic mode, and QoS so each group publishes one merged `TFMessage` after OmniGraph evaluation.
- `exts."isaacsim.ros2.nodes".tfAggregation.enabled`: add a setting for toggling the aggregated TF publisher path.

### Fixed
- Generic ROS 2 service nodes now fail cleanly when requested dynamic service types or native type-support libraries are unavailable instead of dereferencing invalid messages.
- `TestRos2Service` now skips service cases when the requested dynamic service attributes cannot be created in the active ROS 2 environment.
- Stereo CameraInfo tests now subscribe with sensor-compatible QoS so image publishers using best-effort reliability are received reliably in CI.
- Spinning camera golden-image tests now require exact timestamp matches for dynamic camera keyframes and capture an extra rotation when a stream drops a keyframe frame.
- Update ROS 2 Nova Carter test graph paths for the current scenario asset layout.
- Update the Carter stereo ROS 2 example for the current Nova Carter camera and CameraInfo graph layout.
- `set_rotate` now accepts `Gf.Rotation` values for prims that store transforms in `xformOp:transform`.

## [1.21.1] - 2026-07-25
### Fixed
- Respect rotary lidar valid-arc and azimuth-offset attributes when publishing ROS 2 LaserScan messages.

## [1.21.0] - 2026-07-22
### Added
- Public Python config helpers for creating ROS 2 clock, generic publisher, joint states, TF, odometry, camera, RTX lidar, and RTX radar action graphs.
- `set_isaac_namespace` and `set_isaac_name_override` helpers for authoring Isaac robot schema names used by TF frame generation.
- Shared graph shortcut option constants, warning messages, and `radar_supports_basic_aux_output`.

## [1.20.2] - 2026-07-21
### Changed
- Migrated robot asset references from `Isaac/Robots/` to `Isaac/Robots_Multiphysics/` for the new multiphysics-ready USDA assets.

## [1.20.1] - 2026-07-20
### Changed
- Renamed C++ headers from `.h` to `.hpp`; update downstream include directives.

## [1.20.0] - 2026-07-17
### Added
- `interleave_point_cloud` / `fill_point_cloud2_message` helpers and `fill_point_cloud_buffer` binding: build `sensor_msgs/PointCloud2` data from separate per-point arrays with the parallel C++ interleave.

### Changed
- `fillPointCloudBufferHost`: xyz is now `const float*`, declared in the CUDA-free `FillPointCloudBufferHost.hpp`.

## [1.19.0] - 2026-07-16
### Added
- `OgnROS2CameraHelper`: add `rgb_hevc` compressed RGB publishing and pass the selected codec through the compressed image writer.
- `OgnROS2PublishCompressedImage`: allow `hevc` values for `input_format`.

### Fixed
- `CompressedImageManager` now includes the codec in every compressed image writer name, avoiding writer reuse across codecs with matching annotator template names.

## [1.18.18] - 2026-07-15
### Changed
- Replace fixed ROS 2 message waits in camera-info and waypoint-follower tests with predicate-based waits.

## [1.18.17] - 2026-07-13
### Fixed
- Tighten timing checks and clarify failure conditions in some unit tests.

## [1.18.16] - 2026-07-10
### Added
- `ROS2ControlManager` OmniGraph node for configuring the in-process ros2_control ControllerManager.

## [1.18.15] - 2026-07-07
### Changed
- Use supported package-root imports for cross-extension APIs.

## [1.18.14] - 2026-06-29
### Changed
- Publish ROS 2 node APIs explicitly and stop publishing the Kit lifecycle class.

## [1.18.13] - 2026-06-12
### Fixed
- `OgnROS2CameraInfoHelper`: defer the `cv2` import to first use.

## [1.18.12] - 2026-06-09
### Fixed
- Fix linter errors and missing or incomplete docstrings, and update `python_api.md`.

## [1.18.11] - 2026-05-26
### Fixed
- `OgnROS2RtxLidarHelper`: drop the legacy `Camera + IsaacRtxLidarSensorAPI` branch from render-product validation; validation now only accepts `OmniLidar + OmniSensorGenericLidarCoreAPI`.

## [1.18.10] - 2026-05-20
### Changed
- `test_rtx_sensor.TestROS2LaserScanRTX`: replace the per-frame annotator-polling + closest-timestamp lookup with a Writer-based collection. A custom `_GmoCollectorWriter` attached to the render product captures every GMO produced by the SD pipeline in arrival order (deduped by `gmo.timestampNs`), and the ROS subscriber now appends every delivered `LaserScan` rather than only keeping the most recent. The writer is attached only after the publisher discovers the subscriber so both streams begin from the same simulation frame. With the static scene and non-rotational `SICK_nanoScan3` pattern the tail of each stream describes the same ray set, so the test compares `messages[-1]` against `snapshots[-1]` via the bin-tolerant helper. Eliminates the wall-clock-proximity correlation between message stamp and snapshot identity, removing the entire class of "snap picked the wrong scan" flakes.

## [1.18.9] - 2026-05-20
### Fixed
- `test_rtx_sensor.TestROS2LaserScanRTX`: dedup GMO snapshots by `gmo.timestampNs` so each unique scan has a single entry, and look up by the closest annotator wall-clock across every frame the scan was observed (fixes a flaky wrong-scan match when the GMO annotator and the publisher latched different scans on the same simulation frame). Reconstruct the flat-scan bin index using float32 arithmetic to mirror `OgnROS2PublishLaserScan::publishFromGMO` and accept a ±1 bin shift between the publisher's prim-authored `azimuthRange`/`horizontalResolution` and the test's deg→rad→deg round-trip of `angle_min`/`angle_increment` (the C++ binning and the Python recovery can disagree on the integer slot for rays within one ULP of a bin boundary). Adds a diagnostic `carb.log_warn` whenever a strict elementwise comparison would have failed so future regressions surface scan identity, observation history, and per-slot diffs.

## [1.18.8] - 2026-05-18
### Fixed
- `test_joint_state_position_publisher_from_sensor`: command the target via `Articulation.set_dof_position_targets` and wait for joints to converge before asserting (previously compared an uncommanded transient).
- `test_spinning_camera_golden_images`: use a `depth=100` subscriber QoS for both camera streams so the rclpy executor doesn't drop frames under CI load.

## [1.18.7] - 2026-05-18
### Changed
- Add `-> None` annotations to extension lifecycle methods and annotate `srtx_instance` parameters; remove blank lines after docstrings in `ros2_common.py`

## [1.18.6] - 2026-05-15
### Changed
- `OgnROS2CameraHelper`: renamed `compressionType` input to `srtxCompressionType` and marked it as hidden.

## [1.18.5] - 2026-05-14
### Fixed
- SRTX configured sensor-set tests now skip on unsupported platforms before trying to enable `omni.replicator.srtx`, avoiding noisy Kit dependency resolution errors on Windows.

## [1.18.4] - 2026-05-13
### Fixed
- `OgnROS2RtxLidarHelper`: 2D lidar (`LaserScan`) publishing failed because the laser-scan metadata dict carried a `max_points` key that `create_laser_scan_publisher_capsule` does not accept. Pass the laser-scan parameters by explicit keyword and stop computing `max_points` (and the unused `num_channels` / `max_returns` reads) on this path.

## [1.18.3] - 2026-05-12
### Fixed
- `OgnROS2CameraHelper` / `OgnROS2CameraInfoHelper` / `OgnROS2RtxLidarHelper` - `frameSkipCount` deprecation message directs users to set input to 0.
- `OgnROS2RtxLidarHelper` uses new `RtxSensorDebugDrawPointCloud` writer rather than deprecated `RtxLidarDebugDrawPointCloudBuffer`

## [1.18.2] - 2026-05-12
### Fixed
- `OgnROS2CameraInfoHelper`: use render-product-specific SRTX sensor sets so `camera_info` publishes alongside RGB/depth in configured Mega deployments.

## [1.18.1] - 2026-05-11
### Fixed
- `OgnROS2RtxLidarHelper` / `OgnROS2RtxRadarHelper`: `showDebugView` now sets `doTransform=True` on the debug-draw writer when the sensor's `outputFrameOfReference` is anything other than `WORLD` (fixes inverted condition that left points untransformed in SENSOR frame and double-transformed in WORLD frame).
- `OgnROS2PublishPointCloud`: optional metadata fields (intensity, timestamp, emitter/channel/material/tick IDs, hit normal, velocity, object ID, echo ID, tick state, radial velocity) are gated on the matching `output*` boolean inputs in addition to the pointer being non-zero, so wiring a pointer through `IsaacExtractRTXSensorPointCloud` no longer forces the field into the message when the user has not selected it via `selectedMetadata` / `ROS2RtxLidarPointCloudConfig`. **Behavior change**: if you were driving the publisher's metadata pointer inputs directly without setting the matching `output*` flag, you will need to set the flag to keep that field in the published `PointCloud2`.

### Removed
- `OgnROS2PublishPointCloud`: removed the unused `gmoDataPtr` / `gmoBufferSize` / `gmoMaxElements` inputs and the `publishFromGMO` code path. Point cloud publishing always goes through the per-field pointer inputs populated by `IsaacExtractRTXSensorPointCloud`.

### Changed
- `OgnROS2RtxLidarHelper`: marked the `fullScan` input as `deprecated` (was already documented as ignored). The runtime warning now fires once per node lifetime rather than every compute.

## [1.18.0] - 2026-05-07
### Changed
- Enable multitick and remove non-multitick code paths.
- Deprecate `frameSkipCount` and `fullScan` inputs in all helper nodes

### Removed
- build_rtx_sensor_pointcloud_writer and dynamically-generated Writers replaced by single PointCloud writer based on isaacsim.sensors.experimental.nodes.IsaacExtractRtxSensorPointCloud
- test_rtx_sensor replaced by test_rtx_sensor_multitick

## [1.17.13] - 2026-05-07
### Fixed
- `OgnROS2PublishJointState` no longer creates a tensor simulation view when publishing from connected joint-state inputs; the tensor view is created only for the deprecated `targetPrim` path.

## [1.17.12] - 2026-05-04
### Fixed
- `OgnROS2PublishTransformTree`: guard against empty/invalid target prim paths before `GetPrimAtPath`; resolve `toSdfPath` once per entry instead of three times

## [1.17.11] - 2026-05-04
### Fixed
- Properly passthrough arguments for RTX Radar PCL metadata.

## [1.17.10] - 2026-05-01
### Fixed
- Add null check for simulation view in `OgnROS2PublishJointState` and `OgnROS2PublishTransformTree` to prevent crash when physics backend initialization fails

### Added
- Add `test_sim_clock_physics_step` test that validates ROS 2 clock publishing from an `OnPhysicsStep`-driven on-demand graph

## [1.17.9] - 2026-05-01
### Fixed
- Fix flaky waypoint follower tests by using `asyncio.sleep` instead of `simulate_until_condition` to wait for message publication

## [1.17.8] - 2026-04-30
### Fixed
- Fixed `OgnROS2PublishLaserScan` to call `generateBuffers` before `writeData` so output buffers are correctly sized before being populated
- Updated joint state subscriber tests to use `get_dof_positions`/`get_dof_velocities`; extended velocity convergence timeout and broadened multi-joint condition check
- Stabilized camera and camera info tests with fixed frame counts, proper `None` resets before timeline replay, and increased max frame limits
- Replaced manual `threading.Thread` spinning with `start_async_spinning` in waypoint follower tests
- Clean up subscriber queue tests

### Changed
- Added simulate until condition functions to tests to reduce test time

## [1.17.7] - 2026-04-30
### Changed
- Migrated `create_raycast_lidar_sensor` test helper and physics raycast tests to the new `isaacsim.sensors.experimental.physics` 3.0.0 API: call `Raycast.create(...)` (the authoring class) directly, return the authoring object's `paths[0]` instead of going through the runtime sensor, and use plural `translations=[[x, y, z]]` instead of singular `translation=Gf.Vec3d(...)`. The runtime sensor no longer forwards XformPrim attribute access or exposes a `create()` class method, so callers go through the typed authoring accessor.

## [1.17.6] - 2026-04-27
### Fixed
- Fix `OgnROS2CameraHelper` using `is None` instead of `.IsValid()` to check render product prim existence
- Fixed flaky `test_*_subscriber_queue` tests on Windows by splitting each four-subscriber test into `_small` and `_large` variants. Removing the stop/play cycle inside a single test method avoids DDS discovery stale-match on re-play, which `wait_for_subscribers_on_topic` alone cannot work around.
- Fixed race at iteration 0 of `test_transform_tree_subscriber` and `test_transform_tree_subscriber_nova_carter` by waiting for DDS discovery after `timeline.play()` before publishing the first TF.
- Fixed `test_camera_info_sim_time` float-precision flake (`0.1 > 0.0999999996`) by replacing the 1.2x ratio tolerance with an absolute upper bound (≤ 1.0s) after stop/play reset, matching the intent of the assertion.

### Changed
- Refactored `test_subscribers.py` to share a single `_run_queue_test` helper across JointState, Clock, Twist, and AckermannDriveStamped queue tests; each test now does exactly one timeline cycle.

## [1.17.5] - 2026-04-27
### Changed
- Move deprecated extension dependencies to test dependencies
- Migrate test cases to core experimental API

## [1.17.4] - 2026-04-27
### Added
- Support configured SRTX sensor sets for `OgnROS2CameraHelper` and `OgnROS2RtxLidarHelper` by resolving per-render-product sensor-set maps from carb settings and declaring shared render-product path lists before registering local outputs.

## [1.17.3] - 2026-04-24
### Changed
- Query active physics engine at runtime via `omni::physics::IPhysics` and pass it to `createSimulationView` so ROS 2 tensor-backed nodes work with any registered engine
- Add Newton backend test configuration for joint state, pose tree, odometry, and differential base tests

## [1.17.2] - 2026-04-24
### Added
- SRTX-aware code path for publishing camera info topics.

## [1.17.1] - 2026-04-23
### Fixed
- Fixed test failures caused by slow DDS endpoint discovery. Added discovery waits after `timeline.play()` in `test_laser_scan.py`, `test_publisher.py`, `test_subscribers.py`, and `test_semantic_labels.py`.
- Fixed DDS message drops in subscriber queue tests by increasing publisher QoS depth from 1 to `MAX_COUNT` so the DDS writer can buffer all messages during burst publishing.

## [1.17.0] - 2026-04-23
### Added
- `OgnROS2RtxRadarHelper` node for publishing RTX Radar data as `PointCloud2` messages to ROS 2
- Camera and camera info tests (`test_camera.py`, `test_camera_info.py`) using `isaacsim.sensors.experimental.rtx`
- Multitick rendering test coverage for RTX sensor nodes

### Changed
- Updated extension dependency from `isaacsim.sensors.rtx` to `isaacsim.sensors.experimental.rtx`

## [1.16.3] - 2026-04-22
### Fixed
- Removed duplicate `destroy_publisher`/`destroy_node` calls in joint state subscriber test teardown

### Added
- `test_joint_state_subscriber_with_name_override`: verifies `JointNameResolver` correctly maps `isaac:nameOverride` joint names to prim names for articulation control

## [1.16.2] - 2026-04-22
### Fixed
- Make the SRTX sensor-set name used by `OgnROS2CameraHelper` and `OgnROS2RtxLidarHelper` overridable via the `/exts/omni.replicator.srtx/sensorSetName` carb setting. Previously both helpers hard-coded `"default-sensor-set"`, which caused every Isaac Sim bridge in a Mega simulation to register the same sensor-set ID against the shared SRTX runtime stage, clobbering each other and causing `triggerSensorSet ... not found` / `requestCaptureFrames: timed out waiting for previous capture` cascades. The helper `get_srtx_sensor_set_name()` reads the override and falls back to the previous default for standalone Isaac Sim use. The Mega Isaac Sim bridge publishes a per-bridge unique value derived from `<robotStackId>-<robotName>`.

## [1.16.1] - 2026-04-21
### Changed
- Replaced `omni.kit.commands` raycast sensor creation in test helpers with `RaycastSensor.create()` class method

## [1.16.0] - 2026-04-20
### Changed
- Replaced `isaacsim.sensors.physx` dependency with `isaacsim.sensors.experimental.physics` and `isaacsim.sensors.physics.nodes`
- Migrated point cloud and laser scan tests from PhysX lidar to physics raycast sensor (`IsaacReadRaycastSensor`)
- `OgnROS2PublishLaserScan` legacy array path now synthesizes binary hit/miss intensities when `intensitiesData` is unconnected

## [1.15.4] - 2026-04-20
### Fixed
- Fix `test_subscriber.py` and `test_publisher.py` issues where the subscriber and publisher were not properly handling the `vertex_indices` array when it was not set

## [1.15.3] - 2026-04-16
### Fixed
- Fix `OgnROS2PublishPointCloud` issue where the publisher was using its own uninitialized frame ID instead of `db.inputs.frameId()`

## [1.15.2] - 2026-04-15
### Changed
- Updated tests to check fixed sized array edge-cases for publisher and subscriber.

## [1.15.1] - 2026-04-08
- Added `test_bbox.py` with ROS 2 bounding box publisher tests. Includes tight vs loose (i.e., occlusions included vs excluded), and "golden" csv files with ground truth bbox.

### Changed
- Moved bounding-box tests out of `test_camera.py` into `test_bbox.py`

## [1.15.0] - 2026-04-07
### Added
- Add optional multitick support. When enabled, LaserScan and PointCloud2 messages built from GenericModelOutput annotator directly, and all sensor inputs (including RTX Lidar) are assumed to be full frames.

## [1.14.1] - 2026-04-03
### Changed
- Adapt camera test euler angles to new `[roll, pitch, yaw]` input convention
- Added camera TF test verifying 180-degree x-axis rotation is applied to UsdGeomCamera prims in the ComputeTransformTree -> PublishTransformTree pipeline

## [1.14.0] - 2026-04-01
### Changed
- Removed deprecated `isaacsim.core.api` and `isaacsim.core.utils` dependencies
- Migrated to `isaacsim.core.experimental.utils`, `isaacsim.core.experimental.objects`, `isaacsim.core.experimental.prims`, `isaacsim.core.rendering_manager`, and `isaacsim.core.simulation_manager`
- Consolidated `_simulate_async` into shared `common.py` test utility
- Replaced `_find_unique_string_name` with `stage_utils.generate_next_free_path`
- Migrated `OgnROS2RtxLidarHelper` to use `ViewportManager` from `isaacsim.core.rendering_manager`

## [1.13.1] - 2026-03-26
### Changed
- Update the test dependencies to use isaacsim.robot.wheeled_robots.nodes

## [1.13.0] - 2026-03-21
### Added
- SRTX publisher support for ROS 2 image, lidar point cloud, and laser scan topics
- New C++ SRTX publisher classes: ImagePublisher, PointCloudPublisher, and PublisherBase
- SrtxPublisherFactory for creating SRTX-based ROS 2 publishers
- SRTX support in ROS2CameraHelper, ROS2CameraInfoHelper, and ROS2RtxLidarHelper OmniGraph nodes

### Changed
- Refactored OgnROS2PublishPointCloud to use shared PublisherBase/PointCloudPublisher classes

## [1.12.2] - 2026-03-21
### Fixed
- Fix test publisher hang and reduce test time

## [1.12.1] - 2026-03-19
### Fixed
- Fix broken test and reduce test time

## [1.12.0] - 2026-03-17
### Changed
- Updated documentation with AI agent.

## [1.11.0] - 2026-03-10
### Changed
- ROS2PublishTransformTree accepts optional parentFrames, childFrames, translations, and orientations inputs to receive pre-computed transform data from IsaacComputeTransformTree, deprecating direct use of targetPrims

## [1.10.1] - 2026-03-10
### Changed
- Updated test_spinning_camera_golden_images unit test with new golden USD

## [1.10.0] - 2026-03-05
### Changed
- ROS2PublishJointState node now publishes from sensor inputs (e.g. IsaacReadJointState) for joint state data

## [1.9.0] - 2026-03-02
### Changed
- Shifted RTX sensor scan accumulation and post-processing back to host by default to reduce GPU resource contention and improve frametime & frametime consistency. Post-processing-on-device still available as option by setting app.sensors.nv.[modality].outputBufferOnGPU=true.

## [1.8.0] - 2026-02-27
### Changed
- Converted OgnROS2QoSProfile node from Python to C++ for improved performance and consistency with other C++ OG nodes

## [1.7.0] - 2026-02-26
### Added
- ROS 2 H.264 Compressed Image support with H.264 RGB unit tests
- omni.replicator.nv extension automatically enabled by omni.replicator.core and implictily used for HW H.264 encoding

## [1.6.2] - 2026-02-18
### Fixed
- Fix `eFloat` and `eUnknown` cases in `writeNodeAttributeFromMessage` incorrectly hardcoding `"outputs:"` prefix instead of using `inputOutput(isOutput)` and `prependStr`

### Changed
- Move some test assertions into node subscription callbacks

## [1.6.1] - 2026-02-16
### Changed
- Add missing dependency for isaacsim.sensors.physics.nodes

## [1.6.0] - 2026-02-05
### Changed
- Update isaacsim.sensors.physics dependency to isaacsim.sensors.experimental.physics

## [1.5.10] - 2026-02-05
### Removed
- Moved test_menu_graphs to isaacsim.ros2.ui
- Simplified test dependencies
- Replaced fields_to_dtype with sensors_msgs_py.point_cloud2.read_points where applicable
- Cleaned up setUp and tearDown methods in tests

## [1.5.9] - 2026-01-30
### Changed
- Update waypoint follower action graph to use ReadPrimLocalTransform node
- Fix quaternion normalization in ROS2 waypoint follower tests

## [1.5.8] - 2026-01-26
### Added
- Also run ros2 image buffer tests with async rendering handshake enabled

## [1.5.7] - 2026-01-24
### Changed
- Fix issues with menu click and context menu tests being flaky

## [1.5.6] - 2026-01-21
### Added
- Added ros2 image buffer util unit tests

## [1.5.5] - 2025-12-23
### Changed
- Update unit tests to reduce overall test time

## [1.5.4] - 2025-12-19
### Removed
- Remove rendering manager as a test dependency since the extension already depends on it indirectly

## [1.5.3] - 2025-12-11
### Changed
- Update ros2 test case to wait for viewport to be ready
- Increase wait time for test_camera_info.py to 1.25 seconds
- Set resetSimulationTimeOnStop to True for ROS2CameraHelper, ROS2CameraInfoHelper, ROS2RtxLidarHelper

## [1.5.2] - 2025-12-02
### Fixed
- ROS2PublishObjectIdMap correctly generates 128-bit unsigned integer strings from 4xuint32

## [1.5.1] - 2025-12-01
### Added
- ros2_common.build_rtx_sensor_pointcloud_writer to build Writer for ROS2 PointCloud2 for user-selected metadata

### Changed
- ROS2RtxLidarHelper uses ros2_common.build_rtx_sensor_pointcloud_writer

## [1.5.0] - 2025-11-26
### Added
- OgnROS2RtxLidarPointCloudConfig to simplify setting metadata to include in RTX Lidar PointCloud2

### Changed
- OgnROS2RtxLidarHelper now takes selectedMetadata input to specify which metadata to include in RTX Lidar PointCloud2
- OgnROS2PublishPointCloud updated with new CUDA kernel to add RTX Lidar and Radar metadata to PointCloud2 message when data is on device

## [1.4.0] - 2025-11-26
### Changed
- Optimized to use a single publish thread rather than tasks. Can be controlled with /exts/isaacsim.ros2.bridge/publish_with_queue_thread=true|false

## [1.3.0] - 2025-11-24
### Added
- Added support for pinned memory buffers to increase memcpy performance

## [1.2.0] - 2025-11-24
### Changed
- Update code to use new handle interface from isaacsim.ros2.core extension.

## [1.1.1] - 2025-11-20
### Changed
- Cleaned up test_camera_info.py: removed unused imports and centralized visualization flag

## [1.1.0] - 2025-11-10
### Changed
- Removed "viewport" input from ROS2 camera helper node

## [1.0.1] - 2025-11-07
### Changed
- Update to Kit 109 and Python 3.12

## [1.0.0] - 2025-11-02
### Added
- Initial release of `isaacsim.ros2.nodes` extension
- Moved ROS 2 OmniGraph nodes and components from isaacsim.ros2.bridge to this extension
