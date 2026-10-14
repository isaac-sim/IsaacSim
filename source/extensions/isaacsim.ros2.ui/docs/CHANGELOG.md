# Changelog

## [1.9.3] - 2026-10-02
### Fixed
- ROS 2 menu tests share UI setup and cleanup, wait for widgets to become ready, and wait for camera publisher discovery before collecting messages.

## [1.9.2] - 2026-08-04
### Fixed
- `ROS 2 OmniGraphs > Joint States` now creates the migrated joint-state publisher graph.

## [1.9.1] - 2026-07-22
### Changed
- ROS 2 graph shortcut dialogs now import the public graph construction helpers from `isaacsim.ros2.nodes`.
- Removed the `isaacsim.ros2.ui` graph helper API; use `isaacsim.ros2.nodes` instead.
- Sensor graph shortcut dialogs reuse option and warning constants from `isaacsim.ros2.nodes`.

### Fixed
- `TestMenuROS2RadarGraph.test_radar_data_flow` now validates RTX Radar menu graph wiring without running the GPU radar render loop, avoiding CI timeouts after RTX/CUDA render errors.

## [1.9.0] - 2026-07-22
### Added
- Optional render product prim selection in the ROS 2 camera, RTX lidar, and RTX radar graph shortcut dialogs. When specified, the graph uses `IsaacAttachHydraTexture` instead of `IsaacCreateRenderProduct` and reuses the existing render product path without retargeting its camera, resizing it, or authoring RenderVars on it.

### Changed
- ROS 2 graph shortcut dialogs now delegate graph construction to shared helper code.
- Camera graphs include a `ROS2CameraInfoHelper` node when creating a new camera graph.

## [1.8.0] - 2026-07-13
### Changed
- `ROS2 Camera Graph` shortcut adds an RGB compression dropdown for raw, H.264, and HEVC publishing.

### Fixed
- `ROS2 Camera Graph` now updates the RGB topic field through the public `ParamWidget.set_value()` API when the compression dropdown changes.

## [1.7.1] - 2026-07-07
### Changed
- Use supported package-root imports for cross-extension APIs.

## [1.7.0] - 2026-06-25
### Added
- New `Tools > Robotics > ROS 2 OmniGraphs > RTX Radar` menu shortcut that builds an OmniGraph publishing RTX Radar detections as `sensor_msgs/PointCloud2` via `ROS2RtxRadarHelper`. Exposes per-point Radial Velocity, Intensity, and Timestamp metadata as checkboxes; warns when Radial Velocity is selected without the OmniRadar prim advertising the `BASIC` auxiliary output channel.
- `TestMenuROS2RadarGraph` unit tests covering graph creation, the empty-prim null path, helper metadata flag wiring, and end-to-end PointCloud2 data flow on `/radar_point_cloud`.
- `Graph Shortcut` subsection in the RTX Radar tutorial (`docs/isaacsim/ros2_tutorials/tutorial_ros2_rtx_radar.rst`) describing the new menu entry.

### Fixed
- Errors in the "Add to an existing graph" checkbox selection flow for `Ros2CameraGraph`, `Ros2RtxLidarGraph`, and the new `Ros2RtxRadarGraph` when the existing graph was missing its `IsaacCreateRenderProduct` node or different camera prim is present for existing `IsaacCreateRenderProduct` node: `stage_utils.generate_next_free_path` was called with two positional arguments, but `prepend_default_prim` is keyword-only. The call now passes `prepend_default_prim=False` explicitly, matching every other callsite in `og_rtx_sensors.py`.

## [1.6.5] - 2026-06-09
### Fixed
- Fix linter errors and missing or incomplete docstrings.

## [1.6.4] - 2026-05-26
### Fixed
- `Ros2RtxLidarGraph._check_params`: drop the legacy `Camera + IsaacRtxLidarSensorAPI` branch from lidar-prim validation; validator now only accepts `OmniLidar + OmniSensorGenericLidarCoreAPI`.

## [1.6.3] - 2026-04-30
### Fixed
- Fixed crash (SIGSEGV) in `test_odometry_null_conditions` and `test_tf_null_conditions` when empty prim fields are submitted; `_check_params` in `Ros2OdometryGraph` and `Ros2TfPubGraph` now rejects empty required prims before graph creation

### Changed
- TF and Odometry builders now use the IsaacComputeTransformTree node

## [1.6.2] - 2026-04-27
### Removed
- Remove the `omni.isaac.ml_archive` dependency

## [1.6.1] - 2026-04-21
### Removed
- Remove dead root-level `__init__.py`, `extension.py`, and `og_shortcuts_menu.py` (not deployed to build output)

### Changed
- Clean up `__init__.py` exports

## [1.6.0] - 2026-04-01
### Changed
- Removed deprecated `isaacsim.core.utils` dependency
- Migrated to `isaacsim.core.experimental.utils` and `isaacsim.core.rendering_manager`
- Replaced `set_camera_view` with `ViewportManager.set_camera_view`
- Replaced `_simulate_async` with `simulate_until_condition` in test cases
- Replaced `DomeLight` with `DistantLight` in test scene setup

## [1.5.0] - 2026-03-17
### Changed
- Updated documentation with AI agent.

## [1.4.2] - 2026-03-06
### Changed
- Migrated test_menu_graphs imports to use experimental prims and stage utilities (XformPrim, define_prim, add_reference_to_stage)
- Fixed articulation root path in test_joint_states_data_flow and test_odometry_data_flow to use chassis_link
- Fixed SimpleCheckBox widget path for "Publish Robot's TF?" in test_odometry_data_flow

## [1.4.1] - 2026-03-05
### Changed
- Fix api and docs syntax issues

## [1.4.0] - 2026-02-08
### Added
- Added PointCloud2 metadata options to RTX Lidar OG tool
- Added test_menu_graphs

## [1.3.0] - 2026-02-07
### Changed
- Remove Asset Browser from menu
- Use menu.open_content_browser_to_path to open the Content Browser to a specific path as a replacement

## [1.2.0] - 2025-11-25
### Changed
- Set ResetOnStop to True for all Simulation Time OG nodes referenced by ROS Bridge

## [1.0.1] - 2025-11-10
### Fixed
- Replaced deprecated `onclick_fn` with `onclick_action` in menu items to eliminate deprecation warnings
- Registered proper actions for Nova Carter, Leatherback, and iw.hub ROS robot creation menu items

## [1.0.0] - 2025-11-03
### Changed
- Moved all ui components of isaacsim.ros2.bridge to this extension
