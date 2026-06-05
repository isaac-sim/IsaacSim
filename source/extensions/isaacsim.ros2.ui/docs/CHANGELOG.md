# Changelog

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
