# Changelog

## [1.1.8] - 2026-06-09
### Fixed
- Fix linter errors and missing or incomplete docstrings, and update `python_api.md`.

## [1.1.7] - 2026-05-10
### Fixed
- Add `is_valid` to `IMUSensor.get_current_frame()` output
- Fix `EffortSensor.get_sensor_reading()` fallback to use the newest valid buffered reading without mutating buffered state

## [1.1.6] - 2026-05-09
### Fixed
- Remove unused timeline interface lookup from `EffortSensor` callback initialization (6035373)

## [1.1.5] - 2026-04-21
### Fixed
- Removed unused `numpy` import from `EffortSensor`

## [1.1.4] - 2026-04-19
### Fixed
- Fix `IMUSensor.__init__` `UnboundLocalError` when no PhysicsScene exists on stage
- Fix `ContactSensor.__init__` `UnboundLocalError` when no PhysicsScene exists on stage

## [1.1.3] - 2026-04-17
### Fixed
- Fix `EffortSensor.change_buffer_size` creating aliased object references via `np.resize` on object-dtype array
- Fix `EffortSensor.get_sensor_reading` `ZeroDivisionError` when `step_size` is 0 and timestamps are equal
- Fix `ContactSensor.__init__` using `int(1/frequency)` which truncates to 0 for sub-Hz frequencies
- Fix `ContactSensor.__init__` validating `prim_path` instead of `self._body_prim_path` for CollisionAPI check
- Add scalar-first `(w, x, y, z)` quaternion convention to `IMUSensor.get_current_frame` docstring

## [1.1.2] - 2026-03-31
### Deprecated
- Extension deprecated in favor of the Experimental extension `isaacsim.sensors.experimental.physics`

## [1.1.1] - 2026-03-26
### Changed
- Moved Python binding module to `bindings/` subdirectory

## [1.1.0] - 2026-03-04
### Changed
- Added Overview.md and python_api.md and updated docstrings

## [1.0.1] - 2026-02-09
### Fixed
- Fix crash issue when IPhysxSimulation interface is not available

## [1.0.0] - 2026-02-01
### Changed
- Deprecated this extension in favor of isaacsim.sensors.experimental.physics extension
- Moved omnigraph nodes from this extension to isaacsim.sensors.nodes extension. The nodes use the new api from isaacsim.sensors.experimental.physics extension.

## [0.7.0] - 2026-01-28
### Added
- Refactor IMU and contact sensor tests to use experimental core APIs

## [0.6.3] - 2025-12-07
### Changed
- Fix clang tidy issues in cpp code

## [0.6.2] - 2025-12-05
### Changed
- Migrate to Events 2.0.

## [0.6.1] - 2025-12-03
### Changed
- Remove TODOs.

## [0.6.0] - 2025-11-25
### Added
- Add dedicated GPU codepath for IMU to use separate stream and pinned memory buffer

## [0.5.4] - 2025-11-07
### Changed
- Update to Kit 109 and Python 3.12

## [0.5.3] - 2025-10-31
### Changed
- Update deprecated python unittest methods

## [0.5.2] - 2025-10-27
### Changed
- Make omni.isaac.ml_archive an explicit test dependency

## [0.5.1] - 2025-10-22
### Changed
- Remove deprecated time related APIs from CoreNodes interface

## [0.5.0] - 2025-10-17
### Changed
- Migrate PhysX subscription and simulation control interfaces to Omni Physics

## [0.4.3] - 2025-10-02
### Fixed
- Orientation bug for contact and IMU sensor

## [0.4.2] - 2025-09-23
### Changed
- Fixed contact sensor implementation

## [0.4.1] - 2025-08-29
### Changed
- Renamed CARB profiling zones to include [IsaacSim] prefix

## [0.4.0] - 2025-08-07
### Changed
- Use device-generic memory buffer implementation to enable tensor API processing on the GPU for IMU sensor

## [0.3.27] - 2025-07-07
### Fixed
- Correctly enable omni.kit.loop-isaac in test dependency (fixes issue from 0.3.26)

## [0.3.26] - 2025-07-03
### Changed
- Make omni.kit.loop-isaac an explicit test dependency

## [0.3.25] - 2025-06-27
### Fixed
- Use rolling average for the Imu Ogn node test

## [0.3.24] - 2025-06-25
### Changed
- Add --reset-user to test args

## [0.3.23] - 2025-06-18
### Changed
- Track change from isaacsim.core.include Pose.h

## [0.3.22] - 2025-05-31
### Changed
- Use default nucleus server for all tests

## [0.3.21] - 2025-05-19
### Changed
- Update copyright and license to apache v2.0

## [0.3.20] - 2025-05-15
### Changed
- UsdUtilities.h was updated

## [0.3.19] - 2025-05-11
### Changed
- Enable FSD in test settings

## [0.3.18] - 2025-05-10
### Changed
- Remove internal build time dependency

## [0.3.17] - 2025-05-07
### Changed
- Switch to omni.physics interface

## [0.3.16] - 2025-05-02
### Changed
- Remove all Dynamic control compile time dependencies

## [0.3.15] - 2025-04-09
### Changed
- Update all test args to be consistent
- Update Isaac Sim NVIDIA robot asset path
- Update Isaac Sim robot asset path for the IsaacSim folder

## [0.3.14] - 2025-04-04
### Changed
- Version bump to fix extension publishing issues

## [0.3.13] - 2025-03-26
### Changed
- Cleanup and standardize extension.toml, update code formatting for all code

## [0.3.12] - 2025-03-17
### Changed
- Cleanup and rename BridgeApplication to PrimManager for clarity

## [0.3.11] - 2025-03-11
### Changed
- Switch asset root for tests to internal nucleus

## [0.3.10] - 2025-03-05
### Changed
- Update extension codebase to adhere to isaac sim extension structure and file naming  guidelines

## [0.3.9] - 2025-03-04
### Changed
- Update to kit 107.1 and fix build issues

## [0.3.8] - 2025-02-21
### Changed
- Update style format and naming conventions in c++ code, add doxygen docstrings

## [0.3.7] - 2025-02-18
### Fixed
- Make sure prims are valid before using

## [0.3.6] - 2025-01-28
### Fixed
- Windows signing issue

## [0.3.5] - 2025-01-26
### Changed
- Update test settings

## [0.3.4] - 2025-01-21
### Changed
- Update extension description and add extension specific test settings

## [0.3.3] - 2024-12-16
### Changed
- Moved samples to isaacsim.sensors.physics.examples extension.

## [0.3.2] - 2024-12-05
### Changed
- Updated Nova carter path

## [0.3.1] - 2024-11-25
### Fixed
- Error when user attempts to create contact sensor without having selected valid parent prim

## [0.3.0] - 2024-10-31
### Changed
- Moved examples from menu to browser

## [0.2.0] - 2024-10-30
### Changed
- Use USDRT for component initialization using prim types directly rather than traversing stage for better performance

## [0.1.1] - 2024-10-24
### Changed
- Updated dependencies and imports after renaming

## [0.1.0] - 2024-09-24
### Added
- Initial version of isaacsim.sensors.physics
- Includes contact sensor, IMU sensor, effort sensor
