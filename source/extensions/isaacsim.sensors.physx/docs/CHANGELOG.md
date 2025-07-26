# Changelog
## [2.2.27] - 2025-07-07
### Changed
- Add unit test for pybind11 module docstrings

## [2.2.26] - 2025-07-07
### Fixed
- Correctly enable omni.kit.loop-isaac in test dependency (fixes issue from 2.2.25)

## [2.2.25] - 2025-07-05
### Changed
- Update tests to pass without a custom loop runner

## [2.2.24] - 2025-07-03
### Changed
- Make omni.kit.loop-isaac an explicit test dependency

## [2.2.23] - 2025-06-25
### Changed
- Add --reset-user to test args

## [2.2.22] - 2025-06-18
### Changed
- Track change from isaacsim.core.include Pose.h

## [2.2.21] - 2025-05-31
### Changed
- Use default nucleus server for all tests

## [2.2.20] - 2025-05-19
### Changed
- Update copyright and license to apache v2.0

## [2.2.19] - 2025-05-16
### Changed
- Make extension target a specific kit version

## [2.2.18] - 2025-05-15
### Changed
- UsdUtilities.h was updated

## [2.2.17] - 2025-05-11
### Changed
- Enable FSD in test settings

## [2.2.16] - 2025-05-10
### Changed
- Remove internal build time dependency

## [2.2.15] - 2025-05-07
### Changed
- switch to omni.physics interface

## [2.2.14] - 2025-05-02
### Changed
- Remove all Dynamic control compile time dependencies

## [2.2.13] - 2025-04-09
### Changed
- Update all test args to be consistent
- Update Isaac Sim NVIDIA robot asset path

## [2.2.12] - 2025-04-04
### Changed
- Version bump to fix extension publishing issues

## [2.2.11] - 2025-03-26
### Changed
- Cleanup and standardize extension.toml, update code formatting for all code

## [2.2.10] - 2025-03-17
### Changed
- Cleanup and rename BridgeApplication to PrimManager for clarity

## [2.2.9] - 2025-03-11
### Changed
- Switch asset root for tests to internal nucleus

## [2.2.8] - 2025-03-05
### Changed
- Update extension codebase to adhere to isaac sim extension structure and file naming  guidelines

## [2.2.7] - 2025-03-04
### Changed
- Update to kit 107.1 and fix build issues

## [2.2.6] - 2025-02-21
### Changed
- Update style format and naming conventions in c++ code, add doxygen docstrings

## [2.2.5] - 2025-01-28
### Fixed
- Windows signing issue

## [2.2.4] - 2025-01-26
### Changed
- Update test settings

## [2.2.3] - 2025-01-21
### Changed
- Update extension description and add extension specific test settings

## [2.2.2] - 2024-12-18
### Fixed
- Physx Lidar azimuth and zenith units form ReadLidarBeams node

## [2.2.1] - 2024-12-16
### Changed
- Moved lightbeam_sensor example to isaacsim.sensors.physx.examples

## [2.2.0] - 2024-10-30
### Changed
- moved examples from menu to browser

## [2.1.0] - 2024-10-30
### Changed
- Use USDRT for component initialization using prim types directly rather than traversing stage for better performance

## [2.0.2] - 2024-10-28
### Changed
- Remove test imports from runtime

## [2.0.1] - 2024-10-24
### Changed
- Updated dependencies and imports after renaming

## [2.0.0] - 2024-10-04
### Removed
- Ultrasonic sensor.

## [1.0.0] - 2024-09-23
### Added
- Merges omni.isaac.proximity_sensor, isaacsim.sensors.physx, and APIs from omni.isaac.sensor into a single extension.
