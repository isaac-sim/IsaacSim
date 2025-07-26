# Changelog
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
- switch to omni.physics interface

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
- moved examples from menu to browser

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
