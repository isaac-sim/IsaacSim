# Changelog
## [2.1.22] - 2025-07-07
### Fixed
- Correctly enable omni.kit.loop-isaac in test dependency (fixes issue from 2.1.21)

## [2.1.21] - 2025-07-03
### Changed
- Make omni.kit.loop-isaac an explicit test dependency

## [2.1.20] - 2025-06-25
### Changed
- Add --reset-user to test args

## [2.1.19] - 2025-06-25
### Changed
- Fix menu test, Gripper Control Golden, not finding the window given delay. (ETM)

## [2.1.18] - 2025-06-13
### Changed
- Fix menu test timeout

## [2.1.17] - 2025-06-11
### Changed
- Update dependecies

## [2.1.16] - 2025-05-31
### Changed
- Use default nucleus server for all tests

## [2.1.15] - 2025-05-30
### Changed
- Update golden values for unit test

## [2.1.14] - 2025-05-19
### Changed
- Update copyright and license to apache v2.0

## [2.1.13] - 2025-05-10
### Changed
- Enable FSD in test settings

## [2.1.12] - 2025-05-04
### Fixed
- Ui test failures

## [2.1.11] - 2025-05-03
### Changed
- Add ui specific test args

## [2.1.10] - 2025-04-09
### Changed
- Update all test args to be consistent
- Update Isaac Sim NVIDIA robot asset path
- Update Isaac Sim robot asset path

## [2.1.9] - 2025-04-04
### Changed
- Version bump to fix extension publishing issues

## [2.1.8] - 2025-03-26
### Changed
- Cleanup and standardize extension.toml, update code formatting for all code

## [2.1.7] - 2025-03-11
### Changed
- Switch asset root for tests to internal nucleus

## [2.1.6] - 2025-03-10
### Fixed
- Unit test dependencies

## [2.1.5] - 2025-02-11
### Added
- Added tests for omnigraph shortcuts

## [2.1.4] - 2025-01-27
### Changed
- Updated docs link

## [2.1.3] - 2025-01-21
### Changed
- Update extension description and add extension specific test settings

## [2.1.2] - 2025-01-17
### Changed
- Temporarily changed docs link

## [2.1.1] - 2024-12-05
### Changed
- Updated OmniGraph naming

## [2.1.0] - 2024-11-01
### Changed
- Menu name/location

## [2.0.1] - 2024-10-24
### Changed
- Updated dependencies and imports after renaming

## [2.0.0] - 2024-09-30
### Changed
- Extension renamed to isaacsim.robot.manipulators.ui (from omni.isaac.manipulators.ui)

## [1.2.2] - 2024-09-13
### Fixed
- changed pxr.OmniGraphSchema import to OmniGraphSchema

## [1.2.1] - 2024-05-22
### Changed
- docs link changed from internal to external

## [1.2.0] - 2024-05-09
### Changed
- only ask for robot parent prim, automatically search for Articulation Root API under the hood

## [1.1.0] - 2024-04-13
### Added
- needed robot root input separately from articulation root for Articulation Position/Velocity controller graphs
- documentation button for omnigraph shortcuts

## [1.0.0] - 2024-03-20
### Added
- created isaacsim.robot.manipulators.ui
