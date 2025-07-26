# Changelog
## [2.1.21] - 2025-07-07
### Fixed
- Correctly enable omni.kit.loop-isaac in test dependency (fixes issue from 2.1.20)

## [2.1.20] - 2025-07-03
### Changed
- Make omni.kit.loop-isaac an explicit test dependency

## [2.1.19] - 2025-06-30
### Changed
- Add writeTarget.platform to extension.toml

## [2.1.18] - 2025-06-25
### Changed
- Add --reset-user to test args

## [2.1.17] - 2025-05-31
### Changed
- Use default nucleus server for all tests

## [2.1.16] - 2025-05-30
### Changed
- Update golden values for unit test

## [2.1.15] - 2025-05-19
### Changed
- Update copyright and license to apache v2.0

## [2.1.14] - 2025-05-16
### Changed
- Make extension target a specific kit version

## [2.1.13] - 2025-05-11
### Changed
- Enable FSD in test settings

## [2.1.12] - 2025-05-10
### Changed
- Remove internal build time dependency

## [2.1.11] - 2025-05-03
### Fixed
- Missing console bridge symbol

## [2.1.10] - 2025-05-02
### Changed
- Remove all Dynamic control compile time dependencies

## [2.1.9] - 2025-04-21
### Changed
- Update Franka USD path to use variants

## [2.1.8] - 2025-04-11
### Changed
- Update Isaac Sim robot asset path

## [2.1.7] - 2025-04-09
### Changed
- Update all test args to be consistent

## [2.1.6] - 2025-04-04
### Changed
- Version bump to fix extension publishing issues

## [2.1.5] - 2025-03-26
### Changed
- Cleanup and standardize extension.toml, update code formatting for all code

## [2.1.4] - 2025-03-20
### Changed
- Improve doxygen docstrings

## [2.1.3] - 2025-03-11
### Changed
- Switch asset root for tests to internal nucleus

## [2.1.2] - 2025-03-09
### Fixed
- fix failing unit tests

## [2.1.1] - 2025-03-05
### Changed
- Update extension codebase to adhere to isaac sim extension structure and file naming  guidelines

## [2.1.0] - 2025-03-05
### Changed
- Add support for ROS2 Jazzy

## [2.0.5] - 2025-03-04
### Changed
- Update to kit 107.1 and fix build issues

## [2.0.4] - 2025-02-21
### Changed
- Update style format and naming conventions in c++ code, add doxygen docstrings

## [2.0.3] - 2025-01-26
### Changed
- Update test settings

## [2.0.2] - 2025-01-21
### Changed
- Update extension description and add extension specific test settings

## [2.0.1] - 2024-11-19
### Fixed
- Startup test

## [2.0.0] - 2024-11-18
### Removed
- Removed ROS Foxy distro support

## [1.2.0] - 2024-11-11
### Added
- Add the root frame transform if it is not in the list to be rendered (configurable via carb settings)

### Changed
- Unify ROS 2 backend implementations
- Update source code to follow the Isaac Sim's Coding Style Guidelines for C++

## [1.1.0] - 2024-11-04
### Changed
- Updated tf viewer OG node replacing spinOnce function with new initialize ROS2 Node functions

## [1.0.2] - 2024-10-28
### Changed
- Remove test imports from runtime

## [1.0.1] - 2024-10-24
### Changed
- Updated dependencies and imports after renaming

## [1.0.0] - 2024-09-25
### Changed
- Extension renamed to isaacsim.ros2.tf_viewer

## [0.1.1] - 2024-08-26
### Fixed
- Fix the checked status of the menu when the window is closed by using omni.kit.menu helper

## [0.1.0] - 2024-04-12
### Added
- Initial release
