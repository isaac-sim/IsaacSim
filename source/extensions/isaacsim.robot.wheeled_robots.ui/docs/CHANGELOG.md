# Changelog

## [2.3.3] - 2026-06-09
### Fixed
- Fix linter errors and missing or incomplete docstrings, and update `python_api.md`.

## [2.3.2] - 2026-06-02
### Fixed
- Corrected Differential Controller graph shortcut joint names and indices to use left/right wheel order.
- Supported joint index 0 by using -1 as the unset index sentinel.

## [2.3.1] - 2026-04-13
### Removed
- Remove the `omni.isaac.ml_archive` dependency

## [2.3.0] - 2026-03-25
### Changed
- Migrated to experimental APIs (app_utils, prim_utils, stage_utils) replacing deprecated isaacsim.core.utils
- Updated dependency from isaacsim.robot.wheeled_robots to isaacsim.robot.wheeled_robots.nodes
- Converted test file to use SimulationManager, experimental Articulation, and GroundPlane

## [2.2.0] - 2026-03-04
### Changed
- Added Overview.md, python_api.md and updated docstrings

## [2.1.25] - 2026-01-24
### Changed
- Fix issues with menu click and context menu tests being flaky

## [2.1.24] - 2025-10-27
### Changed
- Make omni.isaac.ml_archive an explicit test dependency

## [2.1.23] - 2025-07-07
### Fixed
- Correctly enable omni.kit.loop-isaac in test dependency (fixes issue from 1.4.8)

## [2.1.22] - 2025-07-05
### Changed
- Update tests to pass without a custom loop runner

## [2.1.21] - 2025-07-03
### Changed
- Make omni.kit.loop-isaac an explicit test dependency

## [2.1.20] - 2025-06-27
### Changed
- Retry test if window is not found

## [2.1.19] - 2025-06-25
### Changed
- Add --reset-user to test args

## [2.1.18] - 2025-06-13
### Changed
- Fix menu test timeout

## [2.1.17] - 2025-06-06
### Changed
- Increase timeout for UI tests

## [2.1.16] - 2025-05-31
### Changed
- Use default nucleus server for all tests

## [2.1.15] - 2025-05-30
### Changed
- Update timeouts to fix test

## [2.1.14] - 2025-05-19
### Changed
- Update copyright and license to apache v2.0

## [2.1.13] - 2025-05-10
### Changed
- Enable FSD in test settings

## [2.1.12] - 2025-05-03
### Changed
- Update test settings to fix menu test failures

## [2.1.11] - 2025-04-09
### Changed
- Update all test args to be consistent
- Update Isaac Sim NVIDIA robot asset path

## [2.1.10] - 2025-04-04
### Changed
- Version bump to fix extension publishing issues

## [2.1.9] - 2025-03-26
### Changed
- Cleanup and standardize extension.toml, update code formatting for all code

## [2.1.8] - 2025-03-11
### Changed
- Switch asset root for tests to internal nucleus

## [2.1.7] - 2025-03-04
### Changed
- Update to kit 107.1 and fix build issues

## [2.1.6] - 2025-02-10
### Added
- Added test for differential robot graph shortcut

## [2.1.5] - 2025-01-27
### Changed
- Updated docs link

## [2.1.4] - 2025-01-26
### Changed
- Update test settings

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
- Menu location and name

## [2.0.1] - 2024-10-24
### Changed
- Updated dependencies and imports after renaming

## [2.0.0] - 2024-10-02
### Changed
- Extension renamed to isaacsim.robot.wheeled_robots.ui

## [1.1.2] - 2024-09-13
### Fixed
- Changed pxr.OmniGraphSchema import to OmniGraphSchema

## [1.1.1] - 2024-05-22
### Changed
- Docs link changed from internal to external

## [1.1.0] - 2024-05-09
### Changed
- Only ask for robot parent prim, automatically search for Articulation Root API under the hood

## [1.0.3] - 2024-04-19
### Added
- Reverted the name of "Differential Robots" menu option to "Differential Controller"

## [1.0.2] - 2024-04-14
### Added
- Button to documentation for omnigraph shortcut

## [1.0.1] - 2024-03-25
### Changed
- Option to add to a existing graph for omnigraph controller shortcuts

## [1.0.0] - 2024-02-28
### Added
- Initial version
