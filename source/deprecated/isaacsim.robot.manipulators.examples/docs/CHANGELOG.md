# Changelog

## [2.0.1] - 2026-06-09
### Fixed
- Fix linter errors and missing or incomplete docstrings, and update `python_api.md`.

## [2.0.0] - 2026-04-06
### Deprecated
- Extension deprecated in favor of isaacsim.robot.experimental.manipulators.examples.

### Removed
- FrankaExperimental, FrankaPickPlace, and Franka Stacking (experimental) moved to isaacsim.robot.experimental.manipulators.examples.
- UR10Experimental, UR10FollowTarget, and UR10 Stacking (experimental) moved to isaacsim.robot.experimental.manipulators.examples.
- Interactive pick-place and follow-target UI extensions moved to isaacsim.robot.experimental.manipulators.examples.

## [1.6.1] - 2026-03-05
### Changed
- Experimental API alignment: app_utils for timeline control and SimulationEvent for physics callbacks in follow-target and pick-place examples.

## [1.6.0] - 2026-03-04
### Changed
- Added Overview.md, python_api.md and updated docstrings

## [1.5.0] - 2026-03-02
### Added
- UR (Universal Robots) stacking example

## [1.4.1] - 2026-02-11
### Changed
- Supported multiple Franka robots for pick and place task

## [1.4.0] - 2026-01-10
### Changed
- Updated the folder structure of the extension

### Added
- A simplified stacking class based on core_experimental (NVIDIA Warp APIs)

## [1.3.0] - 2025-12-22
### Changed
- Simplify bin filling example and improve stability, example cubes instead of various parts

## [1.2.4] - 2025-12-07
### Changed
- Updated description

## [1.2.3] - 2025-11-27
### Changed
- Add missing docstrings

## [1.2.2] - 2025-11-24
### Changed
- Update imports from isaacsim.base_samples to isaacsim.examples.base

## [1.2.1] - 2025-11-21
### Removed
- Build window function use

## [1.2.0] - 2025-11-20
### Added
- Franka Pick-and-Place and UR10 Follow Target interactive examples

## [1.1.3] - 2025-10-27
### Changed
- Make omni.isaac.ml_archive an explicit test dependency

## [1.1.2] - 2025-09-26
### Changed
- Update license headers

## [1.1.1] - 2025-08-20
### Added
- UR robot functionality based on Warp APIs
- Target-following task for the UR robot using inverse kinematics (IK)

## [1.1.0] - 2025-07-08
### Changed
- Merged task wrapper and pick-and-place controller into a single file for the Franka robot

## [1.0.17] - 2025-07-07
### Changed
- Updated path to Universal Robot's config file

## [1.0.16] - 2025-07-07
### Fixed
- Correctly enable omni.kit.loop-isaac in test dependency (fixes issue from 1.0.15)

## [1.0.15] - 2025-07-03
### Changed
- Make omni.kit.loop-isaac an explicit test dependency

## [1.0.14] - 2025-06-25
### Changed
- Add --reset-user to test args

## [1.0.13] - 2025-05-31
### Changed
- Use default nucleus server for all tests

## [1.0.12] - 2025-05-19
### Changed
- Update copyright and license to apache v2.0

## [1.0.11] - 2025-05-16
### Changed
- Make extension target a specific kit version

## [1.0.10] - 2025-05-10
### Changed
- Enable FSD in test settings

## [1.0.9] - 2025-04-22
### Changed
- Update to new Surface Gripper

## [1.0.8] - 2025-04-11
### Changed
- Update Isaac Sim robot asset path

## [1.0.7] - 2025-04-09
### Changed
- Update all test args to be consistent

## [1.0.6] - 2025-04-04
### Changed
- Version bump to fix extension publishing issues

## [1.0.5] - 2025-03-26
### Changed
- Cleanup and standardize extension.toml, update code formatting for all code
- Switch asset root for tests to internal nucleus

## [1.0.4] - 2025-02-19
### Changed
- Updated ur10 class's usd path
- Using variant for gripper creation

## [1.0.3] - 2025-01-26
### Changed
- Update test settings

## [1.0.2] - 2025-01-21
### Changed
- Update extension description and add extension specific test settings

## [1.0.1] - 2024-10-24
### Changed
- Updated dependencies and imports after renaming

## [1.0.0] - 2024-09-30
### Changed
- Extension renamed to isaacsim.robot.manipulators.examples (from omni.isaac.manipulators.examples)

## [0.1.0] - 2024-08-09
### Added
- Franka and universal_robots folder
