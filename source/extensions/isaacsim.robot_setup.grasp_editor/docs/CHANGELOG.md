# Changelog

## [2.3.2] - 2026-05-12
### Fixed
- Wrong shape on forces/torques being added into the RigidBody during simulation.
- Spelling mistakes corrected.

## [2.3.1] - 2026-05-08
### Fixed
- Fixed silent failure of Skip Simulation -> Export Grasp; export state is preserved across internal timeline stops and errors are surfaced in the UI.
- Updated user guide to use `object_frame` / `gripper_frame` field names, matching the actual `isaac_grasp` schema written by `DataWriter`.

## [2.3.0] - 2026-03-16
### Changed
- Migrate extension implementation to core experimental API

## [2.2.0] - 2026-03-04
### Changed
- Added Overview.md, python_api.md and updated docstrings

## [2.1.3] - 2025-12-10
### Fixed
- Fixed Events 2.0 assets loaded and timeline play / stop events

## [2.1.2] - 2025-12-05
### Changed
- Migrate to Events 2.0.

## [2.1.1] - 2025-10-27
### Changed
- Make omni.isaac.ml_archive an explicit test dependency

## [2.1.0] - 2025-10-17
### Changed
- Migrate PhysX subscription and simulation control interfaces to Omni Physics

## [2.0.20] - 2025-10-08
### Fixed
- Fix issue where rigid body was re-scaled

## [2.0.19] - 2025-08-26
### Fixed
- Fixed tests for updated gripper asset

## [2.0.18] - 2025-07-07
### Fixed
- Correctly enable omni.kit.loop-isaac in test dependency (fixes issue from 2.0.17)

## [2.0.17] - 2025-07-03
### Changed
- Make omni.kit.loop-isaac an explicit test dependency

## [2.0.16] - 2025-06-25
### Changed
- Add --reset-user to test args

## [2.0.15] - 2025-05-31
### Changed
- Use default nucleus server for all tests

## [2.0.14] - 2025-05-19
### Changed
- Update copyright and license to apache v2.0

## [2.0.13] - 2025-05-16
### Changed
- Make extension target a specific kit version

## [2.0.12] - 2025-05-10
### Changed
- Enable FSD in test settings

## [2.0.11] - 2025-05-07
### Changed
- Switch to omni.physics interface

## [2.0.10] - 2025-04-09
### Changed
- Update all test args to be consistent

## [2.0.9] - 2025-04-04
### Changed
- Version bump to fix extension publishing issues

## [2.0.8] - 2025-03-26
### Changed
- Cleanup and standardize extension.toml, update code formatting for all code

## [2.0.7] - 2025-03-11
### Changed
- Switch asset root for tests to internal nucleus

## [2.0.6] - 2025-03-04
### Changed
- Update to kit 107.1 and fix build issues

## [2.0.5] - 2025-01-26
### Changed
- Update test settings

## [2.0.4] - 2025-01-21
### Changed
- Update extension description and add extension specific test settings

## [2.0.3] - 2024-12-03
### Changed
- Isaac Util menu to Tools->Robotics menu

## [2.0.2] - 2024-10-28
### Changed
- Remove test imports from runtime

## [2.0.1] - 2024-10-24
### Changed
- Updated dependencies and imports after renaming

## [2.0.0] - 2024-10-07
### Changed
- Extension renamed to isaacsim.robot_setup.grasp_editor.

## [1.3.0] - 2024-09-06
### Changed
- Changed metadata fields in Isaac Grasp file that will not interfere with grasp files that have already been created.

## [1.2.1] - 2024-08-28
### Fixed
- Unit test errors due to missing dependencies

## [1.2.0] - 2024-08-14
### Fixed
- Fixed serious bug in supported use-case where selected frames of reference have arbitrary relative transforms from the base frame.

## [1.1.1] - 2024-08-06
### Fixed
- Fix UI bug that allowed users to continue from selection frame prematurely.

## [1.1.0] - 2024-08-02
### Added
- Add Python API with tests to support loading and using imported grasps in Isaac Sim.

## [1.0.0] - 2024-07-23
### Added
- Initial version of Grasp Editor Extension
