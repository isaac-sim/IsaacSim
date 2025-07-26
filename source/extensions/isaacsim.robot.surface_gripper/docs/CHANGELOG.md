# Changelog
## [3.2.6] - 2025-07-07
### Fixed
- Correctly enable omni.kit.loop-isaac in test dependency (fixes issue from 3.2.5)

## [3.2.5] - 2025-07-03
### Changed
- Update extension unit tests to use tensor API from gripper view

## [3.2.4] - 2025-07-03
### Changed
- Make omni.kit.loop-isaac an explicit test dependency

## [3.2.3] - 2025-06-25
### Changed
- Add --reset-user to test args

## [3.2.2] - 2025-06-12
### Fixed
- Fix broken autodocs references in api.rst

## [3.2.1] - 2025-06-10
### Changed
- Remove unnecessary dependencies and imports

## [3.2.0] - 2025-06-06
### Changed
- Update source code to use ops from the experimental core utils extension

## [3.1.5] - 2025-06-03
### Changed
- Fix incorrect licenses and add missing licenses

## [3.1.4] - 2025-06-02
### Fixed
- Ensure Reset Gripper is called at the same time frame as the Open Command.

## [3.1.3] - 2025-06-01
### Changed
- Schema rename

## [3.1.2] - 2025-05-31
### Changed
- Use default nucleus server for all tests

## [3.1.1] - 2025-05-22
### Changed
- Update copyright and license to apache v2.0

## [3.1.0] - 2025-05-20
### Added
- Tensor API for surface gripper control

## [3.0.5] - 2025-05-16
### Changed
- Make extension target a specific kit version

## [3.0.4] - 2025-05-16
### Changed
- Update extension unit tests

## [3.0.3] - 2025-05-15
### Changed
- UsdUtilities.h was updated

## [3.0.2] - 2025-05-11
### Changed
- Enable FSD in test settings

## [3.0.1] - 2025-05-10
### Changed
- Remove internal build time dependency

## [3.0.0] - 2025-04-16
### Changed
- New Surface Gripper component for gripper schema.

## [2.0.13] - 2025-04-09
### Changed
- Update Isaac Sim robot asset path
- Update Isaac Sim robot asset path for the IsaacSim folder

## [2.0.12] - 2025-04-04
### Changed
- Version bump to fix extension publishing issues

## [2.0.11] - 2025-03-26
### Changed
- Cleanup and standardize extension.toml, update code formatting for all code

## [2.0.10] - 2025-03-05
### Changed
- Update extension codebase to adhere to isaac sim extension structure and file naming  guidelines

## [2.0.9] - 2025-03-04
### Changed
- Update to kit 107.1 and fix build issues

## [2.0.8] - 2025-02-21
### Changed
- Update style format and naming conventions in c++ code, add doxygen docstrings

## [2.0.7] - 2025-02-17
### Changed
- Updated docstrings, and code to match new isaac sim style guide

## [2.0.6] - 2025-01-21
### Changed
- Update extension description and add extension specific test settings

## [2.0.5] - 2024-12-03
### Changed
- Create menu to Tools->Robotics->Omnigraphs menu

## [2.0.4] - 2024-10-28
### Changed
- Remove test imports from runtime

## [2.0.3] - 2024-10-24
### Fixed
- Make extension OS specific

## [2.0.2] - 2024-10-24
### Changed
- Updated dependencies and imports after renaming

## [2.0.1] - 2024-10-14
### Fixed
- Minor fix from refactor

## [2.0.0] - 2024-09-27
### Changed
- Extension renamed to isaacsim.robot.surface_gripper

## [1.0.1] - 2024-04-01
### Changed
- Changed Dynamic Control Raycast into Physx scene query raycast.

## [1.0.0] - 2024-03-13
### Changed
- Extension refactored into multiple components.  See isaacsim.robot.surface_gripper.ui.

## [0.8.2] - 2024-03-07
### Changed
- Removed the usage of the deprecated dynamic_control extension

## [0.8.1] - 2024-03-04
### Changed
- Updated omnigraph nodes to use per instance state instead of internal state

## [0.8.0] - 2024-01-08
### Changed
- Moved header files to extension

## [0.7.0] - 2023-11-29
### Changed
- Location on the Menu reverted back to from Create->Isaac->End Effector

## [0.6.0] - 2023-11-20
### Changed
- Location on the Menu changed from Create->Isaac->End Effector to Create->Isaac->Robot.

## [0.5.0] - 2023-09-26
### Changed
- Updated ogn node to prevent issue where it would only call for close on first attempt.
- Automatically move gripper origin out of collision body, with a warning to tell how much to push it forward

## [0.4.1] - 2023-08-11
### Changed
- Updated ogn node prims types from bundle to ogn

## [0.4.0] - 2023-01-03
### Changed
- Update to new action based menu system

## [0.3.2] - 2022-12-01
### Fixed
- CreateSurfaceGripper command documentation update

## [0.3.1] - 2022-12-01
### Fixed
- CreateSurfaceGripper command .do() only returns the created prim and not a tuple

## [0.3.0] - 2022-10-27
### Added
- Moved surface gripper to Create -> Isaac -> End Effector menu
- More Robots, Environments to menu

## [0.2.0] - 2022-10-14
### Added
- Logic for keeping surface gripper attempting to close until it reaches some object

## [0.1.3] - 2022-09-07
### Fixed
- Fixes for kit 103.5

## [0.1.2] - 2022-05-10
### Changed
- Change Tests to use meters as distance unit

## [0.1.1] - 2022-05-03
### Added
- Surface Gripper Omnigraph Node

## [0.1.0] - 2021-07-31
### Added
- Initial version of extension
