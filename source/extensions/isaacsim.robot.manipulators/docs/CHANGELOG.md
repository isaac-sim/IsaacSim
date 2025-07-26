# Changelog
## [3.3.5] - 2025-07-07
### Changed
- Correctly enable omni.kit.loop-isaac in test dependency (fixes issue from 3.3.4)

## [3.3.4] - 2025-07-03
### Changed
- Make omni.kit.loop-isaac an explicit test dependency

## [3.3.3] - 2025-06-25
### Changed
- Add --reset-user to test args

## [3.3.2] - 2025-05-31
### Changed
- Use default nucleus server for all tests

## [3.3.1] - 2025-05-19
### Changed
- Update copyright and license to apache v2.0

## [3.3.0] - 2025-05-12
### Changed
- Update ParallelGripper to be compatible with mimic joint claws that are controlled by a single joint

## [3.2.1] - 2025-05-10
### Changed
- Enable FSD in test settings

## [3.2.0] - 2025-04-22
### Changed
- Update to new Surface Gripper

## [3.1.1] - 2025-04-09
### Changed
- Update all test args to be consistent

## [3.1.0] - 2025-04-07
### Changed
- Removed end_effector_prim_name parameter, end effector prim path is required now

## [3.0.11] - 2025-04-07
### Changed
- Updated ur10 path
- The unit test now use Variant to create the gripper

## [3.0.10] - 2025-04-04
### Changed
- Version bump to fix extension publishing issues

## [3.0.9] - 2025-03-26
### Changed
- Cleanup and standardize extension.toml, update code formatting for all code

## [3.0.8] - 2025-03-11
### Changed
- Switch asset root for tests to internal nucleus

## [3.0.7] - 2025-03-10
### Fixed
- Unit test failure

## [3.0.6] - 2025-03-05
### Changed
- Update extension codebase to adhere to isaac sim extension structure and file naming  guidelines

## [3.0.5] - 2025-02-18
### Fixed
- Post reset gripper issue

## [3.0.4] - 2025-01-26
### Changed
- Update test settings

## [3.0.3] - 2025-01-21
### Changed
- Update extension description and add extension specific test settings

## [3.0.2] - 2024-10-28
### Changed
- Remove test imports from runtime

## [3.0.1] - 2024-10-24
### Changed
- Updated dependencies and imports after renaming

## [3.0.0] - 2024-09-30
### Changed
- extension renamed to isaacsim.robot.manipulators (from omni.isaac.manipulators)

## [2.1.0] - 2024-05-24
### Added
- end_effector_prim_path argument added to the SingleManipulator alongside the end_effector_prim_name

## [2.0.0] - 2024-03-26
### Changed
- Extension factored into multiple components.  See isaacsim.robot.manipulators.ui

## [1.3.1] - 2024-03-21
### Changed
- enabled "add to existing graph" for all the shortcuts

## [1.3.0] - 2024-03-15
### Changed
- moved all python code into a python folder. deleted omni/isaac/manipulators

### Added
- Gripper Controller Node
- automatically populated omnigraph for gripper controller
- pointer to open up the python script that generates the omnigraph for the menu shortcut graphs

## [1.2.0] - 2024-02-07
### Added
- a menu item for the extension to allow for populating common controller omnigraphs
- automatically populated omnigraph for position and velocity controller of any articulation object

## [1.1.0] - 2022-11-24
### Added
- Add functionality to run only first x phases of the pick and place controller (instead of full 10)
- Change parameter name in controller initialization
- Add documentation

## [1.0.2] - 2022-11-22
### Fixed
- Fix typo in `suction_gripper.py` warning message

## [1.0.1] - 2022-08-03
### Fixed
- Bugfix `parallel_gripper.py` `apply_action` internally calling wrong function.

## [1.0.0] - 2022-07-26
### Added
- Added Manipulator class
