# Changelog
## [4.6.10] - 2025-07-07
### Fixed
- Correctly enable omni.kit.loop-isaac in test dependency (fixes issue from 4.6.9)

## [4.6.9] - 2025-07-03
### Changed
- Make omni.kit.loop-isaac an explicit test dependency

## [4.6.8] - 2025-06-25
### Changed
- Add --reset-user to test args

## [4.6.7] - 2025-06-25
### Changed
- Update surface gripper gantry stage to allow manual update of joint targets

## [4.6.6] - 2025-06-11
### Changed
- Update Franka Open Drawer Policy example

## [4.6.5] - 2025-06-06
### Fixed
- Removed the lingering physics callback after stop is pressed

## [4.6.4] - 2025-05-31
### Changed
- Use default nucleus server for all tests

## [4.6.3] - 2025-05-19
### Changed
- Update copyright and license to apache v2.0

## [4.6.2] - 2025-05-10
### Changed
- Enable FSD in test settings

## [4.6.1] - 2025-05-04
### Changed
- Reduce time taken to run tests

### Fixed
- Various unit tests

## [4.6.0] - 2025-05-03
### Changed
- Updated Surface Gripper Example

## [4.5.1] - 2025-04-27
### Fixed
- rename ui unit tests and add ui related test settings

## [4.5.0] - 2025-04-13
### Added
- Franka open drawer policy inference example

## [4.4.15] - 2025-04-09
### Changed
- Update all test args to be consistent
- Update Isaac Sim NVIDIA robot asset path
- Update Isaac Sim robot asset path

## [4.4.14] - 2025-04-04
### Changed
- Version bump to fix extension publishing issues

## [4.4.13] - 2025-03-26
### Changed
- Cleanup and standardize extension.toml, update code formatting for all code

## [4.4.12] - 2025-03-24
### Changed
- Migrate to Events 2.0

## [4.4.11] - 2025-03-11
### Changed
- Switch asset root for tests to internal nucleus

## [4.4.10] - 2025-03-09
### Fixed
- fix failing unit tests

## [4.4.9] - 2025-02-20
### Changed
- Added additional test settings to extension.toml

## [4.4.8] - 2025-01-27
### Changed
- Updated docs link

## [4.4.7] - 2025-01-26
### Changed
- Update test settings

## [4.4.6] - 2025-01-21
### Changed
- Update extension description and add extension specific test settings

## [4.4.5] - 2025-01-17
### Changed
- Temporarily changed docs links

## [4.4.4] - 2025-01-14
### Added
- refactor LoadButtons UI

## [4.4.3] - 2025-01-10
### Added
- Added a few more lines of instructions inside the "Information" tab for multi-robot examples

## [4.4.2] - 2024-12-16
### Fixed
- Error when extension was hot reloaded

## [4.4.1] - 2024-12-04
### Fixed
- Missing import for follow_target_example

## [4.4.0] - 2024-11-27
### Added
- Getting Started Tutorials

## [4.3.0] - 2024-11-25
### Changed
- Base Sample Extension now only have UI Templates. Each example has its own Extension startup and shutdown.

## [4.2.0] - 2024-10-29
### Changed
- Moved all the examples from Menu to Browser

## [4.1.3] - 2024-10-28
### Changed
- Remove test imports from runtime

## [4.1.2] - 2024-10-24
### Changed
- Updated dependencies and imports after renaming

## [4.1.1] - 2024-10-14
### Fixed
- Surface Gripper example tool orientation

## [4.1.0] - 2024-10-07
### Changed
- Updated quadruped dependency to isaacsim.robot.policy.examples
- Moved H1 humanoid exmaple to isaacsim.robot.policy.examples

## [4.0.0] - 2024-09-30
### Changed
- changed extension name to isaacsim.examples.interactive

## [3.0.3] - 2024-09-28
### Fixed
- Surface Gripper extension renaming

## [3.0.2] - 2024-09-16
### Fixed
- Fixed Kaya gamepad example, loading ground and light manually

## [3.0.1] - 2024-08-27
### Fixed
- Fixed glitch with quadruped example

## [3.0.0] - 2024-08-06
### Added
- Reinforcement learning based humanoid and quadruped examples
- Humanoid H1 definition file
- Unit tests for the humanoid H1 example

### Removed
- Optimization based quadruped controller example

## [2.2.1] - 2024-07-17
### Fixed
- Examples world instances clean up at beginning of every example

## [2.2.0] - 2024-07-10
### Removed
- Deprecated omni.isaac.dofbot and removed its usage.

## [2.1.0] - 2024-06-07
### Changed
- Base sample calls set_camera_view as it was removed from World/SimulationContext

## [2.0.0] - 2024-05-16
### Removed
- Franka Nut & Bolt Example

## [1.7.17] - 2024-05-03
### Fixed
- Quadruped example to clear callbacks after it's complete

## [1.7.16] - 2024-04-19
### Fixed
- Updated Path Planning Example for compatibility with Lula 0.10.0

## [1.7.15] - 2024-03-07
### Changed
- Removed the usage of the deprecated dynamic_control extension

## [1.7.14] - 2024-02-02
### Changed
- Updated path to the nucleus extension

## [1.7.13] - 2023-12-13
### Fixed
- Moved nut screwing to start with 3rd nut with slight improvement to centering

## [1.7.12] - 2023-12-11
### Changed
- Force stop bin filling once gripper constraint breaks
- Add small force to induce gripper breaking if it doesn't happen on bin pieces falling

## [1.7.11] - 2023-12-06
### Changed
- Increase quadruped example physics rate to 400hz

## [1.7.10] - 2023-12-03
### Fixed
- Fixed issue with screwing in nut and bolt demo
- Improved accuracy of placement and rotation and increased rotation speed

## [1.7.9] - 2023-11-13
### Fixed
- Updated documentation links for the examples

## [1.7.8] - 2023-10-17
### Fixed
- Changed end offector offset for Franka Stacking Controller in robo party example to [0, 0, 0] instead of [0, 0, -0.015]

## [1.7.7] - 2023-10-12
### Fixed
- Fixed nut tying issue on Windows

## [1.7.6] - 2023-10-12
### Fixed
- Hang on startup due to franka nut and bolt querying nucleus

## [1.7.5] - 2023-10-09
### Changed
- Increased quadruped example gain and physics rate, added notes in the description.

## [1.7.4] - 2023-09-29
### Fixed
- Fixed nut slipping from Franka hand for nut and bolt extension
- Changed num of bolts to two in nut and bolt extension

## [1.7.3] - 2023-09-29
### Fixed
- Fixed collision groups and num_bolts for nut and bolt extension

## [1.7.2] - 2023-09-26
### Fixed
- Changed end offector offset in pick and place examples using Franka to [0, 0, 0] instead of [0, 0, -0.015]

## [1.7.1] - 2023-08-22
### Fixed
- Fixed quadruped example error after reset or stop

## [1.7.0] - 2023-08-15
### Removed
- Moved Cortex Behaviors to isaacsim.cortex.behaviors

## [1.6.5] - 2023-08-07
### Fixed
- Incorrect scaling factor in Omnigraph Keyboard example

## [1.6.4] - 2023-08-02
### Fixed
- Physics/lighting error caused by 100x scale in cube size and camera position

## [1.6.3] - 2023-06-30
### Fixed
- Error in nut+bolt example due to USD schema changes

## [1.6.2] - 2023-05-19
### Fixed
- Missing cortex dependency

## [1.6.1] - 2023-05-09
### Fixed
- Fix for Nut and Bolt Demo Windows Bug

## [1.6.0] - 2023-03-23
### Added
- Cortex Samples

## [1.5.6] - 2023-02-24
### Fixed
- Fix for 6 bolts in Nut and Bolt Demo

## [1.5.5] - 2023-02-20
### Fixed
- Improved Screw Controller in Nut and Bolt Demo

## [1.5.4] - 2023-02-08
### Fixed
- fixed vibrating table in Nut and Bolts Demo

## [1.5.3] - 2023-01-19
### Fixed
- missing button error when running tests

## [1.5.2] - 2023-01-06
### Fixed
- onclick_fn warning when creating UI

## [1.5.1] - 2022-12-09
### Changed
- Docs url for bin filling extension

## [1.5.0] - 2022-12-04
### Added
- Add Nut and Bolt example

## [1.4.0] - 2022-08-30
### Changed
- Remove direct legacy viewport calls

## [1.3.0] - 2022-08-29
### Removed
- Deprecated joint control and read articulation Dynamic control examples. articulation and articulation view provide similar functionality in core and are already documented.

## [1.2.0] - 2022-06-17
### Added
- Path Planning Example with resiazable and movable walls

## [1.1.0] - 2022-06-16
### Added
- Added keep_window_open parameter to BaseSampleExtension to keep a sample's window visible after hot-reloading.

## [1.0.0] - 2022-05-20
### Removed
- ROS examples

## [0.3.0] - 2022-05-05
### Changed
- stage setting changed from cm to m.
- robofactory and roboparty uses hard coded position in meters (instead of cm)

## [0.2.0] - 2022-05-05
### Changed
- Jetbot keyboard example replaced by omnigraph_keyboard, using scripting omnigraph to resizing a cube instead of moving a robot

## [0.1.22] - 2022-04-21
### Changed
- Changed init functions for Franka, UR10, and DofBot controller classes alongside changes to motion_generation

## [0.1.21] - 2022-04-14
### Changed
- Replaced kaya holonomic controller with the generic controller

## [0.1.20] - 2022-03-16
### Changed
- Replaced find_nucleus_server() with get_assets_root_path()
- Jetbot Keyboard example and Kaya Gamepad example are now powered by Omnigraph

## [0.1.19] - 2022-2-10
### Changed
- Updated references to MotionGeneration

## [0.1.18] - 2022-01-27
### Added
- Cleaned up BaseSample UI
- Added Toggle Buttons to FollowTarget Example

## [0.1.17] - 2021-12-09
### Added
- Added a replay follow target example to showcase data logging and how to replay data in simulation.

## [0.1.16] - 2021-12-08
### Added
- Stop button greys out the buttons of the sample so the user presses reset for a proper reset.

## [0.1.15] - 2021-12-07
### Changed
- post_reset is not called after load anymore

### Added
- pre_reset function in base sample

### Fixed
- Follow Target example when adding an obstacle and then resetting

## [0.1.14] - 2021-12-02
### Changed
- Propagation of core api changes
- Rename kaya joystick to kaya gamepad

## [0.1.13] - 2021-11-05
### Changed
- Moved setting world settings logic to BaseSample instead of BaseSampleExtension
- Added pause after load button is pressed.

## [0.1.12] - 2021-11-01
### Changed
- renamed extension to omni.isaac.examples

## [0.1.11] - 2021-11-01
### Changed
- Added RoboFactory sample
- Changed name of multiple tasks to RoboParty Sample
- Added Follow Target sample
- Added Hello World Sample
- Added Simple Stack Sample

## [0.1.10] - 2021-07-26
### Changed
- New UI for Kaya Joystick and Jetbot Keyboard exampls

## [0.1.9] - 2021-07-23
### Changed
- Moved dofbot rmp config to lula package

## [0.1.8] - 2021-07-12
### Added
- add UI Utils to Import URDF

## [0.1.7] - 2021-07-08
### Added
- add dofbot rmp sample

## [0.1.6] - 2021-05-24
### Added
- Added dofbot sample
- Updated to latest physics api

## [0.1.5] - 2021-03-06
### Added
- Franka Replay Sample

## [0.1.4] - 2021-02-17
### Added
- update to python 3.7
- update to omni.kit.uiapp
- Update RMP sample to save data

## [0.1.3] - 2021-01-13
### Added
- Add support for 6DOF RMP target

## [0.1.2] - 2020-12-16
### Added
- RMP sample errors when adding obstacles

## [0.1.1] - 2020-12-14
### Added
- Fix issue with franka sample rmp config files not being found

## [0.1.0] - 2020-12-11
### Added
- Initial version of Isaac Sim Samples Extension
