# Changelog
## [4.1.11] - 2025-07-07
### Fixed
- Correctly enable omni.kit.loop-isaac in test dependency (fixes issue from 4.1.10)

## [4.1.10] - 2025-07-03
### Changed
- Make omni.kit.loop-isaac an explicit test dependency

## [4.1.9] - 2025-06-25
### Changed
- Add --reset-user to test args

## [4.1.8] - 2025-06-11
### Changed
- Update Franka Open Drawer Policy example
- Simplified the observation computation

## [4.1.7] - 2025-05-31
### Changed
- Use default nucleus server for all tests

## [4.1.6] - 2025-05-19
### Changed
- Update copyright and license to apache v2.0

## [4.1.5] - 2025-05-16
### Changed
- Make extension target a specific kit version

## [4.1.4] - 2025-05-10
### Changed
- Enable FSD in test settings
- Add get_physx_simulation_interface().flush_changes() to policy_controller to fix FSD issues

## [4.1.3] - 2025-05-09
### Fixed
- Buf with setting max effort with FSD enabled

## [4.1.2] - 2025-05-02
### Changed
- Update Franka and Anymal USD paths

## [4.1.1] - 2025-04-14
### Changed
- Update Isaac Sim robot asset path

## [4.1.0] - 2025-04-13
### Added
- Franka open drawer policy class

## [4.0.8] - 2025-04-09
### Changed
- Update all test args to be consistent

## [4.0.7] - 2025-04-04
### Changed
- Version bump to fix extension publishing issues

## [4.0.6] - 2025-03-26
### Changed
- Cleanup and standardize extension.toml, update code formatting for all code

## [4.0.5] - 2025-03-11
### Changed
- Switch asset root for tests to internal nucleus

## [4.0.4] - 2025-03-11
### Changed
- Reduce time to complete tests

## [4.0.3] - 2025-01-26
### Changed
- Update test settings

## [4.0.2] - 2025-01-21
### Changed
- Update extension description and add extension specific test settings

## [4.0.1] - 2024-12-22
### Added
- Rootpath input for when the articulation root is not the same as the root prim path

## [4.0.0] - 2024-11-01
### Removed
- Unitree quadruped optimized controller class
- optimized controller based standalone and ROS examples

### Added
- Policy Controller and config loader helpers for Isaac Lab based env config

## [3.0.2] - 2024-10-28
### Changed
- Remove test imports from runtime

## [3.0.1] - 2024-10-24
### Changed
- Updated dependencies and imports after renaming

## [3.0.0] - 2024-10-07
### Changed
- Extension renamed to isaacsim.robot.policy.example.
- Optimization control based robot example removed.
- Moved Humanoid template to robot policy examples

## [2.0.1] - 2024-08-28
### Fixed
- Spot unit test

## [2.0.0] - 2024-08-01
### Added
- RL policy based robot simulation for anymal and spot quadruped

## [1.4.5] - 2024-04-30
### Changed
- Updated unitree folder structure and opt to use unitree models with sensors attached

## [1.4.4] - 2024-03-07
### Changed
- Removed the usage of the deprecated dynamic_control extension

## [1.4.3] - 2024-02-12
### Changed
- Removed IMU sensor from unitree.py (since the controller uses ground truth data)
- Reduced the frequency of osqp solver from every physics step to every 5 physics steps
- Contact sensor now uses the interface instead directly of the python wrapper

## [1.4.2] - 2024-02-02
### Changed
- Updated path to the nucleus extension

## [1.4.1] - 2023-12-11
### Fixed
- Add the missing import statement for the IMUSensor

## [1.4.0] - 2023-12-06
### Changed
- Added torque clamp to the quadruped control in response to physics change
- Behavior changed, investigating performance issue

## [1.3.2] - 2023-08-22
### Fixed
- Fixed robot articulation bug after stop or reset

## [1.3.1] - 2023-06-07
### Changed
- Eliminated dependency on "bezier" python package. Behavior is unchanged.

## [1.3.0] - 2023-02-01
### Removed
- Removed Quadruped class
- Removed dynamic control extension dependency
- Used omni.isaac.sensor classes for Contact and IMU sensors

## [1.2.2] - 2022-12-10
### Fixed
- Updated camera pipeline with writers

## [1.2.1] - 2022-11-03
### Fixed
- Incorrect viewport name issue
- Viewports not docking correctly

## [1.2.0] - 2022-08-30
### Changed
- Remove direct legacy viewport calls

## [1.1.2] - 2022-05-19
### Changed
- Updated unitree vision class to use OG ROS nodes
- Updated ROS1/ROS2 quadruped standalone samples to use OG ROS nodes

## [1.1.1] - 2022-05-15
### Fixed
- DC joint order change related fixes.

## [1.1.0] - 2022-05-05
### Added
- added the ANYmal robot

## [1.0.2] - 2022-04-21
### Changed
- decoupled sensor testing from A1 and Go1 unit test
- fixed contact sensor bug in example and standalone

## [1.0.1] - 2022-04-20
### Changed
- Replaced find_nucleus_server() with get_assets_root_path()

## [1.0.0] - 2022-04-13
### Added
- quadruped class, unitree class (support both a1, go1), unitree vision class (unitree class with stereo cameras), and unitree direct class (unitree class that subscribe to external controllers)
- quadruped controllers
- documentations and unit tests
- quadruped standalone with ros 1 and ros 2 vio examples
