# Changelog
## [4.0.23] - 2025-07-07
### Fixed
- Correctly enable omni.kit.loop-isaac in test dependency (fixes issue from 4.0.22)

## [4.0.22] - 2025-07-05
### Changed
- Update tests to pass without a custom loop runner

## [4.0.21] - 2025-07-03
### Changed
- Make omni.kit.loop-isaac an explicit test dependency

## [4.0.20] - 2025-06-25
### Changed
- Add --reset-user to test args

## [4.0.19] - 2025-06-18
### Fixed
- Fixed WheelBasePoseController for moving forward along -x axis

## [4.0.18] - 2025-06-18
### Changed
- Add units to differential and holonomic controller nodes descriptions

## [4.0.17] - 2025-05-31
### Changed
- Use default nucleus server for all tests

## [4.0.16] - 2025-05-19
### Changed
- Update copyright and license to apache v2.0

## [4.0.15] - 2025-05-16
### Changed
- Make extension target a specific kit version

## [4.0.14] - 2025-05-11
### Changed
- Enable FSD in test settings

## [4.0.13] - 2025-05-10
### Changed
- Remove internal build time dependency

## [4.0.12] - 2025-04-09
### Changed
- Update all test args to be consistent
- Update Isaac Sim NVIDIA robot asset path
- Update Isaac Sim robot asset path for the IsaacSim folder

## [4.0.11] - 2025-04-04
### Changed
- Version bump to fix extension publishing issues

## [4.0.10] - 2025-03-26
### Changed
- Cleanup and standardize extension.toml, update code formatting for all code

## [4.0.9] - 2025-03-20
### Changed
- Improve doxygen docstrings

## [4.0.8] - 2025-03-12
### Fixed
- Fix attribute typo when getting wheel velocities

## [4.0.7] - 2025-03-11
### Changed
- Switch asset root for tests to internal nucleus

## [4.0.6] - 2025-03-05
### Changed
- Update extension codebase to adhere to isaac sim extension structure and file naming  guidelines

## [4.0.5] - 2025-03-04
### Changed
- Update to kit 107.1 and fix build issues

## [4.0.4] - 2025-01-28
### Fixed
- Windows signing issue

## [4.0.3] - 2025-01-26
### Changed
- Update test settings

## [4.0.2] - 2025-01-21
### Changed
- Update extension description and add extension specific test settings

## [4.0.1] - 2024-12-05
### Changed
- Updated Nova carter path

## [4.0.0] - 2024-10-30
### Added
- Enhanced AckermannController class by incorporating new parameters and a higher-fidelity kinematic model to minimize slip in a four-wheel-drive system.
- Added new OgnAckermannController OmniGraph node. Previous OmniGraph node has been deprecated and renamed to OgnAckermannControllerDeprecated.

## [3.0.1] - 2024-10-24
### Changed
- Updated dependencies and imports after renaming

## [3.0.0] - 2024-10-02
### Changed
- extension renamed to isaacsim.robot.wheeled_robots

## [2.3.3] - 2024-07-17
### Fixed
- Fixed acceleration control for the ackermann controller

## [2.3.2] - 2024-07-10
### Changed
- Changed Ackermann controller currentLinearVelocity from double to vectord
- Unit test for enabled acceleration on Ackermann controller altered

## [2.3.1] - 2024-07-08
### Changed
- use omni.isaac.debug_draw instead of omni.debugdraw

## [2.3.0] - 2024-07-03
### Added
- Created Ackermann controller class
- Created Ackermann controller Omnigraph node
- Unit tests for Ackermann controller

## [2.2.0] - 2024-06-14
### Changed
- Deprecated OgnAckermann OmniGraph node

## [2.1.3] - 2024-05-29
### Fixed
- Crash on stage close if twistsubscriber was not run for a single frame

## [2.1.2] - 2024-05-04
### Added
- unit test for robot reset using jetbot

## [2.1.1] - 2024-04-24
### Changed
- Updated API docs

## [2.1.0] - 2024-04-15
### Changed
- Changed desired velocity input name from velocityCommands to inputVelocity to fix a type not match issue

## [2.0.0] - 2024-03-17
### Changed
- Extension refactored into separate components.  See omni.isaac.wheeled_robots.ui for UI components of the extension.

## [1.1.5] - 2024-03-13
### Changed
- added empty string to get_next_free_path's parent argument in Omnigraph shortcuts
- added ability to open python script in the popup dialog for omnigraph shortcuts

## [1.1.4] - 2024-03-12
### Added
- Added acceleration restraints to differential controller

## [1.1.3] - 2024-03-06
### Fixed
- Removed commented out code

## [1.1.2] - 2024-02-24
### Changed
- Update source code to run a build clean of warnings

### Fixed
- Crash on differential controller reset

## [1.1.1] - 2024-02-17
### Changed
- Replaced internalState with perInstanceState for python ogn nodes
- small modifications in the Holonomic node definition
- All custom_reset function in directly set the node using OG API

### Added
- Reset test for holonomic controller

## [1.1.0] - 2024-02-13
### Added
- a menu item for the extension to allow for populating common controller omnigraphs
- automatically populated omnigraph for controlling a differential robot

## [1.0.1] - 2024-02-05
### Changed
- Replaced internalState with perInstanceState for the ogn nodes

## [1.0.0] - 2024-01-30
### Changed
- Refactored wheeled robots extension to use carb plugin and support Cpp code
- Rewrote differential controller node in Cpp
- Removed unused outputs for position and effort control

## [0.9.2] - 2024-01-18
### Changed
- Changed get_assets_root_path to get_assets_root_path_async for the unit tests

## [0.9.1] - 2024-01-08
### Fixed
- Updated description in Ackermann Steering node

## [0.9.0] - 2023-11-28
### Changed
- OgnAckermannSteering node receives inputs in SI units.
- OgnAckermannSteering node now accepts speed and acceleration as desired forward motion inputs.
- OgnAckermannSteering node uses front axel steering angle as input rather than curvature

## [0.8.1] - 2023-11-27
### Fixed
- _wheeled_dof_indices vs _wheel_dof_indices typo

## [0.8.0] - 2023-11-13
### Changed
- Moved wheel base pose controller from motion generation extension

## [0.7.2] - 2023-10-11
### Fixed
- Differential controller will not ignore 0s as max speed inputs.

## [0.7.1] - 2023-10-06
### Changed
- wheeled robot class can now accept a relative path from the default prim to the robot prim when the robot is not the default prim

### Fixed
- Differential controller now resets on simulation Stop

## [0.7.0] - 2023-09-06
### Added
- Ackermann Steering OmniGraph node

## [0.6.5] - 2023-08-10
### Fixed
- Changed robot prim types from bundle to target in Omnigraph

## [0.6.4] - 2023-06-13
### Fixed
- Kit 105.1 update
- use omni.usd instead of omni.usd.utils for get_world_transform_matrix

## [0.6.3] - 2023-03-10
### Fixed
- Error on stop for holonomic and differential controllers

## [0.6.2] - 2023-03-07
### Fixed
- Hardcoded stanley control parameters
- OGN wrapper node could not change stanley control parameters

## [0.6.1] - 2022-08-29
### Fixed
- Issue with targeting using coordinates instead of prim

### Changed
- Removed excessive db.inputs calls to improve efficiency/speed
- Removed path drawing (may add later as an option)

## [0.6.0] - 2022-08-22
### Added
- OgnQuinticPathPlanner
- OgnCheckGoal2D
- OgnStanleyControlPID

## [0.5.8] - 2022-08-08
### Fixed
- Issue with holonomic controller returning an error on reset

## [0.5.7] - 2022-07-22
### Changed
- pulled out the internal state classes for both holonomic and differential controller

## [0.5.6] - 2022-07-19
### Added
- unit tests for nodes and controllers

## [0.5.5] - 2022-06-29
### Added
- doc strings for python files and comments for omnigraph nodes.

## [0.5.4] - 2022-06-01
### Changed
- OgnHolonomicController to use BaseResetNode class from core_nodes

## [0.5.3] - 2022-05-31
### Changed
- OgnDifferentialController to use BaseResetNode class from core_nodes

## [0.5.2] - 2022-05-11
### Changed
- Add Isaac sim category to holonomic robot setup node.

## [0.5.1] - 2022-05-11
### Changed
- holonomic_controller.py allows for flexible wheels rotation axis and stage-up axis

## [0.5.0] - 2022-05-11
### Changed
- holonomic_controller.py becomes robot agnostic

### Added
- OgnHolonomicController Node
- holonomic_robot_usd_setup.py to pull robot attributes from Usd
- OgnHolonomicRobotUsdSetup Node

## [0.4.1] - 2022-05-10
### Fixed
- Minor fixes to OgnDifferentialController

## [0.4.0] - 2022-05-06
### Removed
- OgnGenericDifferentialRobotSetup node

### Changed
- OgnDifferentialController no longer uses bundle inputs

## [0.3.0] - 2022-05-03
### Added
- OgnGenericDifferentialRobotSetup
- wheeled_robot.py

### Changed
- OgnDifferentialController uses bundle inputs

### Fixed
- omnigraph dependency

## [0.2.0] - 2022-04-27
### Added
- OgnDifferentialController

## [0.1.2] - 2022-04-22
### Changed
- Using osqp to solve holonomic controller
- Remove unecessary dependencies

## [0.1.1] - 2022-04-16
### Changed
- Fixed dependency versions

## [0.1.0] - 2022-04-08
### Added
- Initial version
