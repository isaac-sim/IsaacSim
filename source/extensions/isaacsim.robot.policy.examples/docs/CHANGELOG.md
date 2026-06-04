# Changelog

## [5.2.11] - 2026-05-28
### Fixed
- Conditional use of asset until menagerie assets are tested with Isaac lab

## [5.2.10] - 2026-05-21
### Fixed
- Restore the prior physics sim device and fabric state in the cleanup paths of the interactive Quadruped, Go2, and Humanoid examples so the PhysX direct-GPU API flag is not left enabled, which previously caused `PxArticulationJointReducedCoordinate::setDriveTarget` errors when modifying USD in subsequent sessions

### Added
- Unit tests covering the snapshot/restore helpers in `isaacsim.robot.policy.examples.interactive.utils`, plus per-example roundtrip tests that verify the Quadruped, Go2, and Humanoid examples leave the physics sim device and fabric flag unchanged after cleanup

### Changed
- Extract the snapshot/restore physics-state logic shared by the interactive Quadruped, Go2, and Humanoid examples into `isaacsim.robot.policy.examples.interactive.utils`


## [5.2.9] - 2026-05-21
### Fixed
- Spot fall over issue in example by reverting the physics dt to 500hz

## [5.2.8] - 2026-05-20
### Fixed
- Select the asset's `Physics` variant before constructing the `Articulation` in `PolicyController.__init__`, so `UsdPhysics.ArticulationRootAPI` is authored on a descendant prim before `Articulation.fetch_articulation_root_api_prim_paths` resolves the root (prevents `Path.IsValidPathString(NoneType)` crashes)
- Apply `PhysxArticulationAPI` to the articulation root prim in `PolicyController._set_articulation_props` if it is missing, avoiding `Empty typeName` USD errors when the asset's Physics variant does not author the API
- Update default USD paths for Go2 and Spot policy controllers to the nested `Mujoco_Menagerie/<robot>/<robot>/<robot>.usda` layout

## [5.2.7] - 2026-05-20
### Fixed
- Missing _timeline error on policy reset

## [5.2.6] - 2026-05-05
### Added
- Standing test for the Go2 policy that holds a zero command and asserts the robot remains upright

### Fixed
- Select the USD `Physics` variant from `SimulationManager.get_active_physics_engine()` so the Newton-compatible variant is chosen when Newton is the active engine
- Log a warning when the requested USD `Physics` variant is not declared on the robot prim instead of silently selecting a non-existent variant
- Remove the `_set_physics_variant` override in `SpotFlatTerrainPolicy` so it inherits the engine-to-variant mapping from `PolicyController`

## [5.2.5] - 2026-04-23
### Changed
- Decreased test tolerance to pass with newton backend.

## [5.2.4] - 2026-04-23
### Fixed
- Added isaacsim.physics.newton.tensors as extension dep

## [5.2.3] - 2026-04-23
### Changed
- Defer torch import to avoid loading it at startup

## [5.2.2] - 2026-04-08
### Fixed
- Fix physics variant selection to match USD variant names case-insensitively, resolving H1 robot loading failure when variant set uses `Physx` instead of `physx`
- Register `isaacsim.robot.policy.examples.robots` as a public module in the extension manifest

## [5.2.1] - 2026-04-06
### Changed
- Set `reset_xform_op_properties` to True when instantiating the Articulation

## [5.2.0] - 2026-03-17
### Added
- Newton can be used as a physics backend

## [5.1.1] - 2026-03-04
### Changed
- Fix api errors

## [5.1.0] - 2026-03-04
### Changed
- Added Overview.md, python_api.md and updated docstrings

## [5.0.4] - 2026-02-04
### Changed
- Update physics rate for drawer opening test for CPU test

## [5.0.3] - 2025-12-08
### Changed
- Removed rendering manager test time dependency (moved to base sample)

## [5.0.2] - 2025-12-03
### Changed
- Remove TODOs.

## [5.0.1] - 2025-12-02
### Changed
- Removed unecessary dependencies
- Removed remaining experimental api references

## [5.0.0] - 2025-12-01
### Changed
- Changed the backend to experimental API using warp and torch
- Enabled GPU physics to inference policies
- Moved policy based interactive examples to the isaacsim.robot.policy.examples folder

## [4.3.0] - 2025-10-27
### Changed
- Replace import statements with the deprecation function when importing PyTorch
- Make omni.isaac.ml_archive an explicit test dependency

## [4.2.0] - 2025-10-17
### Changed
- Migrate PhysX subscription and simulation control interfaces to Omni Physics

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
- Optimized controller based standalone and ROS examples

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
- Added the ANYmal robot

## [1.0.2] - 2022-04-21
### Changed
- Decoupled sensor testing from A1 and Go1 unit test
- Fixed contact sensor bug in example and standalone

## [1.0.1] - 2022-04-20
### Changed
- Replaced find_nucleus_server() with get_assets_root_path()

## [1.0.0] - 2022-04-13
### Added
- Quadruped class, unitree class (support both a1, go1), unitree vision class (unitree class with stereo cameras), and unitree direct class (unitree class that subscribe to external controllers)
- Quadruped controllers
- Documentations and unit tests
- Quadruped standalone with ros 1 and ros 2 vio examples
