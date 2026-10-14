# Changelog

## [7.2.3] - 2026-10-01
### Fixed
- Defer policy sample construction until an example is selected so asset-server authentication cannot block application startup during browser registration.

## [7.2.2] - 2026-10-01
### Fixed
- Preserve authored MuJoCo joint-limit solver references when applying Newton defaults.

## [7.2.1] - 2026-09-29
### Changed
- Build one grouped Newton actuator per exported actuator group instead of one per joint.

## [7.2.0] - 2026-08-14
### Added
- Binary joint-position actions that map one policy channel to multiple joints.
- Add Franka drawer-policy deployment on Newton using an engine-specific exported policy and task configuration.

### Fixed
- Improve Franka drawer-policy deployment parity using exported task state and current physics poses.

### Security
- Require and verify trusted SHA-256 digests before loading TorchScript policy artifacts.

## [7.1.0] - 2026-08-14
### Added
- Add `DelayedDCMotor` actuator composition for exported Isaac Lab policies.
- Add exported observation-history deployment.
- Add a standalone Unitree H2 custom-policy deployment example.

### Fixed
- Support partial initial joint-position maps and policies that reference a subset of the robot's actuator groups.

## [7.0.2] - 2026-08-07
### Added
- Add Go2 PhysX/Newton regression coverage and atomic policy restart through `RobotPolicyRunner.restart_from_default_state()`.

### Changed
- Temporarily disabled Cartpole tests on the Newton backend

### Fixed
- Improve Go2 and H1 Newton deployment parity with robot-local physics defaults, preserved authored armatures, and clean policy reset and priming after timeline restarts.

## [7.0.0] - 2026-07-31
### Added
- `RobotPolicyRunner`: deploys any bundled robot through one spawn/initialize/step/close lifecycle, driven from the caller's physics callback.
- `PolicySpec` and `PolicyArtifact`, with a `get_<robot>_spec` factory per robot in `isaacsim.robot.policy.examples.bundled`.
- `derive_binding`/`bind_policy` for non-bundled policies.
- A standalone and interactive example per shipped robot, and migration shims at the 6.x import paths.

### Changed
- The exported IO descriptor is the policy-interface source; a deployment must carry `IO_descriptors.yaml` or an explicit binding hook.
- Terms bind by literal joint name, so the Newton H1 example now deploys `h1_minimal.usd`.

### Removed
- `PolicyController` and the six articulation-owning policy classes; use the matching bundled spec.
- `controllers.config_loader`, whose parsing helpers live in `env_config`.
- `OnnxRuntimeModel` and the generic ONNX/array utilities, absorbed by `OnnxPolicyModel`.

## [6.0.3] - 2026-07-28
### Fixed
- `PolicyController.initialize()`: scope all per-DOF configuration, including simulation effort and velocity limits, to the policy's own joints, fixing the shape-mismatch `ValueError` and misapplied limits when another robot shares the same articulation. No-op for single-robot articulations.

### Added
- `get_robot_joint_names_expr()` and `match_joint_names()` in `config_loader` to select a policy's own joints from a shared articulation.

## [6.0.2] - 2026-07-27
### Fixed
- Make ONNX Runtime and its CUDA library bootstrap available in `isaacsim-robot` pip and source installations.

## [6.0.1] - 2026-07-21
### Changed
- Migrated robot asset references from `Isaac/Robots/` to `Isaac/Robots_Multiphysics/` for the new multiphysics-ready USDA assets.

## [6.0.0] - 2026-07-16
### Added
- Add env-config-driven actuator deployment for PhysX and Newton through `isaacsim.core.experimental.actuators`.
- Support exported `ActuatorNetLSTM`, `ActuatorNetMLP`, `DCMotor`, `IdealPDActuator`, `DelayedPDActuator`, and `RemotizedPDActuator` configurations.
- Add hosted Newton policy and environment configuration files for ANYmal C.

### Changed
- Update the hosted PhysX and Newton policy and environment configuration files for Spot.
- Add physics-engine and simulation-device selection to the H1, ANYmal, and Spot standalone examples.
- Exclude Go2 unit tests from the configured PhysX and Newton extension test suites due to known issues.

### Fixed
- Use the same ANYmal C, Spot, and Go2 USD assets as the Isaac Lab policy training configurations.
- Preserve imported joint effort and velocity limits when an exported actuator config leaves `effort_limit_sim` or `velocity_limit_sim` unset.

### Removed
- Remove the legacy `LstmSeaNetwork` helper now that exported actuator configurations use `isaacsim.core.experimental.actuators`.

## [5.3.4] - 2026-07-13
### Changed
- Relax the Go2 forward-motion test tolerance to 0.45 m.

## [5.3.3] - 2026-07-11
### Fixed
- Lower the Go2 locomotion policy test initial spawn height temporarily

## [5.3.2] - 2026-07-07
### Changed
- Use supported package-root imports for cross-extension APIs.
- Update the generated Python API inventory.

## [5.3.1] - 2026-06-29
### Changed
- Stop publishing Kit lifecycle classes from interactive example modules.

## [5.3.0] - 2026-06-23
### Added
- `CartpolePolicy`: add Isaac Lab ONNX policy deployment from exported `policy.onnx` and `env.yaml` files.
- Add strict Cartpole transfer regressions for PhysX- and Newton-trained policies across PhysX and Newton, CPU and CUDA, and Linux and Windows.
- `PolicyController.load_policy`: support ONNX policies and exported env configuration for pre-import rigid-body, articulation, and joint-drive properties.

### Changed
- `PolicyController.initialize`: replace `set_articulation_props` with `reset_to_default_state`; articulation properties are now authored before articulation creation.
- `parse_env_config`: preserve tagged YAML values and support exact, regex, glob, and legacy prefix joint-name matching.
- Use `isaacsim.pip.onnx` for ONNX Runtime while retaining Torch policy support.

## [5.2.12] - 2026-06-09
### Fixed
- Fix linter errors and missing or incomplete docstrings, and update `python_api.md`.

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
