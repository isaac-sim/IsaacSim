# Changelog

## [6.3.7] - 2026-06-09
### Fixed
- Fix linter errors and missing or incomplete docstrings, and update `python_api.md`.

## [6.3.6] - 2026-05-29
### Added
- `GetAllRobotComponents` helper (C++ and Python): returns every prim in a robot's subtree that applies any Isaac robot-schema API (`IsaacRobotAPI`, `IsaacLinkAPI`, `IsaacJointAPI`, `IsaacSiteAPI`, `IsaacReferencePointAPI`). Lets consumers enumerate all schema-tagged frames in one call rather than walking `robotLinks` and re-scanning descendants per consumer.

## [6.3.5] - 2026-05-27
### Fixed
- `GenerateRobotLinkTree`: rewrite the two `UsdPrimRange` loops to the conventional range-for shape (`for (const UsdPrim& prim : UsdPrimRange(root))`) instead of treating the range as an iterator. The previous form did not compile and made `utils.h` unincludable from C++ consumers.
- Add `GetRobotLinkParentMap` helper to `utils.h` for resolving `isaac:physics:robotJoints` body0/body1 relationships into a child-link to parent-link path map.

## [6.3.4] - 2026-05-21
### Fixed
- `GenerateRobotLinkTree` now grafts links and physics joints from nested sub-robots (for example a gripper variant attached at a wrist link) onto the parent's kinematic tree, so forward-kinematics propagation moves the entire assembly. Previously, only the sub-robot's bridging link was reachable from the parent tree, leaving sibling links such as gripper knuckles and fingers behind when the arm was teleported by the Robot Poser. Per sub-robot, prims carrying `IsaacRobotLinkAPI` / `IsaacRobotJointAPI` are preferred; bare `UsdPhysicsRigidBodyAPI` / `UsdPhysicsJoint` prims are picked up only when the sub-robot has no tagged equivalents, so unrelated rigid bodies inside a properly authored sub-robot are not added to the tree.

## [6.3.3] - 2026-05-18
### Fixed
- `robot_schema` now imports `pxr.Usd` during module initialization so `pxr.Usd` annotations resolve in standalone wheel environments.

## [6.3.2] - 2026-04-24
### Fixed
- `PopulateRobotSchemaFromArticulation` and `RecalculateRobotSchema` fall back to `robot_prim` as a synthetic articulation root and sweep `Usd.PrimRange` to apply `JointAPI`/`LinkAPI` when BFS yields no chain, ensuring `robotLinks`/`robotJoints` relationships are populated on vehicle and drone assets.

## [6.3.1] - 2026-04-22
### Added
- `get_allowed_tokens(attribute)` helper that reads `allowedTokens` directly from the bundled `RobotSchema.usda` so Python consumers can populate UI choice lists without duplicating the schema's token lists.

## [6.3.0] - 2026-04-22
### Deprecated
- Deprecated `rangeSensorSchema` plugin types: `RangeSensor`, `Lidar`, `Generic` — used only by the deprecated `isaacsim.sensors.physx` extension. Use `IsaacRaycastSensor` with `isaacsim.sensors.experimental.physics` or `isaacsim.sensors.experimental.rtx` instead.
- Deprecated `IsaacLightBeamSensor` from `isaacSensorSchema` — used only by the deprecated `isaacsim.sensors.physx` extension. Use `IsaacRaycastSensor` with `isaacsim.sensors.experimental.physics` instead.
- `omni.isaac.RangeSensorSchema` Python module now emits `DeprecationWarning` on import
- `IsaacLightBeamSensor` Python wrapper now emits `DeprecationWarning` on construction
- Reorganized C++ sensor tokens to separate active from deprecated entries

## [6.2.0] - 2026-04-20
### Added
- `RecalculateRobotSchema` gains `force_update` kwarg that rebuilds `robotLinks`/`robotJoints` and authors cross-layer `deletedItems`.
- New shared `_discover_articulation_graph` helper with a `root_link_override` input for seeding traversal from the user's chosen base link.
- Articulation-root scoring picks the highest-parentness rigid body when the articulation root is a grouping `Xform`.
- Sub-robot boundary handling registers each sub-robot as one collapsed entry at its first boundary joint.
- `SaveRobotSchemaToRobotLayer` flushes composed state onto the layer authoring `IsaacRobotAPI` only, with referenced-layer support via namespace translation.

## [6.1.0] - 2026-04-17
### Added
- Add `IsaacRaycastSensor` USD schema type, plugInfo entry, sensor tokens, and Python compatibility wrapper

### Changed
- `PopulateRobotSchemaFromArticulation` and `RecalculateRobotSchema` now traverse DFS by default, with a `traversal` kwarg (`"dfs"` | `"bfs"`).
- `PopulateRobotSchemaFromArticulation` only registers `RigidBodyAPI` prims as `robotLinks` and `JointAPI` prims as `robotJoints`; grouping Xforms skipped.
- Targets now written as prepend list ops via `_set_targets_as_prepend`, preserving additive composition with downstream layers.
- `_apply_api` is a no-op when the schema is already applied, preventing `apiSchemas` churn on recalc.

## [6.0.0] - 2026-04-15
### Changed
- Migrated `rangeSensorSchema` and `isaacSensorSchema` from pre-built C++ typed schema libraries (`omni-isaacsim-schema` packman package) to codeless USD schemas
- Removed `omni-isaacsim-schema` packman dependency and associated native library entries
- All C++ consumers now use generic `prim.GetAttribute(token)` and `prim.GetTypeName()` instead of typed schema classes
- Added `sensor_tokens.h` header providing token constants for all sensor schema attributes and type names
- Added Python compatibility wrappers (`omni.isaac.RangeSensorSchema`, `omni.isaac.IsaacSensorSchema`) so existing Python consumers work without changes
- Removed ultrasonic sensor schemas (UltrasonicArray, UltrasonicEmitter, UltrasonicFiringGroup, UltrasonicMaterialAPI) — no code references existed
- Added schema validation tests for both sensor schema and range sensor schema plugin registration

## [5.1.3] - 2026-04-02
### Changed
- Replace `carb.log_*` with Python `logging` module
- Replace `omni.usd.get_world_transform_matrix` with pure `pxr` `UsdGeom.Xformable.ComputeLocalToWorldTransform`
- Replace `CARB_LOG_WARN` with `fprintf(stderr, ...)` in C++ header
- Remove `omni.usd` from extension.toml dependencies

## [5.1.2] - 2026-03-19
### Removed
- Remove the `omni.isaac.ml_archive` dependency

## [5.1.1] - 2026-03-05
### Changed
- Simpify import statements

## [5.1.0] - 2026-03-04
### Changed
- Added Overview.md, python_api.md and updated docstrings

## [5.0.0] - 2026-02-23
### Added
- `KinematicChain` class for building and caching joint chains between arbitrary start/end prims on a robot
- Forward Kinematics computation (`compute_fk`) with per-joint intermediate transforms
- Fused FK + spatial Jacobian computation (`compute_fk_and_jacobian`) in a single pass
- `IKSolver` abstract interface and `IKSolverRegistry` for pluggable IK solver implementations
- Levenberg-Marquardt IK solver (`IKSolverLM`) registered as the default solver, with adaptive damping, null-space bias toward joint centers, step clamping, and joint-limit clipping
- `pose_error` function computing 6-DOF orientation + position error between two transforms
- Math module (`math.py`): `Transform` (SE(3) composition and inversion), `Joint` (screw-axis exponential map for revolute and prismatic joints), quaternion operations (`quat_mul`, `quat_conj`, `quat_rotate`, `axis_angle_to_quat`, `quat_to_matrix`), `skew`, and `adjoint`
- Teleport functionality on `KinematicChain`: `teleport` propagates FK through the kinematic tree and writes USD body transforms; `teleport_anchored` applies joints while keeping a specified link fixed via rigid correction
- Zero-configuration pose computation (`_compute_zero_config_poses`) using static joint local frames
- Joint chain path finding via LCA algorithm (`_collect_chain_joints`) returning ordered joints with forward/backward traversal direction
- Named pose query utilities: `GetAllNamedPoses`, `GetNamedPoseStartLink`, `GetNamedPoseEndLink`, `GetNamedPoseJoints`, `GetNamedPoseJointValues`, `GetNamedPoseJointFixed`, `GetNamedPoseValid`
- `CreateNamedPose` for creating `IsaacNamedPose` prims with relationships and attributes
- Loop detection in articulation traversal (`_discover_articulation_prims`, `PopulateRobotSchemaFromArticulation`, `RecalculateRobotSchema`) using visited-set guards to prevent infinite loops in cyclic joint graphs
- `DetectAndApplySites` and `AddSitesToRobotLinks` for automatic site detection on link child Xforms and registration in the robot schema
- `ApplySiteAPI` function; `ApplyReferencePointAPI` deprecated and redirected to `ApplySiteAPI`
- `ValidateRobotSchemaRelationships` returning valid/invalid link and joint lists
- `RecalculateRobotSchema` that preserves existing relationship order while appending newly discovered items and removing invalid ones
- `RebuildRelationshipAsPrepend` and `EnsurePrependListForRobotRelationships` for proper USD layering of robot relationships

### Changed
- `UpdateDeprecatedSchemas` migrates `ReferencePointAPI` to `SiteAPI` and deprecated DOF offset attributes to `DofOffsetOpOrder` token array
- `UpdateDeprecatedJointDofOrder` collects deprecated per-axis attributes, builds token array, and removes old attributes

## [4.0.6] - 2026-02-23
### Added
- Schema diagram generation script
- Extended test coverage for schema enums, internal helpers, and parsing utilities

## [4.0.5] - 2026-01-28
### Added
- Loop detector on parsing of joints
- Missing SiteAPI functions
- Verify if all found links and joints are added to the schema
- Detect and Create SiteAPIs when applying robot schema
- Update Robot Schema relationships with missing links and joints

### Changed
- Updated deprecated schema checker and fixer

## [4.0.4] - 2025-12-18
### Changed
- Update parsing of all robot joints and links

## [4.0.3] - 2025-12-16
### Changed
- Add source link in compute backwards bodies from joint

## [4.0.2] - 2025-12-07
### Changed
- Add missing docstrings

## [4.0.1] - 2025-11-26
### Changed
- Fixed parsing of robot tree to ignore bodies in joints that are not rigid bodies

## [4.0.0] - 2025-11-14
### Changed
- Updated Robot Schema definitions:
   - Removed Attributes for DofOrder
   - Created DofOrderOP list to be used with DofType tokens
- Updated Add RobotAPI util such that it automatically scans the robot prim for Links and joints and populates it in the traversal order

## [3.6.2] - 2025-11-07
### Changed
- Update to Kit 109 and Python 3.12

## [3.6.1] - 2025-10-30
### Changed
- Fixed issue in `__init__.py` with running with `coverage.py`

## [3.6.0] - 2025-07-10
### Added
- Added `robot_type`, `license`, `version`, `source`, and `changelog` to the Isaac Robot API.

## [3.5.1] - 2025-06-12
### Changed
- Get extension path using `os` module since carb tokens can not be resolved by Sphinx

## [3.5.0] - 2025-05-30
### Changed
- Added Isaac prefix to Robot Schema classes

## [3.4.2] - 2025-05-19
### Changed
- Update copyright and license to apache v2.0

## [3.4.1] - 2025-05-14
### Changed
- Fixed relationship names for robot links and joints.

## [3.4.0] - 2025-05-09
### Changed
- Fixed minor errors on Applying surface gripper API types

### Added
- Added Robot Parsing Utils to generate  Robot Kinematic tree based on the joints available on the schema and their parent-child relationships.

## [3.3.3] - 2025-05-05
### Changed
- Fix API docs.

## [3.3.2] - 2025-04-18
### Changed
- Updates to surface gripper schema

## [3.3.1] - 2025-04-09
### Changed
- Update Isaac Sim robot asset path for the IsaacSim folder

## [3.3.0] - 2025-04-08
### Changed
- Updated Robot Schema to include Surface Gripper.

### Fixed
- Fixed build for robot schema include file

## [3.2.3] - 2025-04-04
### Changed
- Version bump to fix extension publishing issues

## [3.2.2] - 2025-03-26
### Changed
- Cleanup and standardize extension.toml, update code formatting for all code

## [3.2.1] - 2025-03-26
### Changed
- Updates to work with Kit 107.2 and ABI=1

## [3.2.0] - 2025-03-06
### Added
- Added support for isaac:namespace attribute

## [3.1.4] - 2025-03-04
### Changed
- Update to kit 107.1 and fix build issues

## [3.1.3] - 2025-01-28
### Fixed
- Windows signing issue

## [3.1.2] - 2025-01-21
### Changed
- Update extension description and add extension specific test settings

## [3.1.1] - 2024-11-12
### Changed
- Minor update to Robot Schema

## [3.1.0] - 2024-10-31
### Added
- First Version of Robot schema as a codeless implementation
- A few utilities in a c header and python utils fashion to deal with the codeless schema

## [3.0.1] - 2024-10-24
### Changed
- Updated dependencies and imports after renaming

## [3.0.0] - 2024-10-18
### Changed
- Extension renamed to isaacsim.robot.schema.

## [2.1.0] - 2024-06-28
### Added
- Added lightbeam sensor to Isaac Sensor Schema, bumping version

## [2.0.2] - 2024-04-19
### Changed
- Removed omni.usd.schema.physics dependency

## [2.0.1] - 2023-06-21
### Changed
- Direct inheritance for IsA schema IsaacContactSensor, IsaacImuSensor, Lidar, UltrasonicArray, Generic

## [2.0.0] - 2023-06-12
### Changed
- Updated to usd 22.11
- Update to kit 105.1
- Using omni-isaacsim-schema as dependency

## [1.1.0] - 2023-01-21
### Added
- Named Override Attribute

## [1.0.0] - 2022-09-29
### Removed
- Remove DR and ROS bridge schemas

## [0.2.0] - 2022-03-21
### Added
- Add Isaac Sensor USD Schema
- Add Contact Sensor USD Schema
- Add IMU Sensor USD Schema

## [0.1.1] - 2021-08-02
### Added
- Add USS material support

## [0.1.0] - 2021-06-03
### Added
- Initial version of Isaac Sim Usd Schema Extension
