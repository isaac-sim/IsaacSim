# Changelog

## [0.11.6] - 2026-05-10
### Fixed
- Preserve unselected articulation default gains during partial stiffness/damping updates (6042276)
- Fix `SingleArticulation.get_world_velocity()` return description typo while keeping the documented `(6,)` shape (6038995)
- Remove unused timeline interface lookup from `ParticleSystem` callback initialization (6035373)

## [0.11.5] - 2026-05-05
### Changed
- Update the return types of the RigidPrim annotations

## [0.11.4] - 2026-04-22
### Fixed
- Fix `ParticleSystem` setter methods crashing when a prim is missing the required property; now logs an error and skips the prim

## [0.11.3] - 2026-04-19
### Fixed
- Fix `SingleArticulation.initialize` initializing `ArticulationController` before `ArticulationView` (controller receives uninitialized view)

## [0.11.2] - 2026-04-19
### Fixed
- Fix `RigidPrim.get_linear_velocities` UnboundLocalError when `clone=False` (used wrong variable name)
- Fix `RigidPrim.get_angular_velocities` ignoring `clone` parameter (cloned into unused variable)
- Fix `RigidPrim.get_masses` getter silently applying `UsdPhysics.MassAPI` as side effect
- Fix `RigidPrim.get_densities` getter silently applying `UsdPhysics.MassAPI` as side effect
- Fix `RigidPrim.get_sleep_thresholds` getter silently applying `PhysxSchema.PhysxRigidBodyAPI` as side effect
- Fix `RigidPrim.enable_gravities` USD fallback setting `DisableGravity=True` (inverted boolean)
- Fix `RigidPrim.disable_gravities` USD fallback setting `DisableGravity=False` (inverted boolean)
- Fix `RigidPrim.enable_rigid_body_physics` and related methods using `len()` on warp arrays (use `shape[0]`)
- Fix `RigidPrim.set_masses` calling `resolve_indices` twice (redundant second call in USD fallback)
- Fix `RigidPrim.set_coms` `IndexError` on 2D orientation input (use `wxyz2xyzw` instead of manual reorder)
- Fix `RigidPrim.set_default_state` calling `resolve_indices` 4 times (once per field); resolve once
- Fix `Articulation.get_friction_coefficients` skipping zero-valued joints (Python truthiness check)
- Fix `Articulation.get_armatures` skipping zero-valued joints (Python truthiness check)
- Fix `Articulation.switch_control_mode("velocity")` passing `_default_kps` instead of `_default_kds` for warp backend
- Fix `Articulation.set_gains` applying `UsdPhysics.DriveAPI` when both `kps=None` and `kds=None` (early return)
- Fix `Articulation.get_effort_modes` getter silently applying `UsdPhysics.DriveAPI` as side effect
- Fix `Articulation.get_max_efforts` getter silently applying `UsdPhysics.DriveAPI` as side effect
- Fix `Articulation.get_gains` getter silently applying `UsdPhysics.DriveAPI` as side effect in USD fallback
- Fix `Articulation.set_effort_modes` dead code (`if not drive.GetTypeAttr()` always False)
- Fix `Articulation.set_velocities` missing return after not-initialized guard
- Fix `Articulation.get_velocities` missing return after not-initialized guard
- Fix `Articulation.get_linear_velocities` missing return after not-initialized guard
- Fix `Articulation.set_angular_velocities` missing return after not-initialized guard
- Fix `Articulation.get_angular_velocities` missing return after not-initialized guard
- Fix `Articulation.get_joints_state` missing return after not-initialized guard
- Fix `Articulation.set_joint_efforts` zeroing unspecified joints when partial `joint_indices` passed
- Fix `Articulation.apply_action` zeroing joint efforts for unspecified joints when partial `joint_indices` passed
- Fix `Articulation.set_local_poses` raising `ValueError` when `orientations=None` with subset indices
- Fix `XFormPrim.set_local_poses` raising `pxr.ErrorException` (missing continue after error log)
- Fix `XFormPrim._backend2warp` duplicate `elif self._backend == "numpy"` branch (dead code)
- Fix `XFormPrim.get_applied_visual_materials` getter silently applying `MaterialBindingAPI` as side effect
- Fix `XFormPrim.set_world_poses(usd=False)` destroying non-uniform scale (extract scale before `SetRotateOnly`)
- Fix `XFormPrim.get_local_scales` returning NaN when prim has no `xformOp:scale`
- Fix `Prim._on_prim_deletion` falsely invalidating wildcard views (`.*` matches `/` in regex)
- Fix `GeometryPrim.__init__` redundant `to_list(collisions)` conversion inside per-prim loop
- Fix `GeometryPrim.__init__` `IndexError` when collisions array shorter than prim count
- Fix `GeometryPrim.get_contact_offsets` getter silently applying `PhysxCollisionAPI` as side effect
- Fix `GeometryPrim.get_rest_offsets` getter silently applying `PhysxCollisionAPI` as side effect
- Fix `GeometryPrim.get_torsional_patch_radii` getter silently applying `PhysxCollisionAPI` as side effect
- Fix `GeometryPrim.get_min_torsional_patch_radii` getter silently applying `PhysxCollisionAPI` and per-iteration tensor creation
- Fix `GeometryPrim.get_collision_approximations` getter silently applying `MeshCollisionAPI` as side effect
- Fix `GeometryPrim.get_applied_physics_materials` getter silently applying `MaterialBindingAPI` as side effect
- Fix `GeometryPrim` 4 contact-force method return annotations missing `| None`
- Fix `SdfShapePrim._apply_sdf_schema` `UnboundLocalError` when `MeshCollisionAPI` already applied
- Fix `SdfShapePrim.__init__` double-appending prims (parent `Prim.__init__` + subclass loop)
- Fix `ParticleSystem.__init__` silently discarding device setting (add warning on non-cpu fallback)
- Fix `SingleArticulation.dof_properties` `hasLimits` returning False for locked joints (lower==upper!=0)
- Fix `SingleArticulation.dof_properties` dead DC-API migration artifacts (4 commented-out lines)
- Fix `SingleArticulation` 3 methods with dead `is_physics_handle_valid() is None` guard

## [0.11.1] - 2026-04-17
### Fixed
- Add `| None` to 8 `Articulation` property return annotations that can return `None` when uninitialized
- Fix `Articulation.get_drive_types` docstring copy-pasted from `get_dof_limits`
- Fix `SingleArticulation.get_enabled_self_collisions` return annotation from `int` to `np.uint8`
- Fix `RigidPrim.get_current_dynamic_state` docstring example calling `get_default_state` instead of `get_current_dynamic_state`
- Fix `SingleRigidPrim.set_angular_velocity` warning message referencing "articulation state" instead of "rigid body state"

## [0.11.0] - 2026-04-17
### Changed
- Replace deprecated cloth and deformable prim implementations with error stubs since PhysX removed these features
- Update `ParticleSystem` for renamed PhysX attribute

## [0.10.0] - 2026-04-06
### Changed
- Allow to specify joint or DOF indices and set the last one as the default option when accessing data using the `joint_names` parameter

## [0.9.3] - 2026-04-03
### Fixed
- Fix XFormPrim default state not being re-converted when backend changes after initialization (e.g. automatic numpy-to-torch switch for GPU pipelines)

## [0.9.2] - 2026-03-22
### Fixed
- Fix carb.log_error calls passing multiple arguments instead of a single formatted string

## [0.9.1] - 2026-03-18
### Deprecated
- Extension deprecated in favor of the Core Experimental extension `isaaacsim.core.experimental.prims`

## [0.9.0] - 2026-03-12
### Changed
- Added Overview.md, python_api.md and updated docstrings

## [0.8.8] - 2026-03-05
### Changed
- Fix api and docs syntax issues

## [0.8.7] - 2026-01-20
### Removed
- Remove deprecated PhysX residual reporting APIs (`enable_residual_reports`, `get_position_residuals`, `get_velocity_residuals`) from `Articulation` and `SingleArticulation`.

## [0.8.6] - 2026-01-19
### Fixed
- Fixed missing argument `context` in `_reset_fabric_selection` method.

## [0.8.5] - 2026-01-08
### Fixed
- Fixed incorrect method call `set_wind` to `set_winds` in `ParticleSystem` constructor.
- Fixed incorrect attribute name `solverPositionIteration` to `solverPositionIterationCount` in `ParticleSystem.get_solver_position_iteration_counts`.

## [0.8.4] - 2026-01-06
### Changed
- Migrate more events to Events 2.0.

## [0.8.3] - 2025-12-05
### Changed
- Migrate to Events 2.0.

## [0.8.2] - 2025-12-03
### Changed
- Remove TODOs.

## [0.8.1] - 2025-12-01
### Fixed
- Fix the `PhysxCollisionAPI` schema when checking for collision properties

## [0.8.0] - 2025-10-27
### Changed
- Replace import statements with the deprecation function when importing PyTorch
- Make omni.isaac.ml_archive an explicit test dependency

## [0.7.0] - 2025-10-17
### Changed
- Migrate PhysX subscription and simulation control interfaces to Omni Physics

## [0.6.1] - 2025-09-16
### Fixed
- Fix ill-formed SdfPath when querying XFormPrim's applied visual materials

## [0.6.0] - 2025-08-27
### Changed
- Re-write XFormPrim's `get_world_poses`/`set_world_poses` fabric implemenattion using Fabric Scene Delegate and IFabricHierarchy

## [0.5.2] - 2025-08-13
### Fixed
- Auto-compute joint indices when specifying joint names to get articulation's measured joint reaction forces/torques

## [0.5.1] - 2025-07-21
### Changed
- Added explicit `destroy()` method to `Prim`

## [0.5.0] - 2025-07-18
### Changed
- Expose the `reset_xform_properties` argument in single prim classes

## [0.4.4] - 2025-07-07
### Fixed
- Correctly enable omni.kit.loop-isaac in test dependency (fixes issue from 0.4.3)

## [0.4.3] - 2025-07-03
### Changed
- Make omni.kit.loop-isaac an explicit test dependency

## [0.4.2] - 2025-07-01
### Fixed
- Added deprecation note for deformable prims

## [0.4.1] - 2025-06-25
### Changed
- Add --reset-user to test args

## [0.4.0] - 2025-06-09
### Changed
- Move the `_impl/single_prim_wrapper` script to the `impl` folder and update import statements

## [0.3.18] - 2025-05-31
### Changed
- Use default nucleus server for all tests

## [0.3.17] - 2025-05-19
### Changed
- Update copyright and license to apache v2.0

## [0.3.16] - 2025-05-16
### Fixed
- Fix rigid prim's angular velocity unit for USD implementation

## [0.3.15] - 2025-05-10
### Changed
- Enable FSD in test settings

## [0.3.14] - 2025-04-21
### Changed
- Updated franka robot example code

## [0.3.13] - 2025-04-14
### Changed
- Update Isaac Sim robot asset path

## [0.3.12] - 2025-04-09
### Changed
- Update all test args to be consistent

## [0.3.11] - 2025-04-08
### Changed
- Update the get_measured_joint_forces method's docstring to clarify which frame the forces and torques are reported in.

## [0.3.10] - 2025-04-04
### Changed
- Version bump to fix extension publishing issues

## [0.3.9] - 2025-03-26
### Changed
- Cleanup and standardize extension.toml, update code formatting for all code

## [0.3.8] - 2025-03-11
### Changed
- Switch asset root for tests to internal nucleus

## [0.3.7] - 2025-01-26
### Changed
- Update test settings

## [0.3.6] - 2025-01-21
### Changed
- Update extension description and add extension specific test settings

## [0.3.5] - 2024-12-30
### Fixed
- Fixed cleanup for Prim class if it was not initialized correctly

## [0.3.4] - 2024-12-18
### Fixed
- handles_initialized returning the wrong value for a single articulation when articulation controller not initialized

## [0.3.3] - 2024-12-12
### Fixed
- Updated the implementation of articulation inverse dynamic functions to respect the type of articulation (floating base vs fixed based)

## [0.3.2] - 2024-11-19
### Fixed
- XformPrim._set_xform_properties to take in consideration scale:unitsResolve attribute if authored.

## [0.3.1] - 2024-11-08
### Changed
- Changed Articulation.invalidate_physics_callback and RigidPrim.invalidate_physics_callback to weakref callbacks.

## [0.3.0] - 2024-11-05
### Added
- Added joint_names argument to all the joint related methods for ease of specifying which joints to manipulate/ query.

## [0.2.0] - 2024-10-31
### Changed
- Changed .initialize and .post_reset methods to be event triggered
- Changed physX warmup and simulation context creation to be event trigerred

## [0.1.1] - 2024-10-24
### Changed
- Updated dependencies and imports after renaming

## [0.1.0] - 2024-10-18
### Added
- Initial release
