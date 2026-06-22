# Changelog

## [0.8.1] - 2026-06-09
### Fixed
- Fix linter errors and missing or incomplete docstrings.

## [0.8.0] - 2026-06-05
### Changed
- Upgraded Newton pip package from 1.2.0 to 1.2.1

## [0.7.14] - 2026-05-27
### Added
- Perform device check before running simulation in simulation_manager to prevent double initialization when device changes

## [0.7.13] - 2026-05-21
### Added
- Document Newton asset compatibility limitations and workarounds.
- Regression tests for Newton initialization error handling.

## [0.7.12] - 2026-05-19
### Added
- Unit test that tests imported urdf and mjcf asset behavior in both physx and newton

## [0.7.11] - 2026-05-18
### Changed
- Add return type annotations and mypy type:ignore comments across all source files to pass mypy and ruff linting

## [0.7.10] - 2026-05-12
### Changed
- Updated newton pip dependencies to newton 1.2.0rc3, mujoco-warp 3.8.0.2, newton-usd-schemas 0.2.0

## [0.7.9] - 2026-05-08
### Fixed
- Resolve pre-physics articulation link, joint, and DOF metadata by parsing the USD subtree with Newton's importer, mirroring the runtime articulation view's filtering of root and fixed joints, so `Articulation.dof_names` and the corresponding indices are available before physics is initialized and match the order produced once the simulation view is created. Results are cached per `(stage_id, prim_path)` and invalidated on USD stage close events.

## [0.7.8] - 2026-05-04
### Fixed
- Use the newton collision pipeline as default instead of mujoco

- MuJoCo solver: run `Model.collide` once per `simulate()` when `use_mujoco_contacts` is false (Newton collision); skip Newton collide when `use_mujoco_contacts` is true; call `SolverMuJoCo.update_contacts` after every `simulate()` so `contacts.force` and related fields match the solved MuJoCo state (required for tensor contact APIs, not only when MuJoCo runs collision)

## [0.7.7] - 2026-04-27
### Fixed
- Fix unbounded growth of `contact_callbacks` and `step_callbacks` lists when repeatedly subscribing and unsubscribing callbacks. Replace tombstone-based lists with dict-based storage so unsubscribed slots are reclaimed and iteration cost is O(active callbacks)

## [0.7.5] - 2026-04-23
### Fixed
- Try to load USD with flattened stage when composition cycles are present

## [0.7.4] - 2026-04-22
### Fixed
- Run the first simulation step without CUDA graph capture. Warp's link-time optimization compilation for `tile_matmul` / `tile_cholesky` can fail on the first attempt but succeeds on retry. When the first compilation happens inside `wp.capture_begin`, the failure is recorded into the CUDA graph, producing wrong simulation results on all subsequent steps. Skipping graph capture on the first step lets these transient failures resolve before any graph is recorded.

- Forward the user-provided `dt` into `simulate()` when recording the CUDA graph so step times match the requested timestep

- Route `set_dof_actuation_forces` and `get_dof_actuation_forces` through `control.joint_f` so applied joint efforts drive the simulation

## [0.7.3] - 2026-04-17
### Changed
- Update tensor frontend import paths for `omni.physics.tensors` package restructuring

## [0.7.2] - 2026-04-15
### Fixed
- Fix stale model references in all tensor view classes (`NewtonSimView`, `ArticulationSet`, `RigidBodySet`, `RigidContactSet`, and their frontend wrappers)
- Fix `shape_gap` default for static colliders from 0.1m to 0.0
- Add `cleanup_stale_newton_index` to `FabricManager` to remove leftover `newton:index` from prims no longer tracked as dynamic bodies
- Add bounds checking in `set_fabric_transforms` kernel to skip out-of-range body indices

## [0.7.1] - 2026-04-10
### Removed
- Remove the `isaacsim.core.utils` and `omni.isaac.ml_archive` dependencies

## [0.7.0] - 2026-04-01
### Changed
- get_simulation_time_steps_per_second returns int rather than float to comply with omni.physics.core bindings

## [0.6.2] - 2026-03-17
### Fixed
- Newton contact tensor API: apply force sign convention in net and matrix kernels (add for shape0, subtract for shape1) so net contact forces match PhysX expectations.
- Populate contact points from MuJoCo world-space positions when Newton Contacts do not provide rigid_contact_point0/1.

### Changed
- Contact force scaling in tensor view uses SimulationManager.get_physics_dt() instead of Newton stage sim_dt.

## [0.6.1] - 2026-03-17
### Added
- CTRL_DIRECT actuator PD control via tensor API: bridge `set_dof_stiffnesses`, `set_dof_dampings`, and `set_dof_position_targets` to MuJoCo actuator parameters (gainprm, biasprm, ctrl) and set biastype=AFFINE so robot policies (e.g. Go2) work with Newton when USD uses `mujoco:actuator` rather than `physics:driveAPI`.

## [0.6.0] - 2026-03-10
### Changed
- Upgraded Newton pip package from 1.0.0rc3 to 1.0.0.

## [0.5.3] - 2026-03-06
### Changed
- Upgraded Newton pip package from 1.0.0rc2 to 1.0.0rc3.

## [0.5.2] - 2026-03-05
### Changed
- Fix api and docs syntax issues

## [0.5.1] - 2026-03-04
### Changed
- Fix api errors

## [0.5.0] - 2026-03-04
### Changed
- Add Overview.md, python_api.md, SETTINGS.md and update docstrings
- Upgraded Newton pip package from 1.0.0rc1 to 1.0.0rc2.
- Enabled a set of Newton unit tests that are implemented and pass

## [0.4.1] - 2026-02-28
### Changed
- Updated SimulationFns registration to match new omni.physics.core API (`initialize`/`close` replaces removed `attach_stage`/`detach_stage`)

## [0.4.0] - 2026-02-27
### Changed
- Pinned mujoco-warp to 3.5.0.2 for compatibility with newton[sim] 1.0.0rc1

## [0.3.0] - 2026-02-27
### Removed
- Removed omni.warp extension dependency; only omni.warp.core is required

### Changed
- Upgraded Newton to 1.0.0rc1 with updated solver and USD schema handling

## [0.2.0] - 2026-02-24
### Changed
- Moved newton prebundle to isaacsim.pip.newton extension

## [0.1.2] - 2026-02-20
- Fixed broken newton usecases after upgrading the newton/mujoco packages

## [0.1.1] - 2026-02-06
### Added
- Capability check

### Removed
- Data handling from the physics umbrella

## [0.1.0] - 2025-12-23
### Added
- Initial release of isaacsim.physics.newton extension
- Integration with Newton physics engine
- Support for XPBD and MJC integrators
- Stage update node for physics simulation
