# Changelog

## [1.8.8] - 2026-05-11
### Fixed
- Remove unused `unitsResolve` properties (other than `xformOp:scale:unitsResolve`) when resetting the transformation operation attributes of wrapped prims

## [1.8.7] - 2026-05-08
### Added
- Tests verifying that `Articulation` link, joint, and DOF names and indices are non-empty before physics initialization and match the post-physics enumeration for both fixed-base (Franka) and floating-base (Spot) assets.

## [1.8.6] - 2026-05-03
### Fixed
- Resolve descendant articulation targets to their containing articulation root.
- Populate `Articulation` metadata from USD when running with the `remotesim` physics backend.

## [1.8.5] - 2026-05-01
### Fixed
- Switch to the `remotesim` physics engine when other physics engines are still active, even if `remotesim` is already marked active.

## [1.8.4] - 2026-04-28
### Changed
- `Articulation.get_world_poses`, `get_velocities`: skip Warp row-axis fancy-indexing when `indices=None`; column reorder for quaternion xyzw→wxyz still uses the cached index array.
- `Articulation.get_dof_positions`, `get_dof_velocities`, `get_dof_position_targets`, `get_dof_velocity_targets`: skip Warp fancy-indexing when `indices=None` and `dof_indices=None` (guarded by `data.shape[1] == num_dofs` for heterogeneous safety).
- Quaternion reorder index array `[6, 3, 4, 5]` cached per device at module level to avoid repeated allocation on `get_world_poses` / `get_coms` / `set_world_poses` / `set_coms`.

## [1.8.3] - 2026-04-24
### Changed
- Use `SimulationManager.get_active_physics_engine()` for Newton engine detection in `RigidPrim` and `Articulation` instead of checking for `_newton_stage` attribute on the view object.

## [1.8.2] - 2026-04-24
### Fixed
- Removed the obsolete SimState remote-push gate so articulation DOF commands continue writing to SimStateStorage when SimState mode is enabled.

## [1.8.1] - 2026-04-22
### Added
- Add `Articulation._deferred_switch_remotesim()` static method that switches to the remotesim engine after PhysX prim query completes
- Call `_deferred_switch_remotesim()` in `_on_physics_ready` and tear down the PhysX tensor view if the engine was switched

## [1.8.0] - 2026-04-20
### Added
- Add ContactPointData struct, contact event type constants (kContactEventFound/Lost/Persist), ContactEventData struct, ContactReportData struct to IPrimDataReader.h
- Add enableContactReporting() and getContactReport() virtual methods to IPrimDataReader interface
- Add SdfPathToken.h with sdfPathToToken/tokenToSdfPath helpers for PhysX contact body identifiers
- Bump CARB_PLUGIN_INTERFACE version to (2, 2)

## [1.7.6] - 2026-04-17
### Fixed
- Fix link mass inverse test to not add epsilon to denominator

## [1.7.5] - 2026-04-16
### Added
- Add remote simulator support for articulation DOF target writes

## [1.7.4] - 2026-04-13
### Fixed
- Fixed `XformPrim.__init__` raising `TypeError` for non-root articulation links (`raise carb.log_warn(...)` replaced with `carb.log_warn(...)`)

## [1.7.3] - 2026-04-06
### Changed
- Improve exception message related to xformOp reset status

## [1.7.2] - 2026-03-27
### Changed
- Added tests for PhysX tensor-backed world transform path in IPrimDataReader (rigid body positions, quaternion reordering, mixed physics/Fabric prims, tensor vs Fabric consistency)

## [1.7.1] - 2026-03-26
### Changed
- Moved Python binding module to `bindings/` subdirectory

## [1.7.0] - 2026-03-20
### Changed
- Decouple prim data reader interface from implementation; implementation moved to `isaacsim.core.experimental.primdata`
- Add lazy provider loading to enable the primdata extension on-the-fly
- Add configurable provider extension setting

## [1.6.0] - 2026-03-17
### Added
- Added a filterless contact API that captures all contacts for prims

## [1.5.0] - 2026-03-12
### Changed
- Added Overview.md, python_api.md and updated docstrings

## [1.4.0] - 2026-03-09
### Added
- `LinkInfo` struct in `IPrimDataReader.h`: per-link descriptor (path, parentPath) returned by articulation traversal
- `getArticulationLinks()` on `IArticulationDataView`: enumerates `UsdPhysicsRigidBodyAPI` descendants of an articulation root, returning parent-child link relationships
- `getPrimFrameName()` on `IXformDataView`: resolves a prim's frame name, checking `isaac:nameOverride` before falling back to prim name; inherited by all three view types
- `getPrimWorldTransform()` on `IXformDataView`: computes world transform of an arbitrary prim via Fabric, returning position (float[3]) and orientation (float[4], wxyz); inherited by all three view types

## [1.3.1] - 2026-03-05
### Changed
- Fix api and docs syntax issues

## [1.3.0] - 2026-03-03
### Added
- `getDofNames()` and `getDofTypes()` on `IArticulationDataView` for articulation DOF metadata (names and types in articulation order)
- `setArticulationDofMetadata()` on `IPrimDataReader` for Newton backend to supply DOF names and types from Python

## [1.2.1] - 2026-03-02
### Changed
- Enable a set of Newton unit tests that are implemented and pass

## [1.2.0] - 2026-02-27
### Changed
- Add C++ interface to read xform, rigid body and articulation data

## [1.1.4] - 2026-02-27
### Fixed
- Fixed Warp 1.12 compatibility for deformable prims (wp.mat33 row-vector constructor replaced with wp.matrix_from_rows)

## [1.1.3] - 2026-02-12
### Added
- `XformPrim` now creates `FabricHierarchyLocalMatrix` and `FabricHierarchyWorldMatrix` if they don't exist.

## [1.1.2] - 2026-02-06
### Changed
- Update deprecated Warp API calls to their updated names

## [1.1.1] - 2026-02-05
### Changed
- Disable newton physics engine tests

## [1.1.0] - 2026-02-05
### Added
- Add Newton physics engine support for articulation and rigid body tests

## [1.0.1] - 2026-02-04
### Fixed
- Only return values for DOFs that have applied the `PhysxDrivePerformanceEnvelopeAPI` when querying drive model properties

## [1.0.0] - 2026-01-23
### Removed
- Remove deprecated PhysX residual reporting APIs (`enable_residual_reports`, `get_solver_residual_reports`) from `Articulation`

## [0.10.1] - 2026-01-23
### Changed
- Fixed docstring test errors

## [0.10.0] - 2026-01-16
### Added
- Add contact tracking functionality to RigidPrim: `set_enabled_contact_tracking()`, `get_enabled_contact_tracking()`, `get_net_contact_forces()`, `get_contact_force_matrix()`, `get_contact_force_data()`, `get_friction_data()`

## [0.9.8] - 2026-01-14
### Changed
- Slight modification to show how a new physics engine can be added in the future

## [0.9.7] - 2025-12-11
### Removed
- Remove checking for the deformable beta feature, as it is now active by default

## [0.9.6] - 2025-12-05
### Changed
- Migrate to Events 2.0

## [0.9.5] - 2025-12-01
### Fixed
- Fix physics setup when a prim instance is created while the simulation is running

## [0.9.4] - 2025-11-26
### Changed
- Update check condition on DOF to ensure it checks if it's a valid DOF before checking limits

## [0.9.3] - 2025-11-21
### Changed
- Update implementation to Warp 1.10.0
- Update array output in docstrings example due to changes in the NumPy representation

## [0.9.2] - 2025-10-27
### Changed
- Make isaacsim.storage.native an explicit test dependency

## [0.9.1] - 2025-10-22
### Changed
- Replace the use of deprecated core utils functions within implementations

## [0.9.0] - 2025-10-17
### Changed
- Migrate PhysX subscription and simulation control interfaces to Omni Physics

## [0.8.1] - 2025-09-24
### Fixed
- Fix deformable prim's docstrings test

## [0.8.0] - 2025-09-19
### Added
- Add rotation, stress and gradient computation for volume deformable bodies

## [0.7.0] - 2025-09-18
### Added
- Add support for input data expressed as basic Python types (bool, int, float)

## [0.6.2] - 2025-07-28
### Changed
- Trigger the synchronous generation of deformable simulation meshes to ensure data is always available

## [0.6.1] - 2025-07-23
### Fixed
- Fix deformable prim tests golden values

## [0.6.0] - 2025-07-16
### Added
- Add deformable prim for surface and volume deformable bodies

## [0.5.6] - 2025-07-07
### Fixed
- Correctly enable omni.kit.loop-isaac in test dependency (fixes issue from 0.5.5)

## [0.5.5] - 2025-07-03
### Changed
- Make omni.kit.loop-isaac an explicit test dependency

## [0.5.4] - 2025-06-25
### Changed
- Add --reset-user to test args

## [0.5.3] - 2025-06-18
### Changed
- Update docstrings for Warp version change

## [0.5.2] - 2025-06-12
### Fixed
- Fix broken autodocs references in api.rst

## [0.5.1] - 2025-06-07
### Changed
- Set test timeout to 900 seconds

## [0.5.0] - 2025-06-06
### Changed
- Update source code to use the experimental core utils API

## [0.4.3] - 2025-05-31
### Changed
- Use default nucleus server for all tests

## [0.4.2] - 2025-05-23
### Changed
- Rename test utils.py to common.py

## [0.4.1] - 2025-05-19
### Changed
- Update copyright and license to apache v2.0

## [0.4.0] - 2025-05-16
### Changed
- Rename properties related to the wrapped prim paths

## [0.3.2] - 2025-05-16
### Changed
- Make extension target a specific kit version

## [0.3.1] - 2025-05-16
### Fixed
- Fix rigid prim's angular velocity unit for USD backend

## [0.3.0] - 2025-05-12
### Changed
- Update implementation to use experimental material API (visual and physics materials)

## [0.2.0] - 2025-04-29
### Added
- Add support for articulation actuator modeling: advanced joint properties and drive envelope

## [0.1.2] - 2025-04-27
### Fixed
- Added extension specific unit test settings to fix broken tests

## [0.1.1] - 2025-04-21
### Changed
- Update Isaac Sim robot asset path
- Update docstrings for the Franka Panda robot

## [0.1.0] - 2025-03-21
### Added
- Initial release
