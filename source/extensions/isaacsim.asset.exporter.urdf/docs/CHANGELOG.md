# Changelog

## [2.2.10] - 2026-10-02
### Fixed
- Update texture collision regression tests to verify unique filenames and preserved payloads.
- Complete exporter test helper documentation.

## [2.2.9] - 2026-09-30
### Fixed
- Export UDIM textures with constant, uniform, and varying texture-coordinate primvars.
- Reject texture filename collisions that would silently reuse unrelated payloads.
- Prevent shared visual-collision primitives from applying composed scale twice.
- Keep invisible and guide collision geometry out of visuals unless explicitly requested.
- Resolve and deliver concrete UDIM texture tiles for OBJ and MTL exports.

### Security
- Reject remote texture filenames that decode to paths outside the export directory.

## [2.2.8] - 2026-09-25
### Fixed
- Author explicit fixture units and valid limits in exporter regression tests.

## [2.2.7] - 2026-09-19
### Changed
- Load `isaacsim.asset.exporter.urdf` from the independently built `isaacsim-asset` wheel.

## [2.2.6] - 2026-08-31
### Fixed
- Apply composed parent and robot scale to exported primitive geometry dimensions.
- Export X-axis and Y-axis cylinder and capsule orientations correctly.

## [2.2.5] - 2026-08-28
### Fixed
- Reconstruct products of inertia correctly from rotated USD principal axes.

## [2.2.4] - 2026-08-24
### Fixed
- Ignore unauthored or non-finite PhysX DriveAPI `maxForce` values when exporting URDF joint effort limits.

## [2.2.3] - 2026-08-20
### Changed
- Pin `usd-exchange` to version 2.3.0 for reproducible standalone installs.

## [2.2.2] - 2026-08-17
### Fixed
- Export native USD joint effort limits using PhysX DriveAPI `maxForce` when available, falling back to the `urdf:limit:effort` attribute otherwise.

## [2.2.1] - 2026-07-29
### Fixed
- Read joint velocity limits from canonical `newton:velocityLimit` metadata while preserving finite legacy `urdf:limit:velocity` values for backward compatibility.
- Re-apply the root prim's world **scale** (the component that `GetInverse()` removes) to the frame-relative transforms, so it propagates to all links
- Robustness: strip scale from stored link frames, so an inherited scale cannot cancel the re-applied root scale

## [2.2.0] - 2026-07-23
### Added
- Material export: read OmniPBR/MDL parameters (diffuse, roughness, metallic, emissive) from the `.mdl` source (resolved against its authoring layer) when not authored as USD shader inputs; USD inputs take precedence.
- Write roughness/metallic to `.mtl` (`Pr`/`Pm`) and bake a diffuse texture's average color into `Kd`, so material appearance round-trips on reimport.

### Fixed
- Rebase the physical root link's transform relative to the robot prim so ancestor Xform offsets no longer leak into exported joint origins.
- Skip geometry-bearing Xforms spuriously tagged `IsaacSiteAPI`
- `_fmt`: use significant-figure formatting so tiny valid inertias (~1e-10) are preserved instead of rounded to `0`.
- Classify visual/collision meshes by computed (inherited) purpose
- Export unbounded USD revolute joints (`±inf` limits) as URDF continuous joints.
- `export_duplicate_ghost_links`: rename site links that collide with their parent link name instead of always skipping them.

## [2.1.3] - 2026-07-21
### Added
- `UsdToUrdfConverter`: optionally export colliding site ghost links with numeric suffixes.

### Changed
- Migrated robot asset references from `Isaac/Robots/` to `Isaac/Robots_Multiphysics/` for the new multiphysics-ready USDA assets.

### Fixed
- Skip site ghost links whose resolved name matches an existing URDF link (e.g. multiphysics UR10e ``base_link/base_link``).

## [2.1.2] - 2026-07-15
### Fixed
- Explicitly set physics variant in unit test
- Joint friction is now imported to newton:friction rather than physxjoint:jointFriction

## [2.1.1] - 2026-07-15
### Added
- Export `NewtonMassAPI` `newton:inertia` tensor directly to URDF `<inertia>`, taking precedence over principal-axis reconstruction.

## [2.1.0] - 2026-07-02
### Added
- `UsdToUrdfConverter.convert`: add a callback for modifying the completed URDF XML before it is written.

## [2.0.5] - 2026-06-09
### Fixed
- Fix linter errors and missing or incomplete docstrings, and update `python_api.md`.

## [2.0.4] - 2026-05-27
### Fixed
- Fixed bug when nested rigid body are not discovered when the link holding the articulation root api is not a rigid body

## [2.0.3] - 2026-05-05
### Changed
- Preview image

## [2.0.2] - 2026-04-30
### Changed
- Use codeless physx schema

## [2.0.1] - 2026-04-28
### Fixed
- Preserve mesh scale when exporting instanceable geometry containers (e.g. `<link>/geometry` Xform with `instanceable=true` and a non-unit `xformOp:scale`, as in collected Isaac Sim assets like `franka.usd`); the scale is now emitted on the URDF `<mesh scale=...>` attribute instead of being silently dropped
- Replace `Usd.Prim.GetAncestorsRange` (C++-only API) with a `GetParent` walk in mesh prototype path resolution; the previous code raised `AttributeError` whenever a non-instanced mesh was exported

### Changed
- `matrix4_to_origin` now decomposes the matrix via `Gf.Transform` so RPY is computed from the unscaled rotation; `ExtractRotation` on a scaled matrix produced incorrect angles

## [2.0.0] - 2026-04-09
### Changed
- Rewrite as physics-graph-driven converter (UsdToUrdfConverter)

### Added
- Round-trip drive breadcrumbs: `isaac:source_drive` XML comments preserve DriveAPI gains (stiffness, damping, maxForce, targetPosition), MjcActuator parameters, and PhysxJointAPI armature across URDF export/import

### Fixed
- DriveAPI values no longer incorrectly populate URDF `<dynamics>`, `<limit effort>`, or `<calibration reference_position>` — these URDF elements now only contain passive joint properties per the URDF-to-USD concept mapping
- Split into API extension (this) and UI extension (isaacsim.asset.exporter.urdf.ui)
- Remove nvidia-srl-usd-to-urdf dependency
- Remove omni.physics.physx dependency from core API

## [1.5.0] - 2026-04-08
### Changed
- Improve Python API documentation (`config/python_api.md` and/or module docstrings).

## [1.4.5] - 2026-04-02
### Changed
- Update imports for compare usd function as it's moved

## [1.4.4] - 2026-03-25
### Changed
- Replace deprecated onclick_fn with onclick_action for menu registration

## [1.4.3] - 2026-02-24
### Changed
- Temporarily disable nova carter and go2 unit test for urdf converter fix

## [1.4.2] - 2026-02-23
### Changed
- Add explicit omni.physics.physx dependency

## [1.4.1] - 2025-12-14
### Changed
- Update lxml==6.0.2

## [1.4.0] - 2025-10-30
### Changed
- Migrate extension implementation to core experimental API

## [1.3.5] - 2025-10-27
### Changed
- Make omni.isaac.ml_archive an explicit test dependency

## [1.3.4] - 2025-10-09
### Changed
- Switch UI element to remove duplicate code

## [1.3.3] - 2025-10-07
### Changed
- Updated UI to handle local path for mesh

## [1.3.2] - 2025-08-30
### Changed
- Update nvidia-srl-usd-to-urdf==1.0.2
- Update nvidia-srl-usd==2.0.0

## [1.3.1] - 2025-08-28
### Fixed
- Fix test cleanup behavior

## [1.3.0] - 2025-07-21
### Added
- Add unit tests on robots `ur10e`, `2f-140-base`, `nova-carter`,  `tien-kung`, `go2`

## [1.2.5] - 2025-06-02
### Changed
- Update lxml==5.4.0

## [1.2.4] - 2025-05-30
### Changed
- Update nvidia-srl-usd-to-urdf==1.0.1

## [1.2.3] - 2025-05-22
### Changed
- Update copyright and license to apache v2.0

## [1.2.2] - 2025-05-20
### Changed
- Update urdf exporter pip prebundle

## [1.2.1] - 2025-04-04
### Changed
- Version bump to fix extension publishing issues

## [1.2.0] - 2025-04-02
### Changed
- Moved Urdf Exporter into File menu next to the other Exporter
- Options are integrated into the File Exporter dialog
- Removed option to export from USD file without opening it on Stage
- Usd to urdf library upgraded to python 3.11

## [1.1.5] - 2025-03-26
### Changed
- Cleanup and standardize extension.toml, update code formatting for all code

## [1.1.4] - 2025-03-04
### Changed
- Update to kit 107.1 and fix build issues

## [1.1.3] - 2025-01-21
### Changed
- Update extension description and add extension specific test settings

## [1.1.2] - 2025-01-14
### Added

- Missing dependency

## [1.1.1] - 2025-01-13
### Fixed

- Add missing `pathlib` to `exporter/urdf/ui_builder.py`

## [1.1.0] - 2024-10-29
### Fixed
- Fix incorrect URDF export when the USD prims have scaling values

### Changed
- Export .mtl files with the .obj files based on USD material prims
- Include inertia values from USD in URDF
- Use Isaac name override for frame name if it exists
- Add option to use ROS URI file prefix in the exported URDF

## [1.0.2] - 2024-10-28
### Changed
- Remove test imports from runtime

## [1.0.1] - 2024-10-24
### Changed
- Updated dependencies and imports after renaming

## [1.0.0] - 2024-09-27
### Fixed
- Extension renamed to isaacsim.asset.exporter.urdf

## [0.3.2] - 2024-08-28
### Fixed
- Added missing dependency

## [0.3.1] - 2024-07-23
### Fixed
- Removed unnecessary dependencies

## [0.3.0] - 2024-04-26
### Fixed
- Fix cylinder radii check
- Fix checking for duplicate prim names
- Fix issue with joint limits set to inf for revolute joints, set to continuous joint instead
- Fix cylinder scaling issue

### Added
- Replace absolute path and uri path options with mesh path prefix option
- Parse velocity and effort limits from USD and include in URDF
- Add UsdGeom.Cylinder to error message as one of the valid geom prim types
- Add hack to not include camera mesh files when exporting from isaac sim extension
- Add mesh file path char length check
- Add NodeType.SENSOR and functionality to include sensor frames in the URDF

## [0.2.0] - 2023-12-01
### Fixed
- Scaling bug when geometry prim is a child of Xform prim with scaling
- Cylinder scaling issue
- Joint frame alignment when child link transform is not identity
- Exporting revolute joints with inf limits, export as continuous joints now
- Too many root links
- Velocity and effort limits always being set to zero
- Large camera meshes added erroneously to URDF
- Terminal output labeled as error even when export is successful

### Added
- Exporting sensor prim frames to URDF
- Ability to set mesh path prefix
- Optionally setting output to directory (URDF file name automatically set to match USD file name)

## [0.1.1] - 2023-09-19
### Fixed
- Add missing dependencies and remove unused code

## [0.1.0] - 2023-07-27
### Added
- Initial version of Isaac Sim URDF exporter
