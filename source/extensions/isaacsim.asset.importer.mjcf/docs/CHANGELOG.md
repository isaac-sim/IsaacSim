# Changelog

## [3.10.0] - 2026-05-14
### Added
- After `run_asset_transformer_profile` finishes (and `run_multi_physics_conversion` is enabled), `MJCFImporter.import_mjcf()` now post-processes `<output_dir>/payloads/Physics/physx.usda` to combine over-constrained joints (multiple single-axis joints sharing the same body pair) into a single PhysX D6 joint
- The redundant single-axis joints are removed from the PhysX variant. All edits are confined to the PhysX overlay layer; the MuJoCo/Newton variants keep the original per-DOF joint authoring intact
- `MJCFImporterConfig.fix_base` is now a tri-state `bool | None`. `True` adds a world-to-root fixed joint (existing behavior), `False` removes any existing world-to-root fixed joint so the robot becomes floating-base, and the new default `None` leaves the source asset's base authoring untouched.

### Changed
- `MJCFImporter.import_mjcf()` has tighter checks around mjcf xml file names.
- Only adds `MassAPI` when the user sets a non-default density on the links; otherwise links with no mass will not have a `MassAPI` applied.

## [3.9.0] - 2026-05-07
### Changed
- Imported MJCF mimic joints are now expressed exclusively through `NewtonMimicAPI` (via `newton:mimicJoint`, `newton:mimicCoef0`, `newton:mimicCoef1`). The transformer-driven authoring of the equivalent `PhysxMimicJointAPI` has been removed; the runtime consumes the Newton mimic schema directly.
- Imported MJCF articulation roots no longer carry `PhysxArticulationAPI`. Self-collision is authored via `NewtonArticulationRootAPI` (`newton:selfCollisionEnabled`) on top of the standard `UsdPhysics.ArticulationRootAPI`.

## [3.8.0] - 2026-04-29
### Changed
- Updated mujoco-usd-converter to 0.2.0

## [3.7.0] - 2026-04-22
### Changed
- `MJCFImporter.import_mjcf()` now writes all intermediate artifacts (usdex layers, temp stage) to a system temp directory via `tempfile.mkdtemp()` in non-debug mode, instead of the source MJCF directory. This avoids `PermissionError` when importing MJCF assets from read-only locations (packman cache, mounted volumes, installed extension data). In `debug_mode`, intermediates are still written next to the final USD output for inspection.
- Intermediate-artifact cleanup is now wrapped in a `try/finally` so the scratch directory is removed even if an exception is raised mid-conversion.

### Fixed
- With `run_asset_transformer=False`, `import_mjcf()` no longer crashes trying to write to a non-existent output directory. The intermediate stage is now only materialized when the transformer needs it, and the final USD is written directly to the output directory (which is created via `os.makedirs(..., exist_ok=True)`).

## [3.6.0] - 2026-04-22
- Added configs to set default joint dynamics, density, and fix robot
- Added configs to run asset transformer, multi-physics conversion
- Added robot name into the debug folder path to avoid duplication

## [3.5.1] - 2026-04-21
### Changed
- Add deprecation notice to `commands_api.md` for `MJCFCreateImportConfig` and `MJCFCreateAsset` Kit commands

## [3.5.0] - 2026-04-14
### Changed
- Replace Kit extension manager lookup for default profile path with `isaacsim.asset.transformer.rules.DEFAULT_PROFILE_PATH`
- Remove unused `self._extension_path` from extension entrypoint
- Gate `extension.py` import in `__init__.py` so the package can be imported outside Kit

## [3.4.0] - 2026-04-08
### Changed
- Improve Python API documentation (`config/python_api.md` and/or module docstrings).

## [3.3.0] - 2026-03-31
### Changed
- Added `omni.usd.schema.mujoco` dependency for mujoco schema support
- Added `omni.hydra.usdrt_delegate` test dependency
- Removed docstring tests

## [3.2.1] - 2026-03-21
### Fixed
- Fixed test teardown to stop the timeline and flush run-loop frames before the next stage is created, preventing a SIGSEGV crash in `UsdStage::~UsdStage`

## [3.2.0] - 2026-03-04
### Changed
- Added scene import option for the MJCF importer
- MJCF converter version bump to 0.1.0a8

## [3.1.0] - 2026-02-25
### Changed
- Added mjc / newton to physx attribute conversion for multi-physics engine asset support

## [3.0.2] - 2026-02-25
### Changed
- Changed dep from pip prebundle to isaacsim.pip.newton

## [3.0.1] - 2026-02-25
### Changed
- Switched to lazy import for mjcf-usd-converter to fix docs build issue

## [3.0.0] - 2026-01-01
### Changed
- USD exchange backend
- New import format
- New UI design and interface

## [2.5.15] - 2025-12-07
### Changed
- Update description

## [2.5.14] - 2025-11-07
### Changed
- Update to Kit 109 and Python 3.12

## [2.5.13] - 2025-09-26
### Changed
- Update license headers

## [2.5.12] - 2025-09-24
### Changed
- Update Asset converter dependency

## [2.5.11] - 2025-08-27
### Fixed
- Fix the broken documentation links

## [2.5.10] - 2025-08-02
### Changed
- Remove Direct dependency from Omniverse Asset Converter, and use omni.kit.asset_converter

## [2.5.9] - 2025-07-31
### Changed
- Updated compiling dependency for Omniverse Asset Converter

## [2.5.8] - 2025-07-18
### Fixed
- Fixed importing assets where multiple meshes are defined per body
- Fixed placement of bodies when meshes are reused at different transforms
- Fixed material assignment on meshes when material is defined on Mjcf file

## [2.5.7] - 2025-07-17
### Fixed
- Fixed saving texture files (moving to imported folder) for materials on import.
- Deleting partial files created when converting meshes

## [2.5.6] - 2025-07-15
### Fixed
- Fixed application of Collision APIs in the collision scope and in the meshes directly as per OpenUSD Physics.
- Fix cyclic import of robot schema and leave it as sublayer on the base layer

## [2.5.5] - 2025-07-07
### Changed
- Add unit test for pybind11 module docstrings

## [2.5.4] - 2025-07-07
### Fixed
- Correctly enable omni.kit.loop-isaac in test dependency (fixes issue from 2.5.3)

## [2.5.3] - 2025-07-03
### Changed
- Make omni.kit.loop-isaac an explicit test dependency

## [2.5.2] - 2025-06-25
### Changed
- Add --reset-user to test args

## [2.5.1] - 2025-06-24
### Fixed
- Importing bodies with multiple mesh geom

## [2.5.0] - 2025-06-10
### Changed
- Removed unused assets from data folder
- Added missing license file

## [2.4.8] - 2025-05-31
### Changed
- Use default nucleus server for all tests

## [2.4.7] - 2025-05-23
### Changed
- Refactor headers into isaacsim.core.includes
- Add docstrings

## [2.4.6] - 2025-05-19
### Changed
- Update copyright and license to apache v2.0

## [2.4.5] - 2025-05-16
### Changed
- Make extension target a specific kit version

## [2.4.4] - 2025-05-15
### Changed
- Updated MakeRelativePath to use std::string

## [2.4.3] - 2025-05-15
### Changed
- Restructure codebase to align with isaac sim guidelines

## [2.4.2] - 2025-05-10
### Changed
- Enable FSD in test settings

## [2.4.1] - 2025-05-09
### Added
- Extension specific test arguments

## [2.4.0] - 2025-04-09
### Added
- Support for Robot Schema prototype

## [2.3.7] - 2025-03-10
### Changed
- Reverted MeshMergeCollision for now

## [2.3.6] - 2025-02-19
### Fixed
- Relative path for sublayers

## [2.3.5] - 2025-02-13
### Added
- Support for equality constraints

### Fixed
- Fixed bug where multiple tendons to the same joint pair were overriding each other.
- Fixed appropriate conversion of tendon properties to USD.

## [2.3.4] - 2025-01-28
### Fixed
- Windows signing issue

## [2.3.3] - 2025-01-01
### Changed
- Colliders create a MeshMergeCollision at the collisions prim level on robot assembly.

## [2.3.2] - 2024-12-13
### Fixed
- In-place ensuring that meshes, visuals and colliders scopes are invisible.
- Add collision group for colliders out of default prim

## [2.3.1] - 2024-12-09
### Fixed
- Crash when selecting the default density.

## [2.3.0] - 2024-12-09
### Changed
- Ensured Colliders and Visuals do a second stack of referencing so the visual and collider can use the same mesh.

### Fixed
- Collider instancing was picking wrong element when imageable was not a mesh.

## [2.2.3] - 2024-11-22
### Changed
- Only create instanceable prims.
- Use Omniverse ASset Converter.
- Change auto-limit default for joints to true

### Fixed
- Ensure tags are imported even when there are multiple elements.
- Allow import to continue even when inertia is not defined. (issues a warning)

## [2.2.2] - 2024-11-20
### Changed
- Update omni.client import

## [2.2.1] - 2024-10-25
### Changed
- Remove test imports from runtime

## [2.2.0] - 2024-10-13
### Changed
- Update Assets transforms into Translate/orient/scale elements

## [2.1.0] - 2024-10-13
### Changed
- Change the import options

### Removed
- Standalone window import

## [2.0.0] - 2024-09-20
### Changed
- Changed MJCF importer to export USD to base, physics, sensor, and main stage
- Changed joint importer to import joints to the joint scope, and categorized by joint type

## [1.2.2] - 2024-09-17
### Added
- Action registry for menu item

## [1.2.1] - 2024-09-06
### Changed
- Register the mjcf file icon to asset types.

## [1.2.0] - 2024-08-22
### Changed
- ISIM-1670: Move Import workflow to File->Import.
- ISIM-1673: Display mjcf options in the right panel of the Import Dialog Window.

## [1.1.1] - 2024-07-10
### Changed
- Importer frames on imported asset when done through GUI.

## [1.1.0] - 2023-10-03
### Changed
- Structural and packaging changes

## [1.0.1] - 2023-07-07
### Added
- Support for `autolimits` compiler setting for joints

## [1.0.0] - 2023-06-13
### Changed
- Renamed the extension to omni.importer.mjcf
- Published the extension to the default registry

## [0.5.0] - 2023-05-09
### Added
- Support for ball and free joint
- Support for `<freejoint>` tag
- Support for plane geom type
- Support for intrinsic Euler sequences

### Changed
- Default value for fix_base is now false
- Root bodies no longer have their translation automatically set to the origin
- Visualize collision geom option now sets collision geom's visibility to invisible
- Change prim hierarchy to support multiple world body level prims

### Fixed
- Fix support for full inertia matrix
- Fix collision geom for ellipsoid prim
- Fix zaxis orientation parsing
- Fix 2D texture by enabling UVW projection

## [0.4.1] - 2023-05-02
### Added
- High level code overview in README.md

## [0.4.0] - 2023-03-27
### Added
- Support for sites and spatial tendons
- Support for specifying mesh root directory

## [0.3.1] - 2023-01-06
### Fixed
- onclick_fn warning when creating UI

## [0.3.0] - 2022-10-13
### Added
- Added material and texture support

## [0.2.3] - 2022-09-07
### Fixed
- Fixes for kit 103.5

## [0.2.2] - 2022-07-21
### Added
- Add armature to joints

## [0.2.1] - 2022-07-21
### Fixed
- Display Bookmarks when selecting files

## [0.2.0] - 2022-06-30
### Added
- Add instanceable option to importer

## [0.1.3] - 2022-05-17
### Added
- Add joint values API

## [0.1.2] - 2022-05-10
### Changed
- Collision filtering now uses filteredPairsAPI instead of collision groups
- Adding tendons no longer has limitations on the number of joints per tendon and the order of the joints

## [0.1.1] - 2022-04-14
### Added
- Joint name annotation USD attribute for preserving joint names

### Fixed
- Correctly parse distance scaling from UI

## [0.1.0] - 2022-02-07
### Added
- Initial version of MJCF importer extension
