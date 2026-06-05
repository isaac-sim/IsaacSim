# Changelog
## [1.2.2] - 2026-05-22
### Fixed
- `create_conveyor_belt` now applies `UsdPhysics.MeshCollisionAPI` with the `convexHull` approximation when authoring the physics APIs on a `UsdGeomMesh` prim (i.e. when the target mesh and its ancestors do not already have `UsdPhysics.RigidBodyAPI`). Without it PhysX rejected the default `meshSimplification` approximation on the resulting dynamic body and emitted a per-parse error. The API is only authored for mesh prims; analytic shapes (Cube/Sphere/Cylinder/Capsule) keep their schema-correct collision representation.

## [1.2.1] - 2026-04-29
### Fixed
- Restore conveyor texture animation on stop: the StopPlay handler in `OgnIsaacConveyor::initInstance` now performs the texture-translate restore work synchronously from the event callback (subscribing to `omni::timeline::kGlobalEventStop`, mirroring `BaseResetNode`). The previous flow depended on `requestCompute` re-entering the node after the action-graph evaluator had already paused, which silently dropped the restore on stop. Regression introduced in the Kit 107.3 event-dispatcher migration.
- Defer clearing `m_onStart` until after `_collectShaderAttributes` returns so any future early-exit during collection does not leave the cache empty with the flag already cleared.
- Pair USD and USDRT shader-attribute handles in a single `ShaderAttributeBinding` struct so they cannot drift out of lockstep across partial-failure paths.
- Emit a one-shot `CARB_LOG_WARN` when `IStageReaderWriter` is unavailable so the FSD-mirror fallback to USD-only authoring is observable in logs.

- Gate the texture-animation block in `OgnIsaacConveyor::compute()` on a new `m_isPlaying` flag (set/cleared from the StartPlay/StopPlay callbacks). Prevents a "tail tick" fired by OnPlaybackTick immediately after StopPlay from re-authoring one delta of UV translation on top of the just-restored initial value, which manifested as a visible off-by-one-dt drift on every stop.
- Move the texture-translate baseline to a path-keyed `m_initialValuesByPath` map that is the single write-once source of truth. `_collectShaderAttributes` only inserts entries for paths it has never seen; `_restoreInitialTextureTranslations` reads from the map. The previous binding-vector-only preservation could be defeated when the bindings vector was rebuilt (for example on Play-after-Pause), causing Stop to restore to the value reached at the most recent Play instead of the original baseline.

### Added
- `TestConveyorTextureAnimation`: regression coverage for texture-translate advance during play, stop-time restore, post-stop tail-tick non-advance, Play/Pause/Play/Stop baseline preservation, zero-velocity short-circuit, `inputs:animateTexture=false` short-circuit, and FSD/Fabric mirror equality with USD.

## [1.2.0] - 2026-04-21
### Added
- Add `create_conveyor_belt` public Python API as a direct replacement for the Kit command

### Deprecated
- Deprecate `CreateConveyorBelt` Kit command in favor of `create_conveyor_belt()`

### Changed
- Clean up `__init__.py` exports to only expose public API

## [1.1.1] - 2026-04-17
### Fixed
- Remove debug `print(self._prim_path)` left in `CreateConveyorBelt.do()` production code

## [1.1.0] - 2026-04-08
### Changed
- Improve Python API documentation (`config/python_api.md` and/or module docstrings).

## [1.0.24] - 2025-11-07
### Changed
- Update to Kit 109 and Python 3.12

## [1.0.23] - 2025-08-28
### Changed
- Remove RTF wallclock check from 100 conveyor test

## [1.0.22] - 2025-07-07
### Changed
- Cleanup docstring for bindings

## [1.0.21] - 2025-07-07
### Fixed
- Correctly enable omni.kit.loop-isaac in test dependency (fixes issue from 1.0.20)

## [1.0.20] - 2025-07-03
### Changed
- Make omni.kit.loop-isaac an explicit test dependency

## [1.0.19] - 2025-06-28
### Changed
- Increase delay in test to account for slower hardware

## [1.0.18] - 2025-06-25
### Changed
- Add --reset-user to test args

## [1.0.17] - 2025-05-31
### Changed
- Use default nucleus server for all tests

## [1.0.16] - 2025-05-23
### Changed
- Fix docstring error

## [1.0.15] - 2025-05-19
### Changed
- Update copyright and license to apache v2.0

## [1.0.14] - 2025-05-16
### Changed
- Make extension target a specific kit version

## [1.0.13] - 2025-05-11
### Changed
- Enable FSD in test settings

## [1.0.12] - 2025-05-10
- Remove internal build time dependency

## [1.0.11] - 2025-05-07
### Changed
- Switch to omni.physics interface

## [1.0.10] - 2025-04-04
### Changed
- Version bump to fix extension publishing issues

## [1.0.9] - 2025-03-26
### Changed
- Cleanup and standardize extension.toml, update code formatting for all code

## [1.0.8] - 2025-03-05
### Changed
- Update extension codebase to adhere to isaac sim extension structure and file naming  guidelines

## [1.0.7] - 2025-03-04
### Changed
- Update to kit 107.1 and fix build issues

## [1.0.6] - 2025-02-17
### Changed
- Refactored code to isaac sim style guidelines

## [1.0.5] - 2025-01-28
### Fixed
- Windows signing issue

## [1.0.4] - 2025-01-21
### Changed
- Update extension description and add extension specific test settings

## [1.0.3] - 2024-12-11
### Changed
- Fix extension renaming

## [1.0.2] - 2024-10-28
### Changed
- Remove test imports from runtime

## [1.0.1] - 2024-10-24
### Changed
- Updated dependencies and imports after renaming

### Removed
- Removed unecessary dependencies

## [1.0.0] - 2024-09-27
### Changed
- Renamed extension to isaacsim.asset.gen.conveyor

## [0.4.0] - 2024-06-13
### Changed
- Updated Physics API to use SurfaceVelocity instead of rigid Body deprecated modified Velocities.
- Added Velocity Variable in omnigraph to control the conveyor speed

### Removed
- UI elements that didn't move over
- Forcing Kinematics Body on Conveyor Creation

# Changelog

## [0.3.14] - 2024-05-06
### Removed
- Unused config file

# Changelog

## [0.3.13] - 2023-11-01
### Fixed
- Error when Node starts for the first time if it doesn't have a Texture translate attribute.

## [0.3.12] - 2023-08-28
### Changed
- Added standard out fail pattern for the expected no prim found edge case for the ogn test

## [0.3.11] - 2023-08-10
### Changed
- Changed conveyor prim node input type from bundle to target

## [0.3.10] - 2023-06-13
### Changed
- Update to kit 105.1, build system update, omni.usd.utils -> omni.usd, usdPhysics -> pxr/usd/usdPhysics

## [0.3.9] - 2023-02-15
### Fixed
- UI regression where selected track card was not updated correctly

## [0.3.8] - 2023-02-14
### Changed
- Update asset path

## [0.3.7] - 2023-02-09
### Fixed
- Tests Use Fabric to get physx updates
- Error message when generating UI with image URL and destroing element before image finished loading
- Create Conveyor own hidden scope instead of using /Render

## [0.3.6] - 2023-01-25
### Fixed
- Remove un-needed cpp ogn files from extension

## [0.3.5] - 2023-01-06
### Fixed
- onclick_fn warning when creating UI

## [0.3.4] - 2022-12-02
### Fixed
- Adaptive scale for assets not in the same meters per unit than open stage

## [0.3.3] - 2022-12-01
### Fixed
- CreateConveyorBelt command documentation update

## [0.3.2] - 2022-12-01
### Fixed
- CreateConveyorBelt command .do() only returns the created prim and not a tuple

## [0.3.1] - 2022-11-29
### Fixed
- OG Node wouldn't update if the node direction changed
- Assets Reorganization

### Added
- Sorting Assets

## [0.3.0] - 2022-11-22
### Added
- Digital Twin Library Conveyor Authoring tool
- Support for curved conveyors

### Changed
- Default path for creating conveyor command is now at prim parent instead of rigid body prim.

## [0.2.1] - 2022-09-07
### Fixed
- Fixes for kit 103.5

## [0.2.0] - 2022-07-22
### Changed
- Convert node to cpp backend
- Conveyor node renamed to IsaacConveyor

### Added
- Simplified creating multiple conveyors, multiple prims can be selected on creation using menu

## [0.1.1] - 2022-05-10
### Changed
- Change Tests to use meters as distance unit

## [0.1.0] - 2022-03-30
### Changed
- First Version
