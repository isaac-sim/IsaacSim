# Changelog

## [2.4.0] - 2026-10-01
### Added
- Add an "Add to Current Stage" option, enabled by default, so Generate no longer discards stage content.
- Add "Create Ground Plane" and "Create Lighting" options for scenes that already have their own.
- Add an "Overwrite Existing" option, disabled by default, so Generate no longer replaces stage content.
- Add `configure_stage_metadata` and `overwrite_existing` arguments to `HeightmapImporter.create_heightmap()`.
- Add a `terrain_path` property to `HeightmapImporter` reporting where the last successful run built.

### Changed
- Warn before Generate replaces the stage or overwrites existing content.
- Generate now adds a terrain instead of replacing the one before it.
- Build terrain at `occupancyMap_NN` when `/World/occupancyMap` is already taken.
- Skip the ground plane and light, rather than duplicating them, when those paths are already taken.
- Keep the current stage's meters-per-unit, up-axis, and default prim when adding to it.
- Warn when the stage is not Z-up, because terrain is always generated in the XY plane.
- Count anything the importer cannot inspect as your content rather than as its own output.
- Refuse to overwrite a path the current layer cannot clear, such as a variant's or a muted layer's.
- Refuse to generate when `/World` hides the generated paths, naming the prim to activate, load or select.
- Step around a path a muted layer defines indirectly, through a sublayer, a reference, or a variant.
- Count a variant carrying a reference or payload above the generated paths as content that may appear there.

### Fixed
- Honor cell size as meters on stages that are not authored in meters.
- Resize the ground plane when regenerating terrain at a different cell size with "Overwrite Existing" enabled.
- Replace existing content at the ground plane and light paths instead of reusing it.
- Ignore Generate while a previous generation is still running.
- Cancel a generation still in flight when the extension shuts down.
- Remove terrain a failed run half-authored, rather than leaving it to be taken for your content.

## [2.3.5] - 2026-07-07
### Changed
- Use supported package-root imports for cross-extension APIs.

## [2.3.4] - 2026-06-09
### Fixed
- Fix linter errors and missing or incomplete docstrings.

## [2.3.3] - 2026-05-18
### Changed
- Size the heightmap ground plane from the generated occupancy map's world-space bounds (via `UsdGeom.BBoxCache`) instead of the raw image dimensions, and create it after the heightmap instances so it always fully covers the geometry.

### Added
- Added unit tests to capture edge cases.

## [2.3.2] - 2026-05-10
### Fixed
- Show the Heightmap Importer file picker after the Load Image button creates it, unblocking PNG selection from the UI (6083539)
- Raise `ValueError` for non-PIL heightmap inputs before modifying the stage, and propagate image processing failures instead of silently returning an empty heightmap (6132964)

## [2.3.1] - 2026-04-22
### Fixed
- Fix heightmap importer crashing on grayscale images (PIL modes `L`, `I`, `F`) that produce 2D numpy arrays
- Fix `SetDefaultPrim` failing when `/World` prim does not exist; now creates it automatically

## [2.3.0] - 2026-04-08
### Changed
- Improve Python API documentation (`config/python_api.md` and/or module docstrings).

## [2.2.1] - 2026-02-23
### Changed
- Add isaacsim.core.experimental.objects dependency, remove omni.physics.physx dependency for groundplane creation

## [2.2.0] - 2025-11-05
### Changed
- Renamed Block World Generator to Heightmap Importer
- Refactored to separate importer logic from extension UI
- Standardized terminology from "block world" to "heightmap" throughout codebase
- Updated all documentation, comments, and test names to use heightmap terminology

## [2.1.0] - 2025-10-30
### Changed
- Migrate extension implementation to core experimental API

## [2.0.7] - 2025-10-27
### Changed
- Make omni.isaac.ml_archive an explicit test dependency

## [2.0.6] - 2025-05-19
### Changed
- Update copyright and license to apache v2.0

## [2.0.5] - 2025-04-04
### Changed
- Version bump to fix extension publishing issues

## [2.0.4] - 2025-03-26
### Changed
- Cleanup and standardize extension.toml, update code formatting for all code

## [2.0.3] - 2025-01-21
### Changed
- Update extension description and add extension specific test settings

## [2.0.2] - 2024-10-27
### Added
- Test image

## [2.0.1] - 2024-10-24
### Changed
- Updated dependencies and imports after renaming

## [2.0.0] - 2024-10-02
### Added
- Extension renamed to isaacsim.asset.importer.heightmap

## [1.0.0] - 2024-03-11
### Added
- Initial version of Isaac Sim block_world
