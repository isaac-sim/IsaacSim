# Changelog

## [2.3.1] - 2026-06-09
### Fixed
- Fix linter errors and missing or incomplete docstrings.

## [2.3.0] - 2026-04-30
### Changed
- Replace the MDL "Ghost" / "GhostVolumetric" preview material with a viewport selection group tinted green. `ConveyorBuilderWidget` no longer authors `/ConveyorBuilder/conveyorBuilder_Temp_mat` or binds a material to the preview prim; instead it owns a `ConveyorPreviewHighlight` that registers a selection group via `omni.usd.UsdContext.register_selection_group`, sets green outline + shade colours, and re-applies the group on every reference-target swap and Kit selection change. Mirrors the pattern used by `isaacsim.robot.poser.ui.utils.fk_helpers` and `isaacsim.robot_setup.collision_detector.widget`.

### Removed
- `data/Ghost.mdl` and `data/GhostVolumetric.mdl` (no longer referenced)
- `mdl_file` argument to `ConveyorBuilderWidget.__init__` and the matching `Extension.mdl_file` plumbing

## [2.2.1] - 2026-04-21
### Changed
- Replace `omni.kit.commands.execute("CreateConveyorBelt")` call sites with direct `create_conveyor_belt()` API

## [2.2.0] - 2026-04-08
### Changed
- Improve Python API documentation (`config/python_api.md` and/or module docstrings).

## [2.1.20] - 2026-03-06
### Fixed
- Fix shutdown() clearing wrong variable name (`_stage_event_subscription` instead of `_stage_event_sub_selection`), which leaked the selection subscription

## [2.1.19] - 2026-01-06
### Changed
- Migrate more events to Events 2.0.

## [2.1.18] - 2025-12-05
### Changed
- Migrate to Events 2.0.

## [2.1.17] - 2025-07-07
### Fixed
- Correctly enable omni.kit.loop-isaac in test dependency (fixes issue from 2.1.16)

## [2.1.16] - 2025-07-03
### Changed
- Make omni.kit.loop-isaac an explicit test dependency

## [2.1.15] - 2025-06-28
### Changed
- Increase delay in test to account for slower hardware

## [2.1.14] - 2025-06-25
### Changed
- Add --reset-user to test args

## [2.1.13] - 2025-05-31
### Changed
- Use default nucleus server for all tests

## [2.1.12] - 2025-05-19
### Changed
- Update copyright and license to apache v2.0

## [2.1.11] - 2025-05-16
### Changed
- Make extension target a specific kit version

## [2.1.10] - 2025-05-10
### Changed
- Enable FSD in test settings

## [2.1.9] - 2025-04-09
### Changed
- Update all test args to be consistent

## [2.1.8] - 2025-04-04
### Changed
- Version bump to fix extension publishing issues

## [2.1.7] - 2025-03-26
### Changed
- Cleanup and standardize extension.toml, update code formatting for all code

## [2.1.6] - 2025-03-11
### Changed
- Switch asset root for tests to internal nucleus

## [2.1.5] - 2025-03-04
### Changed
- Update to kit 107.1 and fix build issues

## [2.1.4] - 2025-01-26
### Changed
- Update test settings

## [2.1.3] - 2025-01-21
### Changed
- Update extension description and add extension specific test settings

## [2.1.2] - 2025-01-02
### Changed
- Performing any UI logic only once window is built

## [2.1.1] - 2024-12-11
### Changed
- Fix extension renaming

## [2.1.0] - 2024-11-01
### Changed
- Menu name

## [2.0.3] - 2024-10-28
### Changed
- Remove test imports from runtime

## [2.0.2] - 2024-10-24
### Changed
- Updated dependencies and imports after renaming

## [2.0.1] - 2024-10-11
### Changed
- Removed isaacsim core dependencies

## [2.0.0] - 2024-09-27
### Changed
- Renamed extension to isaacsim.asset.gen.conveyor.ui

## [1.1.0] - 2024-06-13
### Fixed
- Fixed an edge case when the first conveyor piece is added - to select the second connection point if it's a multi-connection (so it always goes forward)

### Changed
- Moved the assets source to a configurable setting
- Moved the config path to a configurable setting
- Moved the Conveyor Track Builder to UI.

## [1.0.1] - 2024-05-06
### Changed
- Updated asset path

## [1.0.0] - 2024-03-20
### Changed
- First Version
