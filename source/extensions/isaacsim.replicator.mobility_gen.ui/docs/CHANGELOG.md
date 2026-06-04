# Changelog

## [0.4.4] - 2026-05-13
### Changed
- Clarified the `_cache_stage` docstring around what `export_as_stage` actually inlines.

## [0.4.3] - 2026-04-23
### Removed
- Remove the `omni.isaac.ml_archive` test dependency

## [0.4.2] - 2026-04-22
### Fixed
- Call `save_sensor_overrides` on recording start to persist camera calibration changes made in the UI
- Use `export_as_stage` when caching the scene stage to prevent black images on replay

## [0.4.1] - 2026-04-18
### Changed
- Added return type annotations

## [0.4.0] - 2026-04-08
### Changed
- Migrate from deprecated `isaacsim.replicator.mobility_gen` to `isaacsim.replicator.experimental.mobility_gen`
- Replace legacy world-based simulation control with `SimulationManager` and `SimulationEvent.PHYSICS_POST_STEP` callback
- Replace `set_active_viewport_camera` with `ViewportManager.set_camera`
- Replace `objects.GroundPlane` (core.api) with `GroundPlane` from `isaacsim.core.experimental.objects`
- Use `save_stage` without deprecated `save_and_reload_in_place` argument

## [0.3.0] - 2026-03-04
### Changed
- Added Overview.md, python_api.md and updated docstrings

## [0.2.0] - 2026-02-06
### Added
- Support loading usdz asset

## [0.1.10] - 2025-10-27
### Changed
- Make omni.isaac.ml_archive an explicit test dependency

## [0.1.9] - 2025-09-30
### Fixed
- Add dialog message for incorrect occupancy map paths

## [0.1.8] - 2025-07-07
### Fixed
- Correctly enable omni.kit.loop-isaac in test dependency (fixes issue from 0.1.7)

## [0.1.7] - 2025-07-03
### Changed
- Make omni.kit.loop-isaac an explicit test dependency

## [0.1.6] - 2025-06-27
### Changed
- Fix recording naming for windows filesystems

## [0.1.5] - 2025-06-25
### Changed
- Add --reset-user to test args

## [0.1.4] - 2025-06-04
### Changed
- Added fix for saving stage by setting save_and_reload_in_place=False

## [0.1.3] - 2025-05-31
### Changed
- Use default nucleus server for all tests

## [0.1.2] - 2025-05-19
### Changed
- Update copyright and license to apache v2.0

## [0.1.1] - 2025-05-10
### Changed
- Enable FSD in test settings

## [0.1.0] - 2025-04-28
### Added
- Initial version of MobilityGen UI
