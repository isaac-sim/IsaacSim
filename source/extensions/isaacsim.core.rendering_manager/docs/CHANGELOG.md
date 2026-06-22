# Changelog

## [0.4.3] - 2026-06-09
### Fixed
- Fix linter errors and missing or incomplete docstrings, and update `python_api.md`.

## [0.4.2] - 2026-05-20
### Fixed
- `test_callback` no longer asserts an exact 1:1 mapping between `RenderingManager.render_async()` calls and `NEW_FRAME` dispatches.

## [0.4.1] - 2026-05-17
### Fixed
- `RenderingManager.set_dt()` now updates Fabric's default simulation period settings (`/app/settings/fabricDefaultSimPeriodNumerator` and `/app/settings/fabricDefaultSimPeriodDenominator`) along with USD/timeline timing. Kit reads these defaults when creating Fabric history stages, so newly-created graph/fabric caches use a render-product period that matches the requested rendering dt instead of Fabric's 1/30 s default.

## [0.4.0] - 2026-05-05
### Added
- Adds _ensure_fabric_simulation_time class method to seed /ExternalSimulationTime prim in Fabric for multitick renderer.

## [0.3.2] - 2026-03-26
### Changed
- Updated Python bindings import paths for consistency

## [0.3.1] - 2026-03-25
### Changed
- `ViewportManager.set_camera_view` now delegates look-at matrix computation to `look_at_matrix` from `isaacsim.core.experimental.utils.transform`

## [0.3.0] - 2026-03-04
### Changed
- Added Overview.md, python_api.md and updated docstrings

## [0.2.3] - 2025-12-20
### Changed
- Move import of kit loop runner to a try-except block to handle import exceptions if omni.kit.loop-isaac is not enabled

## [0.2.2] - 2025-11-28
### Fixed
- Fix camera view test case by comparing whether the quaternions represent the same orientation

## [0.2.1] - 2025-11-25
### Changed
- Handle Isaac Sim's loop runner import exceptions

## [0.2.0] - 2025-11-21
### Added
- Add method to set a camera view

### Changed
- Split the manager implementation into two: `RenderingManager` and `ViewportManager`

## [0.1.0] - 2025-11-11
### Added
- Initial release
