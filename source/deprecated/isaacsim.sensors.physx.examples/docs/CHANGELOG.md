# Changelog

## [2.4.3] - 2026-04-28
### Fixed
- `lidar_info.py` example crashed with `AttributeError` when calling `AddTranslateOp` on the lidar prim; wrap the prim in `UsdGeom.Xformable` before adding the op.

## [2.4.2] - 2026-04-20
### Deprecated
- Extension deprecated in favor of the Experimental extension `isaacsim.sensors.physics.examples`

## [2.4.1] - 2026-04-17
### Fixed
- Run `_plot_pattern` in a background thread to avoid blocking Kit's main event loop

## [2.4.0] - 2026-03-04
### Changed
- Added Overview.md and python_api.md and updated docstrings

## [2.3.4] - 2026-01-06
### Changed
- Migrate more events to Events 2.0.

## [2.3.3] - 2025-12-05
### Changed
- Migrate to Events 2.0.

## [2.3.2] - 2025-10-31
### Changed
- Fix invalid escape sequences

## [2.3.1] - 2025-10-27
### Changed
- Make omni.isaac.ml_archive an explicit test dependency

## [2.3.0] - 2025-10-17
### Changed
- Migrate PhysX subscription and simulation control interfaces to Omni Physics

## [2.2.15] - 2025-07-07
### Fixed
- Correctly enable omni.kit.loop-isaac in test dependency (fixes issue from 2.2.14)

## [2.2.14] - 2025-07-03
### Changed
- Make omni.kit.loop-isaac an explicit test dependency

## [2.2.13] - 2025-06-25
### Changed
- Add --reset-user to test args

## [2.2.12] - 2025-05-31
### Changed
- Use default nucleus server for all tests

## [2.2.11] - 2025-05-19
### Changed
- Update copyright and license to apache v2.0

## [2.2.10] - 2025-05-10
### Changed
- Enable FSD in test settings

## [2.2.9] - 2025-04-09
### Changed
- Update all test args to be consistent

## [2.2.8] - 2025-04-04
### Changed
- Version bump to fix extension publishing issues

## [2.2.7] - 2025-03-26
### Changed
- Cleanup and standardize extension.toml, update code formatting for all code

## [2.2.6] - 2025-03-24
### Changed
- Migrate to Events 2.0

## [2.2.5] - 2025-03-11
### Changed
- Switch asset root for tests to internal nucleus

## [2.2.4] - 2025-01-27
### Changed
- Updated docs link

## [2.2.3] - 2025-01-26
### Changed
- Update test settings

## [2.2.2] - 2025-01-21
### Changed
- Update extension description and add extension specific test settings

## [2.2.1] - 2025-01-17
### Changed
- Temporarily changed docs link

## [2.2.0] - 2024-12-16
### Added
- lightbeam_sensor example

## [2.1.1] - 2024-11-25
### Fixed
- Lidar examples pointers and shutting down properly

## [2.1.0] - 2024-10-29
### Changed
- Moved examples from menu to browser

## [2.0.1] - 2024-10-24
### Changed
- Updated dependencies and imports after renaming

## [2.0.0] - 2024-10-04
### Removed
- Ultrasonic sensor example.

## [1.0.1] - 2024-09-03
### Fixed
- Disables "Load Lidar" button in Lidar example while stage is clearing

## [1.0.0] - 2024-03-12
### Added
- Initial version of Isaac Sim Range sensor extension examples
