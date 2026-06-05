# Changelog

## [2.5.1] - 2026-04-20
### Deprecated
- Extension deprecated in favor of the Experimental extension `isaacsim.sensors.physics.ui`

## [2.5.0] - 2026-03-04
### Changed
- Added Overview.md and python_api.md and updated docstrings

## [2.4.3] - 2026-02-18
### Fixed
- Replaced deprecated `onclick_fn` with `onclick_action` in menu items to eliminate deprecation warnings
- Registered proper actions for PhysX Lidar and LightBeam sensor creation menu items

## [2.4.2] - 2026-01-24
### Changed
- Fix issues with menu click and context menu tests being flaky

## [2.4.1] - 2026-01-22
### Changed
- Move menu dictionary to initialize when tests are run rather than at module load time

## [2.4.0] - 2025-12-22
### Added
- Add unit tests.

## [2.3.1] - 2025-10-27
### Changed
- Make omni.isaac.ml_archive an explicit test dependency

## [2.3.0] - 2025-08-12
### Added
- Tashan TS-F-A sensor under "LightBeam" menu

## [2.2.12] - 2025-07-21
### Changed
- Fix error on shutdown

## [2.2.11] - 2025-07-07
### Fixed
- Correctly enable omni.kit.loop-isaac in test dependency (fixes issue from 2.2.10)

## [2.2.10] - 2025-07-03
### Changed
- Make omni.kit.loop-isaac an explicit test dependency

## [2.2.9] - 2025-06-25
### Changed
- Add --reset-user to test args

## [2.2.8] - 2025-05-31
### Changed
- Use default nucleus server for all tests

## [2.2.7] - 2025-05-19
### Changed
- Update copyright and license to apache v2.0

## [2.2.6] - 2025-05-10
### Changed
- Enable FSD in test settings

## [2.2.5] - 2025-04-09
### Changed
- Update all test args to be consistent

## [2.2.4] - 2025-04-04
### Changed
- Version bump to fix extension publishing issues

## [2.2.3] - 2025-03-26
### Changed
- Cleanup and standardize extension.toml, update code formatting for all code

## [2.2.2] - 2025-02-14
### Added
- Sensors to context menu

## [2.2.1] - 2025-01-21
### Changed
- Update extension description and add extension specific test settings

## [2.2.0] - 2024-12-10
### Added
- Lightbeam Sensor button moved to this extension

## [2.1.2] - 2024-12-04
### Changed
- Glyph for the Create Menu

## [2.1.1] - 2024-11-25
### Fixed
- A bug in shutdown code

## [2.1.0] - 2024-11-01
### Changed
- Menu name and location

## [2.0.1] - 2024-10-24
### Changed
- Updated dependencies and imports after renaming

## [2.0.0] - 2024-10-04
### Removed
- Ultrasonic sensor UI elements

## [1.0.1] - 2024-07-17
### Fixed
- Missing omni.kit.context_menu dependency

## [1.0.0] - 2024-03-12
### Added
- Initial version of Isaac Sim Range sensor extension examples
