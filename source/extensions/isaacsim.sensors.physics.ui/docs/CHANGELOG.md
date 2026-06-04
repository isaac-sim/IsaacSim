# Changelog

## [1.2.2] - 2026-04-30
### Changed
- Migrated menu callbacks to the new `isaacsim.sensors.experimental.physics` 3.0.0 API: call `Contact.create()` / `IMU.create()` / `Raycast.create()` (the authoring classes) directly instead of the removed runtime `XSensor.create()` class methods, and use plural `translations`/`orientations` arrays. The IMU menu uses the returned authoring object's `set_visibilities([False])`; the runtime sensor no longer forwards XformPrim attribute access (aligns with `isaacsim.sensors.experimental.rtx`).

## [1.2.1] - 2026-04-21
### Changed
- Replaced `omni.kit.commands` sensor creation with `ContactSensor.create()`, `IMUSensor.create()`, and `RaycastSensor.create()` class methods

## [1.2.0] - 2026-04-17
### Added
- Add menu items for creating solid state, rotating, and beam curtain raycast sensors

## [1.1.1] - 2026-04-09
### Removed
- Remove the `omni.isaac.ml_archive` dependency

## [1.1.0] - 2026-03-04
### Changed
- Added Overview.md and python_api.md and updated docstrings

## [1.0.2] - 2026-02-18
### Fixed
- Replaced deprecated `onclick_fn` with `onclick_action` in menu items to eliminate deprecation warnings
- Registered proper actions for Contact Sensor and IMU Sensor creation menu items

## [1.0.1] - 2026-02-10
### Changed
- IMU and Contact sensor creation commands renamed to include Experimental in their name to avoid name collision with deprecated sensor commands

## [1.0.0] - 2026-02-01
### Added
- Updated to use interfaces from isaacsim.sensors.experimental.physics extension
- Updated menu actions to use new sensor creation commands and experimental prim helpers
- Improved context menu handling and visibility control for created sensor prims

## [0.2.1] - 2026-01-24
### Changed
- Fix issues with menu click and context menu tests being flaky

## [0.2.0] - 2025-12-22
### Added
- Add unit tests.

## [0.1.13] - 2025-10-27
### Changed
- Make omni.isaac.ml_archive an explicit test dependency

## [0.1.12] - 2025-07-07
### Fixed
- Correctly enable omni.kit.loop-isaac in test dependency (fixes issue from 0.1.11)

## [0.1.11] - 2025-07-03
### Changed
- Make omni.kit.loop-isaac an explicit test dependency

## [0.1.10] - 2025-06-25
### Changed
- Add --reset-user to test args

## [0.1.9] - 2025-05-31
### Changed
- Use default nucleus server for all tests

## [0.1.8] - 2025-05-19
### Changed
- Update copyright and license to apache v2.0

## [0.1.7] - 2025-05-10
### Changed
- Enable FSD in test settings

## [0.1.6] - 2025-04-09
### Changed
- Update all test args to be consistent

## [0.1.5] - 2025-04-04
### Changed
- Version bump to fix extension publishing issues

## [0.1.4] - 2025-03-26
### Changed
- Cleanup and standardize extension.toml, update code formatting for all code

## [0.1.3] - 2025-02-14
### Added
- Add Contact and IMU sensors to Viewport and Stage Context Menus

## [0.1.2] - 2025-01-21
### Changed
- Update extension description and add extension specific test settings

## [0.1.1] - 2024-12-12
### Fixed
- Restores missing methods to add IMU and Contact sensors to scene

## [0.1.0] - 2024-12-10
### Added
- Initial version of isaacsim.sensors.physics.ui
