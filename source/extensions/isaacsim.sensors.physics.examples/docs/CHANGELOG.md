# Changelog

## [1.2.4] - 2026-06-09
### Fixed
- Fix linter errors and missing or incomplete docstrings.

## [1.2.3] - 2026-04-30
### Changed
- Migrated examples and tests to the new `isaacsim.sensors.experimental.physics` 3.0.0 API: call `IMU.create()`, `Contact.create()`, `Raycast.create()` (the authoring classes) and wrap the returned authoring object with the runtime sensor for data reads. The deleted `*Backend` classes and the runtime `XSensor.create()` class methods are no longer used; examples and tests use `IMUSensor`, `ContactSensor`, `RaycastSensor` directly. Updated all transform arguments to plural `translations`/`orientations` arrays matching the new API.
- Beam-curtain raycast example now derives the hit-count threshold from each sensor's configured `max_range` instead of a hardcoded `100.0`, so misses (which return `maxRange`) are no longer counted as hits for the 10 m beam curtain.

## [1.2.2] - 2026-04-28
### Fixed
- IMU example raised `AttributeError` when reading orientation; read scalar `orientation_w/x/y/z` fields individually instead of indexing.

## [1.2.1] - 2026-04-21
### Changed
- Replaced `omni.kit.commands` sensor creation with `ContactSensor.create()`, `IMUSensor.create()`, and `RaycastSensor.create()` class methods

## [1.2.0] - 2026-04-17
### Added
- Add example for creating solid state, rotating, and beam curtain raycast sensors

## [1.1.1] - 2026-04-09
### Removed
- Remove the `omni.isaac.ml_archive` dependency

## [1.1.0] - 2026-03-04
### Changed
- Added Overview.md and python_api.md and updated docstrings

## [1.0.1] - 2026-02-10
### Changed
- IMU and Contact sensor creation commands renamed to include Experimental in their name to avoid name collision with deprecated sensor commands

## [1.0.0] - 2026-02-01
### Added
- Updated to use interfaces from isaacsim.sensors.experimental.physics extension
- Updated contact and IMU examples to use the new sensor command APIs and legacy Python interfaces
- Improved example UI lifecycle handling with typed callbacks, stage-close cleanup, and richer docstrings

## [0.2.2] - 2025-12-05
### Changed
- Migrate to Events 2.0.

## [0.2.1] - 2025-10-27
### Changed
- Make omni.isaac.ml_archive an explicit test dependency

## [0.2.0] - 2025-10-17
### Changed
- Migrate PhysX subscription and simulation control interfaces to Omni Physics

## [0.1.15] - 2025-07-07
### Fixed
- Correctly enable omni.kit.loop-isaac in test dependency (fixes issue from 0.1.14)

## [0.1.14] - 2025-07-03
### Changed
- Make omni.kit.loop-isaac an explicit test dependency

## [0.1.13] - 2025-06-25
### Changed
- Add --reset-user to test args

## [0.1.12] - 2025-05-31
### Changed
- Use default nucleus server for all tests

## [0.1.11] - 2025-05-19
### Changed
- Update copyright and license to apache v2.0

## [0.1.10] - 2025-05-10
### Changed
- Enable FSD in test settings

## [0.1.9] - 2025-04-11
### Changed
- Update Isaac Sim robot asset path
- Update Isaac Sim robot asset path for the IsaacSim folder

## [0.1.8] - 2025-04-09
### Changed
- Update all test args to be consistent

## [0.1.7] - 2025-04-04
### Changed
- Version bump to fix extension publishing issues

## [0.1.6] - 2025-03-26
### Changed
- Cleanup and standardize extension.toml, update code formatting for all code

## [0.1.5] - 2025-03-11
### Changed
- Switch asset root for tests to internal nucleus

## [0.1.4] - 2025-01-27
### Changed
- Updated docs link

## [0.1.3] - 2025-01-26
### Changed
- Update test settings

## [0.1.2] - 2025-01-21
### Changed
- Update extension description and add extension specific test settings

## [0.1.1] - 2025-01-17
### Changed
- Temporarily changed docs link

## [0.1.0] - 2024-12-16
### Added
- Initial version of Isaac Sim Physics sensor extension examples
