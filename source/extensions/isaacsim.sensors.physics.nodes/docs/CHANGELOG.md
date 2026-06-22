# Changelog

## [1.5.3] - 2026-06-09
### Fixed
- Fix linter errors and missing or incomplete docstrings.

## [1.5.2] - 2026-04-30
### Changed
- Migrated OGN test fixture (`AntConfig`) and OGN tests to the new `isaacsim.sensors.experimental.physics` 3.0.0 API: call `Contact.create()` / `IMU.create()` (the authoring classes) directly instead of the removed runtime `XSensor.create()` class methods, and use plural `translations` numpy arrays.

## [1.5.1] - 2026-04-21
### Changed
- Replaced `omni.kit.commands` sensor creation in OGN tests with `ContactSensor.create()` class method

## [1.5.0] - 2026-04-17
### Added
- Add Isaac Read Raycast Sensor OmniGraph node

## [1.4.1] - 2026-04-09
### Removed
- Remove the `omni.isaac.ml_archive` dependency

## [1.4.0] - 2026-03-20
### Changed
- Update OmniGraph nodes to use prim path based sensor API instead of integer sensor IDs

## [1.3.0] - 2026-03-05
### Added
- Add Isaac Read Joint State node that reads full joint state (names, positions, velocities, efforts, DOF types, stage units) from an articulation prim

## [1.2.0] - 2026-03-04
### Changed
- Added Overview.md and python_api.md and updated docstrings

## [1.1.0] - 2026-02-27
### Changed
- Migrate nodes to use C++ core experimental prims APIs

## [1.0.1] - 2026-02-10
### Changed
- IMU and Contact sensor creation commands renamed to include Experimental in their name to avoid name collision with deprecated sensor commands

## [1.0.0] - 2026-02-01
### Changed
- Moved nodes from isaacsim.sensors.physics extension to this extension
- Updated to use interfaces from isaacsim.sensors.experimental.physics extension
- Updated contact and IMU examples to use the new sensor command APIs and legacy Python interfaces
