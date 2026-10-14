# Changelog

## [0.2.1] - 2026-10-01
### Fixed
- Support Newton 1.6 drive classes throughout the actuator API, examples, and documentation.

## [0.2.0] - 2026-09-29
### Added
- `ArticulationActuators.from_actuators`: optionally accept a list of DOF names per `ActuatorConfig` to build one grouped actuator.

### Changed
- `_discover_and_build_actuators` batches compatible USD-authored actuators across DOFs and robot instances using Newton's `ModelBuilder`.
- `from_actuators` stores actuators in the supplied group order instead of DOF order

## [0.1.6] - 2026-08-05
### Changed
- Use `joint_target_q` / `joint_target_qd` on Newton actuator control adapters

## [0.1.5] - 2026-07-21
### Changed
- Migrated robot asset references from `Isaac/Robots/` to `Isaac/Robots_Multiphysics/` for the new multiphysics-ready USDA assets.

## [0.1.4] - 2026-07-07
### Changed
- Publish only actuator domain APIs; keep the Kit lifecycle class import-compatible but out of the supported Python API.

## [0.1.3] - 2026-07-02
### Fixed
- `ArticulationActuators.from_actuators` now accepts device argument instead of using the wp default device.

## [0.1.2] - 2026-06-09
### Fixed
- Fix linter errors and missing or incomplete docstrings.

## [0.1.1] - 2026-04-30
### Added
- Lazy importing to improve startup time

## [0.1.0] - 2026-04-30
### Added
- Initial implementation
