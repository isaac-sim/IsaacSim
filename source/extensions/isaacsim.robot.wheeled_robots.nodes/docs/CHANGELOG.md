# Changelog

## [0.1.14] - 2026-09-29
### Fixed
- `DifferentialController`: warn only once per reset for zero-delta updates while retaining invalid-step skipping and negative-delta warnings.

## [0.1.13] - 2026-09-29
### Changed
- `OgnHolonomicController`'s `mecanumAngles` input is now correctly documented as degrees.

## [0.1.12] - 2026-08-28
### Fixed
- `QuinticPathPlanner`: extract signed yaw from `targetPrim` instead of the unsigned rotation angle, which mirrored negative or off-axis goal headings.

## [0.1.11] - 2026-08-17
### Fixed
- `CheckGoal2D`: compare wrapped yaw error with the target orientation.

## [0.1.10] - 2026-07-21
### Changed
- Migrated robot asset references from `Isaac/Robots/` to `Isaac/Robots_Multiphysics/` for the new multiphysics-ready USDA assets.

## [0.1.9] - 2026-07-20
### Changed
- Renamed C++ headers from `.h` to `.hpp`; update downstream include directives.

## [0.1.8] - 2026-07-07
### Changed
- Use supported package-root imports for cross-extension APIs.

## [0.1.7] - 2026-06-29
### Changed
- Hide the Kit lifecycle extension class from the generated public Python API.

## [0.1.6] - 2026-06-23
### Removed
- Removed the unused `omni.physx` dependency. The extension does not reference any `omni.physx` API directly.

## [0.1.5] - 2026-06-10
### Fixed
- Fix linter errors and missing or incomplete docstrings, and update `python_api.md`.

## [0.1.4] - 2026-06-09
### Changed
- Clarify Ackermann Controller `invertSteering` input description for rear-wheel-steered robots.

## [0.1.3] - 2026-06-02
### Fixed
- Clarified DifferentialController velocityCommand output order and units.

## [0.1.2] - 2026-05-11
### Fixed
- Guard navigation nodes against invalid quaternion inputs when computing yaw from OmniGraph orientations.

## [0.1.1] - 2026-04-13
### Removed
- Remove the `omni.isaac.ml_archive` dependency

## [0.1.0] - 2026-03-25
### Added
- Initial release: OmniGraph nodes for wheeled robot controllers
- DifferentialController (C++ node)
- AckermannSteering, AckermannController, CheckGoal2D, HolonomicController, HolonomicRobotUsdSetup, QuinticPathPlanner, StanleyControlPID (Python nodes)
- All Python OGN nodes to experimental APIs (controllers, utilities, USD setup)
