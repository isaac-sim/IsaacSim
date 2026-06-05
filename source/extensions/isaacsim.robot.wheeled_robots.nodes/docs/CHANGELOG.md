# Changelog

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
