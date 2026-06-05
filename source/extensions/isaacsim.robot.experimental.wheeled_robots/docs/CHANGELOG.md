# Changelog

## [0.2.9] - 2026-05-13
### Fixed
- Raise `ValueError` (instead of cryptic `TypeError`) when `HolonomicController` is constructed without `wheel_radius`, `wheel_positions`, `wheel_orientations`, or `mecanum_angles`.

## [0.2.8] - 2026-05-11
### Fixed
- Raise an error when `quintic_polynomials_planner()` cannot find a trajectory satisfying acceleration and jerk constraints.
- Reject non-finite `normalize_angle()` inputs instead of looping indefinitely.
- Raise a clear `ValueError` when `WheeledRobot` is constructed without wheel DOF names or indices.

## [0.2.7] - 2026-04-03
### Changed
- Remove `[::-1]` reversal in `HolonomicController` to match new `[roll, pitch, yaw]` euler angle convention

## [0.2.6] - 2026-03-26
### Fixed
- Fix euler angle convention in HolonomicController causing reversed motion direction
- Add missing public property accessors on HolonomicRobotUsdSetup (wheel_radius, wheel_positions, etc.)

### Changed
- Strengthen HolonomicController test to assert numerical parity with deprecated controller
- Add directional sanity test for pure-forward command

## [0.2.5] - 2026-03-23
### Added
- Add WheeledRobot integration test with kinematic distance assertion

## [0.2.4] - 2026-03-22
### Fixed
- Fix carb.log_error call passing multiple arguments instead of a single formatted string

## [0.2.3] - 2026-03-05
### Changed
- Linting: `__all__` re-exports, docstrings, and type/doc fixes for ruff, mypy, darglint, pydoclint.

## [0.2.2] - 2026-03-05
### Changed
- Fix api and docs syntax issues

## [0.2.1] - 2026-03-02
### Changed
- Removed deprecated libraries and integrated the latest replacements.

## [0.2.0] - 2026-03-01
### Removed
- Unused C++ plugin, bindings, and IWheeledRobots interface (extension is Python-only).
- OmniGraph nodes (DifferentialController, HolonomicController, AckermannController, etc.); use Python API only. For graph nodes use the stable `isaacsim.robot.wheeled_robots` extension.

## [0.1.0] - 2026-02-25
### Added
- Wheeled Robot for Warp based APIs.
