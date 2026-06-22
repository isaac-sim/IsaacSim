# Changelog

## [0.2.1] - 2026-06-09
### Fixed
- Fix linter errors and missing or incomplete docstrings, and update `python_api.md`.

## [0.2.0] - 2026-05-27
### Fixed
- Improve UR10 bin filling motion stability by allowing the lift phase to settle before moving the bin under the filling spout.
- Updated bin filling example to use new motion generation/CuMotion APIs

## [0.1.3] - 2026-05-18
### Fixed
- Improve Robo Factory stacking test reliability by waiting for stacking completion and allowing more time for the Franka move and release phases.

## [0.1.2] - 2026-05-11
### Fixed
- Move default cube placement position to [0.4, 0.2, 0.7] for follow_target.py example so all IK methods can reach default pose.

## [0.1.1] - 2026-05-01
### Fixed
- Move default cube placement target position to [0.0, 0.5, 0.14] so the placed cube is visible and not blocked by the robot arm.

## [0.1.0] - 2026-04-06
### Added
- Migrated experimental manipulator examples from isaacsim.robot.manipulators.examples.
- FrankaExperimental, FrankaPickPlace, and Franka Stacking examples.
- UR10Experimental and UR10 FollowTarget examples.
- Interactive pick-place and follow-target UI extensions.
