# Changelog

## [0.1.2] - 2026-05-11
### Fixed
- Preserve default configuration and velocity safety limits when adding custom PINK IK limits.

## [0.1.1] - 2026-05-01
### Fixed
- Prevented OSQP sparse matrix conversion warnings from being emitted during PINK IK solves.

## [0.1.0] - 2026-03-25
### Added
- Initial PINK (Python Inverse Kinematics) integration with Isaac Sim motion generation API
- PinkIKController implementing BaseController for reactive differential IK
- Robot configuration loading from URDF via Pinocchio
- Transform utilities between Isaac Sim world frame and Pinocchio
- Support for FrameTask, PostureTask, and user-defined tasks, limits, and barriers
