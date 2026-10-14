# Changelog

## [0.4.0] - 2026-08-27
### Changed
- Deliver the controller implementation by staging selected packages from the `isaacsim-robot-motion` wheel in the Kit extension while retaining the existing tests and public import path.

## [0.3.2] - 2026-08-26
### Changed
- Routed controller warnings through `isaacsim.common.logging` to remove the direct Carbonite dependency.

## [0.3.1] - 2026-07-29
### Changed
- Corrected `HolonomicController` test wording: mecanum wheel geometry is read from prim attributes, not an applied API schema.

## [0.3.0] - 2026-07-27
### Added
- `HolonomicController`: single-robot holonomic (omni / mecanum) drive controller wrapping `ControllerHolonomicDrive`, with site-setpoint command, configurable wheel geometry, mecanum roller angles, speed clamping, and CUDA graph capture.

## [0.2.0] - 2026-07-24
### Added
- `AckermannController`: single-robot Ackermann steering controller wrapping `ControllerAckermann`, with site-setpoint and direct-command modes, configurable forward/rotation axes, per-axle radii and track widths, speed and angle clamping, CUDA graph capture, and stateful theta hold near zero speed.

## [0.1.0] - 2026-07-22
### Added
- `DifferentialDriveController`: single-robot differential drive controller wrapping `ControllerDifferentialDrive`, with configurable forward/rotation axes, speed clamping, and CUDA graph capture.
