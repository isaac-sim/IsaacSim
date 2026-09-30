# Changelog

## [Unreleased]

## [7.0.0a1] - 2026-09-16

### Added

- The shared `isaacsim-robot-motion` wheel includes `isaacsim.robot_motion.cumotion`, the cuMotion runtime, and Franka
  and UR10 configurations. The Python module requires generated bindings at import time.

### Changed

- The distribution requires `warp-lang>=1.16.0` and builds with `scikit-build-core>=1.0.3`.
