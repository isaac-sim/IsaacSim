# Changelog

## [Unreleased]

## [7.0.0a1] - 2026-09-16

### Added

- `isaacsim.robot_motion.experimental.motion_generation` provides Kit-independent trajectory following, obstacle
  handling, and controller interfaces through the bundled OpenUSD runtime. The module requires generated Python
  bindings at import time.

### Changed

- The `isaacsim-robot-motion` distribution requires `warp-lang>=1.16.0` and builds with
  `scikit-build-core>=1.0.3`.
