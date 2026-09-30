# Changelog

## [Unreleased]

## [7.0.0a1] - 2026-09-16

### Added

- `isaacsim.core.experimental.objects` provides a Kit-independent deprecated compatibility layer for motion generation.
  It re-exports the required shape and mesh classes from `isaacsim.foundation.objects` in the `isaacsim-deprecated`
  distribution. The module requires generated Python bindings at import time.

### Changed

- `isaacsim-deprecated` is a platform-independent pure-Python wheel that requires `warp-lang>=1.16.0`; its build system
  requires `scikit-build-core>=1.0.3`.
