# Changelog

## [Unreleased]

## [7.0.0a1] - 2026-09-16

### Added

- `isaacsim.core.experimental.prims` provides a Kit-independent deprecated compatibility layer in the
  `isaacsim-deprecated` distribution and requires generated Python bindings at import time. `Prim` aliases
  `isaacsim.foundation.objects`, while `XformPrim` and `GeomPrim` use Foundation implementations with legacy
  non-destructive construction defaults.

### Changed

- `isaacsim-deprecated` is a platform-independent pure-Python wheel that requires `warp-lang>=1.16.0`; its build system
  requires `scikit-build-core>=1.0.3`.
