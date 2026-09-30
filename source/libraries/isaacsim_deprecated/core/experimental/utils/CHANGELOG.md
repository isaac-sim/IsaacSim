# Changelog

## [Unreleased]

## [7.0.0a1] - 2026-09-16

### Added

- `isaacsim.core.experimental.utils` provides a Kit-independent deprecated subset of the `stage`, `prim`, `ops`,
  `backend`, `bounds`, `transform`, and `foundation` USD utilities in the `isaacsim-deprecated` distribution.
- The utilities resolve the current stage through a process-level default stage ID and the USD stage cache. Lazy
  `usdrt` imports allow the pure-OpenUSD path to run without the Fabric runtime.

### Changed

- `isaacsim-deprecated` is a platform-independent pure-Python wheel that requires `warp-lang>=1.16.0`; its build system
  requires `scikit-build-core>=1.0.3`.
