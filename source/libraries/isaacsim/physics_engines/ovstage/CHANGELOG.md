# Changelog

## [Unreleased]

## [7.0.0a1] - 2026-09-16

### Added

- An idempotent `setup()` facility exposes OVStage's private Python and native runtimes without modifying the
  process-wide `pxr` package.
- `get_native_handle()` bridges APIs that consume an OVStage native address.
- `lookup_stage()` returns the Python Stage associated with a handle from `get_native_handle()`.
- `as_native_handle()` normalizes integer handles and nanobind `void*` capsules (`nb_handle`) used when C++ invokes
  Python `initialize` callbacks.

### Changed

- The shared `isaacsim-physics-engines` distribution includes the OvPhysX, Newton, and OVStage providers, depends on
  `isaacsim-physics`, and builds with `scikit-build-core>=1.0.3`.
