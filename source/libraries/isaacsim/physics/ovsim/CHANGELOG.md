# Changelog

## [Unreleased]

## [7.0.0a1] - 2026-09-16

### Added

- Physics interfaces for OV SIM, including `timestamp`-based data callbacks and the exception accessors
  `getComponent()`, `getAttributeName()`, and `getValidAttributeNames()`.
- Simulation parameter requests reject unsupported names.

### Changed

- The `isaacsim-physics` distribution depends on `isaacsim-common` and supports source builds with
  `scikit-build-core>=1.0.3`.
