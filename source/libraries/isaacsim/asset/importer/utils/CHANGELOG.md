# Changelog

## [Unreleased]

### Fixed

- PhysX-to-MJC asset conversion no longer copies drive `targetPosition` to `mjc:ref`. `mjc:ref` is the joint zero point, not a
  target, and the copied degrees were read as radians, which left Newton unstable or outside joint limits (for example the Franka Panda).
- Mesh cleanup, normal and UV generation, and merging raise an error with the optimizer's diagnostic when an
  operation fails or USD Optimize is unavailable, rather than returning an unchanged asset as a successful result.
- The asset wheel pins USD Optimize to the version tested with its OpenUSD dependency, preventing independent
  optimizer upgrades from loading an incompatible OpenUSD runtime.

## [7.0.0a1] - 2026-09-16

### Added

- The independently built `isaacsim-asset` wheel includes `isaacsim.asset.importer.utils`.
- `stage_utils` provides explicit-stage helpers for querying and authoring time-code ranges.
- The asset importer provides PhysX-to-MuJoCo and PhysX-to-Newton conversion helpers.
- Standalone URDF conversion supports Newton and MuJoCo schemas.

### Changed

- The `isaacsim-asset` distribution is a platform-independent pure-Python wheel.
