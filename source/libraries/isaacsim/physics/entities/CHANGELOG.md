# Changelog

## [Unreleased]

## [7.0.0a1] - 2026-09-16

### Added

- C++ and Python wrappers for common physics entities: `PhysicsEntity`, `RigidBodyEntity`,
  and `ArticulationEntity`.
- C++ entity APIs provide concise read-only property accessors such as `numPrims()` and `dofNames()`;
  Python exposes matching idiomatic snake-case properties.

### Changed

- The `isaacsim-physics` distribution depends on `isaacsim-common` and supports source builds with
  `scikit-build-core>=1.0.3`.
