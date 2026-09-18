# Changelog

## [Unreleased]

## [7.0.0a1] - 2026-09-16

### Added

- A backend-neutral simulation manager provides direct lifecycle, callback, scene-query, interaction, and benchmarking
  operations.
- Entity and simulation tensor views support a Warp frontend.
- `publish_transforms_to_stage()` dispatches optional simulation-backend publication of current transforms to the
  attached stage.
- Entity views whose extent is chosen by the caller report shape hints that track the buffers they
  read into, so a subsequent read sizes its allocation from the current extent.
- The C++ API includes `PhysicsManager::configure()`, `PhysicsManager::getSimulatedPhysicsStepCount()`, and full-word
  `EntityView` implementation-registration methods; Python exposes idiomatic snake-case names.

### Changed

- The `isaacsim-physics` distribution depends on `isaacsim-common` and supports source builds with
  `scikit-build-core>=1.0.3`.
