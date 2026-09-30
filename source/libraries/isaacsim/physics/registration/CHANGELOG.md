# Changelog

## [Unreleased]

## [7.0.0a1] - 2026-09-16

### Added

- Free-function APIs register physics simulation backends and tensor factories.
- Shared vocabulary types cover simulation, events, scene queries, interactions, benchmarks, and tensors, including
  `ImplementationKind`, `TensorDescription`, and `TensorSpecification`.
- Python `SimulationFns.initialize` callbacks receive the OVStage address as an `int`, matching
  `physics_manager.initialize`.
- `SimulationFns.publish_transforms_to_stage` lets a backend publish its current simulated transforms to its attached
  stage without advancing simulation.
- `get_active_simulation_id(name)` returns the active simulation registered under a name, matched
  exactly, and raises when more than one active simulation carries that name.
- `TensorRegistry` tracks engines for the lifetime of the process and separately tracks active named simulation
  factories. `list_engines()` reports available engines, while `list_simulations()` reports the names accepted by
  `create_entity` and `create_simulation_view`.
- `TensorDescription::keepAlive` retains tensor storage ownership when required.
- C++ registration introspection provides `getSimulationCount()`, and identifier types provide `computeHash()`.

### Changed

- The `isaacsim-physics` distribution depends on `isaacsim-common` and supports source builds with
  `scikit-build-core>=1.0.3`.
