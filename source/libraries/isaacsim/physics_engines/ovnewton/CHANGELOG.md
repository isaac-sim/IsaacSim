# Changelog

## [Unreleased]

## [7.0.0a1] - 2026-09-16

### Added

- A Kit-independent Newton simulation backend attaches to a caller-owned OVStage through
  `ovnewton.attach_ovstage` and registers under the default simulation name `newton`.
- `register(simulation_name=...)` supports multiple named Newton simulations. `create_entity` and
  `create_simulation_view` resolve each simulation by exact name, and `unregister()` removes its tensor factories.
- Tensor views support `articulation`, `rigid-body`, and `rigid-contact` entities through both
  `TensorRegistry.create_entity` and `SimulationView.create_*_view`.
- Articulation view patterns use USD `ArticulationRootAPI` paths.
- Unsupported SDF and deformable entity types, along with patterns that match no entities, report no support.
- `isaacsim.physics_engines.ovnewton` requires generated Python bindings at import time.

### Changed

- The shared `isaacsim-physics-engines` distribution includes the OvPhysX, Newton, and OVStage providers, depends on
  `isaacsim-physics`, and builds with `scikit-build-core>=1.0.3`.
