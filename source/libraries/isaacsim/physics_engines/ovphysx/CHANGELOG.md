# Changelog

## [Unreleased]

### Changed

- Use the published `ovphysx` and `ovstage` Python packages as the wheel runtime, removing duplicate native libraries
  and plugins from `isaacsim-physics-engines`.

### Fixed

- Initialize OVStage before OvPhysX on Windows and shut it down after the final adapter is released to prevent
  Carbonite dependency-resolution errors during teardown.

## [7.0.0a1] - 2026-09-16

### Added

- A Kit-independent OvPhysX simulation and tensor backend registers automatically on import under the simulation name
  `ovphysx`. Applications can select it through `PhysicsManager.switch_physics_engine()`.
- The backend publishes standalone rigid-body and articulation-link transforms to its attached OVStage through the
  physics manager's backend-neutral callback.
- A private versioned transform journal accompanies authoritative OVStage transforms so compatible consumers can
  acquire complete physics pose batches without rediscovering changes across the full scene.
- Named tensor factories provide all seven entity-view types through `isaacsim.physics.manager.create_entity`.
- Contact and signed distance field (SDF) views derive their extent from caller-provided tensors on every read. Entity
  views begin with capacity for 1,000 contact records or one SDF query point and can be rebuilt to match a new extent.
- Entity patterns that match nothing report no support. `rigid-contact` and `sdf-shape` accept exactly one path pattern.
- Native `disable-gravities` tensor operations support rigid bodies and articulation links.
- Read-only `drive-types` values reflect the live engine state in tensor degree-of-freedom order, using `0` for none,
  `1` for force, and `2` for acceleration. Boolean `uint8` tensor values are normalized to `0` or `1`.
- The package includes the complete native Windows runtime required to activate the OvPhysX backend in standalone
  applications.

### Changed

- The shared `isaacsim-physics-engines` distribution includes the OvPhysX, Newton, and OVStage providers, depends on
  `isaacsim-physics`, and builds with `scikit-build-core>=1.0.3`.
