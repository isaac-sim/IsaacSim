# Changelog

## [1.3.1] - 2026-06-09
### Fixed
- Fix linter errors and missing or incomplete docstrings, and update `python_api.md`.

## [1.3.0] - 2026-05-20
### Added
- `normalize_urdf_for_urdfdom(urdf_text)` helper (`urdf_normalize.py`): rewrites a URDF document in-memory so it satisfies urdfdom 4.0.1's strict parser without modifying any kinematically meaningful values. Inserts default `effort`/`velocity` attributes on `<limit>` elements that omit them, adds a wide-open `<limit>` to `revolute`/`prismatic` joints that have none, adds `k_velocity="0"` to `<safety_controller>` elements missing it, drops empty `<dynamics/>` elements, and removes `<mimic>` elements with no target joint. Idempotent on already-clean URDFs.

### Changed
- `load_cumotion_robot` now pre-processes the URDF through `normalize_urdf_for_urdfdom` and calls `cumotion.load_robot_from_memory` (rather than `cumotion.load_robot_from_file`). This makes the loader accept URDFs produced by `isaacsim.asset.exporter.urdf.UsdToUrdfConverter` and other generators that omit attributes urdfdom requires but the URDF spec / most other parsers do not. Disk content is never mutated.

## [1.2.0] - 2026-05-05
### Added
- `CumotionWorldInterface` now accepts a `device` constructor parameter (any value `wp.get_device` accepts; defaults to `None`, which resolves to warp's current default device). Selects the device used for internally-allocated warp arrays and dispatches the per-frame collider transform composition to either a vectorized NumPy path (CPU) or the Warp kernel (CUDA).
- `compute_collider_transforms_cpu` utility: vectorized NumPy mirror of the Warp collider-transform kernel.

### Changed
- `get_world_to_robot_base_transform` now memoizes its return value and yields the same `wp.array` tuple until the base pose is updated.

## [1.1.2] - 2026-04-23
### Added
- `RmpFlowController` contains `maximum_substep_size` for sub-stepping when more stable integration is required.

## [1.1.1] - 2026-03-16
### Fixed
- TrajectoryOptimizer now functional on Windows.

## [1.1.0] - 2026-03-05
### Changed
- Added Overview.md, python_api.md and updated docstrings

## [1.0.1] - 2026-03-04
### Fixed
- Fixed type annotation errors by adding `from __future__ import annotations` to files using union type syntax.

## [1.0.0] - 2026-03-03
### Changed
- `CumotionWorldInterface.add_oriented_bounding_boxes` now accepts quaternions (w, x, y, z) instead of rotation matrices for the `rotations` parameter.
- `CumotionWorldInterface.add_oriented_bounding_boxes` now accepts warp arrays directly for `centers`, `rotations`, and `half_side_lengths` instead of lists of warp arrays.

## [0.2.1] - 2026-02-27
### Changed
- Graph planner does not use CUDA on windows

### Fixed
- Patches missing dll path on windows for python wheel

### Removed
- TrajectoryOptimizer support on windows

## [0.2.0] - 2026-02-21
### Added
- Initial integration to experimental motion generation API.

## [0.1.0] - 2026-02-21
### Added
- Cumotion pip prebundle
