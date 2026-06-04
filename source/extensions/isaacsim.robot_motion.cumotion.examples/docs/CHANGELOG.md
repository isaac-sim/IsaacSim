# Changelog

## [0.4.0] - 2026-05-19
### Added
- Adds a best-effort per-class GUI warmup in ``async setUp`` (awaited on the Kit loop), prevents Windows hangs.

### Changed
- Refactored all five example sub-extensions (`graph_planner`, `trajectory_optimizer`, `rmp_flow`, `trajectory_generator`, `world_interface`) so each `Scenario` owns its scene loading, async lifecycle, and prim-wrapper teardown, while `UIBuilder` is a strict view that only builds widgets and forwards events.
- `Extension` removes no-op physics step subscriptions.

### Fixed
- `LOAD` no longer hangs when clicked while a previous load is still in flight: pending load tasks are now cancelled and replaced.
- `LOAD` is not cancelled in mid-flight on the graph-planner.

## [0.3.3] - 2026-05-12
### Changed
- Reduces the number of GPU copies needed, since certain warp arrays are only needed for `reset`.

## [0.3.2] - 2026-04-16
### Fixed
- Fixed test failure caused by a leaked `_load_scene_async` task: `UIBuilder` now tracks and cancels in-flight async loads on cleanup and re-load, preventing expired-prim errors when a new stage is created between tests.

## [0.3.1] - 2026-04-14
### Fixed
- `graph_planner` and `trajectory_optimizer` example tests no longer hang: removed `timeline.play()` / frame-wait / `timeline.stop()` from `_load_scene_async`, which could block the Kit update thread (e.g. on CUDA initialisation failures) and prevent `next_update_async()` from ever resolving.
- `test_cspace_button_passes_slider_joint_values` is now significantly faster in both examples: robot config (`load_cumotion_supported_robot`) and slider setup are performed before the nucleus USD load, so the test predicate resolves without waiting for the network round-trip. Task-space tests retain the full wait via a new `_load_until_articulation_ready` helper.
- Reduced `wait_until` poll interval from 250 ms to 50 ms, cutting idle delay after a predicate becomes true.

## [0.3.0] - 2026-03-18
### Added
- Unittests for button presses, slider values, stateful behavior of all GUI components.

### Changed
- All functions for loading assets are async.

### Fixed
- We no longer force physics or rendering dt, which threw an uncapture `RuntimeError` because timeline could be playing.

## [0.2.1] - 2026-03-18
### Changed
- TrajectoryOptimizer example functional on Windows.

## [0.2.0] - 2026-03-05
### Changed
- Added Overview.md, python_api.md and updated docstrings

## [0.1.1] - 2026-03-02
### Changed
- Uses improved QOL motion generation APIs

## [0.1.0] - 2026-02-22
### Added
- Initial release
