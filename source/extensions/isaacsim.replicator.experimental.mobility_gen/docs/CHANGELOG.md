# Changelog

## [0.2.10] - 2026-06-12
### Fixed
- `occupancy_map`: defer the `cv2` import to first use.

## [0.2.9] - 2026-06-10
### Fixed
- Fix linter errors and missing or incomplete docstrings, and update `python_api.md`.

## [0.2.8] - 2026-06-09
### Added
- Teleop support for NuRec scenes with SPG (PPISP): the chase viewport renders through the scene's authored PPISP graph.

### Changed
- Depend on `isaacsim.replicator.nurec_utils`, deduplicating the local NuRec utilities.

## [0.2.7] - 2026-06-07
### Added
- `collect_input()` (exported): caches a scene into a recording — `.usdz` byte-copied whole; `.usd`/`.usda`/`.usdc` collected via `omni.kit.usd.collect`.
- `RecordingSession`: headless build-and-record control extracted from the UI, so recordings can be driven from a script.
- Volume NuRec detection (`OmniNuRecFieldAsset` / `omni:nurec:isNuRecVolume`), in addition to particle scenes.
- `replay_directory.py` options: `--warmup_frames`, `--max_frames`, `--skip_completed`, `--self_contained`.
- `OccupancyMap.from_ros_yaml` loads occupancy maps from Omniverse/URL paths via `omni.client`.
- Warn on NuRec replay when `render_rt_subframes` is below the recommended value.
- Tests for NuRec detection, `RecordingSession`, scene collection, and replay completion-marker validation.

### Changed
- Scene caching copies the input scene and its dependencies instead of flattening the stage (much faster for large scenes).
- Replay runs with multi-GPU rendering disabled.

## [0.2.6] - 2026-06-04
### Fixed
- `MobilityGenCamera.update_state`: guard depth/segmentation/normals/instance-id against empty annotator buffers (as RGB already is), avoiding a "tile cannot extend outside image" crash on replay.

### Removed
- Stale `multiGpu` disable from `[[test]]` args; the Kit 110.1.1 multi-GPU startup crash no longer reproduces.

## [0.2.5] - 2026-05-15
### Added
- `nurec_overrides` module exporting `is_nurec_stage` and `apply_nurec_replay_overrides`. Detects NuRec stages by traversing for `ParticleField` prims; when detected, forces replay flags to the supported subset (RGB only) and re-asserts `/rtx/rtpt/gaussian/skipTonemapping/enabled = False` so splat RGB matches the viewport. The RGB-only restriction and tonemap re-assert reflect the current state of NuRec support in Kit/Omniverse; non-RGB modalities and other render settings are gated here until they are addressed in future versions.
- `replay_directory.py`: invokes `apply_nurec_replay_overrides` after each `load_scenario` so per-recording flag overrides take effect before render-product attachment.

## [0.2.4] - 2026-05-13
### Fixed
- Broken texture/material references on replay when the source scene uses external assets — recordings are now self-contained.
- Windows test failures caused by leaked `np.load` mmap handles blocking tempdir cleanup.

## [0.2.3] - 2026-05-08
### Fixed
- `KeyboardButton._event_callback` and `KeyboardDriver._event_callback`: return `True` for handled WASD key events so Kit's `carb.input` stops propagating them to focused `omni.ui` text fields. Fixes buffered keystrokes (e.g. `wwwwwwwwaaa`) appearing in text fields after teleoperation.

## [0.2.2] - 2026-04-28
### Fixed
- `Module.state_dict_common`: cache buffer list after first call; skip module-tree walk on subsequent steps.
- `MobilityGenWriter.write_state_dict_common`: offload npz disk flush to a background thread; bounded queue (`max_pending=8`) limits memory under back-pressure.
- `MobilityGenRobot.update_state`: cache PhysX joint-index and quaternion-reordering arrays; remove redundant `get_world_poses()` call.
- `MobilityGenRobot.get_pose_2d`: replace `quaternion_to_euler_angles().numpy()` with direct `np.arctan2` yaw, eliminating GPU→CPU sync per step.
- `MobilityGenRobot.get_pose_2d`: fix `NameError` for `np` (missing import).
- `OccupancyMap`: precompute `_freespace_mask_cache` at construction; eliminate duplicate `world_to_pixel_numpy` call in `check_world_point_in_freespace`.
- `GridPoseSampler.sample_px`: fix `ValueError: high <= 0` crash when selected grid block has no freespace; fall back to full-map uniform sampling.

## [0.2.1] - 2026-04-27
### Fixed
- Kit test runner failures: converted test classes from `unittest.TestCase` to `omni.kit.test.AsyncTestCase` so tests are awaitable
- `test_migrate_recording.py`: fixed `FileNotFoundError` by walking upward to locate `standalone_examples/` rather than using a hardcoded `parents[4]` offset that diverges between source and build layouts
- Suppressed expected `parse_sensor_entries` log errors in `stdoutFailPatterns.exclude` so intentional invalid-input tests don't trip the runner

## [0.2.0] - 2026-04-22
### Added
- `MobilityGenMultiSensorRobot` and `MobilityGenSensorRig` for YAML-driven multi-sensor robots; camera rendering is supported, other sensor types (lidar, IMU, radar) are discovered but not yet rendered
- `sensor_overrides.py` module with `save_sensor_overrides`, `apply_sensor_overrides`, and `log_camera_properties`
- Camera calibration overrides (intrinsics, distortion, extrinsics) made in the UI are persisted to `sensor_overrides.usda` at recording time and re-applied during replay
- `generate_sensor_rigs.py` script to discover sensor prims in a robot USD and scaffold `sensor_rig:` YAML blocks
- Added Overview, sensor_rig, module, and adding_a_robot documentation

### Changed
- `state/common/` steps now saved as `.npz` (named arrays) instead of `.npy` (pickled dict); use `migrate_recordings.py` to convert older recordings

## [0.1.1] - 2026-04-18
### Changed
- Added return type annotations, `from __future__ import annotations`, and imperative-mood docstrings

## [0.1.0] - 2026-02-25
### Added
- Initial version of MobilityGen extension migrated to use `isaacsim.core.experimental` API
- Replace `isaacsim.core.prims` with `isaacsim.core.experimental.prims` (`XformPrim`, `Articulation`)
- Replace `isaacsim.core.utils` stage/prim helpers with `isaacsim.core.experimental.utils` equivalents
