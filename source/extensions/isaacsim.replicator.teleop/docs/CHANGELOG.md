# Changelog

## [0.3.5] - 2026-05-18
### Fixed
- `test_teleop_sdg_pick_and_place.py`: updated golden images and fixed a grasp offset

### Changed
- `TeleopManager` subscribes to per-frame updates via Kit Events 2.0 (`omni.kit.app.GLOBAL_EVENT_UPDATE`) instead of deprecated `get_update_event_stream()`.

## [0.3.4] - 2026-05-15
### Added
- golden teleop episode HDF5 (`tests/data/_episode_recorder/episode_floating_xarm_dex3.hdf5`) as a default example for the teleop replay examples

## [0.3.3] - 2026-05-07
### Fixed
- `test_teleop_sdg_pick_and_place.py` split into two independent writers for live and replay
- Fixed reach offset in `floating_xarm` scenario.
- Fixed camera orientation in `ik_dual_ur3_xarm_dex3` profile.

## [0.3.2] - 2026-05-05
### Added
- `floating_xarm.yaml` built-in profile for the solo floating-xArm scenario (VR-origin locomotion).
- `LocomotionController.DEFAULT_LINEAR_STEP` / `DEFAULT_ANGULAR_STEP` class constants.
- End-to-end `test_teleop_sdg_pick_and_place.py` covering all four built-in scenarios with SDG capture and episode replay; adds `isaacsim.test.utils`, `omni.kit.viewport.window`, and `omni.replicator.core` test dependencies.

### Changed
- **Breaking:** `LocomotionController` API renamed `linear_speed` / `angular_speed` (and `set_*`) to `linear_step` / `angular_step`; locomotion now applies per-app-update step sizes instead of wall-clock speeds. Built-in profiles use the new keys.
- `ik_solo_ur3_xarm.yaml` simplified to a right-only solo configuration (left-side floating / IK / grasp blocks removed).

## [0.3.1] - 2026-04-28
### Added
- `pose_backend` arg on `build_teleop_recorder` (forwarded to `EpisodeRecorder`).

### Fixed
- Custom XR anchor actually moves the headset.
- Disconnect returns the headset to its pre-Connect pose.

## [0.3.0] - 2026-04-22
### Added
- Teleop-side `Recordable` plugins, a `build_teleop_recorder(...)` factory, and a `VRRecordingButton` that toggles recording from a VR button — all built on `isaacsim.replicator.episode_recorder`.

### Changed
- Recorder / replayer code moved out to `isaacsim.replicator.episode_recorder`; the teleop extension now only contributes teleop-specific channels.

## [0.2.1] - 2026-04-18
### Changed
- Added return type annotations, `from __future__ import annotations`, imperative-mood docstrings, and `__all__` definitions

## [0.2.0] - 2026-04-16
### Added
- Added FSD backend option

### Changed
- Switced to numpy arrays for most operations

## [0.1.0] - 2026-04-09
### Added
- Initial version of the Teleop extension
