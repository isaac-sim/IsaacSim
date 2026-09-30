# Changelog

## [0.4.4] - 2026-09-04
### Fixed
- `VisualCuesManager`: tolerate updates while the USD stage is temporarily unavailable during replacement.

## [0.4.3] - 2026-08-25
### Changed
- Subscribe to timeline play/stop through Events 2.0 instead of the deprecated timeline event stream.

## [0.4.2] - 2026-07-31
### Fixed
- `get_kit_xr_runtime_state`: try the remaining Kit XR display/enabled probe when the first one raises, instead of reporting `INACTIVE` and letting Teleop open a competing OpenXR session while stereo rendering is actually active.

## [0.4.1] - 2026-07-23
### Fixed
- Share Kit's active OpenXR session through `isaacsim.kit.xr.teleop.bridge` during stereo VR, and fail safely instead of creating a competing session when the bridge is unavailable.
- Keep 2D live controller tracking independent of Kit XR profile settings, and discover XR APIs lazily when the XR experience loads after Teleop.

## [0.4.0] - 2026-07-15
### Added
- Live teleop checks external CloudXR readiness via `prepare_live_cloudxr_env()` before opening an OpenXR session.
- `LocomotionController`: velocity drive mode via `LocomotionDriveMode` (`AUTO`/`TELEPORT`/`VELOCITY`). Velocity commands a physics velocity on a dynamic rigid-body base (real contacts; jointed payloads follow); Teleport keeps the kinematic pose writes that carry parented children. `AUTO` picks Velocity for a dynamic non-kinematic rigid body and Teleport otherwise; an incompatible Velocity request falls back to Teleport. New `linear_speed`/`angular_speed` (m/s, rad/s) settings tune Velocity mode.
- Typed `TeleopFrame` input boundary with live OpenXR, debug-marker, and headless MCAP providers. `TeleopManager.connect()` accepts MCAP replay and optional live MCAP recording paths while preserving existing controller/head snapshot observers.
- Poll Isaac Teleop `DeviceIOSession.update()` unconditionally; its `None` return no longer causes every live input frame to be discarded.
- Pause caller-paced MCAP consumption with the Kit timeline, rewind it on teleop reset, and suppress replayed controller buttons from toggling a new HDF5 recording.
- Stateless scripted-motion helpers for pose trajectories with quaternion slerp and tolerance-gated execution; these helpers are separate from the live teleop path.
- Config-driven floating-gripper functions that discover xArm and Dex3 YAML definitions, build articulation-safe floating roots, and configure floating, trigger-driven grasp, marker, and visual-cue controllers through an opaque context.
- Built-in `floating_xarm_dex3_retargeted.yaml` profile and headless coverage for profile loading, synthetic input routing, real Dex3 alias/drive resolution, USD DriveAPI targets, and measured articulation motion.
- Regression coverage for consistent degree-authored revolute targets through USD DriveAPI and articulation tensor backends.
- Cross-platform capability discovery that keeps debug input, scripted motion, and supported controllers available when the Linux-only Isaac Teleop and PINK packages are absent.

### Changed
- Replace the in-process `CloudXRManager` lifecycle with `cloudxr_env.py` readiness and environment helpers; the CloudXR runtime is started externally.
- Resolve one canonical anchor transform for Kit XR, teleop head/controller poses, and live frame markers. Fixed rotation now preserves the custom prim's initial absolute yaw; follow modes use absolute yaw, offsets use anchor-local axes, and fixed height applies to the shared transform.
- Treat `TeleopManager.connect()` as the transport-only API; command-driven Connect owns the complete marker, tracking-space, and XR-anchor lifecycle.
- Keep custom tracking-space prims read-only. Locomotion carry is unavailable when a custom prim lacks a safely writable translate/orient/scale xform stack.
- Simplify each visual cue to one shadow-free emissive cylinder with 0.5 default opacity.
- Clarify that grasp-config revolute targets use degrees and prismatic targets use stage linear units. Configurations that used radian values to compensate for the former articulation-path behavior must convert those values to degrees.
- Allow optional per-joint grasp drive-property overrides.
- Align the scripted Dex3 grasp center with the marker offset validated by the teleop SDG pick-and-place scenario.
- Create a named, base-link-local TCP Xform for every scripted floating gripper and expose its resolved path.
- Anchor scripted visual cues and TCP cameras to the named TCP frame instead of the gripper base link.
- Load the Isaac Teleop pip prebundle and PINK dependency only on Linux x86_64; live and MCAP connection attempts now report an actionable unavailable-capability status on other platforms.

### Fixed
- Make the extension discoverable on Windows without the optional Isaac Teleop and PINK backends.
- Expose the `IExt` entry point from the Python module declared in `extension.toml`.
- Convert angular grasp targets from USD degrees to tensor radians when controlling an articulation.
- Report actionable retargeting errors when profile aliases do not resolve against TriHand semantics, the grasp config, or controllable USD joints.
- Release visual-cue update state when its USD stage closes or is replaced, preventing stale anonymous-layer edit-target errors.
- Stop velocity-driven locomotion when tracking or the input session is disconnected, and preserve the final physics-integrated base pose when disabling Velocity mode.
- Preserve live input status reporting across command-driven disconnect/reconnect cycles.
- Apply XR anchor offset, rotation, smoothing, and fixed-height settings configured before connecting, and preserve them across reconnects.
- Author generated `/World/XRAnchor` output in an anonymous session layer, reject collisions with user-owned prims, and remove it exactly on disconnect.
- Roll back partially connected sessions when marker, tracking-space, or XR-profile setup fails.
- Hold the last valid shared anchor pose while a custom prim is temporarily unavailable, and restore the configured XR near plane on disconnect.
- Honor the full configured rotation-smoothing time constant instead of imposing a per-frame interpolation floor.

## [0.3.8] - 2026-07-01
### Fixed
- Extension tests: add `omni.hydra.rtx` test dependency for replicatorSDG captures

## [0.3.7] - 2026-06-25
### Changed
- Removed `omni.ui` dependency; UI remains in `isaacsim.replicator.teleop.ui`.

## [0.3.6] - 2026-06-09
### Fixed
- Fix linter errors and missing or incomplete docstrings.

## [0.3.5] - 2026-05-18
### Fixed
- `test_teleop_sdg_pick_and_place.py`: updated golden images and fixed a grasp offset

### Changed
- `TeleopManager` subscribes to per-frame updates via Kit Events 2.0 (`omni.kit.app.GLOBAL_EVENT_UPDATE`) instead of deprecated `get_update_event_stream()`.

## [0.3.4] - 2026-05-15
### Added
- Golden teleop episode HDF5 (`tests/data/_episode_recorder/episode_floating_xarm_dex3.hdf5`) as a default example for the teleop replay examples

## [0.3.3] - 2026-05-07
### Fixed
- `test_teleop_sdg_pick_and_place.py` split into two independent writers for live and replay
- Fixed reach offset in `floating_xarm` scenario.
- Fixed camera orientation in `ik_dual_ur3_xarm_dex3` profile.

## [0.3.2] - 2026-05-05
### Added
- `floating_xarm.yaml` built-in profile for the solo floating-xArm scenario (VR-origin locomotion).
- `LocomotionController.DEFAULT_LINEAR_STEP` / `DEFAULT_ANGULAR_STEP` class constants.
- End-to-end `test_teleop_sdg_pick_and_place.py` covering the four camera-equipped built-in scenarios with SDG capture and episode replay; adds `isaacsim.test.utils`, `omni.kit.viewport.window`, and `omni.replicator.core` test dependencies.

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
