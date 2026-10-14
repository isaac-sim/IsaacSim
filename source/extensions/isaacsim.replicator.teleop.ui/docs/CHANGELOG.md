# Changelog

## [0.4.5] - 2026-10-01
### Changed
- Require a stopped timeline to load profiles or enable Debug Mode; disabling Debug Mode remains available during playback.
- Rename Write Backend to Transform Backend to clarify its scope.

### Fixed
- Reset Debug Mode, input sliders, and profile validation status when the stage closes.

## [0.4.4] - 2026-08-25
### Changed
- Subscribe to timeline play/stop through Events 2.0 instead of the deprecated timeline event stream.

## [0.4.3] - 2026-08-24
### Fixed
- Loading a teleop profile now clears omitted IK and floating prim paths instead of keeping the previous profile's values.

## [0.4.2] - 2026-08-10
### Fixed
- Wait for stage-driven material and menu updates to settle before opening the Teleop window in the floating-controller test.

## [0.4.1] - 2026-07-30
### Fixed
- Release the extension instance when the Teleop UI shuts down.

## [0.4.0] - 2026-07-15
### Added
- Debug squeeze sliders for controller-driven multi-finger grasp input.
- **Visual Cues** panel with a single shadow-free cylinder per controller, configurable reference height and appearance, optional prim overrides, and profile persistence.
- Partial-support UI for platforms without Isaac Teleop: live Connect is disabled while frame markers, Debug Mode, profiles, and supported controllers remain available.

### Changed
- Live **Connect** expects CloudXR to be started in a separate terminal; there are no in-process CloudXR lifecycle controls.
- Clarify that **Custom Anchor** is read-only and shared by XR rendering, markers, and teleop targets. Rotation labels now describe initial-yaw hold versus absolute-yaw following, and Offset is documented in anchor-local axes.

### Fixed
- Make the debug-mode UI discoverable on Windows without the optional Isaac Teleop package.
- Route desktop **Connect** and **Disconnect** through the complete teleop command lifecycle so markers, tracking space, controllers, and the XR anchor are set up and torn down consistently.

## [0.3.4] - 2026-06-09
### Fixed
- Fix linter errors and missing or incomplete docstrings.

## [0.3.3] - 2026-05-05
### Changed
- Locomotion panel exposes per-app-update step sizes (Slide Step / Turn Step) and round-trips them through profiles, matching the renamed `LocomotionController` API.

## [0.3.2] - 2026-05-01
### Changed
- Teleop UI tests now use the shared menu UI test base class for more robust menu interactions in CI.

## [0.3.1] - 2026-04-28
### Added
- `TeleopWindow` activates a pre-session XR anchor on open and restores prior XR settings on close, so the headset tracks real motion before Connect.
- UI tests for the Profiles panel covering load / save round-trips.

### Changed
- Session-panel anchor controls merged into a single **XR Anchor** collapsable.
- **Custom Anchor** uses **Set** / **Clear** (replaces Apply + Enable / Disable).
- Profiles round-trip the Custom Anchor toggle exactly (no auto-activation when loaded as disabled).

## [0.3.0] - 2026-04-22
### Changed
- Removed the record panel: episode recording and replay now live in the standalone `isaacsim.replicator.episode_recorder.ui` window, and teleop channels are contributed to its sessions via a session-injector installed by `TeleopManager`.

## [0.2.1] - 2026-04-18
### Changed
- Added return type annotations and imperative-mood docstrings

## [0.2.0] - 2026-04-16
### Added
- Added FSD backend option

## [0.1.0] - 2026-04-09
### Added
- Initial version of the Teleop UI extension
