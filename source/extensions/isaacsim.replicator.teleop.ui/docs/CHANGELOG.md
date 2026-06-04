# Changelog

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
