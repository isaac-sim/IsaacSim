# Changelog

## [0.1.4] - 2026-06-09
### Fixed
- Fix linter errors and missing or incomplete docstrings.

## [0.1.3] - 2026-05-17
### Fixed
- Suppressed the known `h5py` HDF5 ABI warning during recorder storage import so Windows native Python tests do not treat the warning stderr as a failure.
- The episode record/replay standalone example test now removes its own generated artifacts before recording, keeping output validation repeatable across local reruns.

## [0.1.2] - 2026-05-05
### Changed
- `ArticulationRecordable._discover_link_paths` now returns rigid-body link paths only when any are tagged with `UsdPhysics.RigidBodyAPI`, skipping visual / collision Xformable containers; falls back to all Xformable descendants when no rigid bodies are present. Recorded `link_paths` for the same articulation may shrink accordingly.
- The articulation root path is always preserved at the head of `link_paths` when `include_root` is set, even when only rigid-link descendants are kept.

## [0.1.1] - 2026-04-28
### Added
- `pose_backend` arg on `EpisodeRecorder` / `EpisodeReplayer` (`usd` / `usdrt` / `fabric`, default `usd`). Mid-session FSD toggle silently demotes to `usd` with a one-shot warning instead of crashing.
- `pose_batch_tier_count` / `pose_backend` properties; `PoseBackend` re-exported.

### Fixed
- Replay writes nested xforms / articulations in parents-first tiers, removing the one-frame-lag stutter on prims under a moving parent.
- Missing-xformOps recovery on replay scopes per tier, so later tiers hitting the same error still recover instead of aborting the run.

## [0.1.0] - 2026-04-22
### Added
- Initial release: `EpisodeRecorder` / `EpisodeReplayer` record simulation state to HDF5 via a pluggable `Recordable` protocol and replay it as pure USD writes onto an anonymous sublayer (no physics stepping, no stage mutation).
