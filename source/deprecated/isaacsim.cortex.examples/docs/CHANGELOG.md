# Changelog

## [0.2.3] - 2026-04-28
### Added
- Added `isaacsim.examples.browser` as a dependency so `register_example` is available at startup

## [0.2.2] - 2026-04-27
### Added
- Added interactive samples back to the Robotics Example UI by adding `python.module` in `extension.toml`

## [0.2.1] - 2026-04-24
### Changed
- Fixed import path for `base_sample` module

## [0.2.0] - 2026-04-23
### Added
- Move the `isaacsim.examples.interactive.base_sample` module to this extension

## [0.1.1] - 2026-04-17
### Fixed
- `CortexBase.load_world_async` now calls `setup_scene()` when switching examples with an existing CortexWorld

## [0.1.0] - 2026-04-13
### Deprecated
- Examples deprecated. These examples used to exist in `isaacsim.examples.interactive`. Will be replaced in 7.0 with simple examples and open source equivalents.
