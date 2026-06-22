# Changelog

## [1.2.5] - 2026-06-09
### Fixed
- Fix linter errors and missing or incomplete docstrings, and update `python_api.md`.

## [1.2.4] - 2026-04-22
### Fixed
- Fix module-level physics view state not cleaned up on stage close/reload, causing stale views to accumulate across sessions

## [1.2.3] - 2026-04-20
### Fixed
- Fix `OgnWritePhysicsArticulationView` `KeyError` when randomizing tendon attributes on robots with no fixed tendons

## [1.2.2] - 2026-04-18
### Changed
- Added return type annotations, imperative-mood docstrings, and `__all__` definitions

## [1.2.1] - 2026-04-10
### Removed
- Remove the `omni.isaac.ml_archive` dependency

## [1.2.0] - 2026-04-01
### Added
- Added tests for the articulation and rigid prim views

### Changed
- Registered OGN nodes under `isaacsim.replicator.domain_randomization` namespace with backward compatibility bridges for deprecated context, views, and simulation context

## [1.1.0] - 2026-03-04
### Changed
- Added Overview.md, python_api.md and updated docstrings

## [1.0.1] - 2026-01-13
### Changed
- Update dependencies to use the latest experimental core utils API

## [1.0.0] - 2026-01-12
### Added
- Initial release of experimental domain randomization extension
- Support for `isaacsim.core.experimental.prims` API
- RigidPrim and Articulation view registration and randomization
- SimulationContext randomization support
- Interval-based and reset-triggered randomization gates
- OmniGraph nodes for physics attribute randomization
