# Changelog

## [1.2.12] - 2026-10-01
### Fixed
- `OgnIntervalFiltering`: emit a flat index array and always write `outputs:indices`, so frames with no matching environment clear the previous selection.

### Added
- Test covering `gate.on_interval` gravity updates on positive interval multiples.

## [1.2.11] - 2026-09-23
### Fixed
- Use the normal distribution's standard deviation directly when deriving material-property bucket ranges.

## [1.2.10] - 2026-08-19
### Fixed
- `get_reset_inds`: raise `RuntimeError` when the domain-randomization context is not initialized.

## [1.2.9] - 2026-08-14
### Fixed
- Export the extension `IExt` so stage close clears physics-view registries and the randomization context.
- Connect `numSamples` only when the distribution node exposes that input so sequence distributions can author graphs.

## [1.2.8] - 2026-08-13
### Fixed
- Skip tendon attribute randomization on reset for articulations with no fixed tendons so later writes on the same view still apply.

## [1.2.7] - 2026-07-21
### Changed
- Migrated robot asset references from `Isaac/Robots/` to `Isaac/Robots_Multiphysics/` for the new multiphysics-ready USDA assets.

## [1.2.6] - 2026-07-07
### Changed
- Update the generated Python API inventory.

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
