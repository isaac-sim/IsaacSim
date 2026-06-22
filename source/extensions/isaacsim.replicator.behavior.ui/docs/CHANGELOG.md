# Changelog

## [1.2.1] - 2026-06-09
### Fixed
- Fix linter errors and missing or incomplete docstrings.

## [1.2.0] - 2026-04-22
### Added
- Subscribe to the `isaacsim.replicator.behavior.EXPOSED_VARS_CHANGED` event and refresh the property window when exposed variables are created or removed. This replaces the direct `omni.kit.window.property.request_rebuild()` calls previously issued from the core behavior scripts, allowing the core extension to be decoupled from any UI module and run headless.
- Guarded the `omni.kit.window.property` import so the UI extension can still be imported in headless contexts (the event handler becomes a no-op).

## [1.1.1] - 2026-04-18
### Changed
- Added return type annotations and imperative-mood docstrings

## [1.1.0] - 2026-03-04
### Changed
- Added Overview.md, python_api.md and updated docstrings

## [1.0.6] - 2025-12-01
### Changed
- Update test module import

## [1.0.5] - 2025-05-19
### Changed
- Update copyright and license to apache v2.0

## [1.0.4] - 2025-05-16
### Changed
- Make extension target a specific kit version

## [1.0.3] - 2025-04-04
### Changed
- Version bump to fix extension publishing issues

## [1.0.2] - 2025-03-26
### Changed
- Cleanup and standardize extension.toml, update code formatting for all code

## [1.0.1] - 2025-01-21
### Changed
- Update extension description and add extension specific test settings

## [1.0.0] - 2024-09-29
### Added
- Added property widget to visualize exposed variables
