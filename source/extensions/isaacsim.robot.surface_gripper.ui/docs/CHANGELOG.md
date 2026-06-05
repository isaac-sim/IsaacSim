# Changelog

## [3.1.4] - 2026-04-21
### Changed
- Replace `omni.kit.commands.execute("CreateSurfaceGripper")` call sites with direct `create_surface_gripper()` API

## [3.1.3] - 2026-03-26
### Changed
- Updated Python bindings import paths for consistency

## [3.1.2] - 2026-03-06
### Fixed
- Revoke existing USD ObjectsChanged listener before registering a new one in build_items to prevent listener leaks on repeated property panel rebuilds

## [3.1.1] - 2026-03-05
### Removed
- Remove unused and deprecated extension dependencies

## [3.1.0] - 2026-03-04
### Changed
- Added Overview.md, python_api.md and updated docstrings

## [3.0.3] - 2025-10-27
### Changed
- Make omni.isaac.ml_archive an explicit test dependency

## [3.0.2] - 2025-05-30
### Changed
- Update to typed name for schema instead of hard-coded strings

## [3.0.1] - 2025-05-19
### Changed
- Update copyright and license to apache v2.0

## [3.0.0] - 2025-04-08
### Changed
- Property panel for new Surface Gripper component.

## [2.1.3] - 2025-04-04
### Changed
- Version bump to fix extension publishing issues

## [2.1.2] - 2025-03-26
### Changed
- Cleanup and standardize extension.toml, update code formatting for all code

## [2.1.1] - 2025-01-21
### Changed
- Update extension description and add extension specific test settings

## [2.1.0] - 2024-11-01
### Changed
- Menu name

## [2.0.1] - 2024-10-24
### Changed
- Updated dependencies and imports after renaming

## [2.0.0] - 2024-09-27
### Changed
- Extension renamed to isaacsim.robot.surface_gripper.ui

## [1.0.0] - 2024-03-13
### Added
- Initial version of extension
