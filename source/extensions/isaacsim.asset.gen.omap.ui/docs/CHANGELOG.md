# Changelog

## [2.6.1] - 2026-04-14
### Changed
- UI improvements: unified filename field, auto-updating YAML, window docks to Property panel

## [2.6.0] - 2026-04-08
### Changed
- Improve Python API documentation (`config/python_api.md` and/or module docstrings).

## [2.5.1] - 2026-03-25
### Changed
- Replace deprecated onclick_fn with onclick_action for menu registration

## [2.5.0] - 2026-02-25
### Added
- Save YAML button in the visualization window to save the ROS occupancy map parameters file directly, alongside the existing Save Image button
- Image File Name field in the visualization window to set the image filename used in the YAML; defaults to the stage name
- Update YAML button to rebuild the YAML content with the new filename without regenerating the image
- Save Image dialog now pre-fills the filename from the Image File Name field

## [2.4.3] - 2026-02-23
### Changed
- Add ui test dependency

## [2.4.2] - 2026-02-05
### Changed
- Added dock info for Robot Hierarchy window

## [2.4.1] - 2025-12-05
### Changed
- Migrate to Events 2.0.

## [2.4.0] - 2025-11-05
### Changed
- Refactor codebase and improve docstrings

## [2.3.0] - 2025-10-30
### Changed
- Migrate extension implementation to core experimental API

## [2.2.4] - 2025-10-27
### Changed
- Make omni.isaac.ml_archive an explicit test dependency

## [2.2.3] - 2025-10-08
### Changed
- Fix incorrect image origin calculation

## [2.2.2] - 2025-08-04
### Changed
- Removed duplicated code

## [2.2.1] - 2025-05-19
### Changed
- Update copyright and license to apache v2.0

## [2.2.0] - 2025-04-28
### Changed
- Updated image visualization window to show ROS yaml file and 180 degree rotation as default config for image generation

## [2.1.1] - 2025-04-04
### Changed
- Version bump to fix extension publishing issues

## [2.1.0] - 2025-03-31
### Changed
- Menu items were added by MenuHelpers.

## [2.0.5] - 2025-03-26
### Changed
- Cleanup and standardize extension.toml, update code formatting for all code

### Added
- Layout specifically for Occupancy Map was added.

## [2.0.4] - 2025-01-21
### Changed
- Update extension description and add extension specific test settings

## [2.0.3] - 2024-12-03
### Changed
- Isaac Util menu to Tools->Robotics menu

## [2.0.2] - 2024-10-28
### Changed
- Remove test imports from runtime

## [2.0.1] - 2024-10-24
### Changed
- Updated dependencies and imports after renaming

## [2.0.0] - 2024-10-04
### Changed
- Extension  renamed to isaacsim.asset.gen.omap.ui.

## [1.1.1] - 2024-09-03
### Changed
- Limit Cell Size slider's minimum value to avoid it begin less than or equal to 0

## [1.1.0] - 2024-05-16
### Changed
- Invisible geometry will not be mapped when Use PhysX Collision Geometry is false

## [1.0.1] - 2024-05-15
### Fixed
- Issue when generating occupancy map with prims that have no points

## [1.0.0] - 2024-03-11
### Added
- Initial version of Isaac Sim Occupancy Map UI Extension
