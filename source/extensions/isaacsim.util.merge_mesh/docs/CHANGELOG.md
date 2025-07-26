# Changelog
## [2.0.15] - 2025-07-07
### Fixed
- Correctly enable omni.kit.loop-isaac in test dependency (fixes issue from 2.0.14)

## [2.0.14] - 2025-07-03
### Changed
- Make omni.kit.loop-isaac an explicit test dependency

## [2.0.13] - 2025-06-25
### Changed
- Add --reset-user to test args

## [2.0.12] - 2025-05-31
### Changed
- Use default nucleus server for all tests

## [2.0.11] - 2025-05-19
### Changed
- Update copyright and license to apache v2.0

## [2.0.10] - 2025-05-16
### Changed
- Make extension target a specific kit version

## [2.0.9] - 2025-05-10
### Changed
- Enable FSD in test settings

## [2.0.8] - 2025-04-04
### Changed
- Version bump to fix extension publishing issues

## [2.0.7] - 2025-03-26
### Changed
- Cleanup and standardize extension.toml, update code formatting for all code

## [2.0.6] - 2025-01-21
### Changed
- Update extension description and add extension specific test settings

## [2.0.5] - 2025-01-13
### Changed
- Fixed UI consistency when no prim path is selected for Looks destination

## [2.0.4] - 2024-12-23
### Changed
- Fixed error when UV maps are missing

## [2.0.3] - 2024-12-03
### Changed
- Isaac Util menu to Tools->Robotics menu

## [2.0.2] - 2024-11-08
### Fixed
- Add Texture map to merged assets

## [2.0.1] - 2024-10-24
### Changed
- Updated dependencies and imports after renaming

## [2.0.0] - 2024-09-27
### Changed
- Extension renamed to isaacsim.util.merge_mesh

## [1.0.2] - 2024-07-17
### Fixed
- Missing dependency for tests

## [1.0.1] - 2024-04-18
### Added
- Dependency for tests

## [1.0.0] - 2024-04-12
### Added
- Code-Accessible Merge Mesh functionality (API)
- Merge Command
- Unit Tests
- Disable source assets when merging

### Changed
- Combine Materials now Creates the materials in the provided path if they don't exist

## [0.1.5] - 2024-04-10
### Fixed
- Fix Broken empty Material binding check

## [0.1.4] - 2023-08-09
### Fixed
- Switch from ui.Window to ScrollingWindow wrapper for extension because scrolling was broken

## [0.1.3] - 2023-06-12
### Changed
- Update to kit 105.1, omni.usd.utils -> omni.usd

## [0.1.2] - 2023-02-08
### Fixed
- add primvars normals when merging

## [0.1.1] - 2023-01-06
### Fixed
- onclick_fn warning when creating UI

## [0.1.0] - 2021-07-29
### Added
- Initial version of extension
