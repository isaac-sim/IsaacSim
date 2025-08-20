# Changelog
## [1.2.8] - 2025-07-25
### Fixed
- Update assets path to production

## [1.2.7] - 2025-07-07
### Fixed
- Correctly enable omni.kit.loop-isaac in test dependency (fixes issue from 1.2.6)

## [1.2.6] - 2025-07-03
### Changed
- Make omni.kit.loop-isaac an explicit test dependency

## [1.2.5] - 2025-06-25
### Changed
- Add --reset-user to test args

## [1.2.4] - 2025-05-31
### Changed
- Use default nucleus server for all tests

## [1.2.3] - 2025-05-20
### Changed
- Update copyright and license to apache v2.0

## [1.2.2] - 2025-05-20
### Changed
- Update assets path to staging

## [1.2.1] - 2025-05-10
### Changed
- Enable FSD in test settings

## [1.2.0] - 2025-05-09
### Changed
- Add file utility functions

## [1.1.0] - 2025-04-09
### Changed
- Add authentication callback for ETM tests

## [1.0.12] - 2025-04-09
### Changed
- Update all test args to be consistent

## [1.0.11] - 2025-04-04
### Changed
- Version bump to fix extension publishing issues

## [1.0.10] - 2025-03-26
### Changed
- Cleanup and standardize extension.toml, update code formatting for all code

## [1.0.9] - 2025-03-18
### Fixed
- recursive_list_folders now enforces trailing slashes on paths, preventing path clobber bug

## [1.0.8] - 2025-03-11
### Changed
- Switch asset root for tests to internal nucleus

## [1.0.7] - 2025-02-25
### Changed
- Removed remaining omni.client._omniclient type hints in favor of public API

## [1.0.6] - 2025-01-26
### Changed
- Update test settings

## [1.0.5] - 2025-01-21
### Changed
- Update extension description and add extension specific test settings

## [1.0.4] - 2024-12-20
### Added
- Added synchronous version of is_dir() function

## [1.0.3] - 2024-11-18
### Changed
- omni.client._omniclient to omni.client

## [1.0.2] - 2024-11-01
### Changed
- Deprecated get_nvidia_asset_root_path()
- Deprecated get_isaac_asset_root_path()

## [1.0.1] - 2024-10-24
### Changed
- Updated dependencies and imports after renaming

## [1.0.0] - 2024-10-16
### Changed
- Extension renamed to isaacsim.storage.native

## [0.3.2] - 2024-09-23
### Changed
- Updated to 4.5 asset path
- Updated omni.isaac.version dependency to isaacsim.core.version

## [0.3.1] - 2024-08-05
### Changed
- Updated to 4.2 asset path

## [0.3.0] - 2024-06-30
### Changed
- Set Cloud assets path as default
- Updated to 4.1 asset path

## [0.2.0] - 2024-06-06
### Changed
- Updated get root path functions to throw a runtime error if the root path is not found

## [0.1.2] - 2024-05-20
### Changed
- Updated to 4.0 asset path

## [0.1.1] - 2024-04-24
### Changed
- Reduced timeout.

## [0.1.0] - 2024-02-02
### Added
- Added first version of nucleus.
