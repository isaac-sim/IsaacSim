# Changelog
## [1.4.9] - 2025-07-07
### Fixed
- Correctly enable omni.kit.loop-isaac in test dependency (fixes issue from 1.4.8)

## [1.4.8] - 2025-07-03
### Changed
- Make omni.kit.loop-isaac an explicit test dependency

## [1.4.7] - 2025-06-26
### Changed
- Disable fabric usd notice handler during cloning

## [1.4.6] - 2025-06-25
### Changed
- Add --reset-user to test args

## [1.4.5] - 2025-06-19
### Changed
- Fixed another issue with fabric cloning for partially populated stages

## [1.4.4] - 2025-06-17
### Changed
- Fixed fabric cloning for partially populated stages
- Pre-populated rigid body attributes before cloning

## [1.4.3] - 2025-06-06
### Changed
- Make the cloner extension platform specific

## [1.4.2] - 2025-06-02
### Changed
- Removed omni.population dependency

## [1.4.1] - 2025-05-31
### Changed
- Use default nucleus server for all tests

## [1.4.0] - 2025-05-30
### Changed
- Support for clone in fabric added

## [1.3.12] - 2025-05-19
### Changed
- Update copyright and license to apache v2.0

## [1.3.11] - 2025-05-10
### Changed
- Enable FSD in test settings

## [1.3.10] - 2025-04-11
### Changed
- Update Isaac Sim robot asset path
- Update Isaac Sim robot asset path for the IsaacSim folder

## [1.3.9] - 2025-04-09
### Changed
- Update all test args to be consistent

## [1.3.8] - 2025-04-04
### Changed
- Version bump to fix extension publishing issues

## [1.3.7] - 2025-03-26
### Changed
- Cleanup and standardize extension.toml, update code formatting for all code

## [1.3.6] - 2025-03-11
### Changed
- Switch asset root for tests to internal nucleus

## [1.3.5] - 2025-03-04
### Changed
- Update to kit 107.1 and fix build issues

## [1.3.4] - 2025-01-26
### Changed
- Update test settings

## [1.3.3] - 2025-01-21
### Changed
- Update extension description and add extension specific test settings

## [1.3.2] - 2024-12-29
### Fixed
- Missing argument in grid_cloner

## [1.3.1] - 2024-12-20
### Fixed
- Quaternion precision mismatch issue

## [1.3.0] - 2024-12-17
### Added
- Added `enable_env_ids` flag to clone to enable physics replication env IDs for co-location of clones

## [1.2.0] - 2024-11-26
### Added
- Ability to disable notice handling during cloning process to improve performance

## [1.1.1] - 2024-10-24
### Changed
- Updated dependencies and imports after renaming

## [1.1.0] - 2024-10-10
### Changed
- Physics replication will no longer be unregistered by default when replication is set to False. Instead, an argument is added to `clone` for unregistering replication.

## [1.0.0] - 2024-09-24
### Changed
- Extension renamed to isaacsim.core.cloner

## [0.8.1] - 2024-02-07
### Changed
- Updated path to the nucleus extension

## [0.8.0] - 2024-02-02
### Changed
- Support executing cloner on non-context stage

## [0.7.3] - 2024-01-18
### Changed
- Changed get_assets_root_path to get_assets_root_path_async for the unit tests

## [0.7.2] - 2023-11-10
### Fixed
- Fixed error with GridCloner when offsets are passed
- Add test case with position and orientation offsets for GridCloner

## [0.7.1] - 2023-10-31
### Fixed
- Fixed the order in which xformop is set while cloning. Earlier a set was passed instead of list.

## [0.7.0] - 2023-07-26
### Added
- Exposed API for retrieving clone transorms from GridCloner
- Add unregister when physics replication is disabled

## [0.6.0] - 2023-07-19
### Changed
- When `positions` and `orientations` are None, the cloner keeps the Xform translation and orientation instead of using identity
- Makes the method `replicate_physics` public to allow users to have control over it

## [0.5.0] - 2023-04-27
### Added
- Added option to specify mode of cloning (inherit vs. copy)

## [0.4.1] - 2022-12-09
### Fixed
- Fixed crash when cloning with single environment

## [0.4.0] - 2022-11-18
### Added
- Added option to use omni.physics replication

## [0.3.0] - 2022-07-11
### Changed
- Added setting and deleting the different xform properties used by isaacsim.core.api

## [0.2.1] - 2022-06-23
### Fixed
- Preserve scale property from source prim

## [0.2.0] - 2022-06-15
### Changed
- Optimized cloning and collision filtering logic
- Moved common utility functions to base Cloner class

## [0.1.2] - 2022-05-11
### Fixed
- Added dependency to omni.physx

## [0.1.1] - 2022-05-09
### Added
- Added documentation

### Changed
- Updated filter collision logic. Use inverted collision group filter for more efficient collision group generation.

## [0.1.0] - 2022-03-30
### Added
- Added Initial Classes
