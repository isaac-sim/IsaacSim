# Changelog
## [1.1.14] - 2025-07-17
### Changed
- Updated windows golden images

## [1.1.13] - 2025-07-11
### Fixed
- Texture randomizer uses unique names for materials (WAR for ISIM-4054)

### Changed
- Update sdg pipeline golden images

## [1.1.12] - 2025-07-07
### Fixed
- Correctly enable omni.kit.loop-isaac in test dependency (fixes issue from 1.1.11)

## [1.1.11] - 2025-07-03
### Changed
- Make omni.kit.loop-isaac an explicit test dependency

## [1.1.10] - 2025-06-25
### Changed
- Add --reset-user to test args

## [1.1.9] - 2025-06-08
### Changed
- Update sdg pipeline golden images on windows

## [1.1.8] - 2025-05-31
### Changed
- Use default nucleus server for all tests

## [1.1.7] - 2025-05-23
### Changed
- New sdg pipeline golden images
- Removed npy comparison because of redunancy with png depth comparison
- Renamed output folder to _out_behaviors_sdg with leading underscore for consistency

## [1.1.6] - 2025-05-19
### Changed
- Update copyright and license to apache v2.0

## [1.1.5] - 2025-05-16
### Changed
- Make extension target a specific kit version

## [1.1.4] - 2025-05-12
### Changed
- Using os.path.join to create output directory in sdg tests

## [1.1.3] - 2025-05-10
### Changed
- Enable FSD in test settings

## [1.1.2] - 2025-05-07
### Changed
- Added checks for stage validity when destroying / resetting behaviors

## [1.1.1] - 2025-04-26
### Changed
- SDG pipeline: Increased rgb mean diff tolerance and included depth comparison with small mean diff tolerance
- SDG pipeline: Split into linux and windows golden images

## [1.1.0] - 2025-04-21
### Changed
- Event 2.0 handling
- Updated sdg pipeline golden images

## [1.0.14] - 2025-04-17
### Changed
- changed add_update_semantics to add_labels

## [1.0.13] - 2025-04-09
### Changed
- Update all test args to be consistent

## [1.0.12] - 2025-04-04
### Changed
- Version bump to fix extension publishing issues

## [1.0.11] - 2025-03-26
### Changed
- Cleanup and standardize extension.toml, update code formatting for all code

## [1.0.10] - 2025-03-24
### Changed
- Migrate to Events 2.0

## [1.0.9] - 2025-03-11
### Changed
- Switch asset root for tests to internal nucleus

## [1.0.8] - 2025-01-26
### Changed
- Update test settings

## [1.0.7] - 2025-01-21
### Changed
- Update extension description and add extension specific test settings

## [1.0.6] - 2025-01-09
### Fixed
- Initial behavior not applied if interval is greater than 0

### Added
- Util functions for adding scripts and triggering and waiting for behavior events
- SDG scenario test with golden images

## [1.0.5] - 2024-12-12
### Fixed
- Cleaning up randomizers by calling reset when destroyed during play

## [1.0.4] - 2024-12-05
### Fixed
- Fixed volume stack behavior phyisics material assignment (not needed on parent prim)
- Avoid delta_time=0.0 randomizations through orchestrator.step() stage updates
- Exposed variables removed from fabric as well (avoid UI still showing them)
- Fixed reset simulation case without any performed simulation

## [1.0.3] - 2024-11-27
### Fixed
- Fixed error when removing all the scripts widget during runtime

## [1.0.2] - 2024-11-06
### Fixed
- Fixed stacking bounding box drop area calculation for scaled assets
- Fixed get world rotation util function to use Gf.Transform to get the rotation in case of transforms with scale/shear
- Check for previous colliders/rigid bodies for assets before simulation

### Changed
- Set exposed variables default values

## [1.0.1] - 2024-10-31
### Fixed
- Fixed 'remove_empty_scopes' to check if the prim is valid before searching for Scope or GenericPrim types

## [1.0.0] - 2024-09-29
### Added
- Added initial behavior script based randomizers
