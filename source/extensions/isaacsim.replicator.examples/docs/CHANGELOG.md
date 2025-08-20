# Changelog

## [1.1.31] - 2025-07-18
### Changed
- Reduce path length for test bucket names to avoid issues with long paths

## [1.1.30] - 2025-07-16
### Changed
- Bucketed tests by context. Skipping sdg_ur10_palletizing test on ETM.

## [1.1.29] - 2025-07-07
### Fixed
- Correctly enable omni.kit.loop-isaac in test dependency (fixes issue from 1.1.28)

## [1.1.28] - 2025-07-03
### Changed
- Make omni.kit.loop-isaac an explicit test dependency

## [1.1.27] - 2025-07-01
### Changed
- Update simready assets test example to add references using isaac sim api

## [1.1.26] - 2025-06-25
### Changed
- Add --reset-user to test args

## [1.1.25] - 2025-06-13
### Changed
- Fixed various simready assets example snippet warnings

## [1.1.24] - 2025-06-03
### Changed
- Fix incorrect licenses and add missing licenses

## [1.1.23] - 2025-05-31
### Changed
- Use default nucleus server for all tests

## [1.1.22] - 2025-05-30
### Changed
- increase rt_subframes in examples for more consistent results between updates

## [1.1.21] - 2025-05-23
### Changed
- rename test utils.py to common.py

## [1.1.20] - 2025-05-22
### Changed
- Update copyright and license to apache v2.0

## [1.1.19] - 2025-05-21
### Changed
- Added simready assets example snippettest

## [1.1.18] - 2025-05-16
### Changed
- Make extension target a specific kit version

## [1.1.17] - 2025-05-16
### Added
- added test for starting capturing while the timeline is running
- test utils.py for functions used in multiple tests

### Changed
- more verbose terminal outputs for tests

## [1.1.16] - 2025-05-12
### Changed
- Using os.path.join to create output directory in sdg tests

## [1.1.15] - 2025-05-10
### Changed
- Enable FSD in test settings

## [1.1.14] - 2025-05-10
### Changed
- fixed timeline tests by making sure the timeline is stopped and the looping is set to its original value

## [1.1.13] - 2025-05-09
### Added
- Added data augmentation example test with golden data comparison

## [1.1.12] - 2025-05-07
### Changed
- Changed custom fps example to use duration in seconds as input
- Increased the number of app updates in setUp and tearDown

## [1.1.11] - 2025-04-30
### Changed
- Update event subscriptions to Event 2.0 system

## [1.1.10] - 2025-04-17
### Changed
- changed add_update_semantics to add_labels

## [1.1.9] - 2025-04-09
### Changed
- Update all test args to be consistent
- SDG palletizing example does not need the custom preview event anymore

## [1.1.8] - 2025-04-04
### Changed
- Version bump to fix extension publishing issues

## [1.1.7] - 2025-03-26
### Changed
- Cleanup and standardize extension.toml, update code formatting for all code

## [1.1.6] - 2025-03-24
### Changed
- Migrate to Events 2.0

## [1.1.5] - 2025-03-11
### Changed
- Switch asset root for tests to internal nucleus

## [1.1.4] - 2025-03-05
### Changed
- Update extension codebase to adhere to isaac sim extension structure and file naming  guidelines

## [1.1.3] - 2025-03-04
### Changed
- Update to kit 107.1 and fix build issues

## [1.1.2] - 2025-01-26
### Changed
- Update test settings

## [1.1.1] - 2025-01-21
### Changed
- Update extension description and add extension specific test settings

## [1.1.0] - 2024-12-17
### Added
- added UR10 palletizing demo test

## [1.0.1] - 2024-12-15
### Fixed
- added fixed timestepping for consistent results for custom fps capture test

## [1.0.0] - 2024-12-09
### Added
- created extension with example parts from omni.replicator.isaac
