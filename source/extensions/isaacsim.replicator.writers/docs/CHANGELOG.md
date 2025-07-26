# Changelog
## [1.0.15] - 2025-07-07
### Fixed
- Correctly enable omni.kit.loop-isaac in test dependency (fixes issue from 1.0.14)

## [1.0.14] - 2025-07-03
### Changed
- Make omni.kit.loop-isaac an explicit test dependency

## [1.0.13] - 2025-06-25
### Changed
- Add --reset-user to test args

## [1.0.12] - 2025-05-31
### Changed
- Use default nucleus server for all tests

## [1.0.11] - 2025-05-19
### Changed
- Update copyright and license to apache v2.0

## [1.0.10] - 2025-05-16
### Changed
- Make extension target a specific kit version

## [1.0.9] - 2025-05-10
### Changed
- Enable FSD in test settings

## [1.0.8] - 2025-05-07
### Changed
- PoseWriter test: quaternion comparison both positive and negative quaternions
- PoseWriter test: added float comparison tolerance atol
- PoseWriter test: change the json golden output to store the new replicator rotation (negative of the previous one)

## [1.0.7] - 2025-04-11
### Fixed
- Fixed PoseWriter size output from AABB to Oriented Bounding Box

### Added
- Added PoseWriter test

## [1.0.6] - 2025-04-09
### Changed
- Update all test args to be consistent

## [1.0.5] - 2025-04-04
### Changed
- Version bump to fix extension publishing issues

## [1.0.4] - 2025-03-26
### Changed
- Cleanup and standardize extension.toml, update code formatting for all code

## [1.0.3] - 2025-03-05
### Changed
- Update extension codebase to adhere to isaac sim extension structure and file naming  guidelines

## [1.0.2] - 2025-03-04
### Changed
- Update to kit 107.1 and fix build issues

## [1.0.1] - 2025-01-21
### Changed
- Update extension description and add extension specific test settings

## [1.0.0] - 2024-12-09
### Added
- created extension with writer parts from omni.replicator.isaac
