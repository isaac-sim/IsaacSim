# Changelog
## [2.0.3] - 2025-07-07
### Fixed
- Correctly enable omni.kit.loop-isaac in test dependency (fixes issue from 2.0.2)

## [2.0.2] - 2025-07-03
### Changed
- Make omni.kit.loop-isaac an explicit test dependency

## [2.0.1] - 2025-06-25
### Changed
- Add --reset-user to test args

## [2.0.0] - 2025-06-10
### Changed
- Refactored icon updates to enable batch processing per frame

### Added
- More layers of visiblity checks to prevent issues with extension startup sync conditions
- Cached set of prims that were previously checked for sensor prims to improve performance on frame updates

## [1.2.4] - 2025-06-06
### Changed
- ISIM-3831: Fix test failed.

## [1.2.3] - 2025-06-02
### Changed
- Set default sensor icon visibility to false

## [1.2.2] - 2025-05-31
### Changed
- Use default nucleus server for all tests

## [1.2.1] - 2025-05-29
### Changed
- ISIM-3688: Allow sensor icon scale with distance

## [1.2.0] - 2025-05-27
### Added
- Subscription to TimelineEvents.STOP to force reposition icons

### Changed
- Delete icons when disabling visibility to prevent visibility update timing issues

## [1.1.3] - 2025-05-19
### Changed
- Update copyright and license to apache v2.0

## [1.1.2] - 2025-05-16
### Changed
- Make extension target a specific kit version

## [1.1.1] - 2025-05-10
### Changed
- Enable FSD in test settings

## [1.1.0] - 2025-05-08
### Changed
- Switched to Events 2.0 subscriptions
- Disabled USD update listener when global visibility is off

## [1.0.2] - 2025-05-06
### Changed
- Fix error when populating initial icons when saving a stage

### Added
- Added additional tests to check path errors

## [1.0.1] - 2025-05-03
### Changed
- Updated unit tests with new paths

### Added
- Error handling for extension restarts

## [1.0.0] - 2025-04-29
### Changed
- Reworked to usdrt implementation to eliminate performance impact

### Added
- usdrt dependency

## [0.1.3] - 2025-04-21
### Changed
- Sensor Icon file

## [0.1.2] - 2025-04-11
### Changed
- Fix error when usd notice handler handled deleted prims

## [0.1.1] - 2025-04-09
### Changed
- Update all test args to be consistent

## [0.1.0] - 2025-04-05
### Added
- Initial release
