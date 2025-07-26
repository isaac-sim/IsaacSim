# Changelog
## [1.3.7] - 2025-07-07
### Fixed
- Correctly enable omni.kit.loop-isaac in test dependency (fixes issue from 1.3.6)

## [1.3.6] - 2025-07-03
### Changed
- Make omni.kit.loop-isaac an explicit test dependency

## [1.3.5] - 2025-06-26
### Changed
- Update to latest loop runner code from kit-sdk
- Remove internal category for extension

## [1.3.4] - 2025-06-26
### Added
- Log failure but don't crash if message queue is null

## [1.3.3] - 2025-06-25
### Changed
- Add --reset-user to test args

## [1.3.2] - 2025-05-31
### Changed
- Use default nucleus server for all tests

## [1.3.1] - 2025-05-19
### Changed
- Update copyright and license to apache v2.0

## [1.3.0] - 2025-05-17
### Changed
- Added a getManualMode function and associated python binding
- Added a manualModeEnabled carb setting that can be set per run loop

## [1.2.6] - 2025-04-04
### Changed
- Version bump to fix extension publishing issues

## [1.2.5] - 2025-03-26
### Changed
- Cleanup and standardize extension.toml, update code formatting for all code

## [1.2.4] - 2025-03-04
### Changed
- Update to kit 107.1 and fix build issues

## [1.2.3] - 2025-02-21
### Changed
- Update style format and naming conventions in c++ code, add doxygen docstrings

## [1.2.2] - 2025-01-28
### Fixed
- Windows signing issue

## [1.2.1] - 2025-01-21
### Changed
- Update extension description and add extension specific test settings

## [1.2.0] - 2024-01-08
### Changed
- Moved header files to extension

## [1.1.0] - 2023-10-24
### Changed
- Update to latest loop runner code from kit-sdk and set default settings

## [1.0.2] - 2023-06-12
### Changed
- Update to kit 105.1, pybind11 header locations

## [1.0.1] - 2023-05-19
### Fixed
- Crash on exit due to threading race condition

## [1.0.0] - 2022-09-29
### Changed
-   set_runner_dt to set_manual_step_size
-   setting the dt does not enable manual mode, need to call set_manual_mode(True)

### Added
-   set_manual_mode to enable/disable manual dt during runtime.

## [0.1.0] - 2021-07-09
### Added
-   Initial Isaac RunLoop runner implementation
