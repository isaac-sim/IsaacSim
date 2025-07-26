# Changelog
## [2.2.15] - 2025-07-07
### Fixed
- Correctly enable omni.kit.loop-isaac in test dependency (fixes issue from 2.2.14)

## [2.2.14] - 2025-07-03
### Changed
- Make omni.kit.loop-isaac an explicit test dependency

## [2.2.13] - 2025-06-25
### Changed
- Add --reset-user to test args

## [2.2.12] - 2025-05-31
### Changed
- Use default nucleus server for all tests

## [2.2.11] - 2025-05-28
### Changed
- added new semantics API check
- warning messages are now prefixed with [SDR][Warn] to make it clear that they are recorder related

## [2.2.10] - 2025-05-19
### Changed
- Update copyright and license to apache v2.0

## [2.2.9] - 2025-05-10
### Changed
- Enable FSD in test settings

## [2.2.8] - 2025-04-09
### Changed
- Update all test args to be consistent

## [2.2.7] - 2025-04-04
### Changed
- Version bump to fix extension publishing issues

## [2.2.6] - 2025-03-31
### Changed
- Window added to menu using MenuHelperWindow

## [2.2.5] - 2025-03-26
### Changed
- Cleanup and standardize extension.toml, update code formatting for all code

## [2.2.4] - 2025-03-04
### Changed
- Update to kit 107.1 and fix build issues

## [2.2.3] - 2025-01-27
### Changed
- Updated docs link

## [2.2.2] - 2025-01-21
### Changed
- Update extension description and add extension specific test settings

## [2.2.1] - 2024-10-28
### Fixed
- Added orchestrator.preview call before recording to make sure all frames are written if the timeline is already running

### Added
- Added tests to check the number of frames written

## [2.2.0] - 2024-10-28
### Changed
- Split the recorder functionality into a separate class (synthetic_recorder.py)
- Switched to only warn users when using large resolutions

### Added
- Added basic recorder test

## [2.1.0] - 2024-10-28
### Fixed
- moved from deprecated editor_menu.add_item to omni.kit.menu.utils

### Changed
- Removed unused "omni.pip.cloud" dependency
- Moved recorder to separate window ui class (synthetic_recorder_window.py)

## [2.0.1] - 2024-10-24
### Changed
- Updated dependencies and imports after renaming

## [2.0.0] - 2024-09-23
### Changed
- Extension  renamed to isaacsim.replicator.synthetic_recorder.

## [1.8.1] - 2024-07-31
### Fixed
- Fix deprecation warning and use Semantics instead of pxr.Semantics

## [1.8.0] - 2024-06-18
### Changed
- Capture loop is now using orhcestrator.step in a loop instead of orchestrator.run gueranteeing that the recorder is always in sync with the simulation (useful if custom or exernal randomizations are changing the stage)

### Added
- Verbose checkbox to print the recorder status (e.g. current frame number) to the console

## [1.7.3] - 2024-05-22
### Added
- async delayed ui frame build

## [1.7.2] - 2024-05-21
### Fixed
- writing to s3 by using only folder name instead of full path (ISIM-1133)
- disabled capture on play to fix timeline control issues
- removed step_async WAR (ISIM-943)

## [1.7.1] - 2024-04-23
### Fixed
- using step_async when num_frames=1 (ISIM-943 WAR)

## [1.7.0] - 2023-12-05
### Changed
- removed overwrite/increment/timestamp options from recorder, it is now handled by the backend which is not exposed

### Fixed
- fixed bug if a non valid json file is provided to the custom writer

## [1.6.0] - 2023-08-10
### Added
- custom names for render products
- render products are destroyed after each recording (OM-87164)
- select all and toggle all buttons for writer annotator parameters

### Changed
- increased render product max resolution to 16kx8k

## [1.5.2] - 2023-08-03
### Fixed
- omni.pip.cloud dependency is needed for replicator.core deps

## [1.5.1] - 2023-07-13
### Fixed
- load_config checks for empty json files

## [1.5.0] - 2023-03-13
### Added
- pointcloud_include_unlabelled parameter for pointcloud data

## [1.4.2] - 2023-02-22
### Fixed
- added wait_until_complete for S3 bucket writing one frame less (OM-82465)
- S3 accepting None values for s3_region and s3_endpoint

## [1.4.1] - 2023-02-14
### Fixed
- Synthetic recorder should only subscribe to the type of stage event it needs

## [1.4.0] - 2023-02-07
### Added
- Timeline Control

### Fixed
- Incremental folder naming not supported with S3 (OM-80864)

## [1.3.0] - 2023-01-25
### Removed
- Manual Control UI

### Changed
- Switched to async functions from Replicator 1.7.1 API

## [1.2.2] - 2023-01-06
### Fixed
- onclick_fn warning when creating UI

## [1.2.1] - 2023-01-04
### Fixed
- clean-ups on on_shutdown

## [1.2.0] - 2022-12-09
### Fixed
- Empty strings are no loger saved to config files

### Added
- Refresh path strings to default

## [1.1.1] - 2022-12-06
### Fixed
- Menu toggle value when user closes the window

## [1.1.0] - 2022-11-30
### Added
- Support for loading custom writers

### Changed
- renamed extension.py to synthetic_recorder_extension.py
- renamed extension class from Extension to SyntheticRecorderExtension

### Fixed
- Annotators blocking other annotators of writing data if their requirements are not met

## [1.0.0] - 2022-11-14
### Added
- version using Replicator OV API

## [0.1.2] - 2022-09-07
### Fixed
- Fixes for kit 103.5

## [0.1.1] - 2022-08-02
### Fixed

- Error message when there was no instance data to write

## [0.1.0] - 2021-08-11
### Added
- Initial version of Isaac Sim Synthetic Data Recorder Extension
- Records RGB, Depth, Semantic and Instance segmentation, 2D Tight and Loose bounding box
- Supports multi-viewport recording
