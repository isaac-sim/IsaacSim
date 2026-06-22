# Changelog

## [1.11.6] - 2026-06-10
### Fixed
- Add warmup steps to motion blur example test to make it more stable in CI

## [1.11.5] - 2026-06-09
### Fixed
- Fix linter errors and missing or incomplete docstrings.

## [1.11.4] - 2026-05-21
### Changed
- Augmentation examples now use custom event-based randomization
- Renamed augmentation test tolerances to reflect noise vs no-noise comparisons

## [1.11.3] - 2026-05-20
### Changed
- Update test_data_augmentation to use different tolerance values for RGB and depth images

## [1.11.2] - 2026-05-18
### Added
- Added multiple captures with toggled render product while timeline is running test

## [1.11.1] - 2026-05-05
### Changed
- Enable multitick in all tests
- Update golden images for test_data_augmentation
- Tests now store results in uniquely-named temp directories to avoid run-to-run test pollution

## [1.11.0] - 2026-04-24
### Changed
- Migrated imports from `isaacsim.core.utils` to `isaacsim.core.experimental.utils`
- Removed `omni.isaac.ml_archive` and `isaacsim.core.experimental.utils` from test dependencies (now transitive via `isaacsim.replicator.writers`)
- Removed AMR navigation, object-based SDG, cosmos writer, randomizer snippets, and SimReady snippets tests (moved to snippet-level validation)

## [1.10.0] - 2026-04-20
### Changed
- Added SDG GeomSubset semantic segmentation test for `perSubsetSegmentation` true/false

## [1.9.5] - 2026-04-18
### Changed
- Added return type annotations and `__all__` definitions

## [1.9.4] - 2026-04-15
### Changed
- Add new isaacsim.cortex.examples test time dependency

## [1.9.3] - 2026-03-19
### Changed
- Amr navigation example test: added None to the default environment list which will use a simple ground plane environment

## [1.9.2] - 2026-03-18
### Changed
- SDG deformables test: comparing semantic segmentation labels instead of images

## [1.9.1] - 2026-03-10
### Changed
- Added stricter test tolerances for SDG deformables example test

## [1.9.0] - 2026-03-04
### Changed
- Added Overview.md, python_api.md and updated docstrings

## [1.8.0] - 2026-02-26
### Changed
- Added test for SDG getting started example with replicator enableWriteToFabric and wait_for_render options

## [1.7.2] - 2026-02-21
### Changed
- Fixed SDG physics based volume filling example test to use the omni.physx api

## [1.7.1] - 2026-02-19
### Changed
- SimulationManager device is set back to its default state following the deformable SDG example test

## [1.7.0] - 2026-02-08
### Changed
- Added SDG deformables example test

## [1.6.9] - 2026-02-04
### Changed
- Synched snippets with docs, typos and print statements fixes

## [1.6.8] - 2026-01-05
### Changed
- Augmentation examples use basic empty stage (dome light + ground plane) by default, with optional env_url for custom stages

## [1.6.7] - 2025-12-17
### Changed
- Added missing import for SDG getting started to be in sync with docs

## [1.6.6] - 2025-12-11
### Changed
- Improved simready assets SDG example output results

## [1.6.5] - 2025-12-09
### Changed
- Fixed sequential sphere scan randomizer example to work in script editor in sync with docs

## [1.6.4] - 2025-12-08
### Changed
- Added explicit `.reset()` to events 2.0 subscribers in sync with docs examples

## [1.6.3] - 2025-12-05
### Changed
- Migrate to Events 2.0.

## [1.6.2] - 2025-12-03
### Changed
- Added an app update after switching to pathtracing in the palletizing example test
- Fixed scatter plane parent path in scene based SDG example test
- Fixed SDG box stacking randomizer example test by waiting for the data to be written to disk

## [1.6.1] - 2025-11-27
### Changed
- Make consistent use of SimulationManager

## [1.6.0] - 2025-11-26
### Changed
- Added scene based SDG example test
- Added object based SDG example test
- Added AMR navigation example test
- Switched to RealtimePathTracing in the motion blur example

## [1.5.0] - 2025-10-28
### Changed
- Updated replicator examples to use replicator functional api where applicable
- Writers use explicit backends to write data to disk
- Changed data augmentation tests to use a fixed seed in the kernel functions as well, updated golden images
- UR10 palletizing example uses realtime pathtracing and backend for its writer
- Switched to core.experimental rigid prims where applicable
- Switched to SimulationManager instead of World

## [1.4.1] - 2025-10-27
### Changed
- Make omni.isaac.ml_archive an explicit test dependency

## [1.4.0] - 2025-10-17
### Changed
- Migrate PhysX subscription and simulation control interfaces to Omni Physics

## [1.3.6] - 2025-10-08
### Changed
- Added physics to custom fps example snippet, changed to dome light

## [1.3.5] - 2025-10-07
### Changed
- Updated subscribers and events example snippet to use a custom number of app updates for capturing events

## [1.3.4] - 2025-10-06
### Changed
- Set DLSS (`rtx/post/dlss/execMode`) to Quality mode (2) in the replicator tests
- Added `rt_subframes` to the custom fps example

## [1.3.3] - 2025-10-06
### Fixed
- Fixed data augmentation conversion issues

## [1.3.2] - 2025-09-24
### Changed
- Kept only arxiv paper link in sphere point distribution comments in `test_sdg_randomizer_snippets.py`

## [1.3.1] - 2025-09-23
### Changed
- Updated cosmos writer test example to generate multiple video clips

## [1.3.0] - 2025-09-18
### Changed
- Added cosmos writer tests

## [1.2.2] - 2025-09-15
### Changed
- Switched to isaacsim.test.utils for folder contents validation and goldenimage comparison in tests

## [1.2.1] - 2025-09-01
### Fixed
- Make sure custom writers reset annotators list (`self.annotators = []`) on initialization

## [1.2.0] - 2025-08-22
### Changed
- Moved randomizer snippets tests from `isaacsim.test.collection` to `test_sdg_randomizer_snippets.py`

### Fixed
- Fixed `get_shapes()` new semantics label check in texture randomization test

## [1.1.32] - 2025-08-21
### Changed
- Fix PIL image conversion warnings

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
- Increase rt_subframes in examples for more consistent results between updates

## [1.1.21] - 2025-05-23
### Changed
- Rename test utils.py to common.py

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
- Added test for starting capturing while the timeline is running
- Test utils.py for functions used in multiple tests

### Changed
- More verbose terminal outputs for tests

## [1.1.16] - 2025-05-12
### Changed
- Using os.path.join to create output directory in sdg tests

## [1.1.15] - 2025-05-10
### Changed
- Enable FSD in test settings

## [1.1.14] - 2025-05-10
### Changed
- Fixed timeline tests by making sure the timeline is stopped and the looping is set to its original value

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
- Changed add_update_semantics to add_labels

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
- Added UR10 palletizing demo test

## [1.0.1] - 2024-12-15
### Fixed
- Added fixed timestepping for consistent results for custom fps capture test

## [1.0.0] - 2024-12-09
### Added
- Created extension with example parts from omni.replicator.isaac
