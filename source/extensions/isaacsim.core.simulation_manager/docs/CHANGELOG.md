# Changelog

## [1.3.2] - 2025-07-16
### Fixed
- Check that PhysX Scene API instances are valid before using them

## [1.3.1] - 2025-07-15
### Fixed
- Use carb.log_warn instead of carb.log_warning

## [1.3.0] - 2025-07-14
### Added
- Refactor callback management system to use a dictionary of callback handles
- Allow defult callbacks to be enabled/disabled individually
- Add new callback management APIs: enable_all_default_callbacks, is_default_callback_enabled, get_default_callback_status

## [1.2.7] - 2025-07-07
### Fixed
- Correctly enable omni.kit.loop-isaac in test dependency (fixes issue from 1.2.6)

## [1.2.6] - 2025-07-03
### Changed
- Make omni.kit.loop-isaac an explicit test dependency

## [1.2.5] - 2025-06-30
### Changed
- Omit testing simulation manager interface on ETM as physics steps are handled differently.

## [1.2.4] - 2025-06-25
### Changed
- Add --reset-user to test args

## [1.2.3] - 2025-06-16
### Fixed
- SimulationManager.set_default_physics_scene looks up the physics scene prim path provided in the stage in case it wasn't caught with the stage open event and usd notices.

## [1.2.2] - 2025-06-05
### Fixed
- Added a warning if warmup is not enabled when calling SimulationManager.set_default_physics_scene

## [1.2.1] - 2025-06-03
### Changed
- Removed debugging prints

## [1.2.0] - 2025-06-03
### Changed
- Disable physics warmup/ initialize to be triggered on play when /app/player/playSimulations=False

### Added
- SimulationManger.initialize_physics() for users to control when should initialization happen which will execute 2 physics steps internally

## [1.1.2] - 2025-06-02
### Fixed
- Add physics scene prim paths in the tracked physics scenes in SimulationManager when openning a usd asset.

## [1.1.1] - 2025-05-31
### Changed
- Use default nucleus server for all tests

## [1.1.0] - 2025-05-30
### Changed
- Added support for in memory stages

## [1.0.1] - 2025-05-19
### Changed
- Update copyright and license to apache v2.0

## [1.0.0] - 2025-05-16
### Changed
- Add time acquisition related features to this extension
- getSystemTime, getSimulationTimeMonotonic
- getSimulationTimeAtTime, getSimulationTimeMonotonicAtTime, getSystemTimeAtTime
- Add python bindings for new APIs

## [0.4.5] - 2025-05-16
### Changed
- Remove timeline commit from physx callback

## [0.4.4] - 2025-05-11
### Changed
- Enable FSD in test settings

## [0.4.3] - 2025-05-10
### Changed
- Remove internal build time dependency

## [0.4.2] - 2025-05-07
### Changed
- switch to omni.physics interface

## [0.4.1] - 2025-04-30
### Changed
- Update event subscriptions to Event 2.0 system

## [0.4.0] - 2025-04-13
### Added
- Added get_simulation_time, get_num_physics_steps, step, set_physics_dt, enable_ccd and is_ccd_enabled in SimulationManager
- Added order and name args to SimulationMangager.register_callback
- Added POST_PHYSICS_STEP and PRE_PHYSICS_STEP to IsaacEvents
- Added is_simulating and is_paused apis to SimulationManager

### Removed
- Removed PHYSICS_STEP from IsaacEvents

## [0.3.13] - 2025-04-07
### Added
- Instantiate an internal physics simulation view (Warp frontend) for the experimental implementations

## [0.3.12] - 2025-04-04
### Changed
- Version bump to fix extension publishing issues

## [0.3.11] - 2025-03-26
### Changed
- Cleanup and standardize extension.toml, update code formatting for all code

## [0.3.10] - 2025-03-26
### Changed
- CCD is not supported when using a cuda device, CCD is now automatically disabled if a cuda device is requested.

## [0.3.9] - 2025-03-24
### Changed
- Migrate to Events 2.0

## [0.3.8] - 2025-03-20
### Changed
- Improve doxygen docstrings

## [0.3.7] - 2025-03-05
### Changed
- Update extension codebase to adhere to isaac sim extension structure and file naming  guidelines

## [0.3.6] - 2025-03-04
### Changed
- Update to kit 107.1 and fix build issues

## [0.3.5] - 2025-02-21
### Changed
- Update style format and naming conventions in c++ code, add doxygen docstrings

## [0.3.4] - 2025-01-28
### Fixed
- Windows signing issue

## [0.3.3] - 2025-01-21
### Changed
- Update extension description and add extension specific test settings

## [0.3.2] - 2024-12-04
### Fixed
- Fixed access to invalid Fabric cache Id

## [0.3.1] - 2024-11-25
### Added
- SimulationManager.enable_fabric_usd_notice_handler method to enable/disable fabric USD notice handler.
- SimulationManager.is_fabric_usd_notice_handler_enabled method to query whether fabric USD notice handler is enabled.

## [0.3.0] - 2024-11-15
### Added
- SimulationManager.assets_loading method to query if textures finished loading.

### Changed
- SimulationManager's default backend with gpu pipelines to torch.

## [0.2.1] - 2024-11-08
### Changed
- Changed testing init file

## [0.2.0] - 2024-11-07
### Added
- Changed C++ plugin to follow the naming guidelines.

## [0.1.0] - 2024-10-31
### Added
- Initial release
