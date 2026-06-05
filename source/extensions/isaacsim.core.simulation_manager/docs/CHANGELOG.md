# Changelog

## [1.15.4] - 2026-05-21
### Changed
- `PhysicsScene.get_dt()` and `set_dt()` dispatch based on `PhysxSceneAPI` presence on the prim instead of querying the active engine.
- Remove stale solver APIs (`PhysxSceneAPI`, `MjcSceneAPI`, `NewtonXpbdSceneAPI`) and re-create physics scene wrappers in `SimulationManager._on_engine_switched()` when switching engines.
- Stop unconditionally applying `PhysxSceneAPI` in C++ `UsdNoticeListener` and `PluginInterface`; the API is now applied by the engine-specific Python wrapper (`PhysxScene`).

## [1.15.3] - 2026-05-19
### Fixed
- Raise `RuntimeError` in `PhysicsScene.set_dt()`, `set_enabled_gravity()`, and `set_max_solver_iterations()` when `NewtonSceneAPI` is not applied, matching the `PhysxScene` error-handling pattern.

## [1.15.2] - 2026-05-17
### Fixed
- Restore mode-specific `TimeSampleStorage` handling for render-product reference times. In multitick mode, render-product reference times come from `/ExternalSimulationTime` and can be returned as simulation time directly. In non-multitick mode, render-product reference times come from the renderer's Fabric frame time, so samples are keyed with `IStageReaderWriter::getFrameTime()` and `getSimulationTimeAt()` resolves the simulation time through exact-match or adjacent-sample lookup.

## [1.15.1] - 2026-05-15
### Fixed
- Route base `PhysicsScene` timestep updates to the active physics engine so PhysX scenes do not author Newton timestep values that can hang playback.

## [1.15.0] - 2026-05-05
### Removed
- Removed non-multitick code path.

## [1.14.9] - 2026-05-01
### Changed
- Removed redundant `fetch_results()` after `simulate()` in warmup, simulation-view creation, and `SimulationManager.step()`.

## [1.14.8] - 2026-04-27
### Changed
- Remove USD-attribute workaround in `onPhysicsStep`; `IPhysicsSimulation::getSimulationTimeStepsPerSecond()` now returns the authoritative value, so read it directly.

## [1.14.7] - 2026-04-21
### Changed
- Route Newton simulation view creation through `omni.physics.tensors.create_simulation_view` with `backend="newton"` instead of the Python `isaacsim.physics.newton.tensors` implementation.

## [1.14.6] - 2026-04-21
### Added
- Add `"remotesim"` to `get_active_physics_engine()` and `switch_physics_engine()` return/parameter type hints
- Add `elif engine == "remotesim"` branch in `create_scene()` that returns a lightweight `PhysicsScene` wrapper

## [1.14.5] - 2026-04-20
### Changed
- Multi-tick simulation time is now communicated via the `/ExternalSimulationTime` Fabric prim instead.

## [1.14.4] - 2026-04-17
### Fixed
- Add missing `@staticmethod` decorator to 5 internal event callbacks (`_on_simulation_registry_event`, `_on_stage_opened`, `_on_stage_closed`, `_on_play`, `_on_stop`)
- Fix simulation time using stale steps-per-second from physics runtime when the rate is changed between stop/play cycles. Read the authoritative value from the USD `PhysxSceneAPI::timeStepsPerSecond` attribute instead of relying on `IPhysicsSimulation::getSimulationTimeStepsPerSecond()` which can return a cached value from the previous session.

## [1.14.3] - 2026-04-17
### Changed
- Update `PhysxScene` to use renamed PhysX attribute `GpuMaxDeformableVolumeContacts`

## [1.14.2] - 2026-04-07
### Added
- Add optional multitick support to TimeSampleStorage and PluginInterface. When multitick is enabled, physics time drives rendering time via onPhysicsStep callback.

## [1.14.1] - 2026-04-03
### Changed
- Use local `BindingsPythonUtils.h` from `isaacsim.core.includes` instead of `carb/BindingsPythonUtils.h`
- Refactored stage event subscription to be lazily initialized with null-safety checks for USD context and stage

## [1.14.0] - 2026-04-01
### Changed
- When available, use integer time steps per second and step count to compute simulation time in onPhysicsStep callback to eliminate accumulated bias due to floating-point timestep precision

## [1.13.1] - 2026-03-17
### Fixed
- Correct "broadcast" to "broadphase" in SimulationManager docstrings (collision broadphase algorithm)

## [1.13.0] - 2026-03-12
### Changed
- Added Overview.md, python_api.md and updated docstrings

## [1.12.0] - 2026-03-12
### Changed
- Remove Newton pip prebundle dependency

## [1.11.2] - 2026-03-02
### Changed
- Newton tests enabled; PhysX-specific tests skip when active engine is not PhysX.

## [1.11.1] - 2026-02-25
### Fixed
- Rebuild with new physics package

## [1.11.0] - 2026-02-06
### Changed
- Update deprecated Warp API calls to their updated names

### Added
- Add Newton physics engine support with `switch_physics_engine()` method
- Add `NewtonMjcScene` and `NewtonXpbdScene` classes for Newton solver-specific scene configuration
- Add `PhysicsScene` base class for common physics scene operations

## [1.10.1] - 2026-02-05
### Changed
- Make omni.isaac.ml_archive an explicit test dependency

## [1.10.0] - 2026-02-03
### Added
- When `/rtx/hydra/supportMultiTickRate` is enabled, simulation time is propagated to the run loop

## [1.9.2] - 2026-01-22
### Changed
- Replaced omni.physx start_simulation with omni.physics start_simulation
- Replaced omni.physx simulation event stream with omni.physics simulation event stream

## [1.9.1] - 2026-01-16
### Added
- Add `cleanupInvalidPhysicsScenes()` method to C++ `ISimulationManager` interface to remove tracked physics scenes with invalid prims
- Add `isValid()` method to C++ `PhysicsScene` class to check if the underlying prim is still valid
- Add Python binding for `cleanup_invalid_physics_scenes()` method

### Fixed
- Fix `RuntimeError: Accessed invalid expired 'PhysicsScene' prim` error when physics scene prims become invalid without triggering USD notices (e.g., layer removal operations, session sublayer changes)
- Add stage and root layer validation in `_on_play()` to prevent physics initialization on expired/invalid stages
- Add error handling in `_create_physics_scene()` to gracefully handle physics scene creation failures

## [1.9.0] - 2026-01-14
### Added
- Add supporting APIs for changing physics engines through the omniphysics interfaces

## [1.8.0] - 2026-01-09
### Added
- Add `PhysicsScene` and `PhysxScene` Python class wrappers for high-level physics scene manipulation
- Add `PhysicsScene` C++ class and header for USD Physics Scene prim operations
- Add `get_physics_scene_paths()` function to get all physics scene paths in a stage

## [1.7.3] - 2026-01-08
### Changed
- Change log level of no adjacent samples found for interpolation warning to INFO

## [1.7.2] - 2025-12-07
### Changed
- Run clang tidy

## [1.7.1] - 2025-12-02
### Changed
- Raise a RuntimeError if the physics dt is being set while simulation is running/playing

## [1.7.0] - 2025-11-26
### Added
- Add the `SimulationEvent` enum
- Allow to perform a fabric update when stepping physics

### Changed
- Mark as deprecated the `IsaacEvents` enum and the backend-related methods

## [1.6.2] - 2025-11-25
### Changed
- Make set_physics_dt a classmethod
- Add unit tests for SimulationManager

## [1.6.1] - 2025-11-07
### Changed
- Update to Kit 109 and Python 3.12

## [1.6.0] - 2025-10-27
### Changed
- Replace the use of deprecated core utils functions by the core experimental implementations

## [1.5.0] - 2025-10-17
### Changed
- Migrate PhysX subscription and simulation control interfaces to Omni Physics

## [1.4.4] - 2025-10-09
### Fixed
- Change log level of invalid rational time error to WARN
- Change log level of no data found for time warning to INFO

## [1.4.3] - 2025-09-19
### Fixed
- Reduce verbosity of log messages

## [1.4.2] - 2025-08-29
### Changed
- Renamed CARB profiling zones to include [IsaacSim] prefix

## [1.4.1] - 2025-08-22
### Fixed
- Always return current time if no samples are stored, this occurs if you access the time storage before starting simulation

## [1.4.0] - 2025-08-18
### Changed
- Backend time sampling is now done using a circular buffer that writes/reads rational time samples and properly works with FSD enabled/disabled.

### Fixed
- Duplicated/incorrect timestamp issue

## [1.3.4] - 2025-08-14
### Fixed
- Do not change /physics/outputVelocitiesLocalSpace when fabric is enabled/disabled

## [1.3.3] - 2025-08-08
### Fixed
- Update to event 2.0 system

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
- Switch to omni.physics interface

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
