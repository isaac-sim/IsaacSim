# Changelog

## [2.18.3] - 2026-05-20
### Added
- Added `MinimalRendering` renderer support in `SimulationApp` via the `renderer` launch config option
- Added `minimal_shading_mode` launch config option to set `/rtx/minimal/mode` when using Minimal rendering

## [2.18.2] - 2026-05-17
### Fixed
- `SimulationApp.close()` now flushes Python stdout/stderr before shutdown paths that can terminate the process through fast shutdown, preventing piped test output from being dropped.
- `SimulationApp.close(exit_code=...)` now preserves nonzero script/test failure status when fast shutdown is enabled, removing the need for per-test `os._exit` hooks.

## [2.18.1] - 2026-04-20
### Removed
- Removed optional multitick support; when multitick is enabled, time now routes through Fabric prim via SimulationManager.

## [2.18.0] - 2026-04-13
### Removed
- Removed direct telemetry calls from SimulationApp startup

## [2.17.2] - 2026-04-11
### Fixed
- Replaced `shutdown_and_release_framework()` with `app.shutdown()` in close() to avoid a GIL deadlock where the main thread held the GIL while `carb.tasking` worker threads waited for it during plugin teardown (NVBug 5948099)
- Replaced deprecated `isaacsim.core.utils.carb.get_carb_setting` import with direct `carb.settings` API

### Removed
- Removed shutdown watchdog (no longer needed now that close() avoids the deadlock-prone framework release path)

## [2.17.1] - 2026-04-02
### Added
- Add optional multitick support. When enabled, loop runner resets simulation time to 0.0 on SimulationApp.__init__.

### Changed
- Atexit handler now calls close(wait_for_replicator=False) to avoid hanging during interpreter shutdown
- Added forked watchdog process in close() that sends SIGKILL on timeout to prevent indefinite hangs from native thread deadlocks
- Unload plugins before os._exit() in fast_shutdown path to avoid glibc destructor deadlocks

## [2.17.0] - 2026-03-23
### Added
- Emit telemetry event for app startup duration via isaacsim.core.telemetry

### Changed
- Updated return types

## [2.16.1] - 2026-03-07
### Changed
- Automatically close the application during interpreter shutdown if close() was not called

## [2.16.0] - 2026-03-04
### Changed
- Added Overview.md and python_api.md and updated docstrings

## [2.15.3] - 2026-02-20
### Changed
- Close stage in simulation app close() method to avoid errors

## [2.15.2] - 2026-02-16
### Changed
- Update error message when application fails to start and exit before proceeding to make debugging easier.

## [2.15.1] - 2026-02-13
### Changed
- Fix issue where simulation app close() method would hang if the app was already stopped

## [2.15.0] - 2026-02-11
### Added
- Added separate default render settings for PathTracing and RealTimePathTracing modes

## [2.14.5] - 2026-01-29
### Changed
- Skip explicit stage close in simulation app close() method to avoid crashes

## [2.14.4] - 2026-01-22
### Changed
- is_running method does not require an active USD stage

## [2.14.3] - 2026-01-19
### Changed
- Use close_stage_async method when closing stage to avoid blocking the main thread if available

## [2.14.2] - 2026-01-15
### Changed
- Simulation app close() method now waits for replicator workflows to complete even when using replicator step()

## [2.14.1] - 2025-12-11
### Changed
- Increased MAX_FRAMES in _wait_for_viewport for Windows so NEW_FRAME event fires when expected (again)

## [2.14.0] - 2025-12-10
### Changed
- Change startup behavior so that app ready status is delayed until after the app has started

## [2.13.2] - 2025-12-09
### Changed
- Increased MAX_FRAMES in _wait_for_viewport for Windows so NEW_FRAME event fires when expected

## [2.13.1] - 2025-11-27
### Changed
- Add missing docstrings

## [2.13.0] - 2025-11-21
### Changed
- Change default renderer to RealTimePathTracing

### Added
- Add carb settings for RealTimePathTracing mode

## [2.12.3] - 2025-10-23
### Added
- Fix create_new_stage not working correctly

## [2.12.2] - 2025-09-20
### Fixed
- Fix hang on shutdown by forcing the current stage to close
- Fix issues where simulation app viewport was not fully initialized on startup
- Fix issue where experience could not be None

## [2.12.1] - 2025-09-15
### Added
- Add builtins flag to indicate that the `SimulationApp` class has been launched

## [2.12.0] - 2025-09-10
### Changed
- Added skip_cleanup parameter to close method
- Update docstrings

### Fixed
- Fix hang on startup when waiting for viewport to be ready
- Fix for hang on exit

## [2.11.0] - 2025-09-10
### Fixed
- Force headless mode if DISPLAY environment variable is not set on Linux

## [2.10.1] - 2025-09-08
### Fixed
- Update `SimulationApp` class docstrings to clarify the default behavior when loading experience files

## [2.10.0] - 2025-08-04
### Added
- New 'enable_motion_bvh' config to enable Motion BVH settings via extra_args

## [2.9.2] - 2025-06-19
### Changed
- Remove replicator shutdown workaround
- Add timeout to prevent infinite loop when waiting for USD resource operations to complete on shutdown

## [2.9.1] - 2025-06-18
### Changed
- Optimize shutdown to avoid waiting for replicator to finish writing when it is already stopped

## [2.9.0] - 2025-06-09
### Added
- Add `run_coroutine` method to allow running coroutines using the Kit's asynchronous task engine.

## [2.8.2] - 2025-06-02
### Removed
- No longer set /rtx-default settings

## [2.8.1] - 2025-05-19
### Changed
- Update copyright and license to apache v2.0

## [2.8.0] - 2025-05-17
### Changed
- "/app/player/useFixedTimeStepping" is not set to false by default, set it in your .kit/experience file when using SimulationApp

## [2.7.0] - 2025-05-02
### Added
- Add option to disable viewport updates when running in headless mode

## [2.6.1] - 2025-04-04
### Changed
- Version bump to fix extension publishing issues

## [2.6.0] - 2025-03-31
### Added
- Add Kit args to limit cpu thread count to enhance performance by limiting unnecessary context switching

## [2.5.1] - 2025-03-26
### Changed
- Cleanup and standardize extension.toml, update code formatting for all code

## [2.5.0] - 2025-02-05
### Changed
- Added enable_crashreporter argument that if true will provide crash dumps if the application crashes, default true. Previously when a crash occurred only the outer python process provided crash information.

## [2.4.2] - 2025-01-21
### Changed
- Update extension description and add extension specific test settings

## [2.4.1] - 2024-12-06
### Fixed
- Signal handler to exit cleanly when ctrl-c is pressed

## [2.4.0] - 2024-11-26
### Removed
- Remove livesync_usd entry from SimulationApp configuration

## [2.3.1] - 2024-11-14
### Changed
- Updated default experience list

## [2.3.0] - 2024-11-13
### Changed
- Add extra_args config argument that allows a user to pass in additional arguments from a script

## [2.2.0] - 2024-11-10
### Changed
- Try and load multiple kit files for python app based on what exists

## [2.1.0] - 2024-10-25
### Added
- New config arg "create_new_stage" to create empty stage on startup

## [2.0.3] - 2024-10-24
### Changed
- Updated dependencies and imports after renaming

## [2.0.2] - 2024-10-15
### Fixed
- Rename RayTracedLighting to RaytracedLighting

## [2.0.1] - 2024-10-02
### Changed
- Update dependency omni.isaac.version to isaacsim.core.version

## [2.0.0] - 2024-09-24
### Changed
- Extension renamed to isaacsim.simulation_app

## [1.13.2] - 2024-09-08
### Fixed
- Error on exit caused by stage closure while sdg was finishing writing

## [1.13.1] - 2024-08-03
### Changed
- Added missing tracy parameters to reduce profiling bias on measured performance

## [1.13.0] - 2024-07-16
### Changed
- omni.isaac.version, omni.ui, omni.kit.window.title are not required to use SimulationApp in headless mode

## [1.12.0] - 2024-06-26
### Added
- --ovd="/path/to/capture/" argument that simplifies capturing of physics debugging data

## [1.11.0] - 2024-05-28
### Added
- profiler_backend setting for tracy and nvtx

## [1.10.0] - 2024-05-22
### Added
- SIGINT handler to SimulationApp to force exit when ctrl-c is pressed

## [1.9.0] - 2024-05-13
### Added
- hide_ui to Simulation App to force ui visibility

### Changed
- When headless is set to true, the UI is hidden for performance, hide_ui can be set to false to re-enable the gui

## [1.8.1] - 2024-05-01
### Fixed
- Update for set_phase api change

## [1.8.0] - 2024-04-29
### Added
- max_gpu_count config argument

### Fixed
- Benchmark services include

## [1.7.0] - 2024-04-22
### Changed
- Don't terminate Isaac Sim if the 'isaacsim' module is not imported

## [1.6.3] - 2024-02-29
### Added
- Benchmark metadata

### Changed
- Updated benchmark set_phase() call to correctly record startup time after removing deprecated API

### Removed
- Deprecated benchmark stop_runtime() call

## [1.6.2] - 2024-02-27
### Changed
- Make error message about import isaacsim clearer

## [1.6.1] - 2024-02-26
### Fixed
- Missing app icon issue

## [1.6.0] - 2024-02-22
### Changed
- Rename isaac_sim import statement to isaacsim

## [1.5.2] - 2024-02-14
### Fixed
- Import isaac_sim error in running instance

## [1.5.1] - 2024-01-31
### Fixed
- Crash on exit when using tracy

## [1.5.0] - 2024-01-30
### Changed
- Measures startup time if omni.isaac.benchmark.services is loaded.

## [1.4.7] - 2023-12-13
### Fixed
- set_live_sync method

## [1.4.6] - 2023-11-27
### Changed
- Set /app/player/useFixedTimeStepping to False since the loop runner controls stepping

## [1.4.5] - 2023-10-16
### Changed
- Add flag to SimulationApp close to skip replicator wait for complete

## [1.4.4] - 2023-10-06
### Fixed
- Fix potential error on Kit shutdown when Replicator capture on play is enabled

## [1.4.3] - 2023-08-22
### Fixed
- Missing comma in sync load options
- Various linter issues

### Added
- Faulthandler enabled to print callstack on crash

## [1.4.2] - 2023-06-21
### Fixed
- App framework not working in docker/root environments
- Simulation app startup warning

## [1.4.1] - 2023-02-22
### Added
- Make sure replicator is stopped before calling wait_until_complete on closing application

## [1.4.0] - 2023-02-13
### Added
- Add minimal app framework class

## [1.3.0] - 2023-02-07
### Changed
- Call replicator wait_until_complete on closing application

## [1.2.3] - 2023-01-20
### Fixed
- Startup warnings

## [1.2.2] - 2023-01-18
### Fixed
- Error when viewport extension was not loaded

## [1.2.1] - 2022-12-11
### Fixed
- Error message when closing stage before closing simulation app

## [1.2.0] - 2022-10-25
### Changed
- Prepare UI focuses on content tab and hides samples to improve startup times.

## [1.1.0] - 2022-10-14
### Added
- Fast shutdown config option

### Fixed
- Issue where fast shutdown caused jupyter notebooks to crash

## [1.0.2] - 2022-10-03
### Fixed
- Fixes for kit 104.0

## [1.0.1] - 2022-10-02
### Fixed
- Crash when closing

## [1.0.0] - 2022-09-12
### Removed
- memory_report config flag

## [0.2.1] - 2022-07-25
### Added
- Increase hang detection timeout (OM-55578)

## [0.2.0] - 2022-06-22
### Deprecated

- Deprecated memory report in favor of using statistics logging utility

## [0.1.10] - 2022-06-13
### Added
- Added physics device parameter for setting CUDA device for GPU physics simulation

## [0.1.9] - 2022-04-27
### Changed
- A .kit experience file can now reference other .kit files from the apps folder

## [0.1.8] - 2022-04-13
### Fixed
- Comment in simulation_app.py

## [0.1.7] - 2022-03-31
### Fixed
- Dlss is now loaded properly on startup

## [0.1.6] - 2022-03-24
### Added
- Multi gpu flag to config

### Changed
- Make startup/close logs timestamped

## [0.1.5] - 2022-02-22
### Added
- Windows support

## [0.1.4] - 2022-01-27
### Added
- memory_report to launch config. The delta memory usage is printed when the app closes.
- Automatically add allow-root if running as root user

## [0.1.3] - 2021-12-21
### Changed
- Simulation App starts in cm instead of m to be consistent with the rest of isaac sim.

## [0.1.2] - 2021-12-07
### Added
- reset_render_settings API to reset render settings after loading a stage.
- Fix docstring for antialiasing

## [0.1.1] - 2021-11-30
### Changed
- Remove isaacsim.core.api and omni.physx dependency
- Changed shutdown print statements to make them consistent with startup

## [0.1.0] - 2021-11-30
### Changed
- Tagged Initial version of SimulationApp
