# Changelog

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
- when headless is set to true, the UI is hidden for performance, hide_ui can be set to false to re-enable the gui

## [1.8.1] - 2024-05-01
### Fixed
- update for set_phase api change

## [1.8.0] - 2024-04-29
### Added
- max_gpu_count config argument

### Fixed
- benchmark services include

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
- various linter issues

### Added
- Faulthandler enabled to print callstack on crash

## [1.4.2] - 2023-06-21
### Fixed
- app framework not working in docker/root environments
- simulation app startup warning

## [1.4.1] - 2023-02-22
### Added
- make sure replicator is stopped before calling wait_until_complete on closing application

## [1.4.0] - 2023-02-13
### Added
- add minimal app framework class

## [1.3.0] - 2023-02-07
### Changed
- call replicator wait_until_complete on closing application

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
- fast shutdown config option

### Fixed
- issue where fast shutdown caused jupyter notebooks to crash

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

- deprecated memory report in favor of using statistics logging utility

## [0.1.10] - 2022-06-13
### Added
- added physics device parameter for setting CUDA device for GPU physics simulation

## [0.1.9] - 2022-04-27
### Changed
- a .kit experience file can now reference other .kit files from the apps folder

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
- automatically add allow-root if running as root user

## [0.1.3] - 2021-12-21
### Changed
- Simulation App starts in cm instead of m to be consistent with the rest of isaac sim.

## [0.1.2] - 2021-12-07
### Added
- reset_render_settings API to reset render settings after loading a stage.
- fix docstring for antialiasing

## [0.1.1] - 2021-11-30
### Changed
- Remove isaacsim.core.api and omni.physx dependency
- Changed shutdown print statements to make them consistent with startup

## [0.1.0] - 2021-11-30
### Changed
- Tagged Initial version of SimulationApp
