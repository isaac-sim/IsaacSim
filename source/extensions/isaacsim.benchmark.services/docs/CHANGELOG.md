# Changelog
## [3.0.5] - 2025-07-07
### Fixed
- Correctly enable omni.kit.loop-isaac in test dependency (fixes issue from 3.0.4)

## [3.0.4] - 2025-07-03
### Changed
- Make omni.kit.loop-isaac an explicit test dependency

## [3.0.3] - 2025-06-25
### Changed
- Add --reset-user to test args

## [3.0.2] - 2025-06-18
### Added
- Enabled multi-gpu frametime collection and reporting + average GPU frametime in mGPU setups

## [3.0.1] - 2025-06-18
### Removed
- Removed unused CPU load metrics

## [3.0.0] - 2025-06-16
### Added
- Created validation tooling for render product evaluation

## [2.8.5] - 2025-06-09
### Changed
- Removed unneeded default settings for omni connection

## [2.8.4] - 2025-05-31
### Changed
- Use default nucleus server for all tests

## [2.8.3] - 2025-05-19
### Changed
- Update copyright and license to apache v2.0

## [2.8.2] - 2025-05-10
### Changed
- Enable FSD in test settings

## [2.8.1] - 2025-05-08
### Changed
- Moved main app loop frametime callback to preUpdate event for increased consistency

## [2.8.0] - 2025-04-28
### Changed
- GPU frametime collection is off by default, enabled by config in base_isaac_benchmark

## [2.7.0] - 2025-04-25
### Added
- Added callback for render thread when async rendering is enabled
- Added new field: Rendering FPS for async rendering cases

## [2.6.8] - 2025-04-09
### Changed
- Update all test args to be consistent

## [2.6.7] - 2025-04-04
### Changed
- Version bump to fix extension publishing issues

## [2.6.6] - 2025-03-26
### Changed
- Cleanup and standardize extension.toml, update code formatting for all code

## [2.6.5] - 2025-03-24
### Changed
- Migrate to Events 2.0

## [2.6.4] - 2025-03-11
### Changed
- Switch asset root for tests to internal nucleus

## [2.6.3] - 2025-01-26
### Changed
- Update test settings

## [2.6.2] - 2025-01-21
### Changed
- Update extension description and add extension specific test settings

## [2.6.1] - 2025-01-10
### Fixed
- Division-by-zero error when benchmarked script does not run app update

## [2.6.0] - 2025-01-06
### Changed
- Conditionally enable async benchmark if not running as standalone workflow

## [2.5.0] - 2025-01-02
### Removed
- Remove utils.SyncMode, utils.SampledScope

## [2.4.0] - 2024-11-26
### Added
- Added function to compute frametime metrics statistics using the middle 80% of data points

## [2.3.0] - 2024-11-26
### Added
- Added summary report generation for formatted KPI data

## [2.2.0] - 2024-11-15
### Changed
- Use subscription method to get physics simulation profiling data

## [2.1.0] - 2024-11-03
### Added
- Added OmniPerfKPIFile backend and made default option

## [2.0.3] - 2024-10-29
### Changed
- Updated dependencies and imports after renaming

## [2.0.2] - 2024-10-28
### Changed
- Remove test imports from runtime

## [2.0.1] - 2024-10-24
### Changed
- Updated dependencies and imports after renaming

## [2.0.0] - 2024-09-20
### Changed
- Extension renamed to isaacsim.benchmark.services

## [1.10.2] - 2024-09-09
### Fixed
- Physics time not captured unless app update was called

## [1.10.1] - 2024-09-05
### Fixed
- convert sim_elapsed_time to ms for calculating real-time factor

## [1.10.0] - 2024-08-05
### Changed
- use omni.physdx.get_physx_benchmarks_interface() to get physics simulation profiling data

## [1.9.0] - 2024-07-17
### Changed
- omni.hydra is not a required dependency, but is needed to collect memory stats
- omni.kit.test is a required dependency, not just for running tests

### Removed
- removed unused omni.kit.profiler.window dependency

## [1.8.3] - 2024-07-15
### Added
- Ability to store custom measurements in a specified phase

## [1.8.2] - 2024-07-02
### Changed
- Use psutil to count CPU cores.

## [1.8.1] - 2024-06-25
### Removed
- Stray print statement in IsaacFrameTimeRecorder

## [1.8.0] - 2024-06-07
### Changed
- Always capture physics metrics even if stepping without rendering

## [1.7.0] - 2024-05-01
### Changed
- Removes dependency on isaacsim.core.nodes

## [1.6.0] - 2024-05-01
### Added
- ability to enable/disable frametime and runtime separately when starting a phase

## [1.5.0] - 2024-04-29
### Changed
- cleaned up imports
- updated docstrings
- removed unused functions

## [1.4.2] - 2024-04-15
### Fixed
- Error when zero frames were collected
- Test failure

## [1.4.1] - 2024-03-25
### Fixed
- Frametime collector skips frametime collection if start time is `None` (i.e if `set_phase(start_recording_time=False)` is called

## [1.4.0] - 2024-02-02
### Added
- User can now specify per-benchmark metadata when using BaseIsaacBenchmark, which will persist across phases

### Changed
- Refactors "middleware" in extension
- OsmoKPIFile backend now prints one KPI file per phase, rather than one KPI file per workflow
- Deprecates individual runtime/frametime APIs in BaseIsaacBenchmark, moves functionality to start_phase() and stop_collecting_measurements()
- metrics.measurements.TestRun renamed to metrics.measurements.TestPhase

## [1.3.2] - 2024-02-02
### Changed
- OSMOKPIFile writer logs exact KPI keys, rather than abbreviation.

## [1.3.1] - 2024-02-02
### Changed
- Updated path to the nucleus extension

## [1.3.0] - 2024-01-30
### Added
- IsaacStartupTimeRecorder measures startup time, collected only during "startup" phase

## [1.2.0] - 2024-01-25
### Added
- OsmoKPIFile writer logs KPIs to console.

## [1.1.0] - 2024-01-19
### Added
- Adds new BaseIsaacBenchmark for non-async benchmarking
- Adds new standalone examples for non-async benchmarking
- Adds OsmoKPIFile writer to publish KPIs compatible with OSMO's Kratos backend

### Fixed
- Async benchmark stability on new stage open
- ROS2 bridge camera helper

### Changed
- Move BaseIsaacBenchmark -> BaseIsaacBenchmarkAsync, for (eg) async unit tests

## [1.0.0] - 2024-01-04
### Changed
- Remove depdencies and added classes needed to make benchmarks work independently
- Helpers specific to certain benchmarks were moved to benchmarks extension

## [0.2.0] - 2023-12-14
### Added
- Added ROS2 camera graph helper

## [0.1.1] - 2023-11-30
### Changed
- Use get_assets_root_path_async()

## [0.1.0] - 2023-11-14
### Changed
- Moved utils from omni.isaac.benchmark into omni.isaac.benchmark.services

### Added
- Quasi-initial version
