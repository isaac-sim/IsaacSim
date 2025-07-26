# Changelog
## [1.1.12] - 2025-07-07
### Fixed
- Correctly enable omni.kit.loop-isaac in test dependency (fixes issue from 1.1.11)

## [1.1.11] - 2025-07-03
### Changed
- Make omni.kit.loop-isaac an explicit test dependency

## [1.1.10] - 2025-06-25
### Changed
- Add --reset-user to test args

## [1.1.9] - 2025-06-18
### Changed
- Add scene path setting to extension.toml
- Make default test run for a single frame with an empty stage

## [1.1.8] - 2025-05-31
### Changed
- Use default nucleus server for all tests

## [1.1.7] - 2025-05-19
### Changed
- Update copyright and license to apache v2.0

## [1.1.6] - 2025-05-10
### Changed
- Enable FSD in test settings

## [1.1.5] - 2025-04-09
### Changed
- Update all test args to be consistent

## [1.1.4] - 2025-04-04
### Changed
- Version bump to fix extension publishing issues

## [1.1.3] - 2025-03-26
### Changed
- Cleanup and standardize extension.toml, update code formatting for all code

## [1.1.2] - 2025-03-11
### Changed
- Switch asset root for tests to internal nucleus

## [1.1.1] - 2025-03-04
### Changed
- Update to kit 107.1 and fix build issues

## [1.1.0] - 2025-02-13
### Changed
- Add setting to set the number of app updates for the test exts."isaacsim.benchmark.examples".num_app_updates

## [1.0.4] - 2025-01-26
### Changed
- Update test settings

## [1.0.3] - 2025-01-21
### Changed
- Update extension description and add extension specific test settings

## [1.0.2] - 2024-10-28
### Changed
- Remove test imports from runtime

## [1.0.1] - 2024-10-24
### Changed
- Updated dependencies and imports after renaming

## [1.0.0] - 2024-09-20
### Changed
- Extension renamed to isaacsim.benchmark.examples

## [0.23.0] - 2024-07-15
### Changed
- Remove additional redundant benchmarks now covered by standalone scripts

## [0.22.0] - 2024-05-01
### Changed
- Remove redundant benchmarks that are already covered by standalone scripts

## [0.21.9] - 2024-04-29
### Fixed
- Benchmark service includes

## [0.21.8] - 2024-02-29
### Changed
- Removes deprecated function calls from benchmarks.

## [0.21.7] - 2024-02-07
### Changed
- Updated path to the nucleus extension

## [0.21.6] - 2024-02-06
### Fixed
- O3dyn benchmark crash

## [0.21.5] - 2024-02-05
### Added
- Nova Carter ROS2 benchmark for 3 carters 1 3d lidar each for benchmarking multi-robot nav

## [0.21.4] - 2024-02-01
### Changed
- Changed get_assets_root_path to get_assets_root_path_async for the unit tests

## [0.21.3] - 2024-01-30
### Fixed
- Nova Carter ROS2 benchmark to use /cmd_vel commands rather than articulation control directly

## [0.21.2] - 2024-01-19
### Added
- Fix ROS2 camera helper benchmark issue

## [0.21.1] - 2024-01-18
### Added
- Added 1200p eight cameras benchmark for hawk cameras

## [0.21.0] - 2024-01-16
### Added
- Added Nova Carter Benchmarks (non ROS)

### Changed
- Renamed previous Nova Carter Benchmarks to Nova Carter ROS2

### Removed
- Removed Carter V1 benchmarks

## [0.20.0] - 2024-01-04
### Changed
- Moved helpers from benchmark services extension

## [0.19.0] - 2023-12-14
### Changed
- Now dependent on ROS 2
- Change ROS 1 camera benchmarks to ROS 2 cameras

### Added
- Added Nova Carter benchmarks

## [0.18.4] - 2023-11-15
### Changed
- Now dependent on omni.isaac.benchmark.services

### Removed
- Removed utils module since omni.isaac.benchmark.services is now a dependency

## [0.18.3] - 2023-10-24
### Changed
- Added --no-window and --allow-root by default for testing

## [0.18.2] - 2023-10-09
### Changed
- changed carter bench mark tests from Carter V2 to Carter V1

## [0.18.1] - 2023-08-21
### Changed
- duration of test_benchmark_rtx_radar test from 10 app updates to 600

## [0.18.0] - 2023-08-17
### Added
- test_benchmark_rtx_radar test

### Changed
- test_benchmark_rtx_lidar uses both Rotary and Solid_State to test

### Fixed
- test_benchmark_rtx_lidar crash when multiple tests run

## [0.17.4] - 2023-08-10
### Added
- SDG benchmark render product destruction (supported with Replicator 1.10.1)

## [0.17.3] - 2023-07-25
### Changed
- README to include information about reading a perflab report

## [0.17.2] - 2023-07-20
### Changed
- Renamed ROS camera benchmarks from _ros_1 to _ros1

## [0.17.1] - 2023-07-18
### Added
- Renamed benchmarks to make output easier to process

## [0.17.0] - 2023-07-13
### Added
- Created tests for O3dyn robots, renamed original test_benchmark_robots to test_benchmark_robots_carter

## [0.16.1] - 2023-07-13
### Changed
- Cleaned up benchmark code - unused imports, formatting, etc.

## [0.16.0] - 2023-07-11
### Added
- Test mode for TeamCity, which runs each benchmark for 1 frame instead of 600 frames, just to check if there are no bugs. To enable test mode, set environment variable ISAAC_TEST_MODE to 1.

## [0.15.0] - 2023-07-06
### Changed
- use sync stage load function to get better behavior
- reuse viewport rp for first camera.

## [0.14.0] - 2023-07-04
### Added
- added sync load parameters to setUp in base isaac benchmark class

### Changed
- Update ROS camera test to use render products
- Start ROSCore when running benchmarks

## [0.13.2] - 2023-07-04
### Added
- runtime and frametime recorder to sdg benchmark phase

## [0.13.1] - 2023-07-03
### Added
- moved wait_until_stage_is_fully_loaded_async to helper.py

### Fixed
- unrolled scene generation benchmark loops
- sdg using step_async loop + wait_until_complete_async to make sure data is written to disk in the benchmark phase

## [0.13.0] - 2023-06-30
### Added
- Real Time Factor (RTF) measurement to frametime recorder and benchmark specificially for RTF, used to compare time in simulator vs real time

## [0.12.0] - 2023-06-29
### Added
- ROS 1 camera benchmarks only appear when running Isaac Sim on Linux

## [0.11.0] - 2023-06-29
### Added
- Runtime metric for benchmarks, used to measure total load time in place of framerate data

## [0.10.0] - 2023-06-28
### Added
- Test for 10 robots with camera
- Test 10 robots with lidar and camera

## [0.9.0] - 2023-06-26
### Added
- Test for 100 PhysX Lidar sensors
- Tests for 1-50 robots with no sensor

## [0.8.2] - 2023-06-26
### Fixed
- Removed test loops, benchmark data was not exported because setUp and tearDown were not called for every test

### Changed
- SDG benchmark set default resolution to 720p
- SDG benchmark set writer to None for future replicator changes to cleanup

## [0.8.1] - 2023-06-21
### Fixed
- new (fixed) stage for SDG benchmark

## [0.8.0] - 2023-06-21
### Added
- rtx lidar benchmark

## [0.7.3] - 2023-06-21
### Fixed
- Scene generation crash on pre-existing prim transform attributes

## [0.7.2] - 2023-06-01
### Fixed
- Don't fail on error messages due to missing features on gpu

## [0.7.1] - 2023-05-25
### Changed
- SDG benchmark segment names

## [0.7.0] - 2023-05-18
### Changed
- Use camera class for camera scaling benchmark

### Removed
- PB metric from windows benchmarks

## [0.6.1] - 2023-05-16
### Added
- made SDG benchmark names more descriptive

## [0.6.0] - 2023-05-05
### Added
- phase label to cpu/memory metrics

## [0.5.0] - 2023-04-23
### Added
- Scene generation benchmark
- SDG generation benchmark

## [0.4.1] - 2022-11-17
### Fixed
- missing extensions

## [0.4.0] - 2022-11-16
### Added
- ROS camera benchmark tests
- RTX lidar benchmark tests

### Fixed
- deleting existing sensors/robots/cameras that's already on stage when new rounds of tests start

## [0.3.1] - 2022-10-28
### Changed
- Logging format

## [0.3.0] - 2022-10-24
### Added
- Multi-Robot, multi-robot with lidar, multi-robot with camera tests

## [0.2.0] - 2022-10-24
### Added
- Lidar Benchmark

### Fixed
- Camera Benchmark and logging bugs

## [0.1.0] - 2022-10-05
### Added
- Initial version
