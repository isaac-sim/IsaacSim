# Changelog

## [0.3.2] - 2026-04-28
### Fixed
- `RandomAccelerationScenario.step`: remove redundant `update_state()` call after `write_action()`; pose is unchanged until the next physics step.

## [0.3.1] - 2026-04-23
### Removed
- Remove the `omni.isaac.ml_archive` test dependency

## [0.3.0] - 2026-04-22
### Added
- `WheeledMultiSensorRobot` and `PolicyMultiSensorRobot` base classes for YAML-driven multi-camera robots
- `CarterMultiSensorRobot`, `JetbotMultiSensorRobot`, `H1MultiSensorRobot`, `SpotMultiSensorRobot` concrete robots
- YAML robot configs (`carter.yaml`, `jetbot.yaml`, `h1.yaml`, `spot.yaml`)
- `generate_sensor_rigs.py` script to discover sensor prims in a robot USD and scaffold `sensor_rig:` YAML blocks

## [0.2.2] - 2026-04-18
### Changed
- Added return type annotations, `from __future__ import annotations`, and imperative-mood docstrings

## [0.2.1] - 2026-03-19
### Changed
- Migrate to use `isaacsim.core.experimental.prims` (`Articulation`) in place of `isaacsim.core.prims`
- Force USD payload loading before `Articulation` initialization to ensure `ArticulationRootAPI` is visible

## [0.2.0] - 2026-03-04
### Changed
- Added Overview.md, python_api.md and updated docstrings

## [0.1.12] - 2026-02-06
### Changed
- Change occupancy map radius for Carter Robot

## [0.1.11] - 2025-12-10
### Changed
- Fix USD path for placement of front camera on Carter with latest USD asset
- Fix issue where H1 and Spot policies command must be provided as torch tensor

## [0.1.10] - 2025-10-27
### Changed
- Make omni.isaac.ml_archive an explicit test dependency

## [0.1.9] - 2025-09-30
### Fixed
- Add dialog message for incorrect occupancy map paths

## [0.1.8] - 2025-07-07
### Fixed
- Correctly enable omni.kit.loop-isaac in test dependency (fixes issue from 0.1.7)

## [0.1.7] - 2025-07-03
### Changed
- Make omni.kit.loop-isaac an explicit test dependency

## [0.1.6] - 2025-06-25
### Changed
- Add --reset-user to test args
- Fix SDF paths on windows

## [0.1.5] - 2025-06-12
### Changed
- Fix broken jetbot and nova carter asset links

## [0.1.4] - 2025-06-10
### Changed
- Use default asset root for all assets

## [0.1.3] - 2025-05-31
### Changed
- Use default nucleus server for all tests

## [0.1.2] - 2025-05-19
### Changed
- Update copyright and license to apache v2.0

## [0.1.1] - 2025-05-10
### Changed
- Enable FSD in test settings

## [0.1.0] - 2025-04-28
### Added
- Initial version of MobilityGen Examples
