# Changelog

## [1.5.1] - 2026-06-09
### Fixed
- Fix linter errors and missing or incomplete docstrings.

## [1.5.0] - 2026-05-04
### Added
- RTX Radar menu reorganized into vendor submenus driven by `SUPPORTED_RADAR_CONFIGS`; Generic RTX Radar moved under `NVIDIA`; Texas Instruments IWRL6432AOP added
- RTX Acoustic menu (`Create > Sensors > RTX Acoustic`) with a generic NVIDIA entry; auto-populates per-vendor entries from `SUPPORTED_ACOUSTIC_CONFIGS`

### Changed
- Lidar/Radar/Acoustic menu actions now pass a default variant from `SUPPORTED_*_CONFIGS` so multi-variant-set USDs (e.g. SICK `Product` × `Profile`) materialize a valid prim from a single click

## [1.4.1] - 2026-04-24
### Removed
- Remove the `omni.isaac.ml_archive` test dependency

## [1.4.0] - 2026-04-21
### Changed
- Migrated Lidar creation from `IsaacSensorCreateRtxLidar` Kit command to `Lidar.create()` from `isaacsim.sensors.experimental.rtx`
- Migrated Radar creation from `IsaacSensorCreateRtxRadar` Kit command to `Radar()` from `isaacsim.sensors.experimental.rtx`
- Replaced `isaacsim.sensors.rtx` dependency with `isaacsim.sensors.experimental.rtx`
- Replaced `isaacsim.core.utils.stage` with `isaacsim.core.experimental.utils.stage`
- Updated tests to use experimental API imports

## [1.3.1] - 2026-04-01
### Changed
- Update actions_api.md file

## [1.3.0] - 2026-03-04
### Changed
- Added Overview.md and python_api.md and updated docstrings

## [1.2.3] - 2026-02-18
### Fixed
- Replaced deprecated `onclick_fn` with `onclick_action` in menu items to eliminate deprecation warnings
- Registered proper actions for all RTX Lidar and RTX Radar sensor creation menu items

## [1.2.2] - 2026-01-24
### Changed
- Fix issues with menu click and context menu tests being flaky

## [1.2.1] - 2026-01-22
### Changed
- Move menu dictionary to initialize when tests are run rather than at module load time

## [1.2.0] - 2025-12-22
### Changed
- Refactor unit tests to use isaacsim.test.utils.

## [1.1.9] - 2025-12-05
### Changed
- Change test to use RealTimePathTracing render mode

## [1.1.8] - 2025-12-03
### Changed
- Remove TODOs.

## [1.1.7] - 2025-10-27
### Changed
- Make omni.isaac.ml_archive an explicit test dependency

## [1.1.6] - 2025-08-21
### Changed
- RTX Lidar menu options now use IsaacSensorCreateRtxLidar command to align with other APIs

## [1.1.5] - 2025-08-06
### Changed
- Remove test_helpers_gfx from test dependencies

## [1.1.4] - 2025-07-07
### Fixed
- Correctly enable omni.kit.loop-isaac in test dependency (fixes issue from 1.1.3)

## [1.1.3] - 2025-07-03
### Changed
- Make omni.kit.loop-isaac an explicit test dependency

## [1.1.2] - 2025-06-25
### Changed
- Fixed sensor naming for context menu tests
- Omitted nanoSensor3 from test due to reference load failures.

## [1.1.1] - 2025-06-13
### Changed
- Fix menu test timeout

## [1.1.0] - 2025-06-06
### Changed
- Use isaacsim.sensors.rtx.SUPPORTED_LIDAR_CONFIGS to autogenerate menus, tests

## [1.0.12] - 2025-06-06
### Changed
- Increase timeout for UI tests

## [1.0.11] - 2025-05-31
### Changed
- Use default nucleus server for all tests

## [1.0.10] - 2025-05-27
### Added
- More NVIDIA lidar examples available in menu
- RTX Radar menu option

## [1.0.9] - 2025-05-27
### Changed
- Minor sensor name changes

## [1.0.8] - 2025-05-19
### Changed
- Update copyright and license to apache v2.0

## [1.0.7] - 2025-05-10
### Changed
- Enable FSD in test settings

## [1.0.6] - 2025-04-28
### Fixed
- Add settings to fix dpi for gui based tests

## [1.0.5] - 2025-04-28
### Fixed
- Correctly set prim TypeName to "OmniLidar" for Lidars specified as USDAs

## [1.0.4] - 2025-04-09
### Changed
- Update all test args to be consistent

## [1.0.3] - 2025-04-04
### Changed
- Version bump to fix extension publishing issues

## [1.0.2] - 2025-03-26
### Changed
- Cleanup and standardize extension.toml, update code formatting for all code

## [1.0.1] - 2025-03-20
### Changed
- Add sensor menu to context menus

## [1.0.0] - 2025-03-18
### Added
- Dynamic test generation for RTX Sensor menu items

### Changed
- Update RTX Sensor menu to use OmniSensor prims.

## [0.1.3] - 2025-03-11
### Changed
- Switch asset root for tests to internal nucleus

## [0.1.2] - 2025-01-26
### Changed
- Update test settings

## [0.1.1] - 2025-01-21
### Changed
- Update extension description and add extension specific test settings

## [0.1.0] - 2024-12-10
### Added
- Initial version of isaacsim.sensors.rtx.ui
