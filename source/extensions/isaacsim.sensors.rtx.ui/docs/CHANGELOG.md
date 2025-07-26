# Changelog
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
- increase timeout for UI tests

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
