# Changelog

## [1.4.10] - 2025-07-22
### Fixed
- Fix exception caused if extension is disabled while ros bridge is being enabled

## [1.4.9] - 2025-07-07
### Fixed
- Correctly enable omni.kit.loop-isaac in test dependency (fixes issue from 1.4.8)

## [1.4.8] - 2025-07-03
### Changed
- Make omni.kit.loop-isaac an explicit test dependency

## [1.4.7] - 2025-06-25
### Changed
- Add --reset-user to test args

## [1.4.6] - 2025-06-05
### Changed
- removed the duplicated replicator menu hack

## [1.4.5] - 2025-06-04
### Changed
- Force persistent/exts/omni.anim.navigation.core/navMesh/config/autoRebakeOnChanges to false when extension starts

## [1.4.4] - 2025-05-31
### Changed
- Use default nucleus server for all tests

## [1.4.3] - 2025-05-29
### Changed
- Update replicator menu entry name to match Isaac Sim menu
- removed Isaac Sim Assets [Beta] from default layouts

## [1.4.2] - 2025-05-19
### Changed
- Update copyright and license to apache v2.0

## [1.4.1] - 2025-05-16
### Changed
- Make extension target a specific kit version

## [1.4.0] - 2025-05-15
### Changed
- Removed internal carb setting

## [1.3.11] - 2025-05-10
### Changed
- Enable FSD in test settings

## [1.3.10] - 2025-04-09
### Changed
- Update all test args to be consistent

## [1.3.9] - 2025-04-04
### Changed
- Version bump to fix extension publishing issues

## [1.3.8] - 2025-03-31
### Changed
- added SDG Layout to menu, cleaned up layout folder

## [1.3.7] - 2025-03-26
### Changed
- Cleanup and standardize extension.toml, update code formatting for all code

## [1.3.6] - 2025-03-05
### Changed
- Update extension codebase to adhere to isaac sim extension structure and file naming  guidelines

## [1.3.5] - 2025-01-21
### Changed
- Update extension description and add extension specific test settings

## [1.3.4] - 2024-12-09
### Changed
- menu entries names to reverse some hacks in the menu gui

## [1.3.3] - 2024-12-07
### Fixed
- Removes some deprecation warnings by using MenuItemDescription to build Layouts and Help menus

## [1.3.2] - 2024-11-19
### Fixed
- Startup test

## [1.3.1] - 2024-11-18
### Changed
- omni.client._omniclient to omni.client

## [1.3.0] - 2024-11-10
### Changed
- Moved settings to isaacsim.base.kit

## [1.2.0] - 2024-11-10
### Changed
- Renamed to isaacsim.selector.kit

## [1.1.0] - 2024-11-06
### Changed
- enabled default and visual scripting layout. moved layout folder.

## [1.0.2] - 2024-11-01
### Changed
- Make Isaac Sim Assets Browser in focus by default

## [1.0.1] - 2024-10-24
### Changed
- Updated dependencies and imports after renaming

## [1.0.0] - 2024-10-11
### Changed
- Rename extension to isaacsim.app.setup

## [0.14.2] - 2024-10-02
### Changed
- Update dependency omni.isaac.version to isaacsim.core.version

## [0.14.1] - 2024-09-27
### Fixed
- Missing keyboard bindings for layout menu items

## [0.14.0] - 2024-07-25
### Changed
- Removed imgui calls on app startup to avoid crash when display was not set
- Fixed deprecation warnings

## [0.13.2] - 2024-05-01
### Fixed
- update for set_phase api change

## [0.13.1] - 2024-03-19
### Fixed
- include for benchmark services

## [0.13.0] - 2024-03-18
### Added
- isaac.startup.create_new_stage carb setting to enable/disable stage creation on app startup, enabled by default

## [0.12.3] - 2024-03-07
### Fixed
- Crash after disabling omni.activity extensions to improve performance

## [0.12.2] - 2024-02-29
### Added
- Benchmark metadata

### Changed
- Updated benchmark set_phase() call to correctly record startup time after removing deprecated API

### Removed
- Deprecated benchmark stop_runtime() call

## [0.12.1] - 2024-02-08
### Removed
- Deprecated replicator composer menu item

## [0.12.0] - 2024-01-30
### Changed
- Measures startup time if omni.isaac.benchmark.services is loaded.

## [0.11.2] - 2023-09-21
### Removed
- Remove hotkeys for Help menu

## [0.11.1] - 2023-09-06
### Changed
- Deprecation label for Replicator/Composer menu item

## [0.11.0] - 2023-09-06
### Changed
- Added Replicator YAML menu item

## [0.10.0] - 2023-08-08
### Changed
- Update Help menu

## [0.9.0] - 2023-07-05
### Changed
- Removed settings that were already set in .kit file

## [0.8.2] - 2023-06-07
### Fixed
- Issue where fullscreen and hide UI didn't work

## [0.8.1] - 2023-02-14
### Changed
- Update forums link

## [0.8.0] - 2023-02-08
### Changed
- set omnigraph default settings on app start

## [0.7.0] - 2023-01-23
### Changed
- Removed unused code
- Enabled tests
- Added setting to enable ros bridge on startup

## [0.6.2] - 2022-12-05
### Changed
- Push Help menu to the end

## [0.6.1] - 2022-12-01
### Fixed
- Disable App Selector auto start when running from Help menu (OM-74496)

## [0.6.0] - 2022-12-01
### Changed
- Added replicator menu layout

## [0.5.0] - 2022-11-22
### Changed
- Updated menu icons

## [0.4.1] - 2022-11-08
### Fixed
- Duplicate help menus using same F1 key, switched isaac sim docs to F3

## [0.4.0] - 2022-10-06
### Changed
- Updated menu structure to match create

## [0.3.1] - 2022-09-01
### Changed
- Removed unused call to viewport legacy

## [0.3.0] - 2022-08-15
### Added
- Force PhysX reset on stop to be true

## [0.2.4] - 2022-07-25
### Added
- Increase hang detection timeout (OM-55578)

## [0.2.3] - 2022-05-31
### Changed
- Use carb.tokens to get kit exe path (OM-48462)

## [0.2.2] - 2022-05-17
### Fixed
- Fixed hot reloading

## [0.2.1] - 2022-05-12
### Added
- Use omni.isaac.version.get_version()

## [0.2.0] - 2022-04-29
### Added
- Added Isaac Sim App Selector to the Help Menu.

## [0.1.4] - 2022-03-16
### Added
- Added Isaac Sim full version tag in log file.

## [0.1.3] - 2022-01-24
### Removed
- Moved Nucleus Check to its own extension (OM-43459)

## [0.1.2] - 2021-12-15
### Changed
- Added feature to detect and update downloaded Isaac Sim assets on Nucleus (OM-41819)

## [0.1.1] - 2021-12-01
### Fixed
- Fixed nucleusCheck running during post-install and warm-up (OM-41545)

### Changed
- Update settings to use camel case

## [0.1.0] - 2021-10-22
### Added
- Added first version of setup.
