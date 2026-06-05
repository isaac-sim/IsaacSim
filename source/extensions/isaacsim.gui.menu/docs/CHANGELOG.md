# Changelog

## [2.10.5] - 2026-05-19
### Fixed
- Help menu: `OpenUSD Reference Guide`, `Warp Getting Started`, and `Warp Documentation` now render reliably in `isaacsim.exp.full.kit`. Previously they bound to `source=` items not provided by the app's dependency closure and rendered as orphan separators. Register the actions and `MenuItemDescription`s locally instead.

## [2.10.4] - 2026-03-16
### Changed
- Migrate extension implementation to core experimental API

## [2.10.3] - 2026-03-12
### Changed
- Remove unused golden images

## [2.10.2] - 2026-03-10
### Fixed
- OMPE-84348: rewrote `_test_environment_menu_option` to not use golden images

## [2.10.1] - 2026-03-05
### Fixed
- OMPE-84348: fixed flaky test

## [2.10.0] - 2026-03-04
### Changed
- Add Overview.md, python_api.md and update docstrings

## [2.9.0] - 2026-02-27
### Removed
- Removed Warp Sample Scenes menu item from Help menu (omni.warp dependency removed)

## [2.8.0] - 2026-02-24
### Added
- Robot Self-Collision Detector to Tools > Asset Editors menu

## [2.7.1] - 2026-02-23
### Changed
- Add ticked=True to Asset Check menu item

## [2.7.0] - 2026-02-07
### Changed
- Remove Asset Browser from menu and context menu
- Use menu.open_content_browser_to_path to open the Content Browser to a specific path as a replacement
- Add Utility menu item to check Isaac Sim assets root path

## [2.6.0] - 2026-01-26
### Changed
- Fix broken Isaac Sim documentation links
- Update deprecated onclick_fn to on_click_action
- Added missing docstrings and cleanup code

## [2.5.3] - 2026-01-24
### Changed
- Fix issues with menu click and context menu tests being flaky

## [2.5.2] - 2026-01-22
### Changed
- Move menu dictionary to initialize when tests are run rather than at module load time

## [2.5.1] - 2026-01-06
### Changed
- Migrate more events to Events 2.0.

## [2.5.0] - 2025-12-22
### Changed
- Refactor test_menu to use isaacsim.test.utils.MenuUITestCase and isaacim.gui.components.create_submenu

### Removed
- Move sensor menu UI tests to their own extensions

## [2.4.8] - 2025-12-11
### Changed
- Update golden image for environment test

## [2.4.7] - 2025-12-07
### Changed
- Update description

## [2.4.6] - 2025-11-05
### Changed
- Renamed Block World Generator to Heightmap Importer

## [2.4.5] - 2025-10-27
### Changed
- Make omni.isaac.ml_archive an explicit test dependency

## [2.4.4] - 2025-09-19
### Changed
- Update test to use isaacsim.test.utils for image capture and comparison

## [2.4.3] - 2025-09-07
### Changed
- Update test settings

## [2.4.2] - 2025-08-28
### Changed
- Add delay to menu click for environment test

## [2.4.1] - 2025-08-27
### Fixed
- Fix the broken documentation links

## [2.4.0] - 2025-08-21
### Changed
- Moved help menu links to isaacsim.gui.menu from isaacsim.app.setup

## [2.3.17] - 2025-07-22
### Changed
- Add delay to menu click for sensor tests
- Create new stage rather than clear stage for sensor tests

## [2.3.16] - 2025-07-07
### Fixed
- Correctly enable omni.kit.loop-isaac in test dependency (fixes issue from 1.1.8)

## [2.3.15] - 2025-07-03
### Changed
- Make omni.kit.loop-isaac an explicit test dependency

## [2.3.14] - 2025-06-26
### Changed
- Add increasing delay to menu click for environment test

## [2.3.13] - 2025-06-25
### Changed
- Add --reset-user to test args
- Add delay to menu click for apriltag test

## [2.3.12] - 2025-06-13
### Changed
- Fix incorrect menu name
- Removed actions_api.md

## [2.3.11] - 2025-06-06
### Changed
- Fixed shutdown

## [2.3.10] - 2025-06-06
### Changed
- Rename "Spatial SDG" to "Action and Event Data Generation"

## [2.3.9] - 2025-06-04
### Changed
- Fixed issue with duplicate "Spatial SDG" causing missing menu in tools
- Fixed Replicator menu duplicate issue

## [2.3.8] - 2025-06-03
### Changed
- Add missing Replicator menu items

## [2.3.7] - 2025-05-31
### Changed
- Use default nucleus server for all tests

## [2.3.6] - 2025-05-30
### Changed
- Update timeouts to fix test

## [2.3.5] - 2025-05-29
### Changed
- Added Sensors and Spatial SDG to the Tools menu

## [2.3.4] - 2025-05-29
### Fixed
- Error due to usage of force_legacy_api

## [2.3.3] - 2025-05-19
### Changed
- Update copyright and license to apache v2.0

## [2.3.2] - 2025-05-10
### Added
- SDG Grasping menu item in Replicator submenu

## [2.3.1] - 2025-05-10
### Changed
- Enable FSD in test settings

## [2.3.0] - 2025-05-01
### Added
- Create Robot / Surface Gripper

## [2.2.4] - 2025-04-30
### Changed
- Update event subscriptions to Event 2.0 system

## [2.2.3] - 2025-04-24
### Fixed
- Fixed _build_recent_menu bug

## [2.2.2] - 2025-04-09
### Changed
- Update all test args to be consistent
- Update Isaac Sim NVIDIA robot asset path
- Update Isaac Sim robot asset path
- Update Isaac Sim robot asset path for the IsaacSim folder

## [2.2.1] - 2025-04-04
### Changed
- Version bump to fix extension publishing issues

## [2.2.0] - 2025-04-02
### Added
- URDF Exporter to File menu

## [2.1.3] - 2025-03-31
### Changed
- Layout menu order

## [2.1.2] - 2025-03-26
### Changed
- Cleanup and standardize extension.toml, update code formatting for all code

## [2.1.1] - 2025-03-24
### Changed
- Migrate to Events 2.0

## [2.1.0] - 2025-03-20
### Added
- Isaac Create Menu in Viewport and Stage Context Menus (right click)

## [2.0.13] - 2025-03-18
### Removed
- Dependency on isaacsim.sensors.rtx.ui extension
- RTX Sensor menu item tests (moved to isaacsim.sensors.rtx.ui extension)

## [2.0.12] - 2025-03-11
### Changed
- Switch asset root for tests to internal nucleus

## [2.0.11] - 2025-03-04
### Changed
- Update to kit 107.1 and fix build issues

## [2.0.10] - 2025-02-03
### Fixed
- Updated physics reference link to be based on version automatically and added test

## [2.0.9] - 2025-01-26
### Changed
- Update test settings

## [2.0.8] - 2025-01-21
### Changed
- Update extension description and add extension specific test settings

### Fixed
- Security fix to change os.umask(0) to os.umask(0o777)

## [2.0.7] - 2025-01-08
### Fixed
- Updated physics reference link

## [2.0.6] - 2024-12-12
### Fixed
- Added sensor UI extensions as test dependencies

## [2.0.5] - 2024-12-09
### Changed
- ROS asset separator and undid some hacks

## [2.0.4] - 2024-12-06
### Changed
- Updated OmniGraph menu naming

## [2.0.3] - 2024-12-05
### Changed
- Updated Nova carter path

## [2.0.2] - 2024-12-03
### Changed
- No more Isaac Utils reference
- Glyphs for the Create Asset menu

## [2.0.1] - 2024-11-26
### Changed
- Replicator new menu layout restructure

## [2.0.0] - 2024-10-25
### Removed
- New menu layout

## [1.1.2] - 2024-10-24
### Changed
- Updated dependencies and imports after renaming

## [1.1.1] - 2024-10-15
### Changed
- FIxes typo in Jetbot menu option

## [1.1.0] - 2024-10-04
### Removed
- References to PhysX ultrasonic sensor

## [1.0.0] - 2024-09-30
### Changed
- Extension renamed to isaacsim.gui.menu

## [0.7.3] - 2024-09-05
### Removed
- Grid Room from menu to match documentation

## [0.7.2] - 2024-09-03
### Added
- Missing robots to the menu

### Fixed
- Some naming to match documentation

## [0.7.1] - 2024-08-26
### Added
- Missing environments to the menu

### Fixed
- Generation of April Tag

## [0.7.0] - 2024-08-23
### Added
- Several new robots to robot menu

### Changed
- Modifies robot menu to have multiple submenus based on robot type
- Aligns robot menu ordering with documentation

## [0.6.0] - 2024-08-21
### Changed
- Adds original Franka to robot asset menu
- Renames Franka -> Franka (alt. fingers)
- Moves Ant to Quadruped menu
- Adds Black Grid and Curved Grid menu options.

## [0.5.0] - 2024-07-10
### Removed
- Deprecated omni.isaac.dofbot and removed its usage.

## [0.4.1] - 2024-05-28
### Fixed
- Robotiq asset links

## [0.4.0] - 2024-05-24
### Changed
- Added leatherback

### Fixed
- Broken USD paths

## [0.3.1] - 2024-05-11
### Changed
- Renamed Transporter to iw.hub

## [0.3.0] - 2024-05-07
### Added
- Added menu options for the new robots
- Added humanoid robot section

## [0.2.3] - 2024-04-22
### Fixed
- Sensor menu tests failing due to sensor extension now creates sensors on play, play timeline first before checking for values

## [0.2.2] - 2024-03-07
### Fixed
- Tests failing due to short delay when clicking, increased delay from 10 to 32 (default)

## [0.2.1] - 2024-02-20
### Fixed
- Extra dependencies from tests being incorrectly imported

## [0.2.0] - 2024-01-30
### Changed
- Changed sensor menu tests to verify sensor functionality through the sensor interfaces
- Added golden image test for the environment

## [0.1.0] - 2024-01-19
### Added
- Initial version of extension, moving menu code out of isaacsim.core.utils extension
