# Changelog

## [2.3.0] - 2026-04-10
### Deprecated
- Extension deprecated in favor of the `repo.sh template` CLI system (`./repo.sh template new`)
- Moved to source/deprecated/

## [2.2.1] - 2026-03-17
### Fixed
- Correct "broadcast" to "broadphase" in default physics settings docstring

## [2.2.0] - 2026-03-04
### Changed
- Added Overview.md, python_api.md and updated docstrings

## [2.1.4] - 2026-02-23
### Changed
- Add ticked=True to Extension Generator menu item

## [2.1.3] - 2025-12-15
### Changed
- Fix event name usage.

## [2.1.2] - 2025-12-05
### Changed
- Migrate to Events 2.0.

## [2.1.1] - 2025-10-27
### Changed
- Make omni.isaac.ml_archive an explicit test dependency

## [2.1.0] - 2025-10-17
### Changed
- Migrate PhysX subscription and simulation control interfaces to Omni Physics

## [2.0.12] - 2025-05-19
### Changed
- Update copyright and license to apache v2.0

## [2.0.11] - 2025-05-16
### Changed
- Make extension target a specific kit version

## [2.0.10] - 2025-05-07
### Changed
- Switch to omni.physics interface

## [2.0.9] - 2025-04-09
### Changed
- Update Isaac Sim robot asset path

## [2.0.8] - 2025-04-04
### Changed
- Version bump to fix extension publishing issues

## [2.0.7] - 2025-03-26
### Changed
- Cleanup and standardize extension.toml, update code formatting for all code

## [2.0.6] - 2025-01-27
### Changed
- Updated docs link

## [2.0.5] - 2025-01-21
### Changed
- Update extension description and add extension specific test settings

## [2.0.4] - 2025-01-17
### Changed
- Temporarily changed docs links

## [2.0.3] - 2025-01-13
### Changed
- Move Core connector buttons to Examples

## [2.0.2] - 2024-12-03
### Changed
- Isaac Util menu to Tools->Robotics menu

## [2.0.1] - 2024-10-24
### Changed
- Updated dependencies and imports after renaming

## [2.0.0] - 2024-10-02
### Changed
- Extension renamed to isaacsim.examples.extension.

## [1.7.0] - 2024-03-14
### Fixed
- Fixed code for handling Articulation selection in Configuration Tooling Template. The issue arose from a change in behavior in Core get_prim_object_type()

## [1.6.0] - 2024-02-28
### Fixed
- Fixed code for populating drop-down menu with Articulations in Configuration Tooling Template. This issue arose with Kit behavior changes on stage and timeline events.

## [1.5.2] - 2023-12-02
### Changed
- Updated path to the nucleus extension

## [1.5.1] - 2023-11-13
### Fixed
- Updated documentation link

## [1.5.0] - 2023-10-27
### Added
- Add Scripting Template to demonstrate programming script-like behavior through a standalone UI extension.

### Changed
- Modifies extension.py to massively reduce code duplication.

## [1.4.1] - 2023-08-24
### Fixed
- Fixed bug in Template generator on Windows where user could specify a file path ending in "/".

## [1.4.0] - 2023-08-15
### Fixed
- Fixed bug in Loaded Scenario Template where XFormPrim no longer accepts an Sdf.Path

## [1.3.1] - 2023-08-08
### Fixed
- Switch from ui.Window to ScrollingWindow wrapper for extensions because scrolling was broken

## [1.3.0] - 2023-06-26
### Fixed
- Fixed bug where Configuration Tooling Template could cause a crash in Kit 105.1
- The crash was caused by attempting to initialize Articulation on all stage events. The fix is to subscribe specifically to the ASSETS_LOADED stage event.
- Subscriptions to STOP/PLAY timeline events in the Configuration Tooling Template have been removed, as they didn't acheive anything when running Kit 105.1

## [1.2.0] - 2023-05-03
### Added
- Added limitations to special characters that are considered acceptible in an extension title
- Added XYPlot to "UI Component Library Template"

### Fixed
- Extension templates now use the extension name to generate python module paths to prevent clashing python modules. Previously, all modules were named "scripts" and you could not enable multiple extension templates at once.

## [1.1.0] - 2023-03-17
### Added
- Added "Status Frame" to Extension Template Generator to give feedback to the user to guide their usage. Feedback includes verification that their templates were generated properly.

## [1.0.0] - 2023-02-28
### Changed
- Updated Templates along with breaking changes to isaacsim.gui.components replacing UIFrameWrapper with CollapsableFrame instance of UIElementWrapper
- Updated Configuration Tooling Template to use rebuild() function of a CollapsableFrame instead of pre-allocating 100 invisible frames.

### Added
- Added "UI Component Library Template" to show the usage of each UIElementWrapper

## [0.3.1] - 2023-02-17
### Fixed
- Fixed UI bug in Template Generator that allowed unnamed templates to be created

## [0.3.0] - 2023-02-15
### Added
- Added "Configuration Tooling Template" which allows the user to interact with a loaded robot Articulation through the UI.

### Removed
- Removed "Async Scenario Template" since it doesn't stand out well from "Loaded Scenario Template"

## [0.2.0] - 2023-02-13
### Added
- Added "Loaded Scenario Template" which has a load and reset button that are connected to Core.

## [0.1.3] - 2023-02-01
### Fixed
- Async Scenario Template "Run Scenario" button resets properly

## [0.1.2] - 2023-01-10
### Fixed
- Async Scenario Template now disables buttons until an Articulation is selected

## [0.1.1] - 2023-01-06
### Fixed
- onclick_fn warning when creating UI

## [0.1.0] - 2022-12-09
### Added

- Initial version of Extension Template Generator
