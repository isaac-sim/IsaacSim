# Changelog

## [2.3.7] - 2026-06-09
### Fixed
- Fix linter errors and missing or incomplete docstrings.

## [2.3.6] - 2026-05-18
### Added
- "Base Type" dropdown in the Options frame with three choices (Source / Fixed / Mobile) that drives the tri-state `URDFImporterConfig.fix_base` field.

## [2.3.5] - 2026-05-14
### Fixed
- Fixed Windows UI test flakiness for the ROS 2 URDF import menu by aligning test dependencies and window scaling arguments with other UI menu tests.

## [2.3.4] - 2026-04-27
### Removed
- Remove the `omni.isaac.ml_archive` dependency

## [2.3.3] - 2026-04-23
### Fixed
- Handled RCLError when ROS 2 context is shut down during background service call in RobotDefinitionReader
- Intermediate URDF file now written to a system temp directory instead of the extension build directory. When no explicit USD output folder is set, the USD is also written to the temp directory; set the **USD Output** folder in the UI to control the final location.

### Changed
- Tests preserve temp output on failure for debugging, matching the pattern used in isaacsim.asset.importer.urdf

## [2.3.2] - 2026-04-23
### Changed
- Added robot type dropdown to UI

## [2.3.1] - 2026-04-23
### Changed
- Updated the UI status language on failed/successful imports to be clearer.
- Added error message when package can't be resolved in urdf

## [2.3.0] - 2026-04-21
### Deprecated
- Deprecate `URDFImportFromROS2Node` Kit command in favor of using `RobotDefinitionReader` and `URDFImporter` directly

### Removed
- Remove dead root-level `__init__.py` and `extension.py` (not deployed to build output)

### Changed
- Clean up `__init__.py` exports to only expose public API

## [2.2.0] - 2026-03-19
### Changed
- Updated import path to use isaacsim.gui.components ui utils when the util function is available

## [2.1.0] - 2026-03-17
### Changed
- Updated documentation with AI agent.

## [2.0.1] - 2026-03-06
### Changed
- Use `find_widget_with_retry` and `find_enabled_widget_with_retry` from `isaacsim.test.utils` in tests instead of raw `ui_test.find` calls

## [2.0.0] - 2026-02-07
### Changed
- Updated isaacssim.ros2.urdf to use URDF importer 3.x

## [1.1.22] - 2026-01-09
### Changed
- Make isaacsim.asset.importer.urdf.ui a dependency since this extension heavily depends on and modifies the default urdf importer ui

## [1.1.21] - 2025-10-27
### Changed
- Make omni.isaac.ml_archive an explicit test dependency

## [1.1.20] - 2025-10-06
### Fixed
- Fix cleanup of the ObserverGuard object when using omni.kit.command in script

## [1.1.19] - 2025-07-07
### Fixed
- Correctly enable omni.kit.loop-isaac in test dependency (fixes issue from 1.1.18)

## [1.1.18] - 2025-07-03
### Changed
- Make omni.kit.loop-isaac an explicit test dependency

## [1.1.17] - 2025-06-25
### Changed
- Add --reset-user to test args

## [1.1.16] - 2025-05-31
### Changed
- Use default nucleus server for all tests

## [1.1.15] - 2025-05-24
### Changed
- Update to work with changes to urdf importer

## [1.1.14] - 2025-05-19
### Changed
- Update copyright and license to apache v2.0

## [1.1.13] - 2025-05-10
### Changed
- Enable FSD in test settings

## [1.1.12] - 2025-04-09
### Changed
- Update all test args to be consistent

## [1.1.11] - 2025-04-04
### Changed
- Version bump to fix extension publishing issues

## [1.1.10] - 2025-03-26
### Changed
- Cleanup and standardize extension.toml, update code formatting for all code

## [1.1.9] - 2025-03-24
### Changed
- Migrate to Events 2.0

## [1.1.8] - 2025-01-21
### Changed
- Update extension description and add extension specific test settings

## [1.1.7] - 2025-01-08
### Changed
- Fix Missing dependencies

## [1.1.6] - 2024-12-05
### Changed
- Updated Nova carter path

## [1.1.5] - 2024-12-03
### Changed
- Isaac Util menu to File

## [1.1.4] - 2024-11-18
### Changed
- omni.client._omniclient to omni.client

## [1.1.3] - 2024-11-14
### Changed
- Update omni.isaac.urdf to isaacsim.asset.importer.urdf

## [1.1.2] - 2024-11-09
### Changed
- Updated dependencies and imports after renaming

## [1.1.1] - 2024-10-24
### Changed
- Updated dependencies and imports after renaming

## [1.1.0] - 2024-10-15
### Changed
- Moved ros2 node import workflow UI to File->Menu to align with URDF Importer UI

### Added
- Create `URDFImportFromROS2Node` command

## [1.0.0] - 2024-09-27
### Changed
- Extension renamed to isaacsim.ros2.urdf

## [0.1.0] - 2024-02-28
### Added
- Initial Version
