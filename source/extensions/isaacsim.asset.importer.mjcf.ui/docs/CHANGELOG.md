# Changelog

## [1.4.3] - 2026-05-18
### Added
- "Base Type" dropdown in the Options frame with three choices (Source / Fixed / Mobile) that drives the new tri-state `MJCFImporterConfig.fix_base` field.

## [1.4.2] - 2026-05-14
### Changed
- Updated unit test for the mass api changes

## [1.4.1] - 2026-04-22
### Fixed
- Multi-select import: build an independent `OptionWidget` / config / models per selected MJCF file so edits to one file's panel no longer bleed into another's settings

## [1.4.0] - 2026-04-22
### Changed
- Linting
- Robot Type dropbox
- UI for multi file selection and import

## [1.3.2] - 2026-04-21
### Changed
- Rename `test_command.py` to `test_commands.py` for consistency

## [1.3.1] - 2026-04-20
### Changed
- Updated treeview identifier to filebrowser_grid_view

## [1.3.0] - 2026-04-08
### Changed
- Added type annotations and improved docstrings

## [1.2.0] - 2026-03-31
### Changed
- Use shared ui library from isaacsim.gui.components
- Updated import for compare usd function

## [1.1.1] - 2026-03-21
### Fixed
- Fixed test setUp to call `super().setUp()`, ensuring menus are rebuilt before UI tests run so `File/Import` navigation succeeds on first test invocation

## [1.1.0] - 2026-03-04
### Changed
- Added scene import option to the UI

## [1.0.1] - 2026-02-26
### Changed
- Remove extension api doc

## [1.0.0] - 2026-01-01
### Changed
- Decoupled UI from the MJCF importer
- New UI design and interface
