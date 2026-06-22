# Changelog

## [1.4.3] - 2026-06-09
### Fixed
- Fix linter errors and missing or incomplete docstrings.

## [1.4.2] - 2026-05-18
### Added
- "Base Type" dropdown in the Options frame with three choices (Source / Fixed / Mobile) that drives the new tri-state `URDFImporterConfig.fix_base` field.

## [1.4.1] - 2026-04-22
### Fixed
- Multi-select import: build an independent `OptionWidget` / config / models per selected URDF file so edits to one file's panel no longer bleed into another's settings or ROS package table

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

## [1.3.0] - 2026-04-09
### Added
- Auto-populate the ROS package table from `package://` references found in the selected URDF file on import
- `package_scanner` module that resolves `package://` URIs to filesystem paths by walking parent directories

## [1.2.1] - 2026-04-09
### Removed
- Select file window size constraints

## [1.2.0] - 2026-04-08
### Changed
- Improve Python API documentation (`config/python_api.md` and/or module docstrings).

## [1.1.0] - 2026-03-31
### Changed
- Use shared UI library from isaacsim.gui.components
- Replaced local style definitions with shared base style from `isaacsim.gui.components.style`
- Moved `checkbox_builder`, `dropdown_builder`, `string_filed_builder` imports to `isaacsim.gui.components.ui_utils`

## [1.0.5] - 2026-03-21
### Fixed
- Fixed test setUp to call `super().setUp()`, ensuring menus are rebuilt before UI tests run so `File/Import` navigation succeeds on first test invocation

## [1.0.4] - 2026-03-09
### Fixed
- Hardened subprocess calls to avoid shell=True with string concatenation

## [1.0.3] - 2026-03-05
### Changed
- Linting

## [1.0.2] - 2026-03-05
### Fixed
- Fixed incorrect type annotation for SearchWidget

## [1.0.1] - 2026-02-26
### Changed
- Remove extension api doc

## [1.0.0] - 2026-02-01
### Changed
- URDF importer 3.x user interface

## [0.1.0] - 2026-01-10
### Added
- Initial release of the URDF Importer UI extension
- Decoupled UI components from the core URDF importer extension
- URDF Importer window with file picker and configuration options
- Asset Importer delegate for URDF files
- Joint configuration widgets (stiffness, natural frequency modes)
- Collider and link configuration options
