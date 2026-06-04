# Changelog

## [1.2.3] - 2026-05-14
### Fixed
- Show a Kit warning notification when VS Code cannot be launched because `code` is not available on the system

## [1.2.2] - 2026-04-23
### Fixed
- Eliminated shell injection risk by resolving `code` via `shutil.which()` and removing `shell=True`

## [1.2.1] - 2026-03-25
### Changed
- Replace deprecated onclick_fn with onclick_action for menu registration

## [1.2.0] - 2026-03-09
### Changed
- Extract TCP socket server and Python executor into isaacsim.code_editor.python_server
- Extension now depends on isaacsim.code_editor.python_server for code execution
- Retain only VS Code launcher and menu integration
- Settings moved to isaacsim.code_editor.python_server extension

## [1.1.2] - 2026-03-09
### Fixed
- Hardened subprocess call to avoid shell=True with string concatenation

## [1.1.1] - 2026-03-02
### Changed
- Add Overview.md, add python_api.md, add SETTINGS.md and update docstrings

## [1.1.0] - 2025-06-10
### Changed
- Set the default IP address for the extension server to 127.0.0.1

## [1.0.10] - 2025-06-09
### Changed
- Set the `carb_logs` flag to False by default

## [1.0.9] - 2025-05-19
### Changed
- Update copyright and license to apache v2.0

## [1.0.8] - 2025-05-16
### Changed
- Make extension target a specific kit version

## [1.0.7] - 2025-04-04
### Changed
- Version bump to fix extension publishing issues

## [1.0.6] - 2025-03-26
### Changed
- Cleanup and standardize extension.toml, update code formatting for all code

## [1.0.5] - 2025-01-23
### Fixed
- Remove menu items created when shutting down the extension
- Run socket server closing coroutine thread-safe

## [1.0.4] - 2025-01-21
### Changed
- Update extension description and add extension specific test settings

## [1.0.3] - 2024-12-07
### Fixed
- Moved from deprecated editor_menu.add_item to omni.kit.menu.utils

## [1.0.2] - 2024-10-28
### Changed
- Remove test imports from runtime

## [1.0.1] - 2024-10-24
### Changed
- Updated dependencies and imports after renaming

## [1.0.0] - 2024-09-23
### Changed
- Extension renamed to isaacsim.code_editor.vscode.

## [0.1.0] - 2024-01-24
### Added
-   Initial release
