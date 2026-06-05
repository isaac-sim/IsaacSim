# Changelog

## [0.4.2] - 2026-04-20
### Fixed
- Fix `import_module` issue where the module was not properly stubbed when the optional dependency (e.g. torch) was missing

## [0.4.1] - 2026-04-09
### Fixed
- Skip `exit_app` during stub generation to prevent Kit shutdown when optional dependencies (e.g. torch) are missing

## [0.4.0] - 2026-03-04
### Changed
- Added Overview.md, python_api.md, SETTINGS.md and updated docstrings

## [0.3.3] - 2026-02-07
### Changed
- Remove deprecated asset browser settings

## [0.3.2] - 2025-11-07
### Changed
- Removed code to enable the `omni.isaac.ml_archive` extension when importing PyTorch via `import_module`

## [0.3.1] - 2025-11-05
### Changed
- Enable the `omni.isaac.ml_archive` extension when importing PyTorch via `import_module`

## [0.3.0] - 2025-10-20
### Added
- Expose function to import a deprecated/removed module safely

## [0.2.7] - 2025-05-19
### Changed
- Update copyright and license to apache v2.0

## [0.2.6] - 2025-05-16
### Changed
- Make extension target a specific kit version

## [0.2.5] - 2025-04-30
### Changed
- Update event subscriptions to Event 2.0 system

## [0.2.4] - 2025-04-04
### Changed
- Version bump to fix extension publishing issues

## [0.2.3] - 2025-03-26
### Changed
- Cleanup and standardize extension.toml, update code formatting for all code

## [0.2.2] - 2025-01-21
### Changed
- Update extension description and add extension specific test settings

## [0.2.1] - 2024-10-25
### Fixed
- Fix referenced USD asset paths listing order and use absolute paths

## [0.2.0] - 2024-10-24
### Added
- List referenced USD asset paths containing deprecated OmniGraph nodes in the opened stage

## [0.1.2] - 2024-10-24
### Changed
- Updated dependencies and imports after renaming

## [0.1.1] - 2024-09-26
### Fixed
- Graphs were not functional after renaming types, force reload all graphs if naming changes were made

## [0.1.0] - 2024-09-22
### Added
- Initial release
