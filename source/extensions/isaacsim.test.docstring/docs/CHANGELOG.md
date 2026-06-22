# Changelog

## [1.3.2] - 2026-06-09
### Fixed
- Fix linter errors and missing or incomplete docstrings, and update `python_api.md`.

## [1.3.1] - 2026-04-02
### Fixed
- Replace mutable default arguments (`[]`, `{}`) with `None` defaults across `_doctest.py`, `async_doctest.py`, and `standalone_doctest.py`
- Catch `SystemExit` during doctest execution to prevent application shutdown from docstring examples that call `sys.exit()`
- Add `from __future__ import annotations` for PEP 604 union type syntax

## [1.3.0] - 2026-03-04
### Changed
- Added Overview.md and python_api.md and updated docstrings

## [1.2.1] - 2025-12-07
### Changed
- Update description

## [1.2.0] - 2025-11-28
### Changed
- Add API documentation
- Add missing docstrings
- Add more example usage to documentatiomn

## [1.1.0] - 2025-07-07
### Changed
- Add ability to check docstrings for pybind11 modules

## [1.0.8] - 2025-05-30
### Fixed
- Add 'isaacsim' module to global namespace when checking members

## [1.0.7] - 2025-05-19
### Changed
- Update copyright and license to apache v2.0

## [1.0.6] - 2025-05-16
### Changed
- Make extension target a specific kit version

## [1.0.5] - 2025-04-04
### Changed
- Version bump to fix extension publishing issues

## [1.0.4] - 2025-03-26
### Changed
- Cleanup and standardize extension.toml, update code formatting for all code

## [1.0.3] - 2025-01-21
### Changed
- Update extension description and add extension specific test settings

## [1.0.2] - 2024-11-19
### Fixed
- Startup test

## [1.0.1] - 2024-10-24
### Changed
- Updated dependencies and imports after renaming

## [1.0.0] - 2024-09-23
### Changed
- Extension renamed to isaacsim.test.docstring

## [0.1.0] - 2023-12-19
### Added
- Initial release
