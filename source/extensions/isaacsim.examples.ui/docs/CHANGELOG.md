# Changelog

## [1.2.2] - 2026-05-13
### Fixed
- Fixed nvbug 6107587: the example UI window content overflowed on normal displays, hiding the lower frames. Window content is now wrapped in a `ui.ScrollingFrame` with the vertical scrollbar always on, and the window is given an explicit default `height=600` so the scroll region has a usable bound.

## [1.2.1] - 2026-03-06
### Fixed
- Clear per-frame plot update subscription when window is hidden to stop GLOBAL_EVENT_UPDATE callbacks running in the background

## [1.2.0] - 2026-03-04
### Changed
- Added Overview.md, python_api.md and updated docstrings

## [1.1.10] - 2026-01-06
### Changed
- Migrate more events to Events 2.0.

## [1.1.9] - 2025-12-01
### Changed
- Rename startup.py to test_startup.py

## [1.1.8] - 2025-05-19
### Changed
- Update copyright and license to apache v2.0

## [1.1.7] - 2025-05-16
### Changed
- Make extension target a specific kit version

## [1.1.6] - 2025-04-04
### Changed
- Version bump to fix extension publishing issues

## [1.1.5] - 2025-03-26
### Changed
- Cleanup and standardize extension.toml, update code formatting for all code

## [1.1.4] - 2025-03-24
### Changed
- Migrate to Events 2.0

## [1.1.3] - 2025-01-27
### Changed
- Updated docs link

## [1.1.2] - 2025-01-21
### Changed
- Update extension description and add extension specific test settings

## [1.1.1] - 2025-01-17
### Changed
- Temporarily changed docs links

## [1.1.0] - 2024-10-29
### Changed
- Moved menu entry from "Isaac Examples" to "Window->Examples"

## [1.0.1] - 2024-10-24
### Changed
- Updated dependencies and imports after renaming

## [1.0.0] - 2023-10-14
### Changed
- Extension renamed to isaacsim.examples.ui.

## [0.1.4] - 2023-10-13
### Fixed
- Updated documentation link

## [0.1.3] - 2023-01-06
### Fixed
- onclick_fn warning when creating UI

## [0.1.2] - 2022-11-22
### Fixed
-Limit issues with int field

## [0.1.1] - 2022-01-18
### Added
- Fixes layout issues

## [0.1.0] - 2021-07-10
### Added
- Initial version of Isaac Sim UI Example
