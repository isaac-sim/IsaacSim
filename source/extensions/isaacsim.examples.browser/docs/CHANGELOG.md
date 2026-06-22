# Changelog

## [0.3.2] - 2026-06-09
### Fixed
- Fix linter errors and missing or incomplete docstrings, and update `python_api.md`.

## [0.3.1] - 2026-05-13
### Fixed
- Fixed nvbug 6130501: Example browser detail panel was empty when clicking a synthetic parent category (e.g. "ROS2") whose examples were only registered under sub-categories.
- Added the missing `asyncio` and `omni.kit.app` imports in `property_delegate.py`. The dynamic thumbnail-resize callback referenced both but had no imports, raising `NameError` whenever a selected item had a thumbnail.

### Changed
- Detail view now mirrors a file browser: a category shows its directly-registered examples plus a folder tile for each immediate sub-category. Double-clicking a folder tile drills the tree selection into that sub-category (the selection change is deferred to the next frame so it doesn't tear down the widget tree mid-event).
- Each `ExampleCategoryItem` now owns its examples directly, removing the previous flat-dict / string-prefix lookup in `get_detail_items`.

## [0.3.0] - 2026-03-04
### Changed
- Added Overview.md, python_api.md and updated docstrings

## [0.2.2] - 2025-11-10
### Fixed
- Replaced deprecated `onclick_fn` with `onclick_action` in "Robotics Examples" menu item to eliminate deprecation warnings
- Registered proper toggle action for the examples browser

## [0.2.1] - 2025-08-25
### Changed
- Highlight the selected item label green

## [0.2.0] - 2025-07-28
### Changed
- Add exceptions when deregistering an example that does not exist
- Add exceptions when registering an example with missing parameters

## [0.1.12] - 2025-05-19
### Changed
- Update copyright and license to apache v2.0

## [0.1.11] - 2025-04-04
### Changed
- Version bump to fix extension publishing issues

## [0.1.10] - 2025-03-26
### Changed
- Cleanup and standardize extension.toml, update code formatting for all code

## [0.1.9] - 2025-03-09
### Fixed
- Fix failing unit tests

## [0.1.8] - 2025-01-21
### Changed
- Update extension description and add extension specific test settings

## [0.1.7] - 2025-01-17
### Changed
- Tab to be "Robotics Examples" to match menu

## [0.1.6] - 2025-01-08
### Changed
- Switched from "error" to "warning" when adding examples with existing names
- Deleting empty categories

## [0.1.5] - 2025-01-07
### Changed
- "Isaac Examples" to "Isaac Sim Examples" for consistency

## [0.1.4] - 2024-11-25
### Fixed
- Fixed the disappearing asset tittles

## [0.1.3] - 2024-11-25
### Fixed
- Refreshing browser during hotloading

## [0.1.2] - 2024-11-19
### Fixed
- Startup test

## [0.1.1] - 2024-11-12
### Fixed
- Fix for browser option panel interference

## [0.1.0] - 2024-10-29
### Changed
- First Version
