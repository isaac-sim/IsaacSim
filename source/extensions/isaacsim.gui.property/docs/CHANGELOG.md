# Changelog

## [1.6.4] - 2026-06-09
### Fixed
- Fix linter errors and missing or incomplete docstrings.

## [1.6.3] - 2026-06-08
### Fixed
- Extension test `stdoutFailPatterns.exclude` now ignores benign "The NumPy module was reloaded" log output so successful runs are not marked as failures.

## [1.6.2] - 2026-05-11
### Added
- Array Properties widget now supports editing `string[]` and `token[]` USD attributes alongside the existing scalar/vector numeric arrays.

## [1.6.1] - 2026-04-28
### Added
- Added `Attachment Point API` to Robot Schema UI menu

## [1.6.0] - 2026-04-20
### Added
- New `Tools/Robotics/Joint Inspector` menu opens a dockable window with a searchable robot dropdown scanning `IsaacRobotAPI` prims.
- `JointInspectorWindowManager` supports multiple concurrent windows via `+ New Inspector` button.
- Joint table built on `omni.ui.TreeView` with custom model/delegate, `ui.FloatDrag` cells, and Ctrl/Cmd/Shift row selection.
- Editing a cell on a selected row mirrors the value to every other selected row's matching column.
- Column catalogue across five groups tagged by backend: `Joint Limits`, `Drives`, `Performance Envelope`, `Joint State`, `MuJoCo Joint`.
- Per-axis columns collapse to one when every joint authors at most one axis; fan out only for multi-DOF `D6Joint`.
- Column selection persists across robot switches; unavailable columns dim but stay checkable.
- Hamburger button opens a categorized popup with `PhysX` / `MuJoCo` backend pills that toggle whole backends on/off.
- Joint-name filter is a single rounded input with `fnmatch` wildcards over joint name and full path.
- Robot-picker and columns popups are anchored to host widgets with `WINDOW_FLAGS_NO_MOVE`.
- Typography uses NVIDIA Sans with centralized `_UI_FONT` / `_FONT_SIZE_*` constants.
- New shared `style.py` module centralizing layout, colors, widget styles, and a theme-independent `TOOLTIP_STYLE`.
- `RobotAPIWidget` rewritten with Figma-style layout: editable attribute fields, collapsible changelog, drag-reorderable Robot Joints / Robot Links, `force_update` toggle.
- New `Save to Robot Layer` button flushes composed `robotLinks`/`robotJoints` via `SaveRobotSchemaToRobotLayer`.
- Isaac API schemas registered with Kit's `MultiSchemaPropertiesWidget.__known_api_schemas` to prevent Extra Properties duplication.

### Changed
- Robot-schema widget registrations now use `collapsed_by_default=True`.

## [1.5.1] - 2026-03-05
### Fixed
- Fixed incorrect type annotation (List to list)

## [1.5.0] - 2026-03-04
### Changed
- Add Overview.md, python_api.md, and update docstrings

## [1.4.0] - 2026-02-20
### Added
- Introduced widget for applying IsaacSiteAPI to Xformable prims

## [1.3.0] - 2026-01-28
### Added
- Introduced new widget for setting the IsaacMotionPlanningAPI collisionEnabled attribute

## [1.2.1] - 2025-12-01
### Changed
- Add missing license headers

## [1.2.0] - 2025-11-25
### Added
- Introduced widgets for the Robot Schema

## [1.1.3] - 2025-05-19
### Changed
- Update copyright and license to apache v2.0

## [1.1.2] - 2025-04-04
### Changed
- Version bump to fix extension publishing issues

## [1.1.1] - 2025-03-26
### Changed
- Cleanup and standardize extension.toml, update code formatting for all code

## [1.1.0] - 2025-03-06
### Added
- Introduced new widget for setting the isaac:namespace attribute

## [1.0.4] - 2025-01-21
### Changed
- Update extension description and add extension specific test settings

## [1.0.3] - 2025-01-07
### Fixed
- Dual Name Override Menu entries

## [1.0.2] - 2024-11-19
### Fixed
- Startup test

## [1.0.1] - 2024-10-24
### Changed
- Updated dependencies and imports after renaming

## [1.0.0] - 2024-09-27
### Changed
- Renamed extension to isaacsim.gui.property

## [0.2.3] - 2024-03-22
### Fixed
- Prim Custom Data field can support nested objects (dictionaries) now

## [0.2.2] - 2023-11-14
### Changed
- Prim Custom Data field can support arrays now

## [0.2.1] - 2023-06-12
### Changed
- Update to kit 105.1
- Python 3.11 super().__init__ added

## [0.2.0] - 2023-01-21
### Added
- Name Override widget

## [0.1.1] - 2020-10-09
### Changed
- File structure

## [0.1.0] - 2020-07-08
### Added
- Initial version, supports arrays and json
