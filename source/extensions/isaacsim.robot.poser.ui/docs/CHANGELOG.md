# Changelog

## [1.1.5] - 2026-09-25
### Fixed
- Report unsupported pose transforms at UI callback boundaries and stop failed tracking initialization.

## [1.1.4] - 2026-08-24
### Fixed
- Named pose table and property panel no longer log "Widget[] was destroyed during event or draw" errors.

## [1.1.3] - 2026-07-30
### Changed
- Use Robot Poser's public kinematics imports.

## [1.1.2] - 2026-07-07
### Changed
- Use supported package-root imports for cross-extension APIs.

## [1.1.1] - 2026-03-17
### Changed
- Migrate extension implementation to core experimental API

## [1.1.0] - 2026-03-05
### Changed
- Added Overview.md, python_api.md and updated docstrings

## [1.0.0] - 2025-12-16
### Added
- Initial release of UI for managing robot poses:
  - Provides a window for selecting robots, editing named poses, and configuring inverse kinematics.
  - Enables real-time IK solving, pose editing, and application of poses to robots.
  - Includes property panel integration and visual feedback for IK status.
