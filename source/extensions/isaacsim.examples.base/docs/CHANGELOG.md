# Changelog
## [1.3.1] - 2026-03-04
### Changed
- Changed default physics device to CPU in BaseSampleExperimental

### Fixed
- Fixed physics device reapplication after simulation reset in BaseSampleExperimental

## [1.3.0] - 2026-03-04
### Changed
- Added Overview.md, python_api.md and updated docstrings

## [1.2.0] - 2026-03-03
### Changed
- Replaced timeline API with app utils for simulation control

## [1.1.5] - 2026-01-10
### Changed
- Add functionality to allow setting rendering_dt.

## [1.1.4] - 2026-01-06
### Changed
- Migrate more events to Events 2.0.

## [1.1.3] - 2025-12-07
### Changed
- Added dependency on isaacsim.core.rendering_manager

## [1.1.2] - 2025-12-02
### Changed
- Added a note for the physics scene bug in 1.1.1
- Remove references to core api

## [1.1.1] - 2025-12-01
### Changed
- Creates physics scene if it does not exist on the stage to correctly set physics time steps

## [1.1.0] - 2025-11-24
### Changed
- Moved module from isaacsim.base_samples to isaacsim.examples.base

## [1.0.2] - 2025-11-21
### Removed
- Build window function

## [1.0.1] - 2025-11-20
### Removed
- Dependency on Simulation Context

## [1.0.0] - 2025-10-23
### Added
- Initial release of isaacsim.examples.base extension
- BaseSampleExperimental class for creating sample applications
- BaseSampleUITemplateExperimental class for experimental interactive UI templates
