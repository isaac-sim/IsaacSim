# Changelog

## [1.3.1] - 2026-06-09
### Fixed
- Fix linter errors and missing or incomplete docstrings.

## [1.3.0] - 2026-03-10
### Changed
- Compatible with isaacsim.physics.newton 0.6.0 (Newton 1.0.0).

## [1.2.2] - 2026-03-06
### Changed
- Upgraded Newton pip package from 1.0.0rc2 to 1.0.0rc3.

## [1.2.1] - 2026-03-05
### Changed
- Mujoco schema properties are now sorted alphabetically with the exception of properties that are common to Newton and Mujoco: those are put first in the list.

## [1.2.0] - 2026-03-04
### Added
- Resolver-aware property visibility: Newton and Mujoco schema properties are hidden or disabled when the other resolver (Newton vs MuJoCo) provides the value. Preference is determined by first authored value, otherwise Newton.

### Changed
- Property builders in `mujoco_schemas` and `newton_schemas` use callbacks to hide scene, joint, shape, and material properties when the resolver mapping is provided by the other backend.

## [1.1.0] - 2026-03-04
### Changed
- Add Overview.md, python_api.md and update docstrings

## [1.0.0] - 2026-02-06
### Changed
- Initial submit
