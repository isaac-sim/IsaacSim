# Changelog

## [1.1.4] - 2026-05-11
### Fixed
- `get_gripper_joint_states` returns POSIX joint keys on Windows (replaced `os.path.relpath` with a string slice).

## [1.1.3] - 2026-04-28
### Fixed
- `simulate_physics_async` and `simulate_physics_with_forces_async` now restore the PhysX scene's original `updateType` (or clear it if it was unauthored) via try/finally, instead of permanently leaving the scene set to `Disabled` after manual stepping.

## [1.1.2] - 2026-04-18
### Changed
- Added return type annotations, `from __future__ import annotations`, and imperative-mood docstrings

## [1.1.1] - 2026-03-05
### Changed
- Migrate extension implementation to core experimental API

## [1.1.0] - 2026-03-04
### Changed
- Added Overview.md, python_api.md and updated docstrings

## [1.0.10] - 2026-02-09
### Fixed
- Defer `scipy.stats` import in `sampler_utils.py` to avoid module-level import failure under pycoverage

## [1.0.9] - 2025-07-07
### Fixed
- Correctly enable omni.kit.loop-isaac in test dependency (fixes issue from 1.0.8)

## [1.0.8] - 2025-07-03
### Changed
- Make omni.kit.loop-isaac an explicit test dependency

## [1.0.7] - 2025-06-25
### Changed
- Add --reset-user to test args

## [1.0.6] - 2025-06-03
### Changed
- Fix incorrect licenses and add missing licenses

## [1.0.5] - 2025-05-31
### Changed
- Use default nucleus server for all tests

## [1.0.4] - 2025-05-30
### Changed
- Check if rtree/libspatialindex is installed, if not skip tests

## [1.0.3] - 2025-05-22
### Changed
- Update copyright and license to apache v2.0

## [1.0.2] - 2025-05-20
### Changed
- Workflow fixes and improvements

## [1.0.1] - 2025-05-16
### Changed
- Make extension target a specific kit version

## [1.0.0] - 2025-05-10
### Added
- Extension to sample and execute grasp poses from a mesh.
