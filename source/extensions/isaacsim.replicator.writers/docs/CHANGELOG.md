# Changelog

## [1.5.3] - 2026-05-05
- Enable multitick in all tests
- Tests now store results in uniquely-named temp directories to avoid run-to-run test pollution

## [1.5.2] - 2026-04-28
### Fixed
- `project_pinhole` now returns the screen center for camera points whose homogeneous `w` is near zero, preventing a divide-by-zero crash when projecting points on the camera's projection plane.
- `invert_fisheye_polynomial` now logs a warning with the residual and iteration count when Newton-Raphson fails to converge within `max_iterations`, instead of silently returning the last iterate.

## [1.5.1] - 2026-04-18
### Changed
- Added imperative-mood docstrings and `__all__` definitions

## [1.5.0] - 2026-04-13
### Changed
- Migrate extension implementation to core experimental API
- Mark the `PytorchListener` and `PytorchWriter` implementations as deprecated

## [1.4.1] - 2026-04-11
### Changed
- Add omni.kit.viewport.window to test dependencies

## [1.4.0] - 2026-03-04
### Changed
- Added Overview.md, python_api.md and updated docstrings

## [1.3.2] - 2026-02-18
### Changed
- Add WAR to split Windows camera tests into groups to avoid GPU crashes due to descriptor count

## [1.3.1] - 2026-02-06
### Changed
- Update deprecated Warp API calls to their updated names

## [1.3.0] - 2026-02-03
### Changed
- Added pinholeOpenCV and fisheyePolynomial projection support to pose writer
- Moved DOPE utils to DOPEWriter class

## [1.2.1] - 2025-12-01
### Changed
- Deprecate DOPEWriter and YCBVideoWriter writers
- Deprecate OgnPose and OgnDope nodes

## [1.2.0] - 2025-11-07
### Changed
- Updated pose writer to support explicit backends
- Updated pose writer tests to use golden images and functional API

## [1.1.0] - 2025-10-27
### Changed
- Replace import statements with the deprecation function when importing PyTorch
- Make omni.isaac.ml_archive an explicit test dependency

## [1.0.17] - 2025-09-01
### Fixed
- Make sure custom writers reset annotators list (`self.annotators = []`) on initialization

## [1.0.16] - 2025-08-21
### Changed
- Fix PIL image conversion warnings

## [1.0.15] - 2025-07-07
### Fixed
- Correctly enable omni.kit.loop-isaac in test dependency (fixes issue from 1.0.14)

## [1.0.14] - 2025-07-03
### Changed
- Make omni.kit.loop-isaac an explicit test dependency

## [1.0.13] - 2025-06-25
### Changed
- Add --reset-user to test args

## [1.0.12] - 2025-05-31
### Changed
- Use default nucleus server for all tests

## [1.0.11] - 2025-05-19
### Changed
- Update copyright and license to apache v2.0

## [1.0.10] - 2025-05-16
### Changed
- Make extension target a specific kit version

## [1.0.9] - 2025-05-10
### Changed
- Enable FSD in test settings

## [1.0.8] - 2025-05-07
### Changed
- PoseWriter test: quaternion comparison both positive and negative quaternions
- PoseWriter test: added float comparison tolerance atol
- PoseWriter test: change the json golden output to store the new replicator rotation (negative of the previous one)

## [1.0.7] - 2025-04-11
### Fixed
- Fixed PoseWriter size output from AABB to Oriented Bounding Box

### Added
- Added PoseWriter test

## [1.0.6] - 2025-04-09
### Changed
- Update all test args to be consistent

## [1.0.5] - 2025-04-04
### Changed
- Version bump to fix extension publishing issues

## [1.0.4] - 2025-03-26
### Changed
- Cleanup and standardize extension.toml, update code formatting for all code

## [1.0.3] - 2025-03-05
### Changed
- Update extension codebase to adhere to isaac sim extension structure and file naming  guidelines

## [1.0.2] - 2025-03-04
### Changed
- Update to kit 107.1 and fix build issues

## [1.0.1] - 2025-01-21
### Changed
- Update extension description and add extension specific test settings

## [1.0.0] - 2024-12-09
### Added
- Created extension with writer parts from omni.replicator.isaac
