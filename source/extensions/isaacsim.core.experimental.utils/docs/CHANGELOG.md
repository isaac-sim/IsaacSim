# Changelog

## [0.17.2] - 2026-06-09
### Fixed
- Fix linter errors and missing or incomplete docstrings.

## [0.17.1] - 2026-04-28
### Changed
- Cache `arange` index arrays globally in `ops.resolve_indices` (keyed by `(count, dtype, device)`) so repeated `x=None` calls across all callers reuse the same Warp array without re-allocating

## [0.17.0] - 2026-04-24
### Added
- Add `upgrade_prim_semantics_to_labels` to semantics utils for migrating deprecated `Semantics.SemanticsAPI` to `UsdSemantics.LabelsAPI`

## [0.16.2] - 2026-04-17
### Fixed
- Fix docstring example in `get_simstate_mode()`

## [0.16.1] - 2026-04-16
### Added
- Add `SimStateMode` enum and `get_simstate_mode()` for SimState backend selection

## [0.16.0] - 2026-04-03
### Changed
- **Breaking:** `euler_angles_to_quaternion` and `euler_angles_to_rotation_matrix` now expect `[roll, pitch, yaw]` (XYZ) input order for both extrinsic and intrinsic conventions, enabling clean euler-to-quaternion-to-euler round trips

## [0.15.0] - 2026-04-01
### Added
- Add `get_relative_transform` to xform utils for computing relative transforms between USD prims
- Add `compute_relative_transform` to transform utils for computing relative transforms from world matrices (accepts `list`, `np.ndarray`, or `wp.array`)

## [0.14.0] - 2026-03-25
### Added
- Add `look_at_matrix` to `transform` to compute a USD camera transform (`Gf.Matrix4d`) from eye and target positions, with automatic collinearity fallback

## [0.13.1] - 2026-03-20
### Changed
- Skip update_app tests in async environment

## [0.13.0] - 2026-03-10
### Added
- Add Isaac Sim's `gridroom` stage template

## [0.12.0] - 2026-03-04
### Changed
- Add Overview.md, python_api.md and updated docstrings

## [0.11.4] - 2026-02-06
### Changed
- Update deprecated Warp API calls to their updated names

## [0.11.3] - 2026-02-02
### Fixed
- Fix NaN issue in euler to quaternion conversion utils

## [0.11.2] - 2026-01-27
### Fixed
- Fix inconsistency in euler to quaternion conversion utils

## [0.11.1] - 2026-01-22
### Changed
- Replace the `isaacsim.core.version` extension used for app utils test cases by `omni.pip.cloud`

## [0.11.0] - 2026-01-16
### Added
- Add `get_prim_attribute_names()` to prim utils for listing authored attribute names
- Add `get_prim_attribute_value()` to prim utils for getting attribute values

## [0.10.0] - 2026-01-13
### Added
- Add `quaternion_to_euler_angles()` function to transform utils for converting quaternions to euler angles

## [0.9.0] - 2026-01-07
### Added
- Add `parse_device()` function to ops utils for parsing Warp device specifications
- Add `ensure_api()` function to prim utils for ensuring API schemas are applied to prims

## [0.8.1] - 2025-12-15
### Changed
- Update app module docstrings example and add module summaries for docs purposes

## [0.8.0] - 2025-11-26
### Added
- Add timeline-related functions to app utils

## [0.7.0] - 2025-11-21
### Added
- Add xform utils
- Support USD schemas when getting a prim or prim path

## [0.6.1] - 2025-11-12
### Changed
- Return the input as it is when getting a prim or prim path, provided that the input is of the expected return type

## [0.6.0] - 2025-11-04
### Added
- Add app utils
- Add semantics utils

## [0.5.0] - 2025-10-29
### Added
- Add stage utils functions to:
  - Check whether the stage is loading
  - Generate a string representation of the stage
  - Move a prim to a different location on the stage hierarchy
  - Delete a prim from the stage

## [0.4.0] - 2025-10-22
### Added
- Add prim utils functions to:
  - Find all the prim paths in the stage that match the given (regex) path
  - Check whether a prim corresponds to a non-root link in an articulation

## [0.3.0] - 2025-09-09
### Added
- Add stage utils functions to:
  - Set the current stage units
  - Get and set current stage up axis and time code
  - Save and close the current stage
  - Generate next free path
- Add prim utils function to create prim attributes
- Add foundation utils

## [0.2.3] - 2025-09-05
### Fixed
- Fix test failure

## [0.2.2] - 2025-08-13
### Added
- More functionality has been added to Transform utils for manipulating rotations and quaternions

## [0.2.1] - 2025-07-31
### Added
- Transform utils functions to manipulate rotations and quaternions
- Stage utils function to get the current stage units

## [0.2.0] - 2025-07-16
### Added
- Add function to check if a prim has or not the given API schema(s) applied

### Changed
- Update the predicate function signatures of the prim utils to forward the test prim instance

## [0.1.5] - 2025-07-07
### Fixed
- Correctly enable omni.kit.loop-isaac in test dependency (fixes issue from 0.1.4)

## [0.1.4] - 2025-07-03
### Changed
- Make omni.kit.loop-isaac an explicit test dependency

## [0.1.3] - 2025-06-30
### Changed
- Fix docstrings example

## [0.1.2] - 2025-06-25
### Changed
- Add --reset-user to test args

## [0.1.1] - 2025-06-07
### Changed
- Fix test settings

## [0.1.0] - 2025-06-06
### Added
- Initial release
