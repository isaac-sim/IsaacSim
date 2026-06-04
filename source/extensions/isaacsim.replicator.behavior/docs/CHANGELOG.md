# Changelog
## [1.5.3] - 2026-05-19
### Changed
- Clarified `LocationRandomizer` docstrings for the `frame:useRelativeFrame` and `frame:targetPrimPath` exposed attributes to disambiguate how the random offset is composed with the initial location and the optional target prim.

## [1.5.2] - 2026-05-08
### Fixed
- `TextureRandomizer._apply_behavior` now skips the tick (with a warning) when no texture URLs are configured, instead of letting `numpy.random.Generator.choice` raise on an empty list.

### Added
- Test covering `TextureRandomizer._apply_behavior` with an empty `_texture_urls` list.

## [1.5.1] - 2026-04-28
### Fixed
- `LightRandomizer._reset` now skips cached `inputs:intensity`/`inputs:color` values that were unauthored at setup time and logs a warning instead of writing `None` back to the prim.
- `decompose_rotation` now supports single-axis `xformOp:rotateX|Y|Z` ops by returning a scalar angle and raises `ValueError` for unsupported rotation orders; `set_rotation_with_ops` catches the error and logs a warning.

## [1.5.0] - 2026-04-22
### Changed
- Dropped the `omni.kit.window.property` dependency. The property window refresh is now dispatched via the `isaacsim.replicator.behavior.EXPOSED_VARS_CHANGED` carb event, which `isaacsim.replicator.behavior.ui` subscribes to, so the core behaviors can run headless.

### Fixed
- Made `_setup` idempotent in `TextureRandomizer`, `RotationRandomizer`, `LocationRandomizer`, `LightRandomizer`, and `LookAtBehavior`. A play/pause/play loop (as used by the SDG capture pipeline) previously re-cached the already-randomized state as "initial", so `on_stop` restored stale values (e.g. pallets left bound to a removed randomizer material and rendered gray). Exposed variables are still re-read on every call; prim resolution, initial-state caching, and material creation run only on the first entry of a play session.

## [1.4.2] - 2026-04-18
### Changed
- Added return type annotations, `from __future__ import annotations`, and imperative-mood docstrings

## [1.4.1] - 2026-04-11
### Changed
- Add omni.kit.viewport.window to test dependencies

## [1.4.0] - 2026-03-04
### Changed
- Added Overview.md, python_api.md and updated docstrings

## [1.3.7] - 2026-02-26
### Changed
- Migrated behavior scripting dependency from omni.kit.scripting to omni.behavior.scripting.core

## [1.3.6] - 2026-02-21
### Changed
- Updated SDG physics based volume filling behavior script to use the omni.physx api

## [1.3.5] - 2026-02-03
### Changed
- Update sdg pipeline golden images with new randomization determinism by pausing the timeline before capturing frames

## [1.3.4] - 2026-01-22
### Changed
- Increased test tolerances for golden image comparison until randomizations are deterministic

## [1.3.3] - 2026-01-22
### Changed
- Disable behavior test output until tests are run

## [1.3.2] - 2026-01-06
### Changed
- Migrate more events to Events 2.0.

## [1.3.1] - 2025-12-16
### Changed
- Migrate extension implementation to core experimental API

## [1.3.0] - 2025-12-03
### Changed
- Added explicit seed to randomizers to make them deterministic
- Updated sdg pipeline golden images

## [1.2.1] - 2025-10-27
### Changed
- Make omni.isaac.ml_archive an explicit test dependency

## [1.2.0] - 2025-10-17
### Changed
- Migrate PhysX subscription and simulation control interfaces to Omni Physics

## [1.1.16] - 2025-09-16
### Fixed
- Added kit update after timeline.stop to fix flaky test due to stage loading status

## [1.1.15] - 2025-09-15
### Changed
- Switched to isaacsim.test.utils for golden image comparison in tests

## [1.1.14] - 2025-07-17
### Changed
- Updated windows golden images

## [1.1.13] - 2025-07-11
### Fixed
- Texture randomizer uses unique names for materials (WAR for ISIM-4054)

### Changed
- Update sdg pipeline golden images

## [1.1.12] - 2025-07-07
### Fixed
- Correctly enable omni.kit.loop-isaac in test dependency (fixes issue from 1.1.11)

## [1.1.11] - 2025-07-03
### Changed
- Make omni.kit.loop-isaac an explicit test dependency

## [1.1.10] - 2025-06-25
### Changed
- Add --reset-user to test args

## [1.1.9] - 2025-06-08
### Changed
- Update sdg pipeline golden images on windows

## [1.1.8] - 2025-05-31
### Changed
- Use default nucleus server for all tests

## [1.1.7] - 2025-05-23
### Changed
- New sdg pipeline golden images
- Removed npy comparison because of redunancy with png depth comparison
- Renamed output folder to _out_behaviors_sdg with leading underscore for consistency

## [1.1.6] - 2025-05-19
### Changed
- Update copyright and license to apache v2.0

## [1.1.5] - 2025-05-16
### Changed
- Make extension target a specific kit version

## [1.1.4] - 2025-05-12
### Changed
- Using os.path.join to create output directory in sdg tests

## [1.1.3] - 2025-05-10
### Changed
- Enable FSD in test settings

## [1.1.2] - 2025-05-07
### Changed
- Added checks for stage validity when destroying / resetting behaviors

## [1.1.1] - 2025-04-26
### Changed
- SDG pipeline: Increased rgb mean diff tolerance and included depth comparison with small mean diff tolerance
- SDG pipeline: Split into linux and windows golden images

## [1.1.0] - 2025-04-21
### Changed
- Event 2.0 handling
- Updated sdg pipeline golden images

## [1.0.14] - 2025-04-17
### Changed
- Changed add_update_semantics to add_labels

## [1.0.13] - 2025-04-09
### Changed
- Update all test args to be consistent

## [1.0.12] - 2025-04-04
### Changed
- Version bump to fix extension publishing issues

## [1.0.11] - 2025-03-26
### Changed
- Cleanup and standardize extension.toml, update code formatting for all code

## [1.0.10] - 2025-03-24
### Changed
- Migrate to Events 2.0

## [1.0.9] - 2025-03-11
### Changed
- Switch asset root for tests to internal nucleus

## [1.0.8] - 2025-01-26
### Changed
- Update test settings

## [1.0.7] - 2025-01-21
### Changed
- Update extension description and add extension specific test settings

## [1.0.6] - 2025-01-09
### Fixed
- Initial behavior not applied if interval is greater than 0

### Added
- Util functions for adding scripts and triggering and waiting for behavior events
- SDG scenario test with golden images

## [1.0.5] - 2024-12-12
### Fixed
- Cleaning up randomizers by calling reset when destroyed during play

## [1.0.4] - 2024-12-05
### Fixed
- Fixed volume stack behavior phyisics material assignment (not needed on parent prim)
- Avoid delta_time=0.0 randomizations through orchestrator.step() stage updates
- Exposed variables removed from fabric as well (avoid UI still showing them)
- Fixed reset simulation case without any performed simulation

## [1.0.3] - 2024-11-27
### Fixed
- Fixed error when removing all the scripts widget during runtime

## [1.0.2] - 2024-11-06
### Fixed
- Fixed stacking bounding box drop area calculation for scaled assets
- Fixed get world rotation util function to use Gf.Transform to get the rotation in case of transforms with scale/shear
- Check for previous colliders/rigid bodies for assets before simulation

### Changed
- Set exposed variables default values

## [1.0.1] - 2024-10-31
### Fixed
- Fixed 'remove_empty_scopes' to check if the prim is valid before searching for Scope or GenericPrim types

## [1.0.0] - 2024-09-29
### Added
- Added initial behavior script based randomizers
