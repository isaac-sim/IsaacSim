# Changelog

## [0.7.1] - 2026-05-12
### Fixed
- Depth sensor wrappers returned by `_wrap_depth_sensor_cameras` are now stored in `Extension._depth_sensors` instead of being discarded, preventing a create/destroy cycle that corrupted the RTX pipeline and made asset materials invisible.

## [0.7.0] - 2026-05-07
### Changed
- Sensor list (vendors, models, depth-sensor flags, USD asset paths) is now sourced from `isaacsim.sensors.experimental.rtx.SUPPORTED_CAMERA_CONFIGS`. `Extension.SENSORS` is built at import time from that registry via `get_camera_metadata`, so adding a new camera vendor/model is a one-place change in the registry. The legacy `Extension.SENSORS` shape (`{vendor: {display_name: {prim_prefix, usd_path, is_depth_sensor}}}`) is preserved for backward compatibility.
- Default `prim_prefix` for menu-created sensors is now derived from the USD file stem (with hyphens and dots replaced by underscores) rather than per-asset hand-curated values. User-visible effect: prim names placed via the menu change in some cases (e.g. `/RealsenseD455` -> `/rsd455`, `/Femto` -> `/orbbec_femtomega_v1_0`, `/OAK4D` -> `/oak4_d`). Prim names that already matched the stem (e.g. `/Inspector83x`, `/ZED_X`) are unchanged.
- Menu actions now load camera USDs via `RtxCamera.create()` (experimental) instead of a raw Xform reference. For depth-sensor entries, every Camera in the loaded asset that has a template render product with `OmniSensorDepthSensorSingleViewAPI` is wrapped with `SingleViewDepthCameraSensor`, which copies the template's depth-sensor attributes onto the new render product (matches the prior `SingleViewDepthSensorAsset.initialize()` behavior).

### Added
- Dependency on `isaacsim.sensors.experimental.rtx` for `SUPPORTED_CAMERA_CONFIGS` / `get_camera_metadata` / `RtxCamera` / `SingleViewDepthCameraSensor`.
- Dependency on `isaacsim.core.experimental.utils` for stage utilities used to walk loaded assets when wrapping depth sensors.

### Removed
- Dependency on the deprecated `isaacsim.sensors.camera` extension. The `SingleViewDepthSensorAsset` import is replaced by `RtxCamera.create()` + `SingleViewDepthCameraSensor` from the experimental extension.
- Implicit dependency on the deprecated `isaacsim.core.utils.stage`: `get_next_free_path` replaced by `isaacsim.core.experimental.utils.stage.generate_next_free_path` (same `prepend_default_prim=True` behavior); `clear_stage` (used by `test_camera_context_menu.py`) replaced by `stage_utils.create_new_stage_async()` to match the rest of the experimental test suite.

## [0.6.0] - 2026-05-04
### Added
- Luxonis depth cameras: OAK4-D, OAK4-D Wide, OAK-D Pro PoE, OAK-D Pro W PoE, OAK-D ToF
- SICK Inspector83x and InspectorP61x 2D cameras
- SICK safeVisionary2 and Visionary-T Mini, registered as depth sensors

### Changed
- `test_camera_context_menu_count` derives the expected count from `Extension.SENSORS` instead of a hardcoded sum

## [0.5.1] - 2026-04-21
### Changed
- Added multitick rendering test arguments (`supportMultiTickRate`, `perSensorTickTlas`)
- Updated Overview.md to reference `isaacsim.sensors.experimental.rtx` as the target migration extension

## [0.5.0] - 2026-03-04
### Changed
- Added Overview.md and python_api.md and updated docstrings

## [0.4.2] - 2026-02-18
### Fixed
- Replaced deprecated `onclick_fn` with `onclick_action` in menu items to eliminate deprecation warnings
- Registered proper actions for all camera and depth sensor creation menu items

## [0.4.1] - 2026-01-24
### Changed
- Fix issues with menu click and context menu tests being flaky

## [0.4.0] - 2025-12-22
### Changed
- Refactor unit tests to use isaacsim.test.utils.

## [0.3.0] - 2025-11-06
### Added
- New Realsense category, with D455, D457, and D55 models.

### Removed
- Intel as category

## [0.2.3] - 2025-10-27
### Changed
- Make omni.isaac.ml_archive an explicit test dependency

## [0.2.2] - 2025-09-27
### Added
- Restored SICK Inspector83x

## [0.2.1] - 2025-08-04
### Fixed
- Asset path for ZED X sensor

## [0.2.0] - 2025-07-17
### Changed
- Depth sensors now added using SingleViewDepthSensorAsset API

## [0.1.16] - 2025-07-07
### Fixed
- Correctly enable omni.kit.loop-isaac in test dependency (fixes issue from 0.1.15)

## [0.1.15] - 2025-07-03
### Changed
- Make omni.kit.loop-isaac an explicit test dependency

## [0.1.14] - 2025-06-25
### Changed
- Add --reset-user to test args

## [0.1.13] - 2025-05-31
### Changed
- Use default nucleus server for all tests

## [0.1.12] - 2025-05-19
### Changed
- Update copyright and license to apache v2.0

## [0.1.11] - 2025-05-10
### Changed
- Enable FSD in test settings

## [0.1.10] - 2025-04-16
### Changed
- Assets root path lookup moved to on-click callback, rather than on extension startup

## [0.1.9] - 2025-04-09
### Changed
- Update all test args to be consistent

## [0.1.8] - 2025-04-04
### Changed
- Version bump to fix extension publishing issues

## [0.1.7] - 2025-03-26
### Changed
- Cleanup and standardize extension.toml, update code formatting for all code

## [0.1.6] - 2025-03-20
### Added
- Sensors to context menu

## [0.1.5] - 2025-03-11
### Changed
- Switch asset root for tests to internal nucleus

## [0.1.4] - 2025-01-26
### Changed
- Update test settings

## [0.1.3] - 2025-01-21
### Changed
- Update extension description and add extension specific test settings

## [0.1.2] - 2025-01-06
### Changed
- Updated menu name of SG8-AR0820C-5300-G2A-H120YA, SG8-AR0820C-5300-G2A-H30YA, SG8-AR0820C-5300-G2A-H60SA to SG8S-AR0820C-5300-G2A-H120YA, SG8S-AR0820C-5300-G2A-H30YA, SG8S-AR0820C-5300-G2A-H60SA

## [0.1.1] - 2024-12-20
### Changed
- Corrected menu path of the H30SA camera to H30YA

## [0.1.0] - 2024-12-10
### Added
- Initial version of isaacsim.sensors.camera.ui
