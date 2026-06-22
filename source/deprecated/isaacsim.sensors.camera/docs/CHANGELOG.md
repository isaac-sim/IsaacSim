# Changelog

## [1.7.9] - 2026-06-09
### Fixed
- Fix linter errors and missing or incomplete docstrings, and update `python_api.md`.

## [1.7.8] - 2026-05-18
### Fixed
- Log a warning when `Camera.set_frequency()` / `set_dt()` fall back to "process all frames" because `/app/runLoops/main/rateLimitFrequency` is unset, instead of silently discarding the requested value

## [1.7.7] - 2026-05-10
### Fixed
- Raise a clear error when `Camera.attach_annotator()` is called before `Camera.initialize()`

## [1.7.6] - 2026-05-08
### Fixed
- Fix `CameraView` docstring listing `int32` for `instance_segmentation_fast` and `instance_id_segmentation_fast`; runtime dtype is `uint32` (matches `ANNOTATOR_SPEC` and `semantic_segmentation`)

## [1.7.5] - 2026-04-21
### Changed
- Updated deprecation warning and Overview.md to reference `isaacsim.sensors.experimental.rtx` (was `isaacsim.sensors.experimental.camera`)

## [1.7.4] - 2026-04-17
### Fixed
- Fix `Camera.set_dt` rejecting valid sub-rendering-rate frequencies due to float modulo precision
- Fix `Camera.get_dt` returning -1.0 when rendering frequency is unknown (now returns 0.0)
- Fix `set_fisheye_polynomial_properties` silently skipping valid 0.0 values due to bare truthiness checks
- Fix `set_kannala_brandt_properties` warning message saying "expecting 5" when guard enforces minimum 4
- Fix case-sensitive `"pinhole"` substring check failing for `"opencvPinhole"` in 4 projection methods
- Fix `set_lens_distortion_model("pinhole")` removing all applied schemas instead of only lens distortion schemas

## [1.7.3] - 2026-04-03
### Changed
- Adapt test euler angles to new `[roll, pitch, yaw]` input convention

## [1.7.2] - 2026-03-31
### Deprecated
- Extension deprecated in favor of `isaacsim.sensors.experimental.rtx`

## [1.7.1] - 2026-03-18
### Changed
- Disable `GroundPlane` template on test cases

## [1.7.0] - 2026-03-04
### Changed
- Added Overview.md and python_api.md and updated docstrings

## [1.6.6] - 2026-02-13
### Changed
- OMPE-78945: Split tests into two groups to avoid MMUFault|SMError|||imgui.pixel on Windows-Vk

## [1.6.5] - 2026-02-06
### Changed
- Update deprecated Warp API calls to their updated names

## [1.6.4] - 2026-02-06
### Changed
- omni:rtx:post:depthSensor:outlierRemovalEnabled is now a bool

## [1.6.3] - 2026-02-03
### Changed
- Camera sensor: updated test camera orientation to use experimental rotation utils for euler angles to quaternion

## [1.6.2] - 2026-01-20
### Changed
- Moved to experimental APIs in tests for visual objects and rotation utils

### Fixed
- TestSingleViewDepthSensor.test_getter_setter_methods uses correct initial value for confidenceThreshold.

## [1.6.1] - 2026-01-14
### Fixed
- Cleanup annotators and state properly when the camera is destroyed

## [1.6.0] - 2026-01-13
### Changed
- Camera sensor: switched to use "_fast" version of the annotators where available ("bounding_box_2d_tight_fast", "bounding_box_2d_loose_fast", "instance_segmentation_fast", "instance_id_segmentation_fast")
- Tests: removed `World` from tests, using `timeline.play()` and `timeline.stop()` to provide sensors with data
- Tests: changed to use `SimulationManager` for backend tests
- Tests: changed to use `isaacsim.test.utils.image_comparison` for golden image comparison
- Tests: changed the default environment from loading a USD to creating a simple plane and dome light for tests
- Tests: updated and re-organized golden image data
- Tests: kept debug pointcloud images for future reference

## [1.5.5] - 2026-01-09
### Fixed
- Fixed camera_view.get_data() resolution order issue (height, width) -> (width, height)

## [1.5.4] - 2026-01-06
### Changed
- Migrate more events to Events 2.0.

## [1.5.3] - 2025-12-10
### Fixed
- Removed `do_array_copy=True` workaround in tiled sensor (fixed upstream in replicator.core 1.12.32 by changing strides type from int32 to int64 to avoid warp array arithemtic when getting annotator data)

## [1.5.2] - 2025-12-06
### Changed
- Added validation checks and warmup warnings to camera sensor data methods to handle unavailable data
- Added warmup tests for camera sensor checking for warnings and data availability

## [1.5.1] - 2025-12-05
### Changed
- Migrate to Events 2.0.

## [1.5.0] - 2025-11-26
### Added
- Unit test for get_view_matrix_ros

## [1.4.1] - 2025-11-25
### Fixed
- Fixed issue with tiled sensor data slicing by copying the data from the annotator (do_array_copy=True)

## [1.4.0] - 2025-10-27
### Changed
- Replace import statements with the deprecation function when importing PyTorch
- Make omni.isaac.ml_archive an explicit test dependency

## [1.3.7] - 2025-10-22
### Changed
- Remove deprecated time related APIs from CoreNodes interface

## [1.3.6] - 2025-09-23
### Fixed
- SingleViewDepthSensorAsset correctly sets position, orientation, translation on __init__.

## [1.3.5] - 2025-09-22
### Fixed
- SingleViewDepthSensorAsset.initialize validates depth sensor attributes before setting on render product prims.

## [1.3.4] - 2025-09-04
### Fixed
- Camera.set_opencv_pinhole_properties and Camera.set_opencv_fisheye_properties now correctly set imageSize attribute as Camera._resolution.

## [1.3.3] - 2025-08-21
### Changed
- Fix PIL image conversion warnings

## [1.3.2] - 2025-07-29
### Fixed
- Added error exclusion to ignore depth sensor allocation errors in tests

## [1.3.1] - 2025-07-21
### Changed
- Added explicit `destroy()` method to `CameraView`

### Fixed
- Fixed CameraView to use the correct render product variable `_tiled_render_product`

## [1.3.0] - 2025-07-19
### Added
- SingleViewDepthSensorAsset API to wrap around USDs and automatically create SingleViewDepthSensor objects (including render products) for cameras with associated template render products.

## [1.2.10] - 2025-07-18
### Changed
- Added explicit destroy() method to Camera to manually clean up resources

## [1.2.9] - 2025-07-07
### Fixed
- Correctly enable omni.kit.loop-isaac in test dependency (fixes issue from 1.2.8)

## [1.2.8] - 2025-07-03
### Changed
- Make omni.kit.loop-isaac an explicit test dependency

## [1.2.7] - 2025-06-29
### Fixed
- Camera view issues with different backends and devices (ISIM-3498)
- Avoid slicing None data if data is not yet available in Camera View and Camera sensor

### Changed
- Removed unused local backend variables in Camera sensor
- Update image comparison tolerance for camera view sensor tests

### Added
- Camera view sensor tests for different backends and devices

## [1.2.6] - 2025-06-26
### Changed
- Update unit test golden values and tolerances

## [1.2.5] - 2025-06-25
### Changed
- Add --reset-user to test args

## [1.2.4] - 2025-06-05
### Changed
- Added checks to camera sensor horizontal and vertical aperture to ensure square pixels are maintained
- Updated camera sensor projection golden data with aperture values ensuring square pixels
- Added separate pointcloud tests

## [1.2.3] - 2025-06-04
### Changed
- Switch to get_lens_distortion_model in deprecated methods

## [1.2.2] - 2025-06-03
### Changed
- Fix incorrect licenses and add missing licenses

## [1.2.1] - 2025-05-31
### Changed
- Use default nucleus server for all tests

## [1.2.0] - 2025-05-30
### Added
- SingleViewDepthSensor API

### Changed
- Camera class consistently updates internal _prim member.

## [1.1.2] - 2025-05-27
### Fixed
- Bug causing downstream graphs to be ticked twice per frame

## [1.1.1] - 2025-05-19
### Changed
- Update copyright and license to apache v2.0

### Changed
- Camera.initialize() now optionally attaches RGB annotator
- Camera constructor no longer sets default lens distortion model to OmniLensDistortionFthetaAPI

## [1.0.7] - 2025-05-12
### Changed
- Use pinhole model for camera class by default

## [1.0.6] - 2025-05-10
### Changed
- Enable FSD in test settings

## [1.0.5] - 2025-05-06
### Changed
- Annotator device is taken into account in initialize() method as well of Camera sensor

## [1.0.4] - 2025-05-05
### Changed
- Fix API docs.

## [1.0.3] - 2025-04-30
### Changed
- Update event subscriptions to Event 2.0 system

## [1.0.2] - 2025-04-29
### Changed
- [OMPE-45288] Removed pointcloud cuda singleton data squeeze (1, N, 3) -> (N, 3)

## [1.0.1] - 2025-04-17
### Changed
- Changed add_update_semantics to add_labels

## [1.0.0] - 2025-04-09
### Changed
- Updates Camera API to account for new lens distortion schemas

## [0.4.1] - 2025-04-09
### Changed
- Update all test args to be consistent

## [0.4.0] - 2025-03-27
### Changed
- Added 'annotator_device' parameter to the camera sensor to support GPU data access
- Helper functions can return data with the selected backend

## [0.3.2] - 2025-03-26
### Changed
- Cleanup and standardize extension.toml, update code formatting for all code

## [0.3.1] - 2025-03-26
### Changed
- Updated OpenCV lens distortion schema attribute names

## [0.3.0] - 2025-03-11
### Changed
- Deprecate Camera OpenCV-related APIs in favor of native OpenCV camera models.

## [0.2.12] - 2025-03-11
### Changed
- Switch asset root for tests to internal nucleus

## [0.2.11] - 2025-02-21
### Changed
- Updated annotators in camera sensor to use the `_fast` version where available

## [0.2.10] - 2025-02-17
### Changed
- Camera get_pointcloud method uses the 'pointcloud' annotator if set, otherwise it falls back to a depth-based calculation

### Fixed
- Centered the pointcloud points by adding a half-pixel offset for the depth-based calculation

### Added
- Camera sensor pointcloud specific tests
- Camera sensor 'get_pointcloud()' can return the data in world of camera frame

## [0.2.9] - 2025-01-26
### Changed
- Update test settings

## [0.2.8] - 2025-01-21
### Changed
- Update extension description and add extension specific test settings

## [0.2.7] - 2025-01-14
### Fixed
- Issues when output device did not match the device annotated data was acquired on

## [0.2.6] - 2025-01-06
### Fixed
- Use indexed cuda:{idx} input for warp kernels in camera view class
- Use indexed cuda device to pre-allocate out buffers

## [0.2.5] - 2024-12-31
### Fixed
- Camera sensor tests no longer needs OMPE-28827 WAR

## [0.2.4] - 2024-12-03
### Changed
- Isaac Util menu to Tools->Robotics menu

### Fixed
- Camera view sensor test warp.types.int32 -> warp.types.uint32
- Decreased image comparison threshold with 0.95->0.94

## [0.2.3] - 2024-11-26
### Fixed
- Camera sensor tests fix for colorize param

## [0.2.2] - 2024-11-18
### Fixed
- Fixed camera sensor custom parameter output test

## [0.2.1] - 2024-11-07
### Added
- Added init_params to camera sensor parameters

## [0.2.0] - 2024-11-01
### Added
- Add a tiled camera method to get tiled and batched data from the different annotators/sensors

## [0.1.1] - 2024-10-24
### Changed
- Updated dependencies and imports after renaming

## [0.1.0] - 2024-09-24
### Added
- Initial version of isaacsim.sensors.camera
