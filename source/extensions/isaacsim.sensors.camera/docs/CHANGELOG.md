# Changelog
## [1.3.1] - 2025-07-21
### Changed
- Added explicit `destroy()` method to `CameraView`

### Fixed
- Fixed CameraView to use the correct render product variable `_tiled_render_product`

## [1.3.0] - 2025-07-19
### Added
- SingleViewDepthSensorAsset API to wrap around USDs and automatically create
  SingleViewDepthSensor objects (including render products) for cameras with
  associated template render products.

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
- changed add_update_semantics to add_labels

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
- centered the pointcloud points by adding a half-pixel offset for the depth-based calculation

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
- use indexed cuda:{idx} input for warp kernels in camera view class
- use indexed cuda device to pre-allocate out buffers

## [0.2.5] - 2024-12-31
### Fixed
- Camera sensor tests no longer needs OMPE-28827 WAR

## [0.2.4] - 2024-12-03
### Changed
- Isaac Util menu to Tools->Robotics menu

### Fixed
- camera view sensor test warp.types.int32 -> warp.types.uint32
- decreased image comparison threshold with 0.95->0.94

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
