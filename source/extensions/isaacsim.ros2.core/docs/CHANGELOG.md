# Changelog
## [1.9.4] - 2026-06-12
### Fixed
- `camera_info_utils.compute_relative_pose`: defer the `cv2` import to first use.

## [1.9.3] - 2026-06-10
### Fixed
- `IPCBufferManager` destructor leaked one CUDA virtual address reservation (missing `cuMemAddressFree`) and one exported POSIX file descriptor per buffer on every teardown, and released allocation handles before unmapping them. Tear down in the documented VMM order (unmap, release, address-free) and close the exported file descriptors.
- Dynamic ROS 2 message publishing leaked the previous sequence buffer every tick: `Ros2DynamicMessageImpl::_setArray` re-initialized variable-length array fields without finalizing the prior allocation. Finalize before re-initializing.

## [1.9.2] - 2026-06-09
### Fixed
- Fix linter errors and missing or incomplete docstrings, and update `python_api.md`.

## [1.9.1] - 2026-05-14
### Changed
- Add missing type annotations on `ROS2TestCase` methods and replace a bare `except` in viewport-ready setup

## [1.9.0] - 2026-05-07
### Fixed
- Downgraded bundled Fast-RTPS from 2.6.11 to 2.6.10 in the Humble nv_ros2 linux packages to avoid a regression introduced upstream in 2.6.11.

## [1.8.6] - 2026-04-28
### Changed
- `wait_for_publishers_on_topic` and `wait_for_subscribers_on_topic` now include the last-observed endpoint count in their timeout failure messages to aid flake diagnosis

### Fixed
- Make `Ros2TestBase` use the sourced distro so the test loads the same backend the plugin already initialized.

## [1.8.5] - 2026-04-28
### Fixed
- `read_camera_info()` ignored render product resolution for `opencvPinhole`/`opencvFisheye` lens models, publishing stale `CameraInfo` when the render product was retargeted to a different resolution than the authored `opencv*:imageSize`. Apply the render-product resolution and scale `fx`, `fy`, `cx`, `cy` accordingly.

## [1.8.4] - 2026-04-28
### Fixed
- Raise error if `/exts/omni.replicator.srtx/enabled=true` on non-Linux platforms.

## [1.8.3] - 2026-04-27
### Removed
- Remove the `omni.isaac.ml_archive` and `isaacsim.robot.wheeled_robots` dependencies

## [1.8.2] - 2026-04-24
### Fixed
- Fix joint state publisher writing no position/velocity data when effort retrieval fails

## [1.8.1] - 2026-04-22
### Added
- Added `wait_for_publishers_on_topic` and `wait_for_subscribers_on_topic` helpers to `ROS2TestCase` for waiting on DDS endpoint discovery before asserting on message delivery. Uses wall-clock timeout to handle platforms with no frame rate limiter.

## [1.8.0] - 2026-04-21
### Changed
- Migrated `camera_info_utils` from deprecated `isaacsim.sensors.camera` to `isaacsim.sensors.experimental.rtx` APIs
- Camera info now reads OpenCV distortion coefficients from `OmniLensDistortion` schemata (`OmniLensDistortionOpenCvPinholeAPI`, `OmniLensDistortionOpenCvFisheyeAPI`)
- Removed deprecated `isaacsim.sensors.camera` and `isaacsim.sensors.rtx` dependencies; added `isaacsim.sensors.experimental.rtx` and `isaacsim.sensors.rtx.nodes`
- Removed legacy physical distortion model support (`physicalDistortionModel`, `physicalDistortionCoefficients`)
- Refactored `test_camera_info_utils.py` to use `RtxCamera` and `OmniLensDistortion` schemas

## [1.7.4] - 2026-04-20
### Fixed
- Remove unused isaacsim.sensors.physx dependency

## [1.7.3] - 2026-04-17
### Changed
- Updated internal ROS 2 libraries to support latest simulation interfaces versions (v1.4.0 Humble and v1.5.0 Jazzy)

## [1.7.2] - 2026-04-15
### Fixed
- Fixed handling for fixed-size array types in `ROS2DynamicMessage.cpp` when the input array length differs from the expected size

## [1.7.1] - 2026-04-03
### Fixed
- Added explicit import of `_ros2_core` bindings module for stubgen discoverability
- Added Doxygen `@cond` to hide internal anonymous namespace in `Ros2Distro.h`

## [1.7.0] - 2026-04-01
### Changed
- Removed deprecated `isaacsim.core.api` and `isaacsim.core.utils` dependencies
- Replaced `render_product_utils.py` with `ViewportManager` from `isaacsim.core.rendering_manager`

## [1.6.1] - 2026-03-30
### Changed
- Added `LibraryLoader.h` include to `Ros2Types.h` for SRTX integration support

## [1.6.0] - 2026-03-17
### Changed
- Updated documentation with AI agent.

## [1.5.1] - 2026-03-09
### Fixed
- Hardened subprocess call to avoid shell=True with string concatenation

## [1.5.0] - 2026-02-26
### Added
- CompressedImage message backend

## [1.4.0] - 2026-02-24
### Changed
-  Removed hardcoded ROS 2 distribution checks. Added experimental support for any ROS 2 distribution beyond Jazzy to be sourced and used with Isaac Sim.

## [1.3.1] - 2026-02-20
### Changed
- Fixed ROS 2 service request polling so `takeRequest()` can receive a pending request on its first poll call.
- Added a regression doctest covering first-poll request/response behavior for `Ros2Service`.

## [1.3.0] - 2026-02-01
### Changed
- Removed isaacsim.sensors.experimental.physics dependency

## [1.2.7] - 2026-01-23
### Changed
- Set publish_with_queue_thread extension setting to true

## [1.2.6] - 2026-01-21
### Added
- Added ros2 image buffer utils

## [1.2.5] - 2025-12-23
### Changed
- Added a simulate_until_condition method to simplify test cases

## [1.2.4] - 2025-12-11
### Changed
- Update ros2 test case to wait for viewport to be ready

## [1.2.3] - 2025-12-07
### Changed
- Fix clang tidy issues in cpp code

## [1.2.2] - 2025-11-26
### Changed
- PointCloud2 message backend now optionally supports host-pinned buffer

## [1.2.1] - 2025-11-26
### Added
 - Default setting values to control ROS2PublishImage queue thread optimization.

## [1.2.0] - 2025-11-24
### Added
- Add flag to generateBuffer() to allocate CUDA pinned memory for image buffers

## [1.1.0] - 2025-11-24
### Changed
- Moved handle interface from isaacsim.core.nodes extension to this extension.

## [1.0.2] - 2025-11-20
### Changed
- Use set_lens_distortion_model in TestCameraInfoUtils

## [1.0.1] - 2025-11-07
### Changed
- Update to Kit 109 and Python 3.12

## [1.0.0] - 2025-11-02
### Added
- Initial release of `isaacsim.ros2.core` extension
- Moved core ROS 2 libraries and backend functionality from isaacsim.ros2.bridge to this extension
