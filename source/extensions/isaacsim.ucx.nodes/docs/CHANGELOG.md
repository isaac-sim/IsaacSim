# Changelog

## [1.6.7] - 2026-06-09
### Fixed
- Fix linter errors and missing or incomplete docstrings.

## [1.6.6] - 2026-06-04
### Changed
- `OgnUCXPublishImage`: the image FlatBuffer's `Tensor.shape` is now `[height, width, bytes_per_pixel]` with `ndim = 3` and row-major strides, replacing the previous 1D `[dataSize]` shape. Receivers can now consume the image as a properly-shaped tensor without an external reshape step. `bytes_per_pixel` is derived as `dataSize / (height * width)` so future encodings do not require a code change here. Both the CPU path (`sendCudaBuffer = false`) and the GPU-direct metadata header (`sendCudaBuffer = true`) emit the same shape. Total byte count is still recoverable as `prod(shape)`.

### Fixed
- `python/tests/common.py::get_image_pixel_data_size` now returns `prod(shape)` instead of `shape[0]` so the helper continues to report the pixel byte count under the new 3D shape. Docstring and `unpack_image_message` / `test_camera` comments updated to match.

## [1.6.5] - 2026-06-04
### Fixed
- `OgnUCXCameraHelper.ogn`: the `tag` input default is now the FNV-1a 32-bit hash of `"isaac.Image"` (`270059627`), matching the convention used by the other UCX OGN publishers (`OgnUCXPublishClock`, `OgnUCXPublishJointState`, `OgnUCXSubscribeJointCommand`). Previously the default was the literal `10`, which made `UCXCameraHelper` the only UCX publisher whose default tag did not derive from its FlatBuffer schema name and required consumers to override the input to receive frames.

## [1.6.4] - 2026-05-21
### Fixed
- Made `test_publish_clock.py` more robust by waiting for the UCX listener to accept the client connection and by arming clock receives before triggering manual publish impulses.

## [1.6.3] - 2026-05-14
### Changed
- Add type annotations and docstrings to `OgnUCXCameraHelper`, extension lifecycle methods, and tests to satisfy ruff lint rules.

## [1.6.2] - 2026-05-14
### Fixed
- `test_camera.py`: `receive_image_message` now performs the second `tag_recv` required by the GPU-direct two-message protocol (`sendCudaBuffer=True`, the camera helper default). Previously the test only consumed the metadata FlatBuffer, leaving `image_data` empty and `step = 0`; this caused `test_camera_rgb` and `test_camera_multiple_resolutions` to fail with `0 != width * 3` and silently masked the pixel-data validation in `test_camera_system_time` and `test_camera_frame_skip`. Added matching `step` and `len(image_data)` asserts to the latter two so the CPU-/GPU-direct pixel payload is actually verified in all four tests.
- `tests/common.py`: added `cuda_copy` to `UCX_TLS`. The previous `tcp,self` value had no CUDA memtype module, so the new GPU-direct recv aborted with `UCX ERROR cannot find remote protocol for ... rndv_recv into host memory from cuda/dev[0]` and crashed the test process during shutdown. `cuda_copy` lets UCX stage GPU buffers through host memory before transmitting over TCP.

### Added
- `get_image_pixel_data_size` helper in `tests/common.py` that returns the expected pixel byte count from the Image FlatBuffer's `Tensor.shape[0]`. Used by receivers to detect the GPU-direct two-message protocol and size the follow-up pixel recv.

## [1.6.1] - 2026-05-13
### Changed
- `frameSkipCount` deprecation message in UCXCameraHelper directs users to set input to 0.

### Fixed
- `OgnUCXCameraHelper`: `frameSkipCount` is once again honored when set (`publishStepSize = frameSkipCount + 1`). In 1.5.1, `publishStepSize` was hard-coded to `1`, which silently ignored any user-set `frameSkipCount` even though the deprecation warning fired. The publish gate is now consistent with `OgnROS2CameraHelper` / `OgnROS2RtxLidarHelper`.

## [1.6.0] - 2026-05-08
### Added
- Camera streaming over UCX in `OgnUCXPublishImage` / `OgnUCXCameraHelper`, controlled via the `sendCudaBuffer` bool input.
  - When `false`: pixel bytes are copied to host memory and embedded in a single FlatBuffer message (CPU path).
  - When `true`: a metadata FlatBuffer is sent first, followed by the raw CUDA buffer as a second message on the same UCX tag (UCX preserves in-order delivery within a tag). The receiver pre-allocates a device buffer and posts a tagRecv directly into it, avoiding any CPU copy when the chosen transport supports it. UCX selects the actual transport — GPU-direct RDMA, NVLink, CUDA IPC, or TCP with host staging.
  - Metadata + tensor sends are atomic at queue time: if the tensor send fails to queue, the metadata send is cancelled so the receiver does not desync.
  - Reads `cudaDeviceIndex` from the input into the `DLDevice` metadata for multi-GPU systems.

## [1.5.1] - 2026-05-05
### Changed
- Enable multitick, deprecate `frameSkipCount` in UCXCameraHelper.

## [1.5.0] - 2026-04-17
### Added
- FlatBuffers schemas and package dependency for UCX bridge message types

### Changed
- UCX publish and subscribe nodes now use the Isaac OS FlatBuffers wire format

## [1.4.1] - 2026-04-17
### Fixed
- Fixed crash during shutdown caused by async send request outliving the UCX listener, triggering a close callback on a destroyed mutex

## [1.4.0] - 2026-03-18
### Changed
- Remove `targetPrim` input from `UCXPublishJointState`; node now accepts `jointPositions`, `jointVelocities`, and `jointEfforts` arrays from upstream nodes (e.g. Isaac Read Articulation State)
- Remove `targetPrim` input from `UCXPublishOdometry`; direct data inputs are now the sole data source
- Remove unused `renderProductPath` input from `UCXPublishImage`

## [1.3.3] - 2026-03-05
### Fixed
- Fixed flaky `test_sim_clock` test by increasing UCX send timeout from 1ms to 5000ms and using non-blocking `asyncio.sleep` in the receive wait loop

## [1.3.2] - 2026-03-02
### Changed
- Add Overview.md, public python_api.md and update docstrings

## [1.3.1] - 2026-02-14
### Fixed
- Fixed flaky UCX joint state and odometry tests by increasing connection wait time and adding retry logic

## [1.3.0] - 2025-12-15
### Changed
- Migrate extension implementation to core experimental API

## [1.2.0] - 2025-12-12
### Changed
- Consolidate common functionality (publishMessage) into UcxNode base class.
- Introduce per-sensor data structs (ClockData, ImuData, OdometryData, JointStateData, JointCommandData, ImageMetadata) to separate data extraction from serialization.

## [1.1.3] - 2025-12-11
### Changed
- Try removing listener from registry after node resets
- Fix issues with unit tests
- Reset simulation time on stop by default for UCXCameraHelper

## [1.1.2] - 2025-12-07
### Changed
- Fix issues found by clang tidy

## [1.1.1] - 2025-12-02
### Changed
- Updated deprecated imports to isaacsim.storage.native

## [1.1.0] - 2025-11-25
### Added
- Joint state publishing node
- Joint command subscribing node
- Odometry publishing node
- IMU publishing node
- RGB image publishing node
- Camera helper node for connecting the replicator pipeline with image publishing

## [1.0.0] - 2025-11-18
### Added
- Initial release of UCX Nodes extension
- OmniGraph nodes for high-performance UCX communication
