# Changelog

## [Unreleased]

### Added

- `broadcastView()` and `transposeView()` expose explicit shared strided views while `broadcastTo()` and
  `transpose()` retain their original copy semantics.
- The contiguous `fromBuffer()` overload accepts the original shape/dtype call form alongside bounded strided buffers.
- `sqrt()` and `power()` compute in floating point regardless of the operand dtype.

## [7.0.0a1] - 2026-09-16

### Added

- `Array`, `Dtype`, `Shape`, `Device`, and `DeviceGuard` provide multi-dimensional array operations.
- CUDA-backed array operations support Windows.
- Installed C++ headers expose the public array API; CUDA loader and kernel-export details remain private.
- Production-only CMake configurations support `BUILD_TESTING=OFF`.

### Changed

- The distribution metadata uses the canonical `isaacsim-common` project name.
