# Changelog

## [Unreleased]

## [7.0.0a1] - 2026-09-16

### Added

- `Array`, `Dtype`, `Shape`, `Device`, and `DeviceGuard` provide multi-dimensional array operations.
- CUDA-backed array operations support Windows.
- Installed C++ headers expose the public array API; CUDA loader and kernel-export details remain private.
- Production-only CMake configurations support `BUILD_TESTING=OFF`.

### Changed

- The distribution metadata uses the canonical `isaacsim-common` project name.
