# Changelog

## [Unreleased]

## [7.0.0a1] - 2026-09-16

### Added

- Stable C, C++, and Python profiling events backed by an application-owned Carbonite profiler.
- Lazy zones, frame markers, numeric values, instants, flows, thread names, and safe host attach/detach.
- Lazy C++ convenience macros for profiling events, including explicit capture-mask variants.
- Explicit Python startup and shutdown for the packaged Carbonite profiler.
- Production-only CMake configurations support `BUILD_TESTING=OFF`.

### Changed

- The distribution metadata uses the canonical `isaacsim-common` project name.
