# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [1.0.0] - 2025-08-08

### Added
- Initial release of `isaacsim.xr.input_devices` extension
- Manus gloves integration via C++ tracker with Python bindings
- Vive tracker integration via `pysurvive` (libsurvive) with mock fallback
- Unified Python API: `get_xr_device_integration().get_all_device_data()`
- Left/right wrist mapping resolution between Vive trackers and OpenXR wrists
- Static scene-to-lighthouse transform estimation with clustering and averaging
- Per-device connection status and last-data timestamps
- Sample visualization script for Manus and Vive devices
- Documentation and simple CLI-style tests

### Details
- Update cadence is rate-limited to 100 Hz
- Wrist mapping considers both pairings (WM0→L/WM1→R vs WM1→L/WM0→R),
  accumulates per-frame translation/rotation errors, and chooses the better pairing
- Chosen pairing candidates are clustered (position/orientation margins) and averaged
  to produce a stable `scene_T_lighthouse_static` transform
- Vive tracker poses are transformed into scene coordinates using the static transform
- Manus joint poses (relative to the wrist) and transformed into scene coordinates 
  using the resolved wrist mapping from vive tracker poses

### Dependencies
- Isaac Sim Core APIs
- Omniverse Kit runtime (for logging and extension lifecycle)
- Manus SDK (optional; otherwise uses mock data)
- libsurvive / pysurvive (optional; otherwise uses mock data)
