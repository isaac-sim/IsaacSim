# Changelog
## [2.9.1] - 2026-04-28
### Fixed
- Added warning logs for clarity when rigid body APIs fail

## [2.9.0] - 2026-04-24
### Added
- Add `PhysicsEngine.h` header with `getActivePhysicsEngineName()` to query the active physics simulation backend

### Changed
- `Transforms.h` uses `getActivePhysicsEngineName()` for simulation view creation instead of defaulting to nullptr

## [2.8.0] - 2026-03-20
### Added
- Added `BindingsPythonUtils.h` header for pybind11 bindings utilities

### Fixed
- Fixed brace initialization for `g_kIsaacNameOveride` in `UsdUtilities.h`
- Added Doxygen `@cond` to hide internal template `QuatFromAxisAngle` in `Quat.h`

## [2.7.0] - 2026-03-04
### Changed
- Added Overview.md

## [2.6.1] - 2026-02-06
### Changed
- Optimize GenericBuffer to re-use memory without allocation if requested size fits within existing capacity.

## [2.6.0] - 2026-01-07
### Added
- Add `Defines.h` header with `ISAACSIM_EXPORT` and `ISAACSIM_IMPORT` macros for DLL visibility

## [2.5.4] - 2025-12-06
### Changed
- Run clang tidy
- Migrate BaseResetNode to Events 2.0.

## [2.5.3] - 2025-11-07
### Changed
- Update to Kit 109 and Python 3.12

## [2.5.2] - 2025-10-02
### Changed
- Use cached physics views for PoseTree.h

## [2.5.1] - 2025-08-27
### Changed
- Use the IFabricHierarchy cached transform (with explicit update call) and set it to be the default call in `computeWorldXformNoCache`

## [2.5.0] - 2025-08-07
### Changed
- Use device-generic memory buffer implementation to enable tensor API processing on the GPU for PoseTree computation

## [2.4.1] - 2025-07-28
### Changed
- Defaulting to non-FabricHierarchy usage in `computeWorldXformNoCache`

## [2.4.0] - 2025-07-10
### Added
- Device-generic (CPU or CUDA device) memory buffer implementation

## [2.3.0] - 2025-06-29
### Changed
- Add resizeAsync function to DeviceBufferBase

## [2.2.0] - 2025-06-17
### Changed
- Implement `computeWorldXformNoCache` using Fabric Scene Delegate and IFabricHierarchy

## [2.1.2] - 2025-06-12
### Fixed
- Fix missing parameter description in doxygen comment

## [2.1.1] - 2025-06-04
### Changed
- Changed CUDA_CHECK in ScopedCudaDevice.h to indicate file and line for more verbose error logging

## [2.1.0] - 2025-05-24
### Changed
- DeviceBufferBase<T>.copyAsync accepts passthrough cudaStream argument for cudaMemcpyAsync

## [2.0.7] - 2025-05-23
### Changed
- Refactor and add common headers from mjcf importer and urdf importer
- Add docstrings

## [2.0.6] - 2025-05-19
### Changed
- Update copyright and license to apache v2.0

## [2.0.5] - 2025-05-16
### Changed
- Make extension target a specific kit version

## [2.0.4] - 2025-05-15
### Changed
- Remove unnecessary warnings for empty attributes in UsdUtilities.h

## [2.0.3] - 2025-05-10
### Changed
- Remove internal build time dependency

## [2.0.2] - 2025-05-05
### Changed
- Fix API docs.

## [2.0.1] - 2025-05-03
### Fixed
- Fix issues where name override was incorrectly used if set to empty string

## [2.0.0] - 2025-05-02
### Changed
- Remove all Dynamic control compile time dependencies and move all related headers to the includes/omni/isaac/dynamic_control in the dynamic control extension

## [1.2.4] - 2025-04-09
### Changed
- Update Isaac Sim robot asset path
- Update Isaac Sim robot asset path for the IsaacSim folder

## [1.2.3] - 2025-04-04
### Changed
- Version bump to fix extension publishing issues

## [1.2.2] - 2025-03-26
### Changed
- Cleanup and standardize extension.toml, update code formatting for all code

## [1.2.1] - 2025-03-20
### Changed
- Improve doxygen docstrings

## [1.2.0] - 2025-03-17
### Changed
- Cleanup and rename BridgeApplication to PrimManager for clarity

## [1.1.4] - 2025-03-05
### Changed
- Update extension codebase to adhere to isaac sim extension structure and file naming  guidelines

## [1.1.3] - 2025-03-04
### Changed
- Update to kit 107.1 and fix build issues

## [1.1.2] - 2025-02-21
### Changed
- Update style format and naming conventions in c++ code, add doxygen docstrings

## [1.1.1] - 2025-01-21
### Changed
- Update extension description and add extension specific test settings

## [1.1.0] - 2024-10-30
### Changed
- Use USDRT for component initialization using prim types directly rather than traversing stage for better performance

## [1.0.1] - 2024-10-24
### Changed
- Updated dependencies and imports after renaming

## [1.0.0] - 2024-10-10
### Changed
- Renamed to isaacsim.core.includes

## [0.1.1] - 2024-08-06
### Fixed
- Bug when (re-)creating a buffer of same size on another device

## [0.1.0] - 2023-12-19
### Added
- Initial release
