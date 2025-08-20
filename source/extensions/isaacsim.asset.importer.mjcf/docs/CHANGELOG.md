# Changelog
## [2.5.8] - 2025-07-18
### Fixed
- Fixed importing assets where multiple meshes are defined per body
- Fixed placement of bodies when meshes are reused at different transforms
- Fixed material assignment on meshes when material is defined on Mjcf file

## [2.5.7] - 2025-07-17
### Fixed
- Fixed saving texture files (moving to imported folder) for materials on import. 
- Deleting partial files created when converting meshes

## [2.5.6] - 2025-07-15
### Fixed
- Fixed application of Collision APIs in the collision scope and in the meshes directly as per OpenUSD Physics. 
- Fix cyclic import of robot schema and leave it as sublayer on the base layer

## [2.5.5] - 2025-07-07
### Changed
- Add unit test for pybind11 module docstrings

## [2.5.4] - 2025-07-07
### Fixed
- Correctly enable omni.kit.loop-isaac in test dependency (fixes issue from 2.5.3)

## [2.5.3] - 2025-07-03
### Changed
- Make omni.kit.loop-isaac an explicit test dependency

## [2.5.2] - 2025-06-25
### Changed
- Add --reset-user to test args

## [2.5.1] - 2025-06-24
### Fixed
- Importing bodies with multiple mesh geom

## [2.5.0] - 2025-06-10
### Changed
- Removed unused assets from data folder
- Added missing license file

## [2.4.8] - 2025-05-31
### Changed
- Use default nucleus server for all tests

## [2.4.7] - 2025-05-23
### Changed
- Refactor headers into isaacsim.core.includes
- Add docstrings

## [2.4.6] - 2025-05-19
### Changed
- Update copyright and license to apache v2.0

## [2.4.5] - 2025-05-16
### Changed
- Make extension target a specific kit version

## [2.4.4] - 2025-05-15
### Changed
- Updated MakeRelativePath to use std::string

## [2.4.3] - 2025-05-15
### Changed
- Restructure codebase to align with isaac sim guidelines

## [2.4.2] - 2025-05-10
### Changed
- Enable FSD in test settings

## [2.4.1] - 2025-05-09
### Added
- Extension specific test arguments

## [2.4.0] - 2025-04-09
### Added
- Support for Robot Schema prototype

## [2.3.7] - 2025-03-10
### Changed
- Reverted MeshMergeCollision for now

## [2.3.6] - 2025-02-19
### Fixed
- Relative path for sublayers

## [2.3.5] - 2025-02-13
### Added
- Support for equality constraints

### Fixed
- Fixed bug where multiple tendons to the same joint pair were overriding each other.
- Fixed appropriate conversion of tendon properties to USD.

## [2.3.4] - 2025-01-28
### Fixed
- Windows signing issue

## [2.3.3] - 2025-01-01
### Changed
- Colliders create a MeshMergeCollision at the collisions prim level on robot assembly.

## [2.3.2] - 2024-12-13
### Fixed
- In-place ensuring that meshes, visuals and colliders scopes are invisible.
- Add collision group for colliders out of default prim

## [2.3.1] - 2024-12-09
### Fixed
- Crash when selecting the default density.

## [2.3.0] - 2024-12-09
### Changed
- Ensured Colliders and Visuals do a second stack of referencing so the visual and collider can use the same mesh.

### Fixed
- Collider instancing was picking wrong element when imageable was not a mesh.

## [2.2.3] - 2024-11-22
### Changed
- Only create instanceable prims.
- Use Omniverse ASset Converter.
- Change auto-limit default for joints to true

### Fixed
- Ensure tags are imported even when there are multiple elements.
- Allow import to continue even when inertia is not defined. (issues a warning)

## [2.2.2] - 2024-11-20
### Changed
- Update omni.client import

## [2.2.1] - 2024-10-25
### Changed
- Remove test imports from runtime

## [2.2.0] - 2024-10-13
### Changed
- Update Assets transforms into Translate/orient/scale elements

## [2.1.0] - 2024-10-13
### Changed
- Change the import options

### Removed
- standalone window import

## [2.0.0] - 2024-09-20
### Changed
- Changed MJCF importer to export USD to base, physics, sensor, and main stage
- Changed joint importer to import joints to the joint scope, and categorized by joint type

## [1.2.2] - 2024-09-17
### Added
- Action registry for menu item

## [1.2.1] - 2024-09-06
### Changed
- Register the mjcf file icon to asset types.

## [1.2.0] - 2024-08-22
### Changed
- ISIM-1670: Move Import workflow to File->Import.
- ISIM-1673: Display mjcf options in the right panel of the Import Dialog Window.

## [1.1.1] - 2024-07-10
### Changed
- Importer frames on imported asset when done through GUI.

## [1.1.0] - 2023-10-03
### Changed
- Structural and packaging changes

## [1.0.1] - 2023-07-07
### Added
- Support for `autolimits` compiler setting for joints

## [1.0.0] - 2023-06-13
### Changed
- Renamed the extension to omni.importer.mjcf
- Published the extension to the default registry

## [0.5.0] - 2023-05-09
### Added
- Support for ball and free joint
- Support for `<freejoint>` tag
- Support for plane geom type
- Support for intrinsic Euler sequences

### Changed
- Default value for fix_base is now false
- Root bodies no longer have their translation automatically set to the origin
- Visualize collision geom option now sets collision geom's visibility to invisible
- Change prim hierarchy to support multiple world body level prims

### Fixed
- Fix support for full inertia matrix
- Fix collision geom for ellipsoid prim
- Fix zaxis orientation parsing
- Fix 2D texture by enabling UVW projection

## [0.4.1] - 2023-05-02
### Added
- High level code overview in README.md

## [0.4.0] - 2023-03-27
### Added
- Support for sites and spatial tendons
- Support for specifying mesh root directory

## [0.3.1] - 2023-01-06
### Fixed
- onclick_fn warning when creating UI

## [0.3.0] - 2022-10-13
### Added
- Added material and texture support

## [0.2.3] - 2022-09-07
### Fixed
- Fixes for kit 103.5

## [0.2.2] - 2022-07-21
### Added
- Add armature to joints

## [0.2.1] - 2022-07-21
### Fixed
- Display Bookmarks when selecting files

## [0.2.0] - 2022-06-30
### Added
- Add instanceable option to importer

## [0.1.3] - 2022-05-17
### Added
- Add joint values API

## [0.1.2] - 2022-05-10
### Changed
- Collision filtering now uses filteredPairsAPI instead of collision groups
- Adding tendons no longer has limitations on the number of joints per tendon and the order of the joints

## [0.1.1] - 2022-04-14
### Added
- Joint name annotation USD attribute for preserving joint names

### Fixed
- Correctly parse distance scaling from UI

## [0.1.0] - 2022-02-07
### Added
- Initial version of MJCF importer extension
