# Changelog

## [2.4.19] - 2025-07-07
### Changed
- Add unit test for pybind11 module docstrings

## [2.4.18] - 2025-07-07
### Fixed
- Correctly enable omni.kit.loop-isaac in test dependency (fixes issue from 2.4.17)

## [2.4.17] - 2025-07-03
### Changed
- Make omni.kit.loop-isaac an explicit test dependency

## [2.4.16] - 2025-06-26
### Changed
- Skip tests from ETM that fail to load assets.

## [2.4.15] - 2025-06-25
### Changed
- Add --reset-user to test args

## [2.4.14] - 2025-06-24
### Fixed
- Compound Inertial Origin to the inertia principal axis on import.

## [2.4.13] - 2025-06-12
### Fixed
- Fix section header syntax in api.rst

## [2.4.12] - 2025-06-10
### Fixed
- Cleanup preview.png

## [2.4.11] - 2025-06-09
### Fixed
- Incorrect license header

## [2.4.10] - 2025-05-31
### Changed
- Use default nucleus server for all tests

## [2.4.9] - 2025-05-23
### Changed
- Refactor headers into isaacsim.core.includes
- Restructure to follow extension structure guidelines
- Add docstrings

## [2.4.8] - 2025-05-22
### Fixed
- Importer was setting the base pose at the wrong layer.

## [2.4.7] - 2025-05-20
### Changed
- Update copyright and license to apache v2.0

## [2.4.6] - 2025-05-19
### Changed
- Remove usage of strlen

## [2.4.5] - 2025-05-16
### Changed
- Make extension target a specific kit version

## [2.4.4] - 2025-05-15
### Fixed
- Removed unused code

## [2.4.3] - 2025-05-14
### Fixed
- Fixed issue where the importer was not properly adding the robot schema prim lists and layer assignments as a List Editable.

## [2.4.2] - 2025-05-10
### Changed
- Enable FSD in test settings

## [2.4.1] - 2025-05-09
### Added
- Extension specific test arguments

## [2.4.0] - 2025-04-11
### Changed
- Include Robot Schema definition

## [2.3.13] - 2025-02-27
### Changed
- Disable mesh merge Collision API
- Fix fixed joint merge

## [2.3.12] - 2025-02-06
### Changed
- Viewport now defaults Z as up axis

### Fixed
- Fixed dashes not supported in mesh path error
- Mimic joint limits will now account for the sign of gear

## [2.3.11] - 2025-01-28
### Fixed
- Windows signing issue

## [2.3.10] - 2025-01-13
### Changed
- refactor UI imports to use isaacsim.gui.components

## [2.3.9] - 2025-01-09
### Changed
- Colliders create a MeshMergeCollision at the collisions prim level on robot assembly.

## [2.3.8] - 2025-01-08
### Fixed
- Fix missing map entry to fixed frames
- Revert always ignoring fixed joints

### Added
- Experimental support to mujoco_camera tags

## [2.3.7] - 2024-12-20
### Fixed
- Fix title on example

## [2.3.6] - 2024-12-19
### Fixed
- Fix recursive creation of merged fixed frames

## [2.3.5] - 2024-12-18
### Fixed
- Overriding root prim when it's fixed base

## [2.3.4] - 2024-12-16
### Fixed
- Update sensor linker extension
- Update subscription and removal from examples

## [2.3.3] - 2024-12-16
### Fixed
- Importer fails to parse package when attempting a parse first and import afterwards through python

## [2.3.2] - 2024-12-10
### Fixed
- Layers metadata were not properly being set to Z up and Meters unit
- Importer now does not create Articulation Root when the imported URDF does not contain joints

## [2.3.1] - 2024-12-04
### Fixed
- Errors when loading asset in-stage

## [2.3.0] - 2024-12-03
### Changed
- Examples are now in browser and not menu

## [2.2.3] - 2024-11-20
### Fixed
- omni.kit.test import

## [2.2.2] - 2024-11-20
### Changed
- Updated omni client dependency import

## [2.2.1] - 2024-11-14
### Added
- Support for compliant mimic values

## [2.2.0] - 2024-10-31
### Changed
- Extension renamed from omni.importer.urdf to isaacsim.asset.importer.urdf

## [2.1.3] - 2024-10-25
### Changed
- Moved collision Apply directly on to meshes

## [2.1.2] - 2024-10-25
### Changed
- Remove test imports from runtime

## [2.1.1] - 2024-10-15
### Removed
- Deprecated (removeD) the IsaacSim workflows menu in favor to the File->Import pipeline. Window still available for add-ons

## [2.1.0] - 2024-10-14
### Added
- Support for custom `loop_joint` and `fixed_frame` tags

## [2.0.1] - 2024-10-10
### Changed
- Changed final asset composition to Reference + Payloads with variants to enable/disable sensor and Physics features

## [2.0.0] - 2024-09-26
### Added
- automatic config of Joints Drive gains by Natural Frequency.

## [1.18.0] - 2024-09-25
### Changed
- Updated UI Options

### Deprecated
- Merge fixed joints Option: Now it always merges fixed joints and add child links as reference frames.
- Import inertia tensor Option: Now it always import inertia tensor when available.
- Override Joint Dynamics Option: Now it never override joint dynamics (still uses default when dynamics are not available)

## [1.17.0] - 2024-09-17
### Changed
- Changed the exporter to export multiple usd files where the "base" contains the mesh only, "physics" contains the joints and physical attributes, "sensor" contains the sensor attributes
- The exporter will now categorize the joints by type, and export the same type of joint under the same xform

## [1.16.2] - 2024-09-17
### Added
- Action registry for menu item

## [1.16.1] - 2024-09-06
### Changed
- Register the urdf file icon to asset types.

## [1.16.0] - 2024-08-21
### Changed
- ISIM-1670: Move Import workflow to File->Import.
- ISIM-1673: Display urdf options in the right panel of the Import Dialog Window.

## [1.15.0] - 2024-05-23
### Changed
- Using omniverse_asset_converter to import meshes
- Adding internal reference to meshes and making all meshes instanceable
- Improved logic for mesh reusability
- Improved asset readability by changing the joints creation to its own scope instead of inside each rigid body
- Changed Mimic joints implementation to remove joint velocity limit, and add 20% buffer around joint limits based on primary joint limts.

### Deprecated
- Option to make instanceable. All imports are now instanceable.

## [1.14.1] - 2024-05-23
### Fixed
- Error Message on creating symlink for Windows.

## [1.14.0] - 2024-05-11
### Fixed
- Add Articulation Root to Fixed Joint when fixed base.

## [1.13.1] - 2024-04-29
### Fixed
- Mimic Joints using Mimic Joint API
- Scroll bar in Isaac Sim for UI

## [1.13.0] - 2024-04-04
### Fixed
- Example window UI buttons callbacks

## [1.12.0] - 2024-04-03
### Added
- Parse Camera and Lidar Sensor
- Add code stub for other sensor types currently not in official URDF definition.

## [1.11.0] - 2024-03-26
### Fixed
- Support for GLTF meshes

## [1.10.1] - 2024-03-20
### Fixed
- Ensuring UI frames are always properly aligned

## [1.10.0] - 2024-02-29
### Added
- UI Hooks to Expand URDF Import
- Parse URDF in memory as string

### Fixed
- Compute extent for cylinders axis

## [1.9.0] - 2024-02-26
### Added
- Split URDF Parsing and Importing
- Add utility to select joint drive types prior to import

## [1.8.0] - 2024-02-23
### Fixed
- Joint Axis is maintained when importing if it's aligned with X, Y or Z parent body axis

## [1.7.0] - 2024-02-11
### Fixed
- Fixed importer to ensure rigid body principal axes are taken into account when computing inertia tensor.
- Fixed importing of dynamics damping and friction (when authored).
- Modified handling of massless root links so that they're given zero mass, which results in PhysX autocomputing the minimum mass required for articulation stability,

## [1.6.1] - 2023-12-14
### Fixed
- output USD was accumulating disk size every time it was overwritten
- Instanceable importing was causing the importer to segfault when overwriting.

## [1.6.0] - 2023-12-11
### Fixed
- Changed the AddRobot (and all related bindings and commands) to allow getting the articulation Root prim path directly instead of the base prim, defaulting to getting the base prim.

## [1.5.0] - 2023-12-04
### Fixed
- Fixed setting of articulation root on the actual link prim instead of the default prim. This is needed to deal with letting PhysX parser decide what is the root prim for floating-base systems.

## [1.4.1] - 2023-11-30
### Fixed
- Handling meshes that inadvertertly duplicate the UV maps

## [1.4.0] - 2023-11-30
### Changed
- Prompt users to confirm import path and confirm overwrite before proceeding with importing.

## [1.3.0] - 2023-11-29
### Added
- Handling massless and inertialess links by adding small mass and inertia values

## [1.2.0] - 2023-11-28
### Added
- Flag to decide whether or not to parse mimic joints

## [1.1.4] - 2023-10-18
### Changed
- Update code dependencies

## [1.1.3] - 2023-10-02
### Changed
- Mesh path parser now allows for prefixes other than "package://"

## [1.1.2] - 2023-08-21
### Added
- Added processing of capsule bodies to replace cylinders

### Fixed
- Fixed computation of joint axis. Earlier arbitrary axis were not supported.
- Fixed bug in merging joints when multiple levels of fixed joints exist.

## [1.1.1] - 2023-08-08
### Added
- Added support for the boolean attribute  ``<joint dont_collapse>``: setting this parameter to true in the URDF joint tag prevents the child link from collapsing when the associated joint type is "fixed".

## [1.1.0] - 2023-08-04
### Added
- Suport for direct mimic joints
- Maintain Merged Links as frames inside parent rigid body

### Fixed
- Arbitrary joint axis were adding random 56.7 rotation degrees in the joint axis orientation

## [1.0.0] - 2023-07-03
### Changed
- Renamed the extension to isaacsim.asset.importer.urdf
- Published the extension to the default registry

## [0.5.16] - 2023-06-27
### Fixed
- Support for arbitrary joint axis.

## [0.5.15] - 2023-06-26
### Fixed
- Support for non-diagonal inertia matrix
- Support for convex decomposition mesh collision method

## [0.5.14] - 2023-06-13
### Fixed
- Kit 105.1 update
- Accessing primvars is accessed through the PrimvarsAPI instead of usd convenience functions

## [0.5.13] - 2023-06-07
### Fixed
-  Crash when name was not provided for a material, added check for null pointer and unit test

## [0.5.12] - 2023-05-16
### Fixed
-  Removed duplicated code to copy collision geometry from the mesh visuals.

## [0.5.11] - 2023-05-06
### Added
-  Collision geometry from visual meshes.

## [0.5.10] - 2023-05-04
### Added
- Code overview and limitations in README.

## [0.5.9] - 2023-02-28
### Fixed
- Appearance of carb warnings when setting the joint damping and stiffness to 0 for `NONE` drive type

## [0.5.8] - 2023-02-22
### Fixed
- removed max joint effort scaling by 60 during import
- removed custom collision api when the shape is a cylinder

## [0.5.7] - 2023-02-17
### Added
- Unit test for joint limits.
- URDF data file for joint limit unit test (test_limits.urdf)

## [0.5.6] - 2023-02-14
### Fixed
- Imported negative URDF effort and velocity joint constraints set the physics constraint value to infinity.

## [0.5.5] - 2023-01-06
### Fixed
- onclick_fn warning when creating UI

## [0.5.4] - 2022-10-13
### Fixed
- Fixes materials on instanceable imports

## [0.5.3] - 2022-10-13
### Fixed
- Added Test for import stl with custom material

## [0.5.2] - 2022-09-07
### Fixed
- Fixes for kit 103.5

## [0.5.1] - 2022-09-02
### Changed
- Use omni.kit.viewport.utility when setting camera

## [0.5.0] - 2022-08-30
### Changed
- Remove direct legacy viewport calls

## [0.4.1] - 2022-08-30
### Changed
- Modified default gains in URDF -> USD converter to match gains for Franka and UR10 robots

## [0.4.0] - 2022-08-09
### Added
- Cobotta 900 urdf data files

## [0.3.1] - 2022-08-08
### Fixed
- Missing argument in example docstring

## [0.3.0] - 2022-07-09
### Added
- Add instanceable option to importer

## [0.2.2] - 2022-06-02
### Changed
- Fix title for file picker

## [0.2.1] - 2022-05-23
### Changed
- Fix units for samples

## [0.2.0] - 2022-05-17
### Changed
- Add joint values API

## [0.1.16] - 2022-04-19
### Changed
- Add Texture import compatibility for Windows.
- Revamped UI

## [0.1.15] - 2021-12-20
### Changed
- Fixed bug where missing mesh on part with urdf material assigned would crash on material binding in a non-existing prim.

## [0.1.14] - 2021-12-20
### Changed
- Fix bug where material was indexed by name and removing false duplicates.
- Add Normal subdivision group import parameter.

## [0.1.13] - 2021-12-10
### Changed
- Texture support for OBJ and Collada assets.
- Remove bug where an invalid link on a joint would stop importing the remainder of the urdf. raises an error message instead.

## [0.1.12] - 2021-12-03
### Changed
- Default to save Imported assets on a new USD and reference it on open stage.
- Change base robot prim to also use orientOP instead of rotateOP
- Change behavior where any stage event (e.g selection changed) was resetting some options on the UI

## [0.1.11] - 2021-11-29
### Changed
- Use double precision for xform ops to match isaac sim defaults

## [0.1.10] - 2021-11-04
### Changed
- create physics scene is false for import config
- create physics scene will not create a scene if one exists
- set default prim is false for import config

## [0.1.9] - 2021-10-25
### Added
- Support to specify usd paths for urdf meshes.

### Changed
- distance_scale sets the stage to the same units for consistency
- None drive mode still applies DriveAPI, but keeps the stiffness/damping at zero
- rootJoint prim is renamed to root_joint for consistency with other joint names.

### Fixed
- warnings when setting attributes as double when they should have been float

## [0.1.8] - 2021-10-18
### Added
- Floating joints are ignored but place any child links at the correct location.

### Fixed
- Crash when urdf contained a floating joint

## [0.1.7] - 2021-09-23
### Added
- Default position drive damping to UI

### Fixed
- Default config parameters are now respected

## [0.1.6] - 2021-08-31
### Changed
- Updated to New UI
- Spheres and Cubes are treated as shapes
- Cylinders are by default imported with custom geometry enabled
- Joint drives are default force instead of acceleration

### Fixed
- Meshes were not imported correctly, fixed subdivision scheme setting

### Removed
- Parsing URDF is not a separate step with its own UI

## [0.1.5] - 2021-07-30
### Fixed
- Zero joint velocity issue
- Artifact when dragging URDF file due to transform matrix

## [0.1.4] - 2021-06-09
### Added
- Fixed bugs with default density

## [0.1.3] - 2021-05-26
### Added
- Fixed bugs with import units
- Streamlined UI and fixed missing elements
- Fixed issues with creating new stage on import

## [0.1.2] - 2020-12-11
### Added
- Unit tests to extension
- Add test urdf files
- Fix unit issues with samples
- Fix unit conversion issue with import

## [0.1.1] - 2020-12-03
### Added
- Sample URDF files for carter, franka ur10 and kaya

## [0.1.0] - 2020-12-03
### Added
- Initial version of URDF importer extension
