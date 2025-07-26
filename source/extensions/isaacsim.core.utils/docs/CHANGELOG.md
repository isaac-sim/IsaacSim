# Changelog
## [3.4.5] - 2025-07-07
### Fixed
- Correctly enable omni.kit.loop-isaac in test dependency (fixes issue from 3.4.4)

## [3.4.4] - 2025-07-03
### Changed
- Make omni.kit.loop-isaac an explicit test dependency

## [3.4.3] - 2025-06-25
### Changed
- Add --reset-user to test args

## [3.4.2] - 2025-06-18
### Fixed
- Resolve reference paths between layers when saving the stage

## [3.4.1] - 2025-05-31
### Changed
- Use default nucleus server for all tests

## [3.4.0] - 2025-05-28
### Added
- Add create stage in memory to stage utils

## [3.3.2] - 2025-05-26
### Fixed
- Metrics assembler unit test

## [3.3.1] - 2025-05-22
### Changed
- Update copyright and license to apache v2.0

## [3.3.0] - 2025-05-22
### Added
- Add missing to_numpy function to warp utils

## [3.2.1] - 2025-05-13
### Fixed
- Fixed usdrt API mixup in usd workflow.

## [3.2.0] - 2025-05-12
### Changed
- Updated get stage calls to use stage_utils.get_current_stage()
- Added get_current_stage_id()

## [3.1.2] - 2025-05-11
### Changed
- Enable FSD in test settings

## [3.1.1] - 2025-05-10
### Changed
- Remove internal build time dependency

## [3.1.0] - 2025-05-10
### Changed
- Add utility function to remove deleted references on a stage

## [3.0.3] - 2025-05-10
### Changed
- semantics:add_labels now removes the old SemanticsAPI after upgrading
- semantics:removed keep_old_semantics from upgrade_prim_semantics_to_labels

## [3.0.2] - 2025-05-09
### Changed
- semantics:add_labels now uses functional library from replicator

## [3.0.1] - 2025-05-03
### Changed
- Make omni.usd.metrics.assembler.physics dependency optional

## [3.0.0] - 2025-05-02
### Changed
- Remove all Dynamic control compile time dependencies

## [2.6.0] - 2025-05-02
### Added
- add_reference_to_stage will now check if the units of the stage being referenced match the current stage.

## [2.5.0] - 2025-05-01
### Added
- Implement a Python context manager to set and use a thread-local USD stage

## [2.4.0] - 2025-04-29
### Changed
- Added set_active_viewport_camera method to viewports utilities

## [2.3.1] - 2025-04-21
### Changed
- Updated unit test to use franka.usd instead of franka_alt_finger.usd

## [2.3.0] - 2025-04-17
### Changed
- Added semantics util functions to use the new UsdSemantics.LabelsAPI
- Added deprecation warnings for the old SemanticsAPI
- changed add_update_semantics to add_labels in utils.prims

## [2.2.19] - 2025-04-15
### Changed
- Update to use new apis from SimulationManager

## [2.2.18] - 2025-04-11
### Changed
- Update Isaac Sim robot asset path
- Update Isaac Sim robot asset path for the IsaacSim folder

## [2.2.17] - 2025-04-09
### Changed
- Update all test args to be consistent

## [2.2.16] - 2025-04-07
### Changed
- added prim validation check to some functions in prims.py

## [2.2.15] - 2025-04-04
### Changed
- Version bump to fix extension publishing issues

## [2.2.14] - 2025-03-26
### Changed
- Cleanup and standardize extension.toml, update code formatting for all code

## [2.2.13] - 2025-03-11
### Changed
- Switch asset root for tests to internal nucleus

## [2.2.12] - 2025-03-05
### Changed
- Update extension codebase to adhere to isaac sim extension structure and file naming  guidelines

## [2.2.11] - 2025-03-04
### Changed
- Update to kit 107.1 and fix build issues

## [2.2.10] - 2025-02-26
### Changed
- Update style format and naming conventions in c++ code, add doxygen docstrings

## [2.2.9] - 2025-02-25
### Changed
- Removed remaining omni.client._omniclient type hints in favor of public API

## [2.2.8] - 2025-01-26
### Changed
- Update test settings

## [2.2.7] - 2025-01-21
### Changed
- Update extension description and add extension specific test settings

## [2.2.6] - 2024-12-23
### Fixed
- Removed extra dependencies and imports

## [2.2.5] - 2024-12-11
### Changed
- Move unit tests from isaacsim.core.api

## [2.2.4] - 2024-12-09
### Fixed
- unit test

## [2.2.3] - 2024-11-18
### Changed
- omni.client._omniclient to omni.client

## [2.2.2] - 2024-11-09
### Fixed
- Fix unit test

## [2.2.1] - 2024-11-08
### Fixed
- Fixed backends.transformations.get_world_from_local functions

## [2.2.0] - 2024-11-05
### Added
- Added joint_names argument to ArticulationActions type.

## [2.1.0] - 2024-10-29
### Added
- Added utility for finding the base paths of all Articulations on the stage.

## [2.0.4] - 2024-10-28
### Changed
- Remove test imports from runtime

## [2.0.3] - 2024-10-24
### Fixed
- Make extension OS specific

## [2.0.2] - 2024-10-24
### Changed
- Updated dependencies and imports after renaming

### Fixed
- Backend conversion error when creating prims introduced by view classes replacement

## [2.0.1] - 2024-10-24
### Fixed
- Fabric utils to be compatible with 1.4.0

## [2.0.0] - 2024-10-08
### Changed
- Extension renamed to isaacsim.core.utils.

## [1.0.1] - 2024-03-07
### Changed
- Removed the usage of the deprecated dynamic_control extension

## [1.0.0] - 2024-02-09
### Changed
- Moved menu items to omni.isaac.menu

## [0.7.2] - 2024-02-02
### Changed
- Updated path to the nucleus extension

## [0.7.1] - 2024-01-18
### Changed
- Changed get_assets_root_path to get_assets_root_path_async for the unit tests

## [0.7.0] - 2024-01-11
### Changed
- Moved forklift asset under new Robots > Forklift section
- Renamed forklift asset to Forklift B.

### Added
- New Forklift C asset option under Robots > Forklift

## [0.6.0] - 2023-11-29
### Changed
- Moved End effectors out of Robots menu again
- Removed Factory Franka

## [0.5.4] - 2023-11-20
### Changed
- Moved all End effectors under Robots section

### Fixed
- April tag menu

### Fixed
- Loading Robotiq hand into a path that doesn't start with a number

## [0.5.3] - 2023-10-06
### Added
- Carter V2.4 asset path, removed Carter V2 and V2.3
- Renamed Carter V2.4 Nova Carter

## [0.5.2] - 2023-10-04
### Fixed
- Broken asset paths

## [0.5.1] - 2023-09-13
### Fixed
- Updated path of Kuka KR210 robot to match update to Nucleus Server
- Updated name of Kuka KR210 robot in Create menu.

## [0.5.0] - 2023-09-06
### Changed
- updated Create > Isaac > Robots menu

## [0.4.0] - 2023-08-17
### Added
- getCameraPrimFromRenderProduct function

## [0.3.0] - 2023-08-01
### Added
- Added new robots to Create -> Isaac -> Robots menu

## [0.2.5] - 2023-06-12
### Changed
- Update to kit 105.1, omni.usd.utils renamed omni.usd

## [0.2.4] - 2023-01-06
### Fixed
- onclick_fn warning when creating UI

## [0.2.3] - 2022-11-03
### Fixed
- Default cameras for office, hospital, simple room start at reasonable defaults now

## [0.2.2] - 2022-10-27
### Fixed
- Revamped Create -> Isaac menu

## [0.2.1] - 2022-09-07
### Fixed
- Fixes for kit 103.5

## [0.2.0] - 2022-09-01
### Changed
- removed legacy viewport calls

## [0.1.11] - 2022-03-16
### Changed
- Replaced find_nucleus_server() with get_assets_root_path()

## [0.1.10] - 2021-12-01
### Removed
- isaac.nucleus.default setting moved to isaacsim.core.api

## [0.1.9] - 2021-10-21
### Removed
- lookat_to_quat to isaacsim.core.api.utils.rotations
- get_intrinsics_matrix, backproject_depth, project_depth_to_worldspace to isaacsim.core.api.utils.viewports
- set_up_z_axis to isaacsim.core.api.utils.stage.set_stage_up_axis
- omni.isaac.demo specific math utils
- test_utils.py
- create_background
- quat_to_euler_angles, use isaacsim.core.api.utils.rotations.quat_to_euler_angles

## [0.1.8] - 2021-08-13
### Added
- find_nucleus_server_async with timeout

## [0.1.7] - 2021-07-31
### Removed
- Surface Gripper is its own extension

## [0.1.6] - 2021-07-30
### Removed
- Physics Inspector is its own extension
- Physics Utilities is its own extension
- Merge Mesh is its own extension
- Debug Draw is its own extension

## [0.1.5] - 2021-07-12
### Added
- New UI for Surface Gripper

## [0.1.4] - 2021-05-24
### Added
- add physics utils extension
- add create menu

## [0.1.3] - 2021-02-17
### Added
- update to python 3.7
- update to omni.kit.uiapp

## [0.1.2] - 2021-01-04
### Added
- Fix nucleus utils for new content window

## [0.1.1] - 2020-12-11
### Added
- Add unit tests to extension

## [0.1.0] - 2020-12-03
### Added
- Initial version of Isaac Sim Utils Extension
