# Changelog

## [4.6.10] - 2025-07-23
### Fixed
- Exclude test_world_poses_fabric from ETM

## [4.6.9] - 2025-07-09
### Changed
- Reduce test runtime
- Add timing to test methods

## [4.6.8] - 2025-07-07
### Fixed
- Correctly enable omni.kit.loop-isaac in test dependency (fixes issue from 4.6.7)

## [4.6.7] - 2025-07-03
### Changed
- Make omni.kit.loop-isaac an explicit test dependency

## [4.6.6] - 2025-06-30
### Fixed
- SimulationContext clear_instance will set the physics device to default "cpu"

## [4.6.5] - 2025-06-26
### Fixed
- Fix default rendering dt when set_defaults is false by using the stage's time codes per second as the rendering dt
- Check if manual mode is enabled before returning the manual step size
- Fix joint forces test

## [4.6.4] - 2025-06-25
### Changed
- Add --reset-user to test args

## [4.6.3] - 2025-06-25
### Fixed
- Fix GrounPlane's physical material apply during instance construction

## [4.6.2] - 2025-06-18
### Changed
- Skip articulation and rigid body view tests in ETM

## [4.6.1] - 2025-06-06
### Fixed
- Force commit timeline changes on play_async to properly initialize views

## [4.6.0] - 2025-06-05
### Changed
- Disable PhysX scene's stabilization flag by default

## [4.5.8] - 2025-06-05
### Fixed
- Added timeline.commit() in SimulationContext.play() to trigger warmup directly.

## [4.5.7] - 2025-06-02
### Fixed
- Added initialize_physics() call in SimulationContext.reset and .reset_async in case it wasn't triggered on play event.

## [4.5.6] - 2025-06-02
### Fixed
- Explicitly setting the physics scene prim path in SimulationManager through PhysicsContext

## [4.5.5] - 2025-05-31
### Changed
- Fix bug with getting rendering dt when not using fixed time stepping

## [4.5.4] - 2025-05-31
### Changed
- Use default nucleus server for all tests

## [4.5.3] - 2025-05-30
### Changed
- Update golden values for unit test

## [4.5.2] - 2025-05-28
### Changed
- Support stage in memory in SimulationContext

## [4.5.1] - 2025-05-19
### Changed
- Update copyright and license to apache v2.0

## [4.5.0] - 2025-05-17
### Changed
- Allow for more control over time stepping behavior
- "/app/runLoops/main/rateLimitFrequency" and timeline.set_target_framerate(rendering_hz) are only set if the app has "/app/runLoops/main/rateLimitEnabled" set to true
- isaac sim loop runner set_manual_mode is always used if rendering rate is specified

## [4.4.2] - 2025-05-14
### Changed
- Specify cuboid collider

## [4.4.1] - 2025-05-14
### Added
- Add missing controller classes to API docs

## [4.4.0] - 2025-05-12
### Changed
- Updated get stage calls to use stage_utils.get_current_stage()

## [4.3.6] - 2025-05-10
### Changed
- Enable FSD in test settings

## [4.3.5] - 2025-05-07
### Changed
- switch to omni.physics interface

## [4.3.4] - 2025-05-04
### Changed
- Update stage event handlers to new event system

## [4.3.3] - 2025-04-30
### Fixed
- Reduced gains for cartpole to prevent tunneling

## [4.3.2] - 2025-04-29
### Changed
- Updated Franka alt finger to Franka robot

## [4.3.1] - 2025-04-21
### Changed
- Update Cartpole USD path
- Update articulation view tests boundary and infeasible states

## [4.3.0] - 2025-04-15
### Changed
- Changed reset fabric selection callback to a post physics step

## [4.2.27] - 2025-04-11
### Changed
- Update Isaac Sim robot asset path
- Update Isaac Sim robot asset path for the IsaacSim folder

## [4.2.26] - 2025-04-09
### Changed
- Update all test args to be consistent

## [4.2.25] - 2025-04-04
### Changed
- Version bump to fix extension publishing issues

## [4.2.24] - 2025-03-26
### Changed
- Cleanup and standardize extension.toml, update code formatting for all code

## [4.2.23] - 2025-03-26
### Fixed
- CCD is not supported when using a cuda device, CCD is now automatically disabled if a cuda device is requested.

## [4.2.22] - 2025-03-24
### Changed
- Migrate to Events 2.0

## [4.2.21] - 2025-03-11
### Changed
- Switch asset root for tests to internal nucleus

## [4.2.20] - 2025-03-05
### Changed
- Update extension codebase to adhere to isaac sim extension structure and file naming  guidelines

## [4.2.19] - 2025-03-04
### Changed
- Update to kit 107.1 and fix build issues

## [4.2.18] - 2025-02-21
### Changed
- Update style format and naming conventions in c++ code, add doxygen docstrings

## [4.2.17] - 2025-01-28
### Fixed
- Windows signing issue

## [4.2.16] - 2025-01-27
### Changed
- Changed build repo from constant str to a var for the crash reporter

## [4.2.15] - 2025-01-26
### Changed
- Update test settings

## [4.2.14] - 2025-01-21
### Changed
- Update extension description and add extension specific test settings

## [4.2.13] - 2025-01-13
### Fixed
- Move visual cube down for follow target examples so its visible in default camera angle

## [4.2.12] - 2025-01-06
### Fixed
- Use visual cuboid in follow target obstacle so it can be removed and added dynamically

## [4.2.11] - 2024-12-30
### Fixed
- Reduced contact tests runtime
- Enabled GPU pipeline tests

## [4.2.10] - 2024-12-23
### Fixed
- Remove extra test dependencies
- Disable tests that were not compatible with GPU pipeline

## [4.2.9] - 2024-12-19
### Fixed
- rename articulation_handle to articulation_view

## [4.2.8] - 2024-12-16
### Fixed
- Prevent follow target from going below the ground plane

## [4.2.7] - 2024-12-11
### Changed
- Move unit tests to isaacsim.core.utils
- Improve unit test performance
- fix test_scale_units_resolve

## [4.2.6] - 2024-12-05
### Fixed
- Fixed Rigid Prim and Articulation objects default state gets overwritten after a world.reset()

## [4.2.5] - 2024-11-22
### Removed
- Removed SimulationContext._physics_sim_view.

## [4.2.4] - 2024-11-20
### Changed
- Added SimulationContext._physics_sim_view initialization.

## [4.2.3] - 2024-11-12
### Changed
- Revert SimulationContext.play, .play_async, .pause and .pause_async to use app.update instead of timeline.commit.

## [4.2.2] - 2024-11-08
### Changed
- Changed render calls after timeline changes to timeline.commit calls.

## [4.2.1] - 2024-11-08
### Changed
- Changed local_pose tests in xform_prim_view and rigid_prim_view

## [4.2.0] - 2024-11-06
### Added
- Added residual reporting APIs

## [4.1.4] - 2024-11-06
### Changed
- Nucleus import update

## [4.1.3] - 2024-11-05
### Changed
- removed backend and device arguments from PhysicsContext
- warm start, simulation view creation and post reset ported to SimulationManager

## [4.1.2] - 2024-10-29
### Changed
- Updated dependencies and imports after renaming

## [4.1.1] - 2024-10-24
### Changed
- Updated dependencies and imports after renaming

## [4.1.0] - 2024-10-24
### Changed
- Updated isaacsim.core.api to support live sync with Isaac Lab.

## [4.0.1] - 2024-10-03
### Changed
- Updated omni.isaac.version dependency to isaacsim.core.version

## [4.0.0] - 2024-09-25
### Changed
- Extension renamed to isaacsim.core.api.

## [3.19.5] - 2024-09-17
### Fixed
- .render() call to fetch physX data into fabric in the GPU pipeline
- .render() call implicitly calls update_articulations_kinematic in order to update articulations kinematic before rendering the result

## [3.19.4] - 2024-08-26
### Fixed
- Exempts /Render and child prims when calling clear stage

## [3.19.3] - 2024-08-09
### Fixed
- Do not delete the default renderproduct for the viewport when calling clear stage

## [3.19.2] - 2024-08-09
### Fixed
- missing omni.kit.stage_templates dependency

## [3.19.1] - 2024-07-31
### Fixed
- Fix deprecation warning and use Semantics instead of pxr.Semantics

## [3.19.0] - 2024-07-26
### Added
- Added functions to find missing and incorrect semantic labels
- Added function to tally semantic labels in a scene

## [3.18.1] - 2024-07-18
### Fixed
- Fixed ArticulationController to handle joint indices along with Nan values in the target actions.

## [3.18.0] - 2024-07-16
### Changed
- Make omni.kit.material.library optional, it must be enabled when using OmniGlass helper class

## [3.17.1] - 2024-07-11
### Fixed
- Fixed passing filter_path_expressions as list of list of str in RigidContactView

## [3.17.0] - 2024-06-28
### Added
- Allow find_matching_prim_paths in prim utils to return paths with valid physics APIs applied
- Added a prim_type argument that supports filtering the prims returned by type: articulation and rigid_body are currently supported.

## [3.16.0] - 2024-06-28
### Added
- ArticulationView.pause_motion() and ArticulationView.resume_motion() methods

## [3.15.0] - 2024-06-27
### Fixed
- get_articulation_root_api_prim_path for inputs like "/World/Frank_*" instead of "/World/Franka_.*"

### Changed
- Changed prim_path_expr for XFormPrim, RigidPrimView, ArticulationView, RigidContactView to accept a list of regular expressions.

## [3.14.0] - 2024-06-27
### Changed
- Replaced persistent physics carb settings for the non persistent ones for USD updates.
- Removed update_to_fast_cache from PhysicsContext.set_physx_update_transformations_settings and PhysicsContext.get_physx_update_transformations_settings

## [3.13.1] - 2024-06-14
### Changed
- Moved common test code into common.py

### Fixed
- Unit tests taking longer than expected due to extra app updates
- Reduce print verbosity when running tests

## [3.13.0] - 2024-06-07
### Changed
- SimulationContext and World classes do not change camera on initialization, set_camera_view must be called separately

## [3.12.0] - 2024-05-14
### Added
- added getting world poses through fabric selections in XFormPrimView through usd=False argument

### Changed
- reset_xform_properties arguments defaults to False instead of True in the prim classes initialization.

## [3.11.2] - 2024-05-10
### Fixed
- Added deprecation warning for particle cloth

## [3.11.1] - 2024-05-06
### Added
- Warning notice for TensorFlow int32 tensor to interoperability function docstrings

## [3.11.0] - 2024-04-25
### Added
- Friction reporting API to GeometryPrim and GeometryPrimView

## [3.10.0] - 2024-04-17
### Changed
- Remove joint velocities computation using finite difference with TGS solver

## [3.9.2] - 2024-04-17
### Fixed
- numpy.bool deprecation

## [3.9.1] - 2024-04-17
### Fixed
- removed the deprecated physX force sensor

## [3.9.0] - 2024-04-16
### Added
- Friction reporting API

## [3.8.2] - 2024-04-15
### Fixed
- Carb plugin used for crash metadata

## [3.8.1] - 2024-04-08
### Added
- Added crash metadata

## [3.8.0] - 2024-03-24
### Added
- Utility function to get the prim path that has the Articulation Root API

### Changed
- Get the prim path that has the Articulation Root API when the ArticulationView class is instantiated

## [3.7.0] - 2024-03-19
### Added
- Interops utility to convert tensors/arrays between ML frameworks (Warp, PyTorch, JAX, TensorFlow and NumPy)

## [3.6.0] - 2024-03-14
### Changed
- Changed the extension structure to allow for Cpp files to be added.
- Changed the find_matching_prim_paths util to a cpp implementation for higher perf.

## [3.5.1] - 2024-03-13
### Fixed
- Fixed get_prim_object_type function in prim utils.

## [3.5.0] - 2024-03-11
### Added
- Enable joint velocities computation using finite difference with TGS solver

## [3.4.0] - 2024-03-07
### Changed
- Removed the usage of the deprecated dynamic_control extension

## [3.3.8] - 2024-02-05
### Fixed
- Check for PhysicsScene USD type to identify whether a physics scene is already defined (OM-115537)

## [3.3.7] - 2024-02-02
### Changed
- Updated path to the nucleus extension
- Added deprecation warnings to the nucleus functions in nucleus.py

## [3.3.6] - 2024-02-02
### Changed
- Changed get_assets_root_path to get_assets_root_path_async for the unit tests

### Fixed
- Always set update to USD flag when fabric is enabled

## [3.3.5] - 2024-01-31
### Changed
- Update asset paths to 4.0

## [3.3.4] - 2024-01-10
### Fixed
- Fix undefined variables when clone=False for force APIs

## [3.3.3] - 2023-12-18
### Changed
- Add examples to docstrings, fix type annotations, and improve description. Affected modules: isaacsim.core.api.scenes (scene, scene_registry)

## [3.3.2] - 2023-12-13
### Fixed
- set_live_sync method

## [3.3.1] - 2023-12-12
### Fixed
- Unit tests

## [3.3.0] - 2023-12-1
### Changed
- Add more articulation metadata for OM-116002
- Fix contact reporter API schema preparation for OM-116001
- Fix joint limits OM-91153

## [3.2.1] - 2023-12-01
### Changed
- Add examples to docstrings, fix type annotations, and improve description. Affected modules: isaacsim.core.api.world (world) and isaacsim.core.api.simulation_context (simulation_context)

## [3.2.0] - 2023-11-30
### Changed
- Add get_assets_root_path_async(). Fix for OM-112464.
- Add get_full_asset_path_async()
- Add get_server_path_async()

## [3.1.0] - 2023-11-29
### Changed
- /app/runLoops/main/rateLimitEnabled in standalone workflow will be set to False

### Added
- set_block_on_render and get_block_on_render to control waitIdle flag

## [3.0.6] - 2023-11-29
### Changed
- Apply codespell to fix common misspellings and typos

### Fixed
- quats_to_rot_matrices under torch utils to handle 1 dimensional input and batched
- pad method under torch utils

### Added
- added rot_matrices_to_quats method to torch utils

## [3.0.5] - 2023-11-28
### Fixed
- Forward the density parameter to the RigidPrimView instance in RigidPrim class constructor
- Fix argument typo when applying a physics material to a ground plane object

### Changed
- Add examples to docstrings, fix type annotations, and improve description. Affected modules: isaacsim.core.api.objects (capsule, cone, cuboid, cylinder, sphere, ground_plane)

## [3.0.4] - 2023-11-27
### Fixed
- Add missing worker thread parsing from sim config

## [3.0.3] - 2023-11-27
### Changed
- Add examples to docstrings, fix type annotations, and improve description. Affected modules: isaacsim.core.api.prims (xform_prim, xform_prim_view, rigid_prim, rigid_prim_view, rigid_contact_view, geometry_prim, geometry_prim_view, base_sensor)

## [3.0.2] - 2023-11-24
### Fixed
- Fix indices used to set the GeometryPrimView collision API properties
- Fix the indices comparison that prevented applying several physical materials to a GeometryPrimView object

## [3.0.1] - 2023-11-20
### Changed
- Add examples to docstrings, fix type annotations, and improve description. Affected modules: isaacsim.core.api.articulations (articulation, articulation_view) and isaacsim.core.api.robots (robot, robot_view)

### Fixed
- Use articulation view metadata to get DOF types given specific DOF names
- Remove unexpected keyword argument when printing the stage via the stage utils

## [3.0.0] - 2023-11-15
### Changed
- Removed create_hydra_texture; use rep.create.render_product from omni.replicator.core instead.

## [2.12.1] - 2023-11-14
### Changed
- Add examples to docstrings, fix type annotations, and improve description. Affected modules: isaacsim.core.api.utils (bounds, carb, constants, extension, mesh, physics, prim, stage)

## [2.12.0] - 2023-11-09
### Changed
- Update asset paths to 2023.1.1

## [2.11.0] - 2023-10-31
### Added
- test for get_joint_position in test_articulation to catch the sign switch that happens when articulation joints have different body0 and body1 than expected.

### Fixed
- renamed utils.torch.rotations quat_to_rot_matrices to quats_to_rot_matrices to be consistent with numpy, originally named function is now a redirect function

## [2.10.2] - 2023-10-12
### Fixed
- Bug in ArticulationController with np.isnan

## [2.10.1] - 2023-10-10
### Fixed
- Fixed bug in matrix_to_euler_angles, euler_to_rot_matrix
- Apply physics warm start in async play and add render_async

## [2.10.0] - 2023-10-04
### Added
- euler_to_rot_matrix returns the pre multiplicative matrix and not the post multiplicative one as before and in numpy format instead of Gf.Rotation.
- added extrinsic argument to torch rotation utils: compute_rot, quat_from_euler_xyz, get_euler_xyz, euler_angles_to_quats
- added methods to torch rotation utils: quat_to_rot_matrices, matrices_to_euler_angles
- added extrinsic argument to numpy rotation utils: quats_to_euler_angles, euler_angles_to_quats
- added extrinsic argument to rotation utils: euler_angles_to_quat, quat_to_euler_angles, matrix_to_euler_angles, euler_to_rot_matrix

## [2.9.4] - 2023-09-30
### Fixed
- Correctly set GPU pipeline when it is missing from SimConfig

## [2.9.3] - 2023-09-29
### Fixed
- Propagate physX to Fabric when stepping physX and kit separately

## [2.9.2] - 2023-09-27
### Fixed
- Articulation Root mismatch with the default state in ArticulationView

## [2.9.1] - 2023-09-26
### Fixed
- Fixed a bug in a rigid_prim_view unit test

## [2.9.0] - 2023-09-22
### Added
- Add missing GPU collision stack size API in physics context

## [2.8.0] - 2023-09-20
### Fixed
- Fixed gpu device parsing.
- Added gpu flushing as a physics callback to handle the extension workflow and the standalone one alike.
- Removed the destructor calls from SimulationContext and World for possible bugs concerning them being Singletons.

## [2.7.7] - 2023-09-18
### Fixed
- GeometryPrimView.is_collision_enabled to read values set through USD and not the class
- GeometryPrimView.get_physics_visual_materials typo to use physics material rather than visual material

## [2.7.6] - 2023-09-13
### Fixed
- None in numpy arrays in current numpy version 1.25.2 gets converted to nan, propagated this change to ArticulationController.apply_action

## [2.7.5] - 2023-09-12
### Changed
- Changed get_world_pose fabric util to check for the attribute before getting it to avoid warnings

## [2.7.4] - 2023-08-28
### Changed
- Added warnings to OgnIsaacReadFilePath

### Fixed
- Fixed the ``__new__`` method in the :class:`SimulationContext` class to work for inherited class.

## [2.7.3] - 2023-08-28
### Fixed
- Updated utils.viewports.set_camera_view to check both x and y of the position and target

## [2.7.2] - 2023-08-25
### Changed
- Updated view port unit test to accept identical fx and fy

### Fixed
- Add default values for World transform attributes when fetching fabric in _get_world_pose_transform_w_scale method

## [2.7.1] - 2023-08-16
### Fixed
- Vertical Aperture used from reading the horizontal aperture usd property and multiplying it by resolution ratio to conform to the square pixels assumption in place. (setting and getting intrinsics in viewports utils)

## [2.7.0] - 2023-08-11
### Added
- Add function create_viewport_for_camera() to viewports.py

## [2.6.0] - 2023-08-09
### Added
- get_local_pose and get_world_pose in xforms utils which will go through fabric if the prim exists there, otherwise it will read it from USD

### Changed
- XFormPrimView get_local_poses and get_world_poses uses the new methods available in xforms utils to query the poses from Fabric and USD alike
- timeline is stopped if initialize simulation context is called

### Fixed
- Crash when initializing world if is_playing was true

## [2.5.1] - 2023-08-08
### Fixed
- Fixed memory leak in ArticulationView, RigidPrimView and SimulationContext stemming from tensor api views
- Fixed minor typo in articulation view
- Fixed the order of parsing of the stage in the physics context warm start operation

## [2.5.0] - 2023-08-04
### Added
- Added SdfShapeView class for handling shapes signed-distance function

## [2.4.1] - 2023-08-01
### Changed
- Split World reset_async into scene construction and simulation start.

## [2.4.0] - 2023-08-01
### Added
- Added a new API for reading detailed contact data

## [2.3.1] - 2023-07-19
### Fixed
- set_camera_view now works when the camera is directly above or directly below the target (x and y positions are equal)

## [2.3.0] - 2023-07-10
### Added
- Added support for Warp backend for rigid classes

## [2.2.0] - 2023-07-10
### Added
- Added core wrappers for physx and physics.tensors deformable body APIs

## [2.1.0] - 2023-06-21
### Added
- Added new APIs for joint force sensors and dof effort sensors

## [2.0.0] - 2023-06-13
### Fixed
- Kit 105.1 update
- UsdShaderConnecatbleAPI explicit for OmniGlass and PreviewSurface constructors
- Renamed flatcache to fabric (enable_flatcache -> enable fabric in PhysicsContext) and sim_params dict key use_flatcache -> use_fabric
- Extension omni.physx.flatcache renamed to omni.physx.fabric
- Schema attributes moved from CLoth API to Particle API in ClothPrimView class
- SimulationContext sets TimeCodesPerSecond attribute and the timeline's target framrate to the rendering frequency to account for the decoupling of stage updates and app updates.
- set_prop_val move from omni.usd.utils to omni.usd

## [1.49.0] - 2023-05-31
### Added
- Added OBB functions compute_obb and compute_obb_corners to utils.bounds
- Added OBB tests

## [1.48.0] - 2023-04-18
### Changed
- Add function get_transform_with_normalized_rotation to utils.transformation.py
- Function returns transformation matrix after removing scale factor

## [1.47.0] - 2023-03-31
### Changed
- Update asset paths to 2023.1.0

## [1.46.3] - 2023-03-01
### Fixed
- Added joint efforts to the get_applied_action method in ArticulationView and Articulation (was returning None previously)

## [1.46.2] - 2023-02-14
### Fixed
- Simulation context should only subscribe to the type of stage event it needs

## [1.46.1] - 2023-02-09
### Added
- Added unit test for clear_instance() causing SimulationContext to self-delete on construction

## [1.46.0] - 2023-02-08
### Added
- texture_translate to omni_pbr.py

## [1.45.2] - 2023-02-05
### Fixed
- several unit test errors/warnings

## [1.45.1] - 2023-02-03
### Fixed
- fixed minor issues in cloth prims

## [1.45.0] - 2023-01-31
### Added
- destroy_all_viewports util

## [1.44.1] - 2023-01-23
### Fixed
- Warnings about no hydra render context when running tests
- Warning about SimulationContext/World needing re-init after a new stage should only happen if they were previously initialized.

## [1.44.0] - 2023-01-17
### Changed
- Make omni.kit.viewport.window and omni.kit.viewport.utility optional dependencies

## [1.43.0] - 2023-01-06
### Changed
- Update asset paths to 2022.2.1

## [1.42.0] - 2022-12-12
### Changed
- Update asset paths to 2022.2.0

## [1.41.1] - 2022-12-10
### Fixed
- refactored cloth API for consistency with other core prims

## [1.41.0] - 2022-12-10
### Added
- Added set_targets to prim utils

## [1.40.4] - 2022-12-09
### Fixed
- Data Logger mutating the data frames variable when saving is called.
- Simulation Context/ World used to revert the frame rate to the cached value before initializing SimulationContext/ World when a new stage is created. In kit 104 the setting is reset automatically when a new stage is loaded. So the explicit  caching and reverting to the original frame rate is removed from Simulation Context/ World.

## [1.40.3] - 2022-12-08
### Fixed
- deprecated get_joint_efforts in favor of get_applied_joint_efforts

## [1.40.2] - 2022-12-07
### Fixed
- Fixes getter/ setter for sleep threshold

## [1.40.1] - 2022-12-05
### Fixed
- RigidContactView for GeometryPrim/GeometryPrimView
- get_joint_efforts in Articulation is now consistent with that of ArticulationView class
- API doc fixes

## [1.40.0] - 2022-12-02
### Added
- Sleep threshold setter and getter in RigidPrimView

## [1.39.0] - 2022-11-30
### Added
- Particle cloth simulation support through physics.tensors extension

## [1.38.0] - 2022-11-29
### Added
- Added articulationView APIs to get the commanded and computed joint efforts

## [1.37.1] - 2022-11-29
### Fixed

- unit test for get_body_index function

## [1.37.0] - 2022-11-23
### Added

- get_body_index to return queried body index

## [1.36.1] - 2022-11-23
### Fixed
- API fixes and unit test improvements

## [1.36.0] - 2022-11-22
### Added
- Moved ArticulationSubset here from `omni.isaac.motion_generation`
- Updated ArticulationSubset to handle sparse ArticulationActions. Previously, it None-padded the ArticulationAction.
- Some modifications to ArticulationSubset to simplify the error checking code and change member names.
- Updates ArticulationMotionPolicy to use the sparse API.

## [1.35.0] - 2022-11-19
### Added
- Base Sensor Class

## [1.34.0] - 2022-11-19
### Added
- Backend utils in torch and numpy: pas, stack, matmul, sin, cos, inverse and transpose_2d

## [1.33.0] - 2022-11-18
### Changed
- moved util functions from core/utils/pose_generation.py to coreutils /transformations.py, /mesh.py, /random.py
- updated ycb_video_writer.py get_mesh_vertices_relative_to utils import to mesh.py

### Added
- mesh.py and random.py

## [1.32.2] - 2022-11-18
### Fixed
- Fixed set_local_poses when indices are provided

## [1.32.1] - 2022-11-16
### Fixed
- Fixed device for ArticulationView max_efforts

## [1.32.0] - 2022-11-09
### Added
- isaacsim.core.api.utils.render_product

## [1.31.1] - 2022-11-03
### Fixed
- Fixed order of matrix multiplications for variable source_to_target_column_major_tf in function get_relative_transform() in utils/pose_generation.py

## [1.31.0] - 2022-10-31
### Added
- Added RigidContactView class and APIs within RigidPrimView to retrieve net contact forces and contact forces from a subset of prims (OM-64746)

## [1.30.2] - 2022-10-25
### Changed
- Set persistent.isaac.asset_root.nvidia to main NVIDIA asset path (OM-64173)

## [1.30.1] - 2022-10-24
### Changed
- Changed default values of shape sizes to be 1.0 instead of 0.05

## [1.30.0] - 2022-10-17
### Added
- allow applying rigid body forces in local coordinates and also at a position

## [1.29.0] - 2022-10-16
### Added
- moved standalone pose estimation example utils to core.utils

## [1.28.2] - 2022-10-15
### Fixed
- bug in sphere.py and cylinder.py where incorrect prim type was used in IsA check

## [1.28.1] - 2022-10-03
### Changed
- Cuboids default size parameter from 0.05 to 1.

## [1.28.0] - 2022-09-29
### Changed
- Allow manual dt to be set if loop runner is available outside of SimulationApp

## [1.27.1] - 2022-09-28
### Changed
- Use blocking update_simulation call in warm_start

### Fixed
- disable_rigid_body_physics

## [1.27.0] - 2022-09-12
### Added
- get_id_from_index to convert a legacy viewport id index into a proper viewport id

## [1.26.1] - 2022-09-07
### Fixed
- Fixes for kit 103.5

## [1.26.0] - 2022-09-02
### Changed
- reset_xform_ops now resets to isaac sim defaults

### Added
- clear_xform_ops, reset_and_set_xform_ops
- set_prim_hide_in_stage_window, set_prim_no_delete
- add_aov_to_viewport

## [1.25.0] - 2022-08-31
### Changed

- removed unused velocity argument from set_camera_view
- removed default arguments from set_camera_view to make it more general
- switch to omni.kit.viewport.utility instead of viewport legacy

### Added

- viewport helper functions: get_viewport_names and get_window_from_id

## [1.24.4] - 2022-08-31
### Changed

- Update paths to 2022.2

### Added

- get_window_from_id to viewport.py

## [1.24.3] - 2022-08-17
### Fixed

- fixes `set_max_efforts` function: device must be on cpu

## [1.24.2] - 2022-08-17
### Fixed

- Reshape jacobian shape to match with shape of jacobian tensor.

## [1.24.1] - 2022-08-15
### Fixed

- Articulation Controller bugfix: `get_applied_action` was indexing joint_positions even if simulation is not running.

## [1.24.0] - 2022-08-14
### Added

- get_semantics to return all semantic APIs applied onto a prim

## [1.23.3] - 2022-08-09
### Fixed

- Articulation bugfix: `get_max_efforts` was always returning the `max_efforts` from PhysX instead of the joint-indices result when `clone=True`.
- Articulation bugfix: `get_linear_velocity`, `get_angular_velocity` and `get_joint_velocities` was calling view's method twice once with indices then once without.  The second time should be selecting the single element of the batch array from the `result` rather than calling the method again.

## [1.23.1] - 2022-08-03
### Fixed

- Articulation bugfix: `get_joint_positions` was calling view's method twice once with indices then once without.  The second time should be selecting the single element of the batch array from the `result` rather than calling the method again.

## [1.23.0] - 2022-07-26
### Added
- Added joint_indices to the different get_joint_* methods in the Articulation.
- Increase hang detection timeout (OM-55578)

## [1.22.1] - 2022-07-25
### Added
- Added setting gravity from sim config in physics context.

## [1.22.0] - 2022-07-21
### Added
- Added reset_xform_properties parameter to view classes for efficiency when the objects already have the right set of xform properties.

## [1.21.0] - 2022-07-21
### Added
- Added new APIs for ArticulationView and RigidPrimView

## [1.20.0] - 2022-07-17
### Changed
- single prim classes inheritance structure to avoid duplication of code

## [1.19.0] - 2022-07-16
### Added
- added get_first_matching_parent_prim, is_prim_non_root_articulation_link to prim utils

### Changed
- get_all_matching_child_prims to return a list of prims instead of a list of prim_paths
- get_first_matching_child_prim returns a prim instead of a prim path

## [1.18.0] - 2022-06-23
### Changed
- statistics.py moved to omni.isaac.statistics_logging extension

## [1.17.0] - 2022-06-22
### Changed
- Size to be a float for Cuboid instead of 3 dimensional (scale to be used instead for consistency with USD)

## [1.16.0] - 2022-06-16
### Changed
- save_stage allows in place saving without reloading stage.

## [1.15.2] - 2022-06-13
### Added
- Parse GPU device ID from carb settings /physics/cudaDevice.

### Fixed
- Fixed GPU buffer attribute mismatch in physics context config parsing.

## [1.15.1] - 2022-06-02
### Fixed
- handles_initialized in Articulation class

## [1.15.0] - 2022-05-30
### Changed
- move and rename persistent.isaac.asset_root.cloud from assets_check extension

## [1.14.0] - 2022-05-26
### Changed
- Replaced .check on physics views with an event callback for efficiency.
- Adds checking for prim/prms in remove-object

## [1.13.2] - 2022-05-26
### Added
- Added APIs to get/set Enable Scene Query Support attribute

## [1.13.1] - 2022-05-25
### Changed
- Renamed copyAssetsURL to cloudAssetsURL.

## [1.13.0] - 2022-05-24
### Fixed
- Setting pd gains in the gpu pipeline.

## [1.12.0] - 2022-05-17
### Fixed
- Object classes to use RigidViews if initialized
- Bug is set_local_poses in RigidPrimView and ArticulationView

### Added
- Physics Handles check to avoid calling tensor api when the view is not valid.
- Added persistent.isaac.asset_root.default
- Added get_full_asset_path()

## [1.11.0] - 2022-05-16
### Fixed
- Object classes to wrap existing prims without changing its properties
- Setting gains to persist across resets

### Changed
- Passing physics materials instead of physics material path along with its properties.

## [1.10.0] - 2022-05-12
### Added
- initialize_physics function in World and SimulationContext

### Fixed
- GPU warmup

## [1.9.0] - 2022-05-12
### Changed
- Use omni.isaac.version.get_version()

### Added
- Added persistent.isaac.asset_root.nvidia and persistent.isaac.asset_root.isaac setting
- Added get_nvidia_asset_root_path() and get_isaac_asset_root_path()
- Added get_url_root() and verify_asset_root_path()

### Removed
- Removed persistent.isaac.nucleus.default setting
- Removed find_nucleus_server() and find_nucleus_server_async()

## [1.8.0] - 2022-05-06
### Changed
- Removing redundant api in ArticulationView and RigidPrimView
- Raise Exceptions when using set_linear_velocities and set_angular_velocities with the gpu pipeline

## [1.7.0] - 2022-05-06
### Changed
- RigidPrim class using RigidPrimView, GeometryPrim uses GeometryPrimView and Articulation uses ArticulationView class

## [1.6.9] - 2022-05-05
### Added
- Disable GPU usage warnings from tensor APIs in Core APIs
- articulation: added an accessor for getting the default state. (previously you could only set it)

## [1.6.8] - 2022-05-05
### Changed
- Added the option to enable flatcache in physics_context
- Disabled updateToUsd in physics_context when flatcache is enabled to allow faster load time

## [1.6.7] - 2022-05-03
### Changed
- Reorganized the functions in World and SimulationContext to make them clearer to understand

### Added
- Added reset(), reset_async(), clear() methods to SimulationContext

### Fixed
- Reset in World was always resetting the physics sim view

## [1.6.6] - 2022-05-02
### Changed
- Update DOF path parsing in ArticulationView to use tensor API directly
- Use tensor APIs when available for DOF properties

## [1.6.5] - 2022-04-28
### Added
- API to enable/disable omni.physx.flatcache extension in PhysicsContext
- API to track whether GPU pipeline is enabled

### Fixed
- issue with getting next stage free path slash parsing

## [1.6.4] - 2022-04-27
### Added
- density in rigid prim view

### Removed
- Sim start in XFormPrim view and ArticulationView doesn't create a dummy physics view anymore

## [1.6.3] - 2022-04-27
### Fixed
- missing args for `convert()` method

## [1.6.2] - 2022-04-26
### Fixed
- Fixed create_prim method to support sequence data type
- Fixed prim interfaces to use sequence data type for setters and getters for pose and velocities
- Added method `convert()` to backend utils to convert into respective object container

## [1.6.1] - 2022-04-21
### Added
- Added checks for setters/getters of Geometry prim in the case collision is disabled

### Changed
- replaced find_nucleus_server() with get_assets_root_path()
- Adapts the hierarchy of classes in object prims: The inheritance is as follows:
    - Visual<Obj>(GeometryPrim): collision is disabled
    - Fixed<Obj>(Visual<Obj>): collision is enabled
    - Dynamic<Obj>(Fixed<Obj>, RigidPrim): collision is enabled and rigid body API applied (which enables the influence of external forces)

### Fixed
- Fixed issue with specifying a USD path for a view regex

## [1.6.0] - 2022-04-18
### Fixed
- Fixed assets version file check.
- acceleration spelling mistake in articulation_controller

## [1.5.2] - 2022-04-18
### Fixed
- cleaned up imports and comments in the utils

## [1.5.1] - 2022-04-15
### Fixed
- fixing visibility on XFormPrimView
- docstring issues

## [1.5.0] - 2022-04-14
### Added
- An argument to clear scene registry only
- rotation and cross product util functions

### Fixed
- Deleting a reference always when trying to delete a prim under the ref
- Physics start on construction of XFormPrim to be able to use dc interface to query if its under an articulation.
- XFormPrimView: fixed setting translation on init

## [1.4.0] - 2022-04-13
### Changed
- world.py: add step_sim param to step() paralleling the render flag

## [1.3.0] - 2022-04-10
### Changed
- XFormPrim class to use XFormPrimView class internally
- Changed default value of visibility in the XFormPrim class

### Added
- added rotation conversion functions to and from quaternions

## [1.2.0] - 2022-04-08
### Added
- Added implementations of set_gains, set_max_efforts, set_effort_modes, switch_control_modes and the their getters in ArticulationView.
- Forced physics to start on init of ArticulationView to initialize the num_dofs and other variables.
- Added unit tests for ArticulationView.
- Added initial docstrings for the added functions.

## [1.1.0] - 2022-04-05
### Added
- added pose_from_tf_matrix() to isaacsim.core.api.utils.transformations

## [1.0.0] - 2022-03-31
### Added
- First version of Tensor API integration

## [0.3.2] - 2022-03-17
### Fixed
- converting gains from dc to usd units when saving to usd

## [0.3.1] - 2022-03-16
### Changed
- replaced find_nucleus_server() with get_assets_root_path()

### Added
- added get_assets_server()

## [0.3.0] - 2022-02-23
### Added
- set gains in usd option is added to the articulation controller

## [0.2.9] - 2022-02-14
### Added
- get/set_rigid_body_enabled to isaacsim.core.api.utils.physics
- default predicate to isaacsim.core.api.utils.prims.get_all_matching_child_prims
- test_prims to isaacsim.core.api.tests

### Changed
- _list and _recursive_walk in omni.isaac.nucleus to list_folder and recursive_list_folder

## [0.2.8] - 2022-02-14
### Fixed
- Fix setting of local pose in XFormPrim constructor

## [0.2.7] - 2022-02-13
### Fixed
- Use a SDF Change block when deleting prims
- Do not delete /Render/Vars prim when clearing stage

### Added
- is_prim_hidden_in_stage

## [0.2.6] - 2022-02-10
### Fixed
- GeometryPrim was not setting the collision approximation type correctly

## [0.2.5] - 2022-02-04
### Changed
- isaac.nucleus.default is now a persistent carb setting

## [0.2.4] - 2022-02-02
### Added
- dof_names property to Articulation

## [0.2.3] - 2022-01-26
### Added
- enable/disable rigid_body_physics for RigidPrims
- enable_gravity() for Articulation

### Fixed
- disable_gravity() for Articulation was enabling gravity

## [0.2.2] - 2022-01-21
### Added
- remove_all_semantics util function
- add set_intrinsics_matrix function

### Changed
- get_intrinsics_matrix uses vertical_aperture set on camera prim
- set_camera_view can take a user specified camera path

## [0.2.1] - 2022-01-20
### Changed
- kinematics.py to omni.isaac.motion_generation extension

## [0.2.0] - 2022-01-11
### Changed
- physx and usd transformations update parameters are read from carb

### Added
- Added set_defaults to SimulationContext, World and PhysicsContext

## [0.1.12] - 2021-12-16
### Changed
- Added feature to detect and update downloaded Isaac Sim assets on Nucleus (OM-41819)

## [0.1.11] - 2021-12-08
### Changed
- recompute_extents now takes an argument to include children in recomputation

## [0.1.10] - 2021-12-01
### Added
- isaac.nucleus.default setting moved from isaacsim.core.utils

### Fixed
- XformPrim now checks if orient is in single or double precision before setting

### Changed
- gf_quatf_to_np_array and gf_quatd_to_np_array to gf_quat_to_np_array
- control_index to time_step_index in pre_step in BaseTask

## [0.1.9] - 2021-11-29
### Added
- get_memory_stats to return a dictionary with memory usage statistics

## [0.1.8] - 2021-11-08
### Added
- Propagated scale and visible args to different objects inheriting from XFormPrim object.
- is_simulating to SimulationContext

### Changed
- Split add_ground_plane to add_default_ground_plane and add_ground_plane.
- Changed initialize_handles to initialize.

## [0.1.7] - 2021-11-06
### Added
- Added separate functions to set different physics scene settings.
- Added error handling in PhysicsContext.
- Added doc strings to PhysicsContext.
- create_bbox_cache, compute_aabb, compute_combined_aabb

### Changed
- moving offset logic to base task to move the task assets accordingly
- changed name of PhysicsScene to PhysicsContext
- renamed mesh.py to bounds.py

## [0.1.6] - 2021-11-04
### Added
- OmniPBR visual material
- Added get_physics_scene in SimulationContext
- clear in world
- get_extension_path_from_name
- is_prim_no_delete

### Changed
- default visual materials and physics materials prim paths
- prim_type is default to Xform for create_prim
- type changed to prim_type for add_reference_to_stage
- get_prim_at_descendent_path -> get_first_matching_child_prim
- get_prims_path_at_descendent_tree -> get_all_matching_child_prims
- check_ancestral -> is_prim_ancestral

### Fixed
- default pose was resetting using local pose
- local pose in Rigid Body was missing an argument
- clear in Scene

### Removed
- set_extension_enabled

## [0.1.5] - 2021-11-01
### Added
- Renamed Cube objects to Cuboids
- Generalized Cubes to Cuboids instead
- Added support for OmniGlass get_applied_visual material
- Changed parent_prim_path to prim_path in OmniGlass
- Unit tests for time stepping
- Switched visual_material_path to visual_material when passed to the different objects

## [0.1.4] - 2021-10-29
### Added
- FixedCuboid class
- Calculate_metrics, is_done and async step functions to World
- More stepping examples to time_stepping.py
- Unit tests for time stepping

### Changed
- Made BaseTask methods non abstract
- Renamed set_physics_dt to set_simulation_dt for simulation context
- physics_dt and rendering_dt separated
- editor_callback change to render_callback
- Moved PhysicsScene to physics_scene.py

## [0.1.3] - 2021-10-21
### Changed
- Renamed view_ports.py to viewports.py
- renamed nucleus_utils.py to nucleus.py

### Added
- disable_extension isaacsim.core.api.utils.extensions
- lookat_to_quat to isaacsim.core.api.utils.rotations
- get_intrinsics_matrix, backproject_depth, project_depth_to_worldspace to isaacsim.core.api.utils.viewports
- set_up_z_axis to isaacsim.core.api.utils.stage.set_stage_up_axis

## [0.1.2] - 2021-10-20
### Added
- Added articulation gripper class
- Deleted PD in the namings under articulation controller
- Added base tasks for stacking, pick_place and follow_target
- Added an IK solver under utils

## [0.1.1] - 2021-10-18
### Added
- Added *_callback_exists methods under SimulationContext
- Added object_exists method under Scene

### Changed
- The behavior of .play() method under SimulationContext to always do 1 step with rendering for dc to function properly.

## [0.1.0] - 2021-10-15
### Added
- Added first version of core.
