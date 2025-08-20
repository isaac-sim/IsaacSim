# Changelog

## [3.2.16] - 2025-07-25
### Fixed
- Fix issue where multiple upstream connections to execIn were causing duplicate node executions
- Fix issue where correct frame time was not returned by Simulation Time annotator

## [3.2.15] - 2025-07-18
### Fixed
- Disable `xformOp` resetting when instantiating the articulation wrapper in the articulation state/controller nodes

## [3.2.14] - 2025-07-17
### Fixed
- Fix issue where explicit render product destruction caused crashes on stage cleanup

## [3.2.13] - 2025-07-07
### Fixed
- Correctly enable omni.kit.loop-isaac in test dependency (fixes issue from 3.2.12)

## [3.2.12] - 2025-07-05
### Changed
- Update tests to pass without a custom loop runner

## [3.2.11] - 2025-07-03
### Changed
- Removed bundles from OgnIsaacReadWorldPose and added unit test

## [3.2.10] - 2025-07-03
### Changed
- Make omni.kit.loop-isaac an explicit test dependency

## [3.2.9] - 2025-06-25
### Changed
- Add --reset-user to test args

## [3.2.8] - 2025-06-24
### Changed
- Add extension specific test args to extension.toml

## [3.2.7] - 2025-06-18
### Changed
- Track change from isaacsim.core.include Pose.h

## [3.2.6] - 2025-06-13
### Changed
- Switched CUDA files to Apache license.

## [3.2.5] - 2025-06-04
### Changed
- Changed CUDA_CHECK in ScopedCudaDevice.h to indicate file and line for more verbose error logging

## [3.2.4] - 2025-05-31
### Changed
- Use default nucleus server for all tests

## [3.2.3] - 2025-05-30
### Changed
- Update golden values for unit test

## [3.2.2] - 2025-05-19
### Changed
- Update copyright and license to apache v2.0

## [3.2.1] - 2025-05-19
### Fixed
- Unit tests failing due to non-deterministic behavior

## [3.2.0] - 2025-05-16
### Changed
- Deprecate all time related functions, use isaacsim.core.simulation_manager APIs instead

## [3.1.1] - 2025-05-15
### Changed
- UsdUtilities.h was updated

## [3.1.0] - 2025-05-13
### Changed
- Added IsaacReadSimulationTimeAnnotator node
- Added IsaacReadSystemTimeAnnotator node

## [3.0.2] - 2025-05-11
### Changed
- Enable FSD in test settings

## [3.0.1] - 2025-05-11
### Changed
- Remove internal build time dependency

## [3.0.0] - 2025-05-10
### Changed
- IsaacReadSimulationTime and IsaacReadSystemTime use referenceTimeNumerator and referenceTimeDenominator directly
- Simplified annotator structure for getting sim and system time
- Modified input arguments for python bindings to take a tuple instead of a rational time object which could not be created by the user

### Removed
- deprecated function and their python bindings getSimTimeAtSwhFrame, getSimTimeMonotonicAtSwhFrame, getSystemTimeAtSwhFrame, get_sim_time_at_swh_frame, get_sim_time_monotonic_at_swh_frame, get_system_time_at_swh_frame
- removed IsaacReadTimes node

## [2.2.17] - 2025-05-09
### Added
- Unit test in test_articulation_controller to verify joint_indices comparison logic functions properly

## [2.2.16] - 2025-05-06
### Fixed
- Fix issues where np array comparison was leading to broadcast errors if joint_indices had different shapes

## [2.2.15] - 2025-05-02
### Fixed
- Fix issues where name override was incorrectly used if set to empty string

## [2.2.14] - 2025-05-02
### Changed
- Remove all Dynamic control compile time dependencies

## [2.2.13] - 2025-04-29
### Fixed
- prim validation in OgnIsaacComputeOdometry node

## [2.2.12] - 2025-04-15
### Changed
- Use SimulationManager APIs to get time and simulation state

### Fixed
- Fix OgnIsaacScaleToGromStageUnit.ogn

## [2.2.11] - 2025-04-11
### Changed
- Update Isaac Sim robot asset path
- Update Isaac Sim robot asset path for the IsaacSim folder

## [2.2.10] - 2025-04-09
### Changed
- Update all test args to be consistent

## [2.2.9] - 2025-04-04
### Changed
- Version bump to fix extension publishing issues

## [2.2.8] - 2025-03-26
### Changed
- Cleanup and standardize extension.toml, update code formatting for all code

## [2.2.7] - 2025-03-24
### Changed
- Migrate to Events 2.0

## [2.2.6] - 2025-03-20
### Changed
- Improve doxygen docstrings

## [2.2.5] - 2025-03-11
### Changed
- Switch asset root for tests to internal nucleus

## [2.2.4] - 2025-03-09
### Fixed
- fix failing unit tests

## [2.2.3] - 2025-03-05
### Changed
- Update extension codebase to adhere to isaac sim extension structure and file naming  guidelines

## [2.2.2] - 2025-03-04
### Changed
- Update to kit 107.1 and fix build issues

## [2.2.1] - 2025-02-25
### Changed
- Update style format and naming conventions in c++ code, add doxygen docstrings

## [2.2.0] - 2025-02-21
### Added
- Added globalLinearVelocity and globalLinearAcceleration inputs to IsaacComputeOdometry

## [2.1.5] - 2025-01-28
### Fixed
- Windows signing issue

## [2.1.4] - 2025-01-26
### Changed
- Update test settings

## [2.1.3] - 2025-01-21
### Changed
- Update extension description and add extension specific test settings

## [2.1.2] - 2024-12-13
### Fixed
- Updated simulation gate to not execute if the input step is 0.

## [2.1.1] - 2024-12-11
### Fixed
- Compute Odometry node to validate chassis prim input.

## [2.1.0] - 2024-10-28
### Added
- Isaac Joint Name Resolver node which checks for any joint prims with isaac:nameOverride attributes that match the provided names and updates those names to their corresponding original prim names.

## [2.0.2] - 2024-10-28
### Changed
- Remove test imports from runtime

## [2.0.1] - 2024-10-24
### Changed
- Updated dependencies and imports after renaming

## [2.0.0] - 2024-10-08
### Changed
- Extension renamed to isaacsim.core.nodes.

## [1.16.3] - 2024-08-28
### Fixed
- Unit test failures due to omni.graph.nodes MakeArray being deprecated in favor of ConstructArray

## [1.16.2] - 2024-08-15
### Fixed
- Handling of writers when they are added via attach_writer function

## [1.16.1] - 2024-07-16
### Fixed
- Missing omni.isaac.dynamic_control dependency

## [1.16.0] - 2024-07-05
### Added
- Add OgnIsaacPassthroughImagePtr node
- register_node_writer_with_telemetry and register_annotator_from_node_with_telemetry helper functions

## [1.15.6] - 2024-06-25
### Fixed
- ArticulationController Node allowing for multiple types of control

## [1.15.5] - 2024-05-17
### Added
- New BaseWriterNode.attach_writer API enables attaching specific writer to specific render product, rather than all writers to single render product.

## [1.15.4] - 2024-05-16
### Fixed
- Fix Isaac Run One Simulation Frame node name

## [1.15.3] - 2024-05-09
### Changed
- Update source code to run a build clean of warnings

## [1.15.2] - 2024-05-02
### Fixed
- Physics step node update to latest kit.

## [1.15.1] - 2024-05-01
### Fixed
- Reset articulation controller inputs on stop

## [1.15.0] - 2024-04-28
### Added
- Time Splitter node

## [1.14.4] - 2024-04-18
### Fixed
- Check for joint indices need to account for the possibility of an array input

## [1.14.3] - 2024-04-18
### Added
- Telemetry for writers and annotators

## [1.14.2] - 2024-04-18
### Fixed
- Update IStageUpdate usage to fix deprecation error

## [1.14.1] - 2024-04-18
### Fixed
- Fault with Articulation Controller node when it receives no joint name or indices inputs

## [1.14.0] - 2024-04-04
### Added
- New Isaac Read System Time node to allow users to incorporate latest system time or system time at specified simulation frame (swh frame number).

## [1.13.3] - 2024-03-26
### Fixed
- Unit tests

## [1.13.2] - 2024-03-18
### Fixed
- Bug in Isaac Get Wold Pose Node's orientation

## [1.13.1] - 2024-03-12
### Fixed
- Multi GPU support in RGBA to RGB node

## [1.13.0] - 2024-03-11
### Added
- Isaac Get Wold Pose Node

## [1.12.3] - 2024-03-04
### Changed
- Updated omnigraph nodes to use per instance state instead of internal state

### Fixed
- Render products not disabling on stop
- RGBA to RGB node running twice per frame

## [1.12.2] - 2024-03-01
### Fixed
- Issue where render product is not cleared when opening a stage
- Render product resolution and camera prim not updating
- Camera info issue when path was blank

## [1.12.1] - 2024-02-24
### Changed
- Update source code to run a build clean of warnings

## [1.12.0] - 2024-02-22
### Changed
- Converted Isaac Simulation Gate and Read Camera Info nodes to C++
- Read camera info node no longer accept viewport as input

## [1.11.1] - 2024-02-21
### Added
- Replaced internalState with perInstanceState for the articulation controller node

## [1.11.0] - 2024-02-13
### Added
- Articulation State node

## [1.10.1] - 2024-02-05
### Changed
- Replaced internalState with perInstanceState for the ogn nodes
- Updated path to the nucleus extension

## [1.10.0] - 2024-01-30
### Added
- Run One Frame Simulation Node

## [1.9.0] - 2024-01-23
### Added
- Real Time Factor Node

## [1.8.1] - 2024-01-18
### Changed
- Changed get_assets_root_path to get_assets_root_path_async for the unit tests

## [1.8.0] - 2024-01-08
### Changed
- Moved header files into extension

## [1.7.2] - 2024-01-03
### Changed
- Removed usePath checkbox from OgnIsaacArticulationController node. Now if the robotPath field is populated, targetPrim is ignored.

- GUI users won't notice any behavior changes. Standalone workflow users must ensure any references to usePath in the script is removed.

## [1.7.1] - 2023-12-01
### Fixed
- Annotator unit test due to replicator update

## [1.7.0] - 2023-11-08
### Added
- On Physics Step Node

## [1.6.1] - 2023-09-21
### Changed
- Read file node will return False if file path does not exist or is invalid

## [1.6.0] - 2023-09-19
### Changed
- Add enabled flag to create render product node

## [1.5.0] - 2023-08-31
### Changed
- Added a default noop node to SDG pipeline helper nodes so that the graph is not deleted on stop

### Fixed
- ReadTimes node not passing execution state properly

## [1.4.3] - 2023-08-25
### Changed
- added stdout fail pattern for the expected no prim found edge case for the ogn test

## [1.4.2] - 2023-08-24
### Fixed
- Fixed camera info is empty bug in test_camera

## [1.4.1] - 2023-08-15
### Changed
- Changed omnigraph prim from bundle to target for OgnIsaacComputeOdometry, OgnIsaacArticulationController, OgnIsaacCreateRenderProduct, OgnIsaacSetCameraOnRenderProduct

## [1.4.0] - 2023-08-15
### Changed
- Use Replicator Annotator class for nodes that can provide data
- Point Cloud node returns width and height of data

## [1.3.2] - 2023-08-11
### Fixed
- Import error in OgnIsaacGetViewportRenderProduct

## [1.3.1] - 2023-08-09
### Fixed
- Added time code settings for test_physics_num_steps
- Vertical Aperture used from reading the horizonal aperture usd property and multiplying it by resolution ratio to conform to the square pixels assumption in place. (DepthToPointCloud and IsaacReadCameraInfo nodes)

## [1.3.0] - 2023-08-03
### Changed
- RGBA and Depth to PCL nodes use raw ptrs instead of arrays to improve perf

## [1.2.0] - 2023-07-31
### Added
- Added tracking for the number of physics step

## [1.1.0] - 2023-07-06
### Removed
- unused writer and node template attachment systems

## [1.0.0] - 2023-06-13
### Added
- IsaacReadTimes node
- get_sim_time_at_time
- get_sim_time_monotonic_at_time
- get_system_time_at_time
- IsaacReadTimesAOV node template
- IsaacReadTimes node template

### Changed
- Update to kit 105.1, build system update
- IsaacArticulationController targetPrim now optional
- getSimulationTimeAtSwhFrame now getSimulationTimeAtTime with rational time
- getSimulationTimeMonotonicAtSwhFrame now getSimulationTimeMonotonicAtTime with rational time
- getSystemTimeAtSwhFrame now getSystemTimeAtTime with rational time
- [SENSOR NAME]IsaacSimulationGate nodes to [RENDERVAR]IsaacSimulationGate to match synthetic data standard
- deprecated get_sim_time_at_swh_frame
- deprecated get_sim_time_monotonic_at_swh_frame
- deprecated get_system_time_at_swh_frame

### Removed
- IsaacReadSystemTime node
- swhFrameTime input/output from IsaacConvertRGBAToRGB node

## [0.24.0] - 2023-05-31
### Added
- Support for custom distortion type/values on a camera  to read camera info node

## [0.23.2] - 2023-03-12
### Fixed
- Pressing pause should not reset classes derrived from BaseResetNode

## [0.23.1] - 2023-02-21
### Fixed
- Missing fisheye parameters for read camera info

## [0.23.0] - 2023-02-21
### Added
- BaseWriterNode for nodes that have to attach and detach writers

### Fixed
- RGBAToRGB Node should pass buffer size and SWH frame number

## [0.22.2] - 2023-02-14
### Fixed
- Core nodes should only subscribe to the type of stage event it needs

## [0.22.1] - 2023-01-25
### Fixed
- remove un-needed cpp ogn files from extension

## [0.22.0] - 2023-01-09
### Added
- interface for caching and retreiving handles

## [0.21.0] - 2022-12-10
### Changed
- IsaacSimulationGate step value can now be set to zero to stop execution

### Added
- function to handle writer activation requests to avoid race conditions from camera helpers.

## [0.20.0] - 2022-12-05
### Added
- IsaacSetCameraOnRenderProduct Node
- render product support for ReadCameraInfo
- fisheye parameter support for ReadCameraInfo
- utility function to cache writer attach calls until the next frame
- ogn tests for IsaacCreateRenderProduct, IsaacReadCameraInfo

### Changed
- Deprecate viewport support in ReadCameraInfo

### Fixed
- Errors with SDG template registration
- Errors in default ogn tests

## [0.19.1] - 2022-12-01
### Fixed
- Articulation Controller Node skipping single array inputs

## [0.19.0] - 2022-11-09
### Added
- IsaacCreateRenderProduct node

## [0.18.0] - 2022-11-09
### Added
- IsaacGetViewportRenderProduct node

## [0.17.0] - 2022-10-04
### Changed
- CreateViewport node only creates one viewport
- CreateViewport takes a name as input, falls back onto viewportId as the name if name is not set

## [0.16.1] - 2022-10-03
### Fixed
- Fixes for kit 104.0

## [0.16.0] - 2022-09-29
### Added
- linearAcceleration, angularAcceleration to IsaacComputeOdometry

## [0.15.0] - 2022-09-28
### Added
- python bindings and tests for timing related APIs

## [0.14.3] - 2022-09-12
### Added
- unit test for create viewport node

### Fixed
- CreateViewport node uses legacy viewport ID which used to be the viewport index, the index is now converted to ID

## [0.14.2] - 2022-09-07
### Fixed
- Fixes for kit 103.5

## [0.14.1] - 2022-09-02
### Fixed
- bug with hiding /Render prim when it didn't exist

## [0.14.0] - 2022-08-31
### Changed
- Use omni.kit.viewport.utility instead of legacy viewport APIs

## [0.13.0] - 2022-08-09
### Added
- utility function to cache node activations until the next frame. This solves an issue where activating node templates from other nodes would cause a race condition

### Fixed
- IsaacSetViewportResolution node forces window aperture to reset if the resolution is changed.

## [0.12.2] - 2022-07-22
### Fixed
- In OgnIsaacArticulationController, added validity check for joint_indicies list to prevent unnecessary warning message

### Added
- Additional unit test for Articulation Controller node for cases where no joint names or indices were given

## [0.12.1] - 2022-07-21
### Fixed
- In OgnIsaacArticulationController, added validity check for joint_indicies list to prevent unnecessary warning message

## [0.12.0] - 2022-07-20
### Changed
- IsaacComputeOdometry takes either an articulation root or a valid rigid body prim for the chassisPrim input

## [0.11.4] - 2022-07-06
### Fixed
- Quaternion input descriptions

## [0.11.3] - 2022-07-05
### Fixed
- Kit 104 build error

## [0.11.2] - 2022-07-03
### Fixed
- Extension will still load if replicator templates fail to register. This prevents dependent extensions from also failing to load due to a replicator.core failure

## [0.11.1] - 2022-06-30
### Added
- Unit test for Articulation Controller node

## [0.11.0] - 2022-06-22
### Added
- Added node to read file contents from path

## [0.10.0] - 2022-06-08
### Added
- Added node to read OS environment variables

## [0.9.0] - 2022-05-31
### Changed
- Added node to set viewport resolution
- Articulation controller only initializes on start

## [0.8.2] - 2022-05-19
### Changed
- Added "step" input to OgnIsaacSimulationGate

## [0.8.1] - 2022-05-18
### Added
- Utility function to set target prims on OG nodes

## [0.8.0] - 2022-05-16
### Added
- Register nodes used in SDG pipeline

## [0.7.0] - 2022-05-14
### Added
- ReadSystemTime node
- SimulationGate node

## [0.6.3] - 2022-05-11
### Changed
- Articulation Handle is refreshed at every compute

## [0.6.2] - 2022-05-11
### Changed
- Joint indices now is part of ArticulationAction type in ArticulationControllerNode

## [0.6.1] - 2022-05-06
### Changed
- De-bundled ArticulationControllerNode

### Fixed
- Crash when stepping physics without playing timeline

## [0.6.0] - 2022-05-05
### Changed
- Moved ReadSImulationTime to core nodes category

### Added
- OgnIsaacScaleToFromStageUnit

### Fixed
- Node unit tests

## [0.5.2] - 2022-05-04
### Changed
- OgnIsaacGenerate32FC1 to cpp
- OgnIsaacGenerateRGBA and OgnIsaacConvertRGBAToRGB to use token type for encoding input/ouput
- Added execOut to OgnIsaacConvertRGBAToRGB

## [0.5.1] - 2022-05-03
### Changed
- Output data types to vectord and quatd in Isaac Compute Odometry node
- Articulation controller node takes bundles

## [0.5.0] - 2022-05-02
### Added
- OgnIsaacConvertDepthToPointCloud

## [0.4.1] - 2022-04-29
### Fixed
- Fixed bug with validating encoding input in OgnIsaacConvertRGBAToRGB

## [0.4.0] - 2022-04-26
### Added
- OgnIsaacGenerate32FC1
- OgnIsaacCreateViewport
- OgnIsaacReadCameraInfo
- OgnIsaacArticulationController

### Changed
- Cleanup UI node names

### Fixed
- fixed issue with swh frame not working when simulation was stopped.

## [0.3.0] - 2022-04-25
### Changed
- renamed OgnIsaacRGBAToRGB to OgnIsaacConvertRGBAToRGB
- renamed OgnIsaacTestGenerateRGBA to OgnIsaacGenerateRGBA
- using a global clock for simulation time
- added ability to get simulation time from swhFrameNumber

## [0.2.1] - 2022-04-22
### Changed
- Renamed odometry node to OgnIsaacComputeOdometry

## [0.2.0] - 2022-04-18
### Added
- RGBA to RGB and RGBA generator nodes

## [0.1.1] - 2022-04-01
### Fixed
- Added missing omni.graph dependency for tests

## [0.1.0] - 2022-03-28
### Added
- Added first version of core omnigraph nodes.
