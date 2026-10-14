# Changelog

## [7.2.0] - 2026-09-22
### Changed
- Deliver the motion generation implementation from the Kit-independent `isaacsim-robot-motion` library wheel. The `isaacsim.robot_motion.experimental.motion_generation` import path and public API are unchanged; the extension stages the wheel in `pip_prebundle` and depends on the `isaacsim.common`, `isaacsim.core.experimental.objects`, `isaacsim.core.experimental.prims`, `isaacsim.core.experimental.utils`, and `isaacsim.robot_motion.schema` extensions.

### Removed
- Remove the in-extension Python implementation (now provided by the library wheel). Behavioral tests remain in the extension and run against the wheel-provided implementation.

## [7.1.3] - 2026-09-04
### Changed
- Prevent transform reset during world binding

## [7.1.2] - 2026-08-26
### Changed
- Replaced checking local scaling to checking global uniform scaling

## [7.1.1] - 2026-08-10
### Changed
- `get_shape_type` and scene-query plane detection use USD `Usd.Prim.IsA(<type name>)` instead of the shape `are_of_type` classifier.
- Route motion-generation warnings through `isaacsim.common.logging` and configure the Kit logging channel during extension startup.

## [7.1.0] - 2026-07-22
### Changed
- `WorldBinding` reads obstacle geometry, scale, and collision-enabled state during initialization and synchronizes only transforms at runtime.
- Use core experimental bounds utilities for world AABB and untransformed OBB collision approximations.
- Use core experimental prim helpers for prototype filtering, attribute access, and mesh topology reads.

### Removed
- Remove USDRT property change tracking and the `WorldBinding.synchronize` and `WorldBinding.synchronize_properties` methods. Use `synchronize_transforms` for runtime pose updates.

## [7.0.0] - 2026-07-06
### Changed
- Updates the class names.

## [6.1.4] - 2026-07-03
### Changed
- Emit `carb.log_warn` on `BaseController` error paths when `forward()` returns `None` or `reset()` fails.

## [6.1.3] - 2026-06-09
### Fixed
- Fix linter errors and missing or incomplete docstrings, and update `python_api.md`.

## [6.1.2] - 2026-05-05
### Changed
- `JointState` and `SpatialState` build their per-row valid views (`positions`, `velocities`, `position_indices`, etc.) lazily on first property access, reusing zero-copy NumPy slices instead of eagerly allocating `wp.array`s in the constructor. Public API and semantics are unchanged.

## [6.1.1] - 2026-03-04
### Fixed
- Fixed type annotation errors by adding `from __future__ import annotations` to files using union type syntax.

## [6.1.0] - 2026-03-04
### Changed
- Added Overview.md and python_api.md and updated docstrings

## [6.0.0] - 2026-03-03
### Changed
- `add_oriented_bounding_boxes` use pure warp-arrays, accepts a quaternion for local rotation instead of a rotation matrix.
- `OBB` uses quaternion instead of rotation matrix to represent its rotation.

## [5.2.0] - 2026-03-02
### Added
- Improved error message when setting/getting ObstacleConfigurations on prims that don't exist.
- `SceneQuery` can accept a single include/exclude group.
- `JointState.from_name` and `JointState.from_index` now accept 2D-array inputs, as long as the size of the first dimension is 1.

### Changed
- Makes more explicit the existing functionality that ObstacleRepresentation can be initialized via string
- Adds an explicit check that the string maps to a valid ObstacleRepresentation
- Examples/docs to use the improved QOL interfaces.

### Fixed
- bug-fix: default safety tolerances no longer override shape safety tolerances in `ObstacleStrategy` if set afterwards.

## [5.1.0] - 2026-02-17
### Added
- `TrackableApi.MOTION_GENERATION_COLLISION` support for querying and tracking prims with `IsaacMotionPlanningAPI` instead of requiring physics collision APIs.
- `SceneQuery` now supports finding prims with `IsaacMotionPlanningAPI` applied via `TrackableApi.MOTION_GENERATION_COLLISION`.
- `WorldBinding` now supports tracking collision enabled state via the `isaac:motionPlanning:collisionEnabled` attribute when using `TrackableApi.MOTION_GENERATION_COLLISION`, allowing users to control collision checking independently from physics simulation.

## [5.0.0] - 2026-02-16
### Changed
- `BodyState` renamed to `SpatialState` for clarity and consistency (not only used for bodies).
- `RobotState.tool_frames` renamed to `RobotState.sites`, which is more general.
- `JointState` and `SpatialState` now require known state-spaces. This prevents combining `RobotState` objects which are not intended for the same control-space.
- `JointState` has much greater flexibility to combine, i.e. `joint_0` can be controlled in position while `joint_1` is controlled in effort. The logic to combine states has also been simplified.
- `SpatialState` has much greater flexibility to combine, i.e. `frame_0` can be controlled in orientation while `frame_1` is controlled in position. The logic to combine states has also been simplified.

### Added
- `JointState.from_name` and `JointState.from_index` constructors, intended for construction of `JointState` objects by users.
- `SpatialState.from_name` and `SpatialState.from_index` constructors, intended for construction of `SpatialState` objects by users.

## [4.0.0] - 2026-02-16
### Changed
- `WorldBinding.synchronize` is split across two separate functions which can be called independently (`synchronize_transforms` or `synchronize_properties`).
- The `WorldBinding.synchronize_properties` does some caching to perform less prim and property lookups.

### Removed
- `WorldBinding.synchronize_properties` no longer tracks the local matrix attribute. The RT change tracking on this attributes was updating for every object which moved, which was not the intended purpose, and caused inefficient updates.

## [3.0.0] - 2026-02-13
### Changed
- `TrajectoryFollower.reset` now has semantically correct meaning. When `True` is returned, it means we can start to call `forward` on the controller.
- `TrajectoryFollower` now follows a clear workflow: set_trajectory() --> reset() --> forward()
- `TrajectoryFollower` deletes the trajectory and start time whenever the trajectory time is out of bounds.

## [2.1.1] - 2026-02-09
### Changed
- Reset function of `TrajectoryFollower` bug fix, it was returning `False` when it should have returned `True`.

## [2.1.0] - 2026-02-05
### Added
- `WorldBinding`, `WorldInterface` and basic utility function for scaling validation of ancestor prims.

## [2.0.0] - 2026-02-04
### Changed
- All `wp.vec3` and `wp.quat` types were changed to `wp.array`.
- `JointState`, `BodyState` and `RootState` can all now have entries which are `None`, meaning they are not defined (for example, `JointState.velocity=None` indicating velocity is not known).
- The `combine_robot_states` function is more flexible.

## [1.1.0] - 2026-02-02
### Added
- `SceneQuery` and `TrackableApi`, for querying objects within an AABB for those with a certain API.

## [1.0.0] - 2026-01-31
### Added
- Additional control structures, in particular the `SequentialController` and `ParallelController`

### Changed
- `BaseController`, function `reset` now returns `True` if it is successful.
- `BaseController` function `forward` now returns an `Optional[RobotState]`. `None` indicates no valid output.
- `Trajectory` no longer has a (redundant) function for `get_joint_names`
- `Trajectory` returns an `Optional[RobotState]`. `None` indicates no valid output.
- `MinimalTimeTrajectory` is now named `MinimalTimeJointTrajectory`

### Removed
- `Action` type is no longer used, as `RobotState` is more general.

## [0.2.0] - 2026-01-27
### Added
- Introduces a full obstacle strategy and configurations.
- Adds collision approximation utilities

## [0.1.1] - 2026-01-25
### Changed
- Update license headers

## [0.1.0] - 2026-01-12
### Added
- Initial release
