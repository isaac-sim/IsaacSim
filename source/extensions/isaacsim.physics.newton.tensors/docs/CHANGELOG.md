# Changelog

## [0.8.1] - 2026-10-02
### Fixed
- Rigid body, articulation and rigid contact views report `check() == False` and fail their queries once Newton re-initializes and rebuilds its model and `state_0`, instead of reading released buffers; recreate the views.

## [0.8.0] - 2026-09-28
### Changed
- Updated Newton to 1.6.1rc1.

### Fixed
- Use coordinate-layout indices for Newton position targets, including floating-base robots.

## [0.7.2] - 2026-09-28
### Fixed
- Honor boolean masks in articulation and rigid-body masked setters.

## [0.7.1] - 2026-09-25
### Fixed
- Added set/get joint friction properties to the articulation view.

## [0.7.0] - 2026-09-22
### Added
- Compute articulation gravity and Coriolis compensation forces using Newton inverse dynamics on CPU and GPU.

### Fixed
- Preserve articulation indices when gathering batched dynamics tensors.

## [0.6.5] - 2026-09-21
### Fixed
- Transform body-local Newton rigid-contact points into world space correctly on CPU and GPU.

## [0.6.4] - 2026-08-21
### Fixed
- Apply off-center and local-frame rigid-body and articulation wrenches correctly on CPU and GPU.

## [0.6.3] - 2026-08-20
### Changed
- Add Newton solver-switching coverage to rigid-body tensor view tests.

## [0.6.2] - 2026-08-17
### Added
- Adding get/set gravity test

## [0.6.1] - 2026-08-12
### Fixed
- Fixed get/set gravity crash with simulation tensor view

## [0.6.0] - 2026-08-11
### Added
- `get_friction_data` support for Newton rigid-contact tensor views.

## [0.5.0] - 2026-08-11
### Changed
- Updated the Newton pip package to 1.5.0.

## [0.4.0] - 2026-08-05
### Changed
- Updated Newton pip package to 1.5.0rc2
- Use `Control.joint_target_q` / `joint_target_qd` (Newton 1.5 rename)
- Use `ModelFlags` for Newton 1.5 solver refresh notifications

## [0.3.2] - 2026-07-29
### Fixed
- Adding Jacobian and Mass Matrix Computation to the tensor api

## [0.3.1] - 2026-07-23
### Fixed
- Avoid repeated Newton initialization attempts after failure.

## [0.3.0] - 2026-07-21
### Changed
- Updated Newton pip package to 1.4.0

## [0.2.2] - 2026-07-20
### Changed
- Renamed C++ headers from `.h` to `.hpp`; update downstream include directives.

## [0.2.1] - 2026-07-13
### Fixed
- Resolve rigid contact sensor body paths with prefix and shape matching, including shapes under the sensor USD path.
- Return empty raw contact buffers instead of failing when Newton contact forces are not yet available.

## [0.2.0] - 2026-07-10
### Changed
- Updated Newton pip package to 1.4.0rc1

### Fixed
- Make the effort actuator-bridge test compare peak joint velocity instead of the final-step velocity.

## [0.1.10] - 2026-06-29
### Changed
- Remove internal Warp kernel helpers from the generated public API inventory.

## [0.1.9] - 2026-06-09
### Fixed
- Fix linter errors and missing or incomplete docstrings, and update `python_api.md`.

## [0.1.8] - 2026-06-05
### Changed
- Updated Newton pip package to 1.2.1

## [0.1.7] - 2026-05-19
### Fixed
- Update Go2 test asset path to `Mujoco_Menagerie/unitree_go2/go2/go2.usda` to match the new nested asset layout

## [0.1.6] - 2026-05-18
### Changed
- Add return type annotations and mypy type:ignore comments to pass mypy and ruff linting

## [0.1.5] - 2026-05-14
### Changed
- Updated newton pip dependencies to newton 1.2.0rc4, mujoco-warp 3.8.0.2, newton-usd-schemas 0.2.0

## [0.1.4] - 2026-05-05
### Added
- Log Python exceptions in C++ articulation view bridge calls (`_notifyJointDofPropertiesChanged`, `_syncCtrlDirectActuatorGains`, `_syncCtrlDirectPositionTargets`) instead of silently swallowing them

- Actuator bridge tests covering effort control, position targets, and PD gain overrides through the high-level `Articulation` API on a quadruped robot

## [0.1.3] - 2026-05-04
### Fixed
- Swap contact normal and separation sign convention between sensorA/sensorB in CPU and GPU contact data kernels to match MuJoCo 3.7 output

## [0.1.2] - 2026-05-01
### Added
- Implement `getAccelerations` for CPU and GPU rigid body views using Newton's `body_qdd` extended state attribute

## [0.1.1] - 2026-04-22
### Fixed
- Fix contact point positions for MuJoCo backend: detect world-space contact positions from `mjw_data.contact.pos` and skip body-local-to-world rotation in CPU and GPU contact data kernels

- Fix contact force dt scaling in `BaseRigidContactView` to apply `sim_dt / user_dt` matching the Python tensor implementation

## [0.1.0] - 2026-04-14
### Added
- C++ tensor backend implementing `omni.physics.tensors` interfaces for Newton physics, with CPU and GPU (Warp/CUDA) implementations of `SimulationView`, `ArticulationView`, `RigidBodyView`, and `RigidContactView`. Supports three device configurations (CPU sim + CPU view, GPU sim + CPU view, GPU sim + GPU view) for articulation and rigid body state, force/torque application, and contact queries.
