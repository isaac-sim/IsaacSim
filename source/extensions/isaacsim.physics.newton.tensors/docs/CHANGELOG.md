# Changelog

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
