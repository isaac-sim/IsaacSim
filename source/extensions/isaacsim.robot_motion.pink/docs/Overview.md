# Overview

The isaacsim.robot_motion.pink extension provides access to the PINK (Python Inverse Kinematics) library within Isaac Sim's motion generation framework. PINK formulates differential inverse kinematics as a quadratic program (QP) with weighted tasks and safety constraints, powered by the Pinocchio rigid-body dynamics library.

## Concepts

### PINK Integration

PINK solves differential inverse kinematics by composing weighted task objectives (end-effector tracking, posture regularization, velocity damping) with inequality constraints from joint limits, velocity limits, and control barrier functions. The extension wraps this into Isaac Sim's `BaseController` interface for seamless integration with simulation workflows.

### Robot Configuration

Robot setup requires a URDF file describing the kinematic chain. Pinocchio loads the URDF and provides forward kinematics, Jacobians, and frame placement. An optional SRDF file can configure self-collision exclusion pairs.

## Key Components

### {class}`PinkRobot <isaacsim.robot_motion.pink.PinkRobot>`

The {class}`PinkRobot <isaacsim.robot_motion.pink.PinkRobot>` dataclass holds the Pinocchio model, data, and controlled joint names. Load robots using {func}`load_pink_robot <isaacsim.robot_motion.pink.load_pink_robot>` for custom URDFs or {func}`load_pink_supported_robot <isaacsim.robot_motion.pink.load_pink_supported_robot>` for pre-configured robots bundled with the extension.

### {class}`PinkIKController <isaacsim.robot_motion.pink.PinkIKController>`

{class}`PinkIKController <isaacsim.robot_motion.pink.PinkIKController>` implements the `BaseController` interface using PINK's `solve_ik`. On each `forward()` call it updates task targets from the setpoint, solves the QP to obtain a joint velocity, and integrates the configuration. The controller supports configurable frame tasks, posture regularization, arbitrary user-supplied tasks, limits, and barriers.

## Integration

The extension integrates with Isaac Sim's experimental motion generation framework through the `isaacsim.robot_motion.experimental.motion_generation` dependency. The controller implements the `BaseController` interface, ensuring compatibility with `ControllerContainer`, `ParallelController`, `SequentialController`, and `TrajectoryFollower` from the broader motion generation ecosystem.
