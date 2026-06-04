# Overview

```{deprecated} 6.0.0
This extension is deprecated in favor of `isaacsim.robot_motion.experimental.motion_generation` and `isaacsim.robot_motion.cumotion`.
```

The isaacsim.robot_motion.motion_generation extension provides a comprehensive framework for generating collision-aware robot motion in Isaac Sim. This extension implements both real-time reactive motion policies and trajectory generation capabilities, with a primary focus on Lula-based motion algorithms that enable dynamic obstacle avoidance and smooth robot control.

<div align="center">

```{mermaid}
graph TD
    %% Inheritance relationships
    WorldInterface --> MotionPolicy
    WorldInterface --> KinematicsSolver
    WorldInterface --> PathPlanner
    MotionPolicy --> RmpFlow
    KinematicsSolver --> LulaKinematicsSolver
```

</div>

## Concepts

### Motion Policies vs Trajectories

The extension distinguishes between two fundamental approaches to robot motion:

**Motion Policies** are real-time reactive algorithms that compute joint targets dynamically based on the current robot state and environment. The {class}`MotionPolicy <isaacsim.robot_motion.motion_generation.MotionPolicy>` interface enables collision-aware movement to targets while adapting to dynamic obstacles in real-time.

**Trajectories** represent pre-computed continuous-time paths that specify desired joint positions and velocities over a time interval. The {class}`Trajectory <isaacsim.robot_motion.motion_generation.Trajectory>` interface supports both configuration space and task space trajectory generation with time-optimal planning capabilities.

### World Interface

The {class}`WorldInterface <isaacsim.robot_motion.motion_generation.WorldInterface>` provides a standardized way for motion algorithms to understand and interact with the USD scene. It translates USD world representations into formats that motion policies and path planners can process, supporting various obstacle types including cuboids, spheres, capsules, cylinders, and cones.

## Key Components

### {class}`RmpFlow <isaacsim.robot_motion.motion_generation.RmpFlow>` Motion Policy

{class}`RmpFlow <isaacsim.robot_motion.motion_generation.RmpFlow>` is the primary motion policy implementation, providing real-time reactive control that smoothly guides robots to task space targets while avoiding dynamic obstacles. It supports both configuration space and end-effector targeting, with built-in collision avoidance and null space optimization.

Key features include:
- Real-time obstacle avoidance with dynamic updates
- Configurable substep integration for stability
- Internal state tracking and visualization capabilities
- Support for both position and orientation targets

### Kinematics Solvers

The extension provides kinematics solving capabilities through the {class}`KinematicsSolver <isaacsim.robot_motion.motion_generation.KinematicsSolver>` interface, with {class}`LulaKinematicsSolver <isaacsim.robot_motion.motion_generation.LulaKinematicsSolver>` as the primary implementation. These solvers compute forward and inverse kinematics for robot frames, supporting:

- Multi-frame kinematics calculations
- Collision-aware inverse kinematics (when supported)
- Configurable solver parameters and tolerances
- Joint limit enforcement and optimization

### {class}`Trajectory <isaacsim.robot_motion.motion_generation.Trajectory>` Generators

Two specialized trajectory generators enable path planning in different coordinate systems:

**{class}`LulaCSpaceTrajectoryGenerator <isaacsim.robot_motion.motion_generation.LulaCSpaceTrajectoryGenerator>`** creates time-optimal trajectories connecting configuration space waypoints using spline-based interpolation with velocity, acceleration, and jerk limit enforcement.

**{class}`LulaTaskSpaceTrajectoryGenerator <isaacsim.robot_motion.motion_generation.LulaTaskSpaceTrajectoryGenerator>`** converts task space waypoints into joint space trajectories, supporting linear path interpolation between positions and orientations for specified end-effector frames.

### Articulation Wrappers

Several wrapper classes bridge the gap between motion generation algorithms and Isaac Sim's articulation system:

**{class}`ArticulationMotionPolicy <isaacsim.robot_motion.motion_generation.ArticulationMotionPolicy>`** wraps {class}`MotionPolicy <isaacsim.robot_motion.motion_generation.MotionPolicy>` implementations to work directly with SingleArticulation objects, handling joint subset management and physics timestep calculations.

**{class}`ArticulationTrajectory <isaacsim.robot_motion.motion_generation.ArticulationTrajectory>`** converts continuous-time {class}`Trajectory <isaacsim.robot_motion.motion_generation.Trajectory>` objects into discrete ArticulationActions that can be applied to simulated robots.

**{class}`ArticulationKinematicsSolver <isaacsim.robot_motion.motion_generation.ArticulationKinematicsSolver>`** simplifies kinematics computations for simulated robots, automatically handling current joint states and returning results in ArticulationAction format.

## Integration

The extension integrates with Isaac Sim's core articulation and physics systems through the isaacsim.core.api dependency. Motion policies and trajectories output ArticulationAction objects that work seamlessly with Isaac Sim's PD control system, where joint targets are achieved through the equation: `kp*(target_position - current_position) + kd*(target_velocity - current_velocity)`.

The {class}`MotionPolicyController <isaacsim.robot_motion.motion_generation.MotionPolicyController>` provides a standardized controller interface that integrates motion policies with Isaac Sim's controller framework, enabling easy integration into simulation scenarios and robot control pipelines.
