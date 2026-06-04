# Overview

The isaacsim.robot_motion.cumotion extension provides access to cuMotion planners and controllers within Isaac Sim's motion generation framework. This extension bridges NVIDIA's cuMotion library with Isaac Sim, offering high-performance motion planning, trajectory generation, and reactive control for robotic manipulators.

## Concepts

### cuMotion Integration

The extension serves as a wrapper around NVIDIA's cuMotion library, translating between Isaac Sim's coordinate systems and data structures and cuMotion's internal representations. All cuMotion operations work in the robot base frame, while Isaac Sim typically works in the world frame - the extension handles these coordinate transformations automatically.

### Robot Configuration

Robot setup requires both URDF (Unified Robot Description Format) and XRDF (Extended Robot Description Format) files. The URDF describes the robot's kinematic structure, while the XRDF contains cuMotion-specific configuration parameters for planning and control.

## Key Components

### {class}`CumotionRobot <isaacsim.robot_motion.cumotion.CumotionRobot>`

The {class}`CumotionRobot <isaacsim.robot_motion.cumotion.CumotionRobot>` dataclass encapsulates all robot-specific data needed for cuMotion operations, including the robot description, kinematics solver, and controlled joint names. You can load robots using {func}`load_cumotion_robot <isaacsim.robot_motion.cumotion.load_cumotion_robot>` for custom configurations or {func}`load_cumotion_supported_robot <isaacsim.robot_motion.cumotion.load_cumotion_supported_robot>` for pre-configured robots.

### {class}`CumotionWorldInterface <isaacsim.robot_motion.cumotion.CumotionWorldInterface>`

{class}`CumotionWorldInterface <isaacsim.robot_motion.cumotion.CumotionWorldInterface>` manages collision geometry and obstacle tracking between Isaac Sim's USD scene and cuMotion's collision world. It handles coordinate frame transformations and provides methods to add various collision primitives (spheres, cubes, meshes, planes, capsules) and update their poses dynamically.

### Motion Planners

**{class}`GraphBasedMotionPlanner <isaacsim.robot_motion.cumotion.GraphBasedMotionPlanner>`** implements sampling-based planning algorithms (RRT variants) for finding collision-free paths. It supports planning to configuration space targets, full pose targets (position + orientation), and translation-only targets where orientation is unconstrained.

**{class}`TrajectoryOptimizer <isaacsim.robot_motion.cumotion.TrajectoryOptimizer>`** uses cuMotion's optimization-based approach to generate smooth, collision-free trajectories. It optimizes over multiple cost functions including smoothness, collision avoidance, and goal achievement.

### Controllers and Generators

**{class}`RmpFlowController <isaacsim.robot_motion.cumotion.RmpFlowController>`** implements cuMotion's RMPflow algorithm for reactive motion control. This controller continuously computes joint commands based on multiple task-space and configuration-space attractors, providing smooth and collision-aware motion.

**{class}`TrajectoryGenerator <isaacsim.robot_motion.cumotion.TrajectoryGenerator>`** creates smooth, time-optimal trajectories from discrete waypoints without collision checking. It handles conversion between configuration space and task space representations.

### {class}`CumotionTrajectory <isaacsim.robot_motion.cumotion.CumotionTrajectory>`

{class}`CumotionTrajectory <isaacsim.robot_motion.cumotion.CumotionTrajectory>` wraps cuMotion trajectories within Isaac Sim's trajectory interface, allowing queries for robot states at specific times along continuous-time trajectories.

## Integration

The extension integrates with Isaac Sim's experimental motion generation framework through the `isaacsim.robot_motion.experimental.motion_generation` dependency. Controllers implement the `BaseController` interface, trajectories implement the `Trajectory` interface, and world interfaces implement the `WorldInterface` interface, ensuring compatibility with Isaac Sim's broader motion generation ecosystem.
