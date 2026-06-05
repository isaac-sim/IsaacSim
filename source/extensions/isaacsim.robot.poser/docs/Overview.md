# Overview

The isaacsim.robot.poser extension provides functionality for authoring robot poses through inverse kinematics and managing named pose libraries. This extension enables developers to programmatically solve IK problems, store solutions as reusable named poses, and apply joint configurations to robots in both simulation and non-simulation contexts.

```{image} ../../../../source/extensions/isaacsim.robot.poser/data/preview.png
---
align: center
---
```


## Key Components

### {class}`RobotPoser <isaacsim.robot.poser.RobotPoser>`

The {class}`RobotPoser <isaacsim.robot.poser.RobotPoser>` class serves as the primary interface for IK-based pose authoring. It wraps a kinematic chain and provides high-level methods for solving inverse kinematics problems.

```python
# Create a poser for a specific robot and kinematic chain
poser = RobotPoser(stage, robot_prim, start_prim, end_prim)

# Solve IK for a target transform
target_transform = Transform(position=[0.5, 0.2, 0.8], orientation=[1, 0, 0, 0])
result = poser.solve_ik(target_transform)

# Apply the solution to the robot
if result.success:
    poser.apply_pose(result.joints)
```

The poser automatically handles unit conversions between radians (for computation) and native USD units (degrees for revolute joints), manages solution seeding for consistent results, and provides anchored positioning to maintain fixed reference points during pose application.

When {meth}`solve_ik <isaacsim.robot.poser.RobotPoser.solve_ik>` is called without an explicit `seed=` and no previous solution is cached, the poser tries a small ladder of starting configurations — the joint-limit midpoint, a few deterministic random restarts within the joint limits, and finally zeros — and returns the first attempt that converges. This avoids silent failures on redundant arms (e.g. Franka Panda 7-DOF) where the all-zero configuration is in the wrong convergence basin. For latency-sensitive code paths and for targets close to a known configuration, callers should still pass an explicit `seed=` to skip the ladder and run the solver exactly once.

### {class}`PoseResult <isaacsim.robot.poser.PoseResult>`

{class}`PoseResult <isaacsim.robot.poser.PoseResult>` encapsulates the outcome of IK solving or named pose queries. It contains joint values, success status, kinematic chain information, and target pose details. This data structure serves as the standard format for pose information exchange throughout the system.

### Joint State Management

The extension provides standalone functions for applying joint configurations without requiring a full {class}`RobotPoser <isaacsim.robot.poser.RobotPoser>` instance:

- {func}`apply_joint_state <isaacsim.robot.poser.apply_joint_state>` applies joint values via FK during non-simulation or DOF targets during simulation
- {func}`apply_joint_state_anchored <isaacsim.robot.poser.apply_joint_state_anchored>` applies joint values while keeping a specified anchor prim at its original world position

These functions automatically detect the simulation state and choose the appropriate application method.

## Functionality

### Named Pose Library

The extension implements a complete named pose management system that persists poses as USD prims within the robot asset. Named poses are stored under a standardized scope and can be exported/imported as JSON files for sharing between assets or projects.

```python
# Store a successful IK solution as a named pose
store_named_pose(stage, robot_prim, "pick_position", pose_result)

# Apply a named pose later
apply_pose_by_name(stage, robot_prim, "pick_position")

# Export all poses for backup or sharing
export_poses(stage, robot_prim, "/path/to/poses.json")
```

### IK Solving

The extension integrates with the robot schema's IK solver system to provide configurable inverse kinematics solving. It supports solution seeding, convergence tolerance adjustment, and joint locking through solver parameters.

### Robot Validation

{func}`validate_robot_schema <isaacsim.robot.poser.validate_robot_schema>` ensures robot prims carry the required IsaacRobotAPI schema before attempting pose operations, providing early validation for robotic workflows.

## Integration

The extension uses **omni.kit.menu.utils** to integrate pose management capabilities into the Kit interface, enabling users to access robot posing functionality through standard menu systems. It builds upon isaacsim.robot.schema for kinematic chain management and forward kinematics operations.
