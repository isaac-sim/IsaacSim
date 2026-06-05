# Overview

```{deprecated} 6.0.0
This extension is deprecated in favor of `isaacsim.robot.experimental.wheeled_robots` and `isaacsim.robot.wheeled_robots.nodes`.
```

The `isaacsim.robot.wheeled_robots` extension provides Python and C++ classes for controlling wheeled mobile robots in Isaac Sim. It includes robot wrappers, multiple controller types covering differential, holonomic, and Ackermann drive systems, path planning utilities, and OmniGraph nodes for graph-based robot control.

## Key Components

### Robots

**{class}`WheeledRobot <isaacsim.robot.wheeled_robots.WheeledRobot>`** wraps an articulation prim to provide a high-level interface for wheeled robot control. It manages joint states and accepts drive commands through `apply_wheel_actions()` using `ArticulationAction` objects, supporting mixed command modes (effort, velocity, and position).

**{class}`HolonomicRobotUsdSetup <isaacsim.robot.wheeled_robots.HolonomicRobotUsdSetup>`** reads wheel geometry parameters from USD for holonomic (mecanum) robots. It extracts wheel radii, positions, orientations, and mecanum angles needed to construct a {class}`HolonomicController <isaacsim.robot.wheeled_robots.HolonomicController>`.

### Controllers

**{class}`DifferentialController <isaacsim.robot.wheeled_robots.DifferentialController>`** converts throttle and steering (or linear/angular velocity) commands into left and right wheel velocities for two-wheeled differential drive robots.

**{class}`HolonomicController <isaacsim.robot.wheeled_robots.HolonomicController>`** computes per-wheel velocities for omnidirectional (mecanum) robots from forward, lateral, and yaw commands, accounting for wheel geometry and mecanum roller angles.

**{class}`AckermannController <isaacsim.robot.wheeled_robots.AckermannController>`** implements bicycle-model Ackermann steering, returning both steering angles and per-wheel rotation velocities from steering angle, speed, and acceleration inputs.

**{class}`WheelBasePoseController <isaacsim.robot.wheeled_robots.WheelBasePoseController>`** provides a higher-level controller that drives a wheeled robot to a target 2D pose (position + heading) by combining proportional control with an underlying differential or holonomic controller.

**{class}`StanleyControl <isaacsim.robot.wheeled_robots.StanleyControl>`** implements the Stanley lateral path-tracking algorithm for path following with configurable gains.

**{class}`QuinticPathPlanner <isaacsim.robot.wheeled_robots.QuinticPathPlanner>`** generates smooth quintic polynomial trajectories between waypoints for wheeled robot navigation.

## Basic Usage

```python
from isaacsim.robot.wheeled_robots.robots import WheeledRobot
from isaacsim.robot.wheeled_robots.controllers import DifferentialController
from isaacsim.core.utils.types import ArticulationAction

# Wrap an articulation as a wheeled robot
jetbot = WheeledRobot(
    prim_path="/World/Jetbot",
    name="jetbot",
    wheel_dof_names=["left_wheel_joint", "right_wheel_joint"],
)

# Create a differential controller
controller = DifferentialController(
    name="diff_ctrl", wheel_radius=0.035, wheel_base=0.1
)

# In your simulation loop:
action = controller.forward(command=[0.5, 0.0])  # [linear, angular]
jetbot.apply_wheel_actions(action)
```

## Integration

The extension integrates with `isaacsim.core.api` for articulation and controller base classes. The extension also provides OmniGraph nodes for graph-based robot control, which can be combined with Isaac Sim's action graph system for complete autonomous navigation pipelines.
