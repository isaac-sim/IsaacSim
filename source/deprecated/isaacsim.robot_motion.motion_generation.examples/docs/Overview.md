# Overview

```{deprecated} 6.0.0
This extension is deprecated. Use `isaacsim.robot_motion.experimental.motion_generation` and `isaacsim.robot_motion.cumotion` instead.
```

The isaacsim.robot_motion.motion_generation.examples extension provides interactive UI-based examples for robot motion generation concepts within Isaac Sim. This extension demonstrates various motion planning algorithms and trajectory generation techniques through hands-on examples with Franka and UR10 robots.

Each example registers a menu entry under **Motion Generation Examples** (RMPflow, Kinematics, RRT, Trajectory Generation).

## Key Components

### Example modules

The extension consists of four specialized example modules, each focusing on different aspects of robot motion generation:

#### RMP Flow
The RMP Flow module demonstrates Riemannian Motion Policies (RMP) for reactive motion generation. It provides an interactive interface for loading Franka robot scenarios and running RMP-based motion planning algorithms that generate smooth, collision-aware trajectories in real-time.

#### RRT
The RRT module showcases Rapidly-exploring Random Tree path planning algorithms. Users can experiment with probabilistic sampling-based motion planning techniques that efficiently explore the robot's configuration space to find feasible paths from start to goal configurations.

#### Kinematics
The Kinematics module focuses on fundamental robot kinematics concepts. It provides demonstrations of forward and inverse kinematics calculations, joint space manipulations, and the relationship between joint configurations and end-effector poses.

#### Trajectory Generator
The Trajectory Generator module presents comprehensive trajectory planning capabilities for UR10 robots. It covers configuration space trajectories, task space trajectories, and advanced trajectory planning methods, demonstrating how to generate smooth, time-parameterized paths for robot motion.

### {class}`UIBuilder <isaacsim.robot_motion.motion_generation.examples.rmp_flow.UIBuilder>` Architecture

Each example module implements a {class}`UIBuilder <isaacsim.robot_motion.motion_generation.examples.rmp_flow.UIBuilder>` class that creates a standardized interface with two main control sections:

**World Controls** - Provides Load and Reset buttons for scene management, allowing users to initialize robot scenarios and restore default states.

**Scenario Controls** - Contains interactive buttons for starting and stopping motion generation demonstrations, with state-aware feedback to indicate execution status.

The {class}`UIBuilder <isaacsim.robot_motion.motion_generation.examples.rmp_flow.UIBuilder>` classes handle timeline integration, physics step callbacks, and stage event management to ensure proper synchronization between the UI and Isaac Sim's simulation systems.

## Functionality

### Interactive demonstrations

Each example provides hands-on exploration of motion generation concepts through visual demonstrations. Users can load pre-configured robot scenarios, adjust parameters, and observe real-time motion execution within the Isaac Sim environment.

### Event system integration

The extension integrates with Isaac Sim's timeline and physics systems to provide responsive examples. Physics step callbacks ensure continuous updates during simulation, while timeline and stage event handling maintains proper state management throughout the example lifecycle.

### Automated scene setup

The examples automatically handle scene initialization including lighting setup, camera positioning, and robot asset loading. This ensures optimal visualization conditions for each demonstration without requiring manual configuration.

## Dependencies

The extension uses `isaacsim.robot_motion.lula` for accessing the Lula planning library functionality that powers the motion generation algorithms. It integrates with `isaacsim.robot_motion.motion_generation` to leverage the core motion planning infrastructure. The `isaacsim.gui.components` dependency provides the UI element wrappers used throughout the example interfaces.
