# Overview

```{deprecated} 6.0.0
This extension is deprecated in favor of `isaacsim.robot_motion.experimental.motion_generation` and `isaacsim.robot_motion.cumotion`.
```

The isaacsim.robot_motion.lula_test_widget extension provides a testing interface for robot motion planning using Lula kinematics solvers and RmpFlow motion generation algorithms. This extension creates a dockable UI window that enables users to select articulated robots from the scene, load robot configuration files, and test various motion planning scenarios including inverse kinematics, trajectory generation, and RmpFlow-based motion control.

```{image} ../../../../source/deprecated/isaacsim.robot_motion.lula_test_widget/data/preview.png
---
align: center
---
```


## Key Components

### {class}`LulaTestScenarios <isaacsim.robot_motion.lula_test_widget.LulaTestScenarios>`

**{class}`LulaTestScenarios <isaacsim.robot_motion.lula_test_widget.LulaTestScenarios>` is the core testing framework that manages all motion planning scenarios.** This class provides comprehensive testing capabilities for robotic motion planning scenarios, including inverse kinematics target following, obstacle avoidance with RmpFlow, custom trajectory execution, and sinusoidal target tracking.

The class manages visual elements like targets, obstacles, coordinate frames, and trajectory waypoints, providing both interactive control and automated scenario execution with real-time visualization of end-effector frames and collision spheres for debugging.

#### Test Scenarios

The framework supports several distinct testing scenarios:

- **Inverse Kinematics Target Following**: Sets up scenarios where robots follow targets using the `on_ik_follow_target()` method with configurable end-effector frames
- **Custom Trajectory Execution**: Creates trajectory scenarios with waypoints forming rectangular paths via `on_custom_trajectory()`, with dynamic waypoint management through `add_waypoint()` and `delete_waypoint()`
- **RmpFlow Target Following with Obstacles**: Implements obstacle avoidance scenarios using `on_rmpflow_follow_target_obstacles()` with wall obstacles and target cubes
- **Sinusoidal Target Tracking**: Provides sinusoidal motion patterns through `on_rmpflow_follow_sinusoidal_target()` with configurable frequency and radius parameters

#### Visualization and Debugging

The framework includes comprehensive visualization tools:

```python
# Visualize end-effector frames
scenarios.visualize_ee_frame(articulation, ee_frame)

# Toggle RmpFlow debug mode for collision sphere visualization
scenarios.toggle_rmpflow_debug_mode()
```

#### Scenario Management

{class}`LulaTestScenarios <isaacsim.robot_motion.lula_test_widget.LulaTestScenarios>` provides methods for managing scenario lifecycle:

- `full_reset()`: Performs complete reset of all scenario data and Lula components
- `scenario_reset()`: Resets current scenario by clearing targets, obstacles, and trajectories
- `update_scenario()`: Updates scenarios based on type and parameters, particularly for sinusoidal patterns
- `get_next_action()`: Computes articulation actions for the current scenario with end-effector visualization updates

### File Filtering Utilities

The extension provides specialized file filtering functions for robot configuration:

- {func}`is_yaml_file <isaacsim.robot_motion.lula_test_widget.is_yaml_file>` and {func}`is_urdf_file <isaacsim.robot_motion.lula_test_widget.is_urdf_file>`: Check file extensions for YAML and URDF formats
- {func}`on_filter_yaml_item <isaacsim.robot_motion.lula_test_widget.on_filter_yaml_item>` and {func}`on_filter_urdf_item <isaacsim.robot_motion.lula_test_widget.on_filter_urdf_item>`: Filter functions for file browser items to display only relevant robot configuration files

### Extension Interface

The Extension class manages the UI window and robot selection interface, providing:

- Interactive robot articulation selection from the current stage via `get_all_articulations()`
- Robot configuration loading with support for YAML robot descriptions and URDF files
- Real-time articulation property updates through `get_articulation_values()`
- Comprehensive UI panels for kinematics testing, trajectory generation, and RmpFlow configuration

## Functionality

The extension integrates multiple motion planning approaches within a unified testing interface. Users can select articulated robots from the scene, load robot configuration files, and execute various motion planning algorithms with real-time visualization and debugging capabilities.

Key functionality includes inverse kinematics solving with target following, RmpFlow-based motion planning with obstacle avoidance, custom trajectory generation and execution, and sinusoidal target tracking scenarios. The extension supports both interactive control and automated scenario execution with comprehensive visual debugging tools.

## Integration

The extension integrates with the Isaac Sim robotics ecosystem through dependencies on isaacsim.robot_motion.lula for kinematics solving and isaacsim.robot_motion.motion_generation for RmpFlow motion policies. It utilizes isaacsim.gui.components for UI elements and connects with the physics simulation through **omni.physics** for robot articulation control.
