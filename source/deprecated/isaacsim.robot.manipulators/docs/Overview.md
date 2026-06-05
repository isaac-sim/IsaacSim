# Overview

```{deprecated} 6.0.0
This extension is deprecated. Refer to `isaacsim.robot.experimental.manipulators.examples` for recommended alternatives.
```

The `isaacsim.robot.manipulators` extension provides high-level Python classes for controlling robotic manipulators in Isaac Sim. This extension covers the complete manipulator stack: robot representation, gripper abstractions, task-level controllers, and an OmniGraph node for gripper control.

## Key Components

### {class}`SingleManipulator <isaacsim.robot.manipulators.SingleManipulator>`

The {class}`SingleManipulator <isaacsim.robot.manipulators.SingleManipulator>` class serves as the primary interface for manipulator control, built on top of the articulation system. It encapsulates a complete manipulator system consisting of the robotic arm, end effector, and optional gripper components.

Key features include:
- **End Effector Tracking**: Monitors the rigid body corresponding to the end effector through the `end_effector` property, enabling access to world pose and motion data
- **Gripper Integration**: Optional gripper attachment through the `gripper` property for pick-and-place operations
- **Physics Integration**: Utilizes PhysX tensor API through the `initialize()` method for efficient physics simulation
- **State Management**: Provides `post_reset()` functionality to restore default states after timeline resets

The class inherits from `SingleArticulation`, extending it with manipulator-specific functionality while maintaining compatibility with the broader articulation framework.

### Grippers

The extension provides an abstract gripper framework and two concrete implementations:

**{class}`Gripper <isaacsim.robot.manipulators.Gripper>`** is the abstract base class for all gripper types. It extends `SingleRigidPrim` and defines the interface for opening, closing, and querying gripper state. All gripper implementations must provide `open()`, `close()`, `get_action()`, and state management methods.

**{class}`ParallelGripper <isaacsim.robot.manipulators.ParallelGripper>`** controls two-finger parallel grippers through joint position control. It supports configurable open/closed joint positions, action deltas for incremental movement, and optional mimic joint mode where only a single drive joint is commanded.

**{class}`SurfaceGripper <isaacsim.robot.manipulators.SurfaceGripper>`** wraps the surface gripper physics interface for suction-cup style grippers. It manages gripper state through the `isaacsim.robot.surface_gripper` C++ backend, translating high-level open/close commands into the underlying surface gripper API calls.

### Controllers

**{class}`PickPlaceController <isaacsim.robot.manipulators.PickPlaceController>`** implements a state machine that manages pick-and-place operations through a sequence of predefined phases. These phases include moving to a picking position, approaching and grasping the object, lifting, moving to the target location, and placing the object. It coordinates end-effector control and gripper actions across all phases.

**{class}`StackingController <isaacsim.robot.manipulators.StackingController>`** extends the pick-and-place workflow to stack multiple objects in a specified order. It sequences {class}`PickPlaceController <isaacsim.robot.manipulators.PickPlaceController>` calls to move objects from their initial positions onto a growing stack.

## Integration

The extension builds upon `isaacsim.core.api` for base articulation and controller classes, and `isaacsim.robot.surface_gripper` for the surface gripper physics backend. The controllers and grippers are designed to work together with {class}`SingleManipulator <isaacsim.robot.manipulators.SingleManipulator>`, providing a complete manipulator control stack from low-level gripper actuation to high-level task execution.
