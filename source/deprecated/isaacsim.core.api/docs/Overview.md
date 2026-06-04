# Overview

```{deprecated} 6.0.0
This extension is deprecated in favor of the Core Experimental extensions: `isaacsim.core.experimental.*`.
```

The `isaacsim.core.api` extension provides APIs for controlling simulation state and physics scenes within Isaac Sim. This extension serves as the primary interface for managing simulation execution, physics interactions, and USD object manipulation in robotics and AI simulation workflows.

## Key Components

### Core Classes

{class}`World <isaacsim.core.api.World>` serves as the primary orchestration layer for scene management, providing control over simulation stepping, reset operations, and object lifecycle management. It manages the integration between physics simulation and USD stage operations.

{class}`SimulationContext <isaacsim.core.api.SimulationContext>` manages simulation state, physics stepping, and rendering parameters. It provides the foundational control for simulation timing, physics solver settings, and stage initialization.

{class}`PhysicsContext <isaacsim.core.api.PhysicsContext>` controls physics scene configuration including gravity, collision settings, and solver parameters. It provides direct access to physics scene properties and performance tuning options.

### Controllers

{class}`ArticulationController <isaacsim.core.api.ArticulationController>` provides high-level control for robotic articulated systems with position, velocity, and effort control modes. It supports joint-level control and kinematic state queries.

{class}`BaseController <isaacsim.core.api.BaseController>` defines the base interface for all control systems with standardized initialization, stepping, and reset behaviors.

{class}`BaseGripperController <isaacsim.core.api.BaseGripperController>` extends controller functionality for gripper-specific operations including open/close commands and grasp detection.

### Objects

The extension provides geometric primitive creators for common shapes including spheres, boxes, cylinders, capsules, and cones. Each primitive supports Dynamic, Fixed, and Visual variants:
- **Dynamic variants** include rigid body physics properties for simulation
- **Fixed variants** create static collision geometry  
- **Visual variants** provide rendering-only geometry

{class}`GroundPlane <isaacsim.core.api.GroundPlane>` creates infinite ground surfaces with configurable physics materials and visual properties.

### Materials

{class}`PhysicsMaterial <isaacsim.core.api.PhysicsMaterial>` defines collision and friction properties for rigid body interactions.

{class}`VisualMaterial <isaacsim.core.api.VisualMaterial>` serves as the base for rendering materials with support for textures and shader parameters.

{class}`DeformableMaterial <isaacsim.core.api.DeformableMaterial>` and {class}`DeformableMaterialView <isaacsim.core.api.DeformableMaterialView>` provide material properties for soft body and cloth simulations.

### Tasks

{class}`BaseTask <isaacsim.core.api.BaseTask>` defines the framework for simulation tasks with observation, action, and reward computation interfaces.

{class}`FollowTarget <isaacsim.core.api.FollowTarget>`, {class}`PickPlace <isaacsim.core.api.PickPlace>`, and {class}`Stacking <isaacsim.core.api.Stacking>` implement specific robotic task scenarios with configurable objectives and success metrics.

### Robots

{class}`Robot <isaacsim.core.api.Robot>` provides a high-level interface for single robot management including initialization, control, and state queries.

{class}`RobotView <isaacsim.core.api.RobotView>` extends robot functionality to handle multiple robot instances efficiently with batch operations.

## Functionality

The extension integrates physics simulation capabilities through omni.physics and omni.physx.tensors, enabling precise control over physical interactions and dynamics. It provides wrappers for USD objects that simplify working with Universal Scene Description assets in simulation contexts. The extension also includes utilities for managing both physics and visual materials, allowing users to define realistic material properties that affect simulation behavior and rendering.

Additionally, the extension incorporates computational tools through omni.pip.compute for advanced mathematical operations and omni.warp.core for high-performance parallel computing tasks commonly required in robotics simulations.