# Overview

The isaacsim.physics.newton extension integrates Newton physics simulation into Isaac Sim, providing an alternative physics engine to PhysX with support for advanced solvers including XPBD and MuJoCo backends. This extension enables high-performance physics simulation with CUDA graph capture optimization and tensor-based interfaces for machine learning workflows.

## Key Components

### Physics Engine Management

The extension provides APIs to manage multiple physics engines within Isaac Sim. The {func}`get_available_physics_engines <isaacsim.physics.newton.get_available_physics_engines>` function lists all registered physics engines with their active status, while {func}`get_active_physics_engine <isaacsim.physics.newton.get_active_physics_engine>` returns the currently active engine name. This allows applications to query and switch between different physics backends dynamically.

### Newton Physics Interface

The core physics control is accessed through {func}`acquire_physics_interface <isaacsim.physics.newton.acquire_physics_interface>`, which returns a NewtonPhysicsInterface for controlling simulation parameters and execution. The interface manages simulation stepping, state synchronization, and provides access to the underlying Newton solver systems.

### Stage Management

The {func}`acquire_stage <isaacsim.physics.newton.acquire_stage>` function provides access to the NewtonStage object, which handles the simulation stage and USD integration. This stage object manages the physics scene representation and coordinates with the broader Isaac Sim USD workflow.

### Configuration System

**{class}`NewtonConfig <isaacsim.physics.newton.NewtonConfig>`** serves as the primary configuration class, following IsaacLab's pattern of separating simulation-level parameters from solver-specific settings. Key configuration areas include:

- **Performance Settings**: CUDA graph capture, physics frequency, and substep control
- **USD Integration**: Fabric synchronization, PhysX tracker coordination, and joint processing options  
- **Contact Parameters**: Stiffness, damping, friction coefficients, and restitution settings
- **Joint Settings**: Limit parameters, armature values, and PD controller scaling

### Solver Configurations

The extension supports multiple solver backends through specialized configuration classes:

**{class}`XPBDSolverConfig <isaacsim.physics.newton.XPBDSolverConfig>`** configures the Extended Position-Based Dynamics solver, an implicit integrator for rigid and soft body simulation. Parameters include iteration counts, relaxation values for different constraint types, and compliance settings for joint behaviors.

**{class}`MuJoCoSolverConfig <isaacsim.physics.newton.MuJoCoSolverConfig>`** provides extensive configuration for the MuJoCo Warp solver backend, including constraint limits, solver type selection, integrator options, and actuator gear mappings. This solver can operate in pure MuJoCo CPU mode or utilize the mujoco_warp GPU acceleration.

## Integration

The extension integrates with Isaac Sim's unified physics interface through **omni.physics**, allowing applications to switch between Newton and other physics engines seamlessly. The isaacsim.core.simulation_manager dependency provides coordination with the broader simulation workflow, while usdrt.scenegraph enables direct Fabric integration for high-performance scene graph access.

Performance optimization is achieved through CUDA graph capture, which can be controlled via the `capture_graph_physics_step` setting. The extension can automatically become the active physics engine on startup through the `auto_switch_on_startup` setting.
