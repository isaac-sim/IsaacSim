# Overview

```{deprecated} 6.0.0
This extension is deprecated in favor of `isaacsim.core.experimental.prims`.
```

isaacsim.core.prims provides a comprehensive set of APIs for reading and writing state information to different types of prims in Isaac Sim. This extension serves as a high-level wrapper around USD prims, offering both individual prim management through Single classes and efficient batch operations through View classes for physics simulation environments.

<div align="center">

```{mermaid}
graph TD
    %% Inheritance relationships
    XFormPrim --> GeometryPrim
    XFormPrim --> RigidPrim
    XFormPrim --> Articulation
    XFormPrim --> ClothPrim
    XFormPrim --> DeformablePrim
    GeometryPrim --> SdfShapePrim
```


</div>

## Key Components

### Prim View Classes

The extension provides view classes that efficiently handle collections of similar prims using regex patterns for path matching:

**{class}`Articulation <isaacsim.core.prims.Articulation>`** wraps multiple articulated body prims (like robots) for batch joint control, kinematic state management, and physics property configuration. It supports position/velocity targets, effort control, and comprehensive dynamics queries.

**{class}`RigidPrim <isaacsim.core.prims.RigidPrim>`** manages collections of rigid body prims with batch operations for poses, velocities, forces, and mass properties. It includes contact force tracking and physics material application.

**{class}`GeometryPrim <isaacsim.core.prims.GeometryPrim>`** handles geometric collision shapes with advanced collision detection features, physics material binding, and contact force monitoring between specified filter prims.

**{class}`ClothPrim <isaacsim.core.prims.ClothPrim>`** controls cloth simulation parameters including spring stiffnesses, damping, pressure for inflatable dynamics, and particle-based collision settings.

**{class}`DeformablePrim <isaacsim.core.prims.DeformablePrim>`** manages soft body deformables with tetrahedral mesh properties, material assignments, and finite element simulation parameters.

**{class}`ParticleSystem <isaacsim.core.prims.ParticleSystem>`** coordinates particle-based simulations with solver settings, collision parameters, and system-wide properties like wind and neighborhood sizes.

**{class}`SdfShapePrim <isaacsim.core.prims.SdfShapePrim>`** provides signed distance field queries for complex geometry collision detection with configurable resolution and margin parameters.

**{class}`XFormPrim <isaacsim.core.prims.XFormPrim>`** serves as the base transformation container with pose, scale, visibility, and material application capabilities.

### Single Prim Classes

Individual prim management classes ({class}`SingleArticulation <isaacsim.core.prims.SingleArticulation>`, {class}`SingleRigidPrim <isaacsim.core.prims.SingleRigidPrim>`, {class}`SingleGeometryPrim <isaacsim.core.prims.SingleGeometryPrim>`, etc.) offer direct control over single entities with the same feature set as their view counterparts but optimized for individual operations.

## Functionality

### State Management
All prim classes support default state configuration and current state queries. Default states are automatically applied during reset operations, enabling consistent initial conditions across simulation runs.

### Physics Integration
The extension integrates deeply with PhysX through **omni.physx.tensors**, providing GPU-accelerated physics queries and state manipulation. Physics handles are automatically managed and validated.

### Batch Operations
View classes use vectorized operations for efficient multi-prim manipulation, supporting numpy, torch, and warp tensor backends for performance-critical applications.

### Contact Tracking
Advanced contact force monitoring allows detailed collision analysis between specific prim sets, with support for normal forces, friction data, and contact geometry information.
