# Overview

The isaacsim.core.experimental.prims extension provides high-level wrapper classes for manipulating USD prims and their attributes during simulation. It offers object-oriented interfaces that simplify common operations on different types of prims, from basic transformations to complex physics properties, while supporting batch operations across multiple prims using regular expressions and efficient NumPy-style broadcasting.

<div align="center">

```{mermaid}
graph TD
    Prim --> XformPrim
    XformPrim --> GeomPrim
    XformPrim --> RigidPrim
    XformPrim --> DeformablePrim
    XformPrim --> Articulation
```

</div>

## Key Components

### Base {class}`Prim <isaacsim.core.experimental.prims.Prim>` Wrapper

The {class}`Prim <isaacsim.core.experimental.prims.Prim>` class serves as the foundational wrapper for managing USD prims in the stage. It handles prim path resolution, including regular expression matching for batch operations, and provides basic validation and access to underlying USD prim objects.

### Transformation Control

{class}`XformPrim <isaacsim.core.experimental.prims.XformPrim>` extends the base functionality to handle transformation operations including world and local poses, scaling, visibility, and material applications. It provides methods for setting default states and resetting prims to those states, along with comprehensive transformation operation management.

### Articulated Bodies

{class}`Articulation <isaacsim.core.experimental.prims.Articulation>` wraps prims with Root {class}`Articulation <isaacsim.core.experimental.prims.Articulation>` API to provide comprehensive control over robotic systems. It manages degrees of freedom (DOFs), joints, and links with support for position, velocity, and effort control modes. The class handles drive properties, friction parameters, solver settings, and provides access to advanced features like Jacobian matrices, mass matrices, and gravity compensation forces.

### Rigid Body Dynamics

{class}`RigidPrim <isaacsim.core.experimental.prims.RigidPrim>` handles prims with Rigid Body API applied, offering control over mass properties, velocities, and force applications. It supports contact tracking with customizable filtering and provides access to detailed contact information including forces, points, and friction data.

### Geometric Collision

{class}`GeomPrim <isaacsim.core.experimental.prims.GeomPrim>` manages geometric prims for collision detection, providing control over collision approximation methods, contact/rest offsets, and torsional patch properties. It handles collision API management and physics material applications for accurate collision behavior.

### Deformable Bodies

{class}`DeformablePrim <isaacsim.core.experimental.prims.DeformablePrim>` supports both surface and volume deformable bodies, managing nodal positions and velocities, kinematic targeting, and physics material properties. It provides access to stress and gradient information for advanced deformable simulation analysis.

## Functionality

### Batch Operations

All wrapper classes support operating on multiple prims simultaneously through regular expression path matching and NumPy-style broadcasting. This enables efficient manipulation of large numbers of similar objects with minimal code.

### Multi-Backend Support

The extension operates across multiple simulation backends including USD, USD-RT, Fabric, and tensor-based physics systems, automatically selecting the appropriate backend based on simulation state and requirements.

### Default State Management

Prims can have default states defined and restored, supporting complex reset scenarios for simulation environments and robotic systems.

### Physics Integration

Deep integration with physics simulation provides access to advanced properties like mass matrices, Jacobians, contact forces, and solver parameters, enabling sophisticated control and analysis of physical systems.
