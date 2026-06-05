# Overview

The isaacsim.core.experimental.objects extension provides high-level classes for creating and wrapping USD primitive objects and lights within Isaac Sim scenes. This extension offers a comprehensive set of geometric shapes, lighting elements, and specialized objects like ground planes with consistent APIs for creation, manipulation, and property management.

<div align="center">

```{mermaid}
graph TD
    %% Inheritance relationships
    Shape --> Capsule
    Shape --> Cone
    Shape --> Cube
    Shape --> Cylinder
    Shape --> Plane
    Shape --> Sphere
    
    Light --> CylinderLight
    Light --> DiskLight
    Light --> DistantLight
    Light --> DomeLight
    Light --> RectLight
    Light --> SphereLight
```

</div>

## Key Components

### {class}`Shape <isaacsim.core.experimental.objects.Shape>` Classes

The extension provides wrapper classes for standard USD geometric primitives, all inheriting from the base {class}`Shape <isaacsim.core.experimental.objects.Shape>` class:

**Basic Shapes**
- {class}`Cube <isaacsim.core.experimental.objects.Cube>` - Primitive rectilinear cubes centered at origin
- {class}`Sphere <isaacsim.core.experimental.objects.Sphere>` - Primitive spheres with configurable radius
- {class}`Cylinder <isaacsim.core.experimental.objects.Cylinder>` - Primitive cylinders with closed ends along specified axis
- {class}`Capsule <isaacsim.core.experimental.objects.Capsule>` - Primitive cylinders capped by half-spheres
- {class}`Cone <isaacsim.core.experimental.objects.Cone>` - Primitive cones with apex pointing toward positive axis
- {class}`Plane <isaacsim.core.experimental.objects.Plane>` - Primitive planes (limited rendering support in Hydra)

**Complex Geometry**
- {class}`Mesh <isaacsim.core.experimental.objects.Mesh>` - USD {class}`Mesh <isaacsim.core.experimental.objects.Mesh>` prims supporting custom geometry with points, faces, normals, and subdivision specifications

All shape classes support geometric properties like radii, heights, sizes, and axes configuration, plus physics properties including collision detection, contact offsets, and material application.

### {class}`Light <isaacsim.core.experimental.objects.Light>` Classes  

The extension provides comprehensive lighting support through classes inheriting from the base {class}`Light <isaacsim.core.experimental.objects.Light>` class:

- {class}`SphereLight <isaacsim.core.experimental.objects.SphereLight>` - Omnidirectional light emitted from a sphere
- {class}`RectLight <isaacsim.core.experimental.objects.RectLight>` - Directional light emitted from a rectangle surface  
- {class}`CylinderLight <isaacsim.core.experimental.objects.CylinderLight>` - {class}`Light <isaacsim.core.experimental.objects.Light>` emitted outward from a cylinder (no end caps)
- {class}`DiskLight <isaacsim.core.experimental.objects.DiskLight>` - {class}`Light <isaacsim.core.experimental.objects.Light>` emitted from one side of a circular disk
- {class}`DistantLight <isaacsim.core.experimental.objects.DistantLight>` - Parallel light from distant source along -Z axis
- {class}`DomeLight <isaacsim.core.experimental.objects.DomeLight>` - Environmental lighting from distant external environment

All light classes support intensity, exposure, color temperature, diffuse/specular multipliers, and normalization controls.

### Specialized Objects

**{class}`GroundPlane <isaacsim.core.experimental.objects.GroundPlane>`**
The {class}`GroundPlane <isaacsim.core.experimental.objects.GroundPlane>` class creates composite ground plane objects with both collision ({class}`Plane <isaacsim.core.experimental.objects.Plane>`) and rendering ({class}`Mesh <isaacsim.core.experimental.objects.Mesh>`) components. This addresses the limitation that USD {class}`Plane <isaacsim.core.experimental.objects.Plane>` prims are not supported by Hydra rendering while providing proper physics collision.

## Functionality

### Object Creation and Wrapping

Classes follow a unified pattern for both creating new USD prims and wrapping existing ones:
- If prim paths exist, wrappers are placed over existing prims
- If paths don't exist, new prims are created and wrapped
- Supports regular expressions for path matching
- Broadcasting of parameters follows NumPy rules for efficient bulk operations

### Physics Integration

{class}`Shape <isaacsim.core.experimental.objects.Shape>` classes provide comprehensive physics capabilities:
- Collision detection enabling/disabling
- Contact and rest offset configuration for collision behavior
- Torsional patch radii for friction calculations
- Physics material application with support for descendants

### Property Management

All classes support:
- Position, translation, orientation, and scale transformations
- Geometric property queries and modifications
- Extent calculations and updates
- Type validation through static `are_of_type` methods
- Instance retrieval through static `fetch_instances` methods

### Advanced {class}`Mesh <isaacsim.core.experimental.objects.Mesh>` Features

The {class}`Mesh <isaacsim.core.experimental.objects.Mesh>` class supports sophisticated geometry operations:
- Custom point and normal specifications
- Face definitions with vertex indices and counts
- Crease and corner specifications for subdivision surfaces
- Multiple subdivision schemes (Catmull-Clark, loop, bilinear)
- Hole indices for creating invisible faces
