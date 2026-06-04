# Overview

The Isaac Sim Physics API Editor provides a UI tool for applying and removing physics APIs on USD prims. This extension focuses on collision shape configuration and physics component management, offering support for triangle mesh, convex hull, and convex decomposition collision types.

```{image} ../../../../source/extensions/isaacsim.util.physics/data/preview.png
---
align: center
---
```


## UI Components

### Physics API Controls

The extension creates menu items that allow users to apply or remove physics APIs on selected prims through simple UI interactions. The main operations include applying collision APIs with different approximation shapes and removing physics-related components entirely.

### Progress Tracking

During batch operations on multiple selected prims, the extension displays progress information to keep users informed about the processing status. This is particularly useful when working with large selections or complex prim hierarchies.

## Functionality

### Collision API Management

**Collision Application**: Users can apply collision APIs to selected prims with three approximation options:
- Triangle Mesh (none): Uses the exact mesh geometry for collision detection
- Convex Hull: Creates a convex wrapper around the mesh
- Convex Decomposition: Breaks complex meshes into simpler convex parts

**Collision Removal**: The extension can selectively remove only collision-related APIs while preserving other physics components.

### Physics API Cleanup

The extension provides comprehensive physics API removal that strips away all physics-related components including rigid body APIs, articulation APIs, character controllers, contact reports, triggers, materials, mass properties, and various joint types.

### Prim Validation and Traversal

The extension includes intelligent prim filtering that validates geometric primitives (cylinders, capsules, cones, spheres, cubes) and meshes with valid point data. It can traverse prim hierarchies while respecting visibility settings and rigid body constraints.

## Integration

The extension integrates with isaacsim.gui.components for UI elements and **omni.physx** for physics simulation capabilities. It works directly with USD stage selections and applies changes through standard USD APIs and PhysX utilities.

For instanceable prims, the extension applies CollisionAPI and MeshCollisionAPI directly, while for regular prims it uses PhysX utilities to configure collision shapes with the appropriate approximation methods.
