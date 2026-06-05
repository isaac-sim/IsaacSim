# Overview

The isaacsim.core.experimental.materials extension provides high-level wrappers for creating and managing USD materials in Isaac Sim. This extension offers a unified API to work with both physics and visual materials, allowing developers to programmatically create, configure, and manipulate material properties for simulation and rendering purposes.

<div align="center">

```{mermaid}
graph TD
    %% Inheritance relationships
    PhysicsMaterial --> RigidBodyMaterial
    PhysicsMaterial --> SurfaceDeformableMaterial
    PhysicsMaterial --> VolumeDeformableMaterial
    VisualMaterial --> OmniGlassMaterial
    VisualMaterial --> OmniPbrMaterial
    VisualMaterial --> PreviewSurfaceMaterial
```

</div>

## Key Components

### Physics Materials

**Physics materials control physical interaction properties for simulation.**

The {class}`PhysicsMaterial <isaacsim.core.experimental.materials.PhysicsMaterial>` base class serves as the foundation for all physics-based materials. It provides common functionality for managing friction, density, and other physical properties that affect how objects interact during simulation.

{class}`RigidBodyMaterial <isaacsim.core.experimental.materials.RigidBodyMaterial>` handles materials for rigid body physics, supporting properties like static friction, dynamic friction, restitution coefficients, and densities. It also provides compliant contact settings for spring-damper contact effects and combine modes that control how material properties interact during collisions.

{class}`SurfaceDeformableMaterial <isaacsim.core.experimental.materials.SurfaceDeformableMaterial>` manages materials for deformable surfaces, including mechanical properties like Young's modulus and Poisson's ratio, surface-specific parameters like thickness and stiffness values for stretch, shear, and bend behaviors.

{class}`VolumeDeformableMaterial <isaacsim.core.experimental.materials.VolumeDeformableMaterial>` covers materials for volumetric deformable bodies, providing similar mechanical properties to surface materials but applied to solid volumes rather than surfaces.

### Visual Materials

**Visual materials control appearance and rendering properties.**

The {class}`VisualMaterial <isaacsim.core.experimental.materials.VisualMaterial>` base class provides the foundation for all appearance-related materials. It offers methods for setting and getting shader input values, managing material parameters that affect how surfaces appear under lighting conditions.

{class}`PreviewSurfaceMaterial <isaacsim.core.experimental.materials.PreviewSurfaceMaterial>` wraps USD Preview Surface materials, supporting standard PBR properties including diffuse color, metallic and roughness values, emission, opacity, and normal mapping parameters.

{class}`OmniPbrMaterial <isaacsim.core.experimental.materials.OmniPbrMaterial>` provides comprehensive access to Omniverse's physically-based rendering system with extensive control over albedo, reflectivity, ambient occlusion, emissive properties, opacity, normal mapping, UV coordinates, and geometry effects like edge rounding.

{class}`OmniGlassMaterial <isaacsim.core.experimental.materials.OmniGlassMaterial>` specializes in realistic glass rendering with properties for color tinting, volume absorption, surface roughness, index of refraction, reflection characteristics, and thin-walled glass simulation.

## Functionality

### Material Creation and Management

The extension supports both creating new materials and wrapping existing USD material prims. When prim paths don't exist, new material prims are automatically created. When paths exist, wrappers are placed over existing materials.

All material classes provide type validation through `are_of_type()` methods to verify whether prims at given paths are valid for specific material types. The `fetch_instances()` methods automatically detect material types and return appropriate wrapper instances.

### Property Configuration

Physics materials offer specialized setters and getters for their respective properties. {class}`RigidBodyMaterial <isaacsim.core.experimental.materials.RigidBodyMaterial>` provides friction coefficient management, restitution control, density settings, and compliant contact configuration. Deformable materials add mechanical properties like Young's modulus, Poisson's ratio, and material-specific parameters.

Visual materials use a unified `set_input_values()` and `get_input_values()` interface for managing shader parameters. This approach provides flexibility to configure any supported shader input while maintaining type safety and proper data broadcasting.

### Batch Operations

All materials support batch operations through index-based parameter modification. Properties can be set for all wrapped prims simultaneously or selectively applied to specific indices, enabling efficient management of multiple materials with similar or varied configurations.
