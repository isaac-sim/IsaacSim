# Overview

```{deprecated} 6.0.0
This extension has been deprecated and replaced by the Scene Optimizer.
```

The isaacsim.util.merge_mesh extension provided tools for combining multiple meshes into a single USD prim. It supported resetting mesh origins to the world origin and consolidating materials with matching characteristics into a unified material list.

## Key Components

### {class}`MeshMerger <isaacsim.util.merge_mesh.MeshMerger>`

The {class}`MeshMerger <isaacsim.util.merge_mesh.MeshMerger>` class provided utility functions for merging multiple USD meshes into a single mesh. It handled the merging of mesh geometry, materials, geometry subsets, and texture coordinates from multiple source meshes into a single output mesh with comprehensive control over merge behavior.

**Key capabilities included:**
- Combining mesh geometry and material data from multiple sources
- Handling geometry subsets and texture coordinate merging
- Supporting configurable material consolidation options
- Managing mesh transform and origin settings

### {class}`MergeMeshesCommand <isaacsim.util.merge_mesh.MergeMeshesCommand>`

The {class}`MergeMeshesCommand <isaacsim.util.merge_mesh.MergeMeshesCommand>` class provided a command interface for merging selected meshes and all children of given prims into a single mesh. It offered options to control merge behavior including transform handling, source deactivation, and material consolidation.

**Configuration options included:**
- Clear Parent Transform: Set mesh origin at world origin or preserve first element origin
- Deactivate source assets: Set source prims to inactive after merge operation
- Combine Materials: Redirect materials to a single folder and share materials with matching names

## Functionality

The extension was accessible from the **Tools > Robotics > Asset Editors** menu.
