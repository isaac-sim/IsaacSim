# Overview

```{image} ../../../../source/extensions/isaacsim.asset.exporter.urdf/data/preview.png
```

The isaacsim.asset.exporter.urdf extension exports USD robot assets to URDF (Unified Robot Description Format) files. It extracts joint hierarchies, link geometries, inertia properties, and physics parameters from USD stages and writes them in standard URDF XML format for use with ROS and other robotics frameworks.

## Functionality

- **USD to URDF conversion**: Exports articulated robot assets with proper joint types, limits, and dynamics
- **Inertia extraction**: Retrieves mass, center of mass, and inertia tensors from USD physics properties or PhysX-computed values
- **Mesh export**: Exports visual and collision meshes with configurable path prefixes (`file://`, `package://`, or relative paths) for ROS package compatibility
- **Collision visualization**: Supports visualization of exported collision geometry for validation
