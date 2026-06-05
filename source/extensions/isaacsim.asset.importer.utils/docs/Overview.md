# Overview

This extension provides shared utility functions for asset importers. It includes:

- **Collision from Visuals**: Apply collision APIs using visual geometry.
- **Mesh Merge Helpers**: Merge mesh groups via Scene Optimizer.
- **Self-Collision Utilities**: Enable articulation self-collision flags.
- **URDF/MJCF Conversion Helpers**: Convert joint and actuator attributes between URDF, MJCF, and PhysX.
- **Asset Structure Profiles**: Run Asset Transformer profiles for packaging.

## Dependencies

This extension depends on:
- `isaacsim.asset.transformer`: Asset structure profile execution.
- `omni.scene.optimizer.core`: Mesh merge and scene optimization utilities.
