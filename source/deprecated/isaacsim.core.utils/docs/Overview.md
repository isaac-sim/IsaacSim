# Overview

```{deprecated} 6.0.0
This extension is deprecated in favor of `isaacsim.core.experimental.utils`.
```

The isaacsim.core.utils extension provides a comprehensive set of utility functions for working with USD stages, physics simulation, mathematics, and rendering operations within Isaac Sim. This extension serves as a foundational toolkit that simplifies common development tasks through high-level APIs for stage manipulation, geometry processing, coordinate transformations, and data interchange between different computational frameworks.

## Functionality

### USD Stage Management
The extension provides comprehensive stage operations including creating, opening, saving, and manipulating USD stages. Users can traverse stage hierarchies, manage prim lifecycles, and handle references and payloads. Stage utilities support both synchronous and asynchronous operations for file I/O and stage updates.

### Prim Operations and Manipulation
Extensive prim utilities enable creating, querying, and modifying USD prims with support for attributes, properties, and metadata. The extension handles prim hierarchies, parent-child relationships, and provides advanced search capabilities using regex patterns and predicate functions.

### Physics Integration
Physics utilities facilitate rigid body management, simulation control, and raycasting operations. The extension integrates with Omni Physics to provide seamless physics-enabled prim manipulation and simulation stepping with configurable timing and callback support.

### Mathematical Operations
Comprehensive mathematical utilities support coordinate transformations, quaternion operations, rotation matrix conversions, and pose calculations. The extension provides both single-value and vectorized operations for efficient batch processing of geometric data.

### Multi-Framework Data Interchange
The extension includes comprehensive interoperability utilities for converting data between NumPy, PyTorch, JAX, TensorFlow, and Warp arrays. This enables seamless integration across different computational frameworks commonly used in robotics and simulation workflows.

### Rendering and Viewport Control
Rendering utilities provide viewport management, camera control, and render product configuration. Users can create custom viewports, manipulate camera transforms, configure AOVs (Arbitrary Output Variables), and manage intrinsic camera parameters.

### Mesh and Geometry Processing
Geometry utilities support mesh analysis, bounding box calculations, and vertex processing. The extension provides tools for computing axis-aligned and oriented bounding boxes, extracting mesh vertices in different coordinate systems, and performing geometric transformations.

## Key Components

### Commands System
The extension registers several commands for common operations:

{class}`IsaacSimSpawnPrim <isaacsim.core.utils.IsaacSimSpawnPrim>` creates new prims with USD references and applies transformations using physics-aware positioning.

{class}`IsaacSimTeleportPrim <isaacsim.core.utils.IsaacSimTeleportPrim>` relocates existing prims with proper physics handling for articulated objects.

{class}`IsaacSimScalePrim <isaacsim.core.utils.IsaacSimScalePrim>` applies scaling transformations to prims.

{class}`IsaacSimDestroyPrim <isaacsim.core.utils.IsaacSimDestroyPrim>` removes prims from the stage with optimized performance for batch operations.

### Data Structures
Several specialized data classes facilitate state management:

{class}`DataFrame <isaacsim.core.utils.DataFrame>` encapsulates simulation data at specific time steps with time indexing.

{class}`XFormPrimState <isaacsim.core.utils.XFormPrimState>` and {class}`XFormPrimViewState <isaacsim.core.utils.XFormPrimViewState>` manage transformation states for single and multiple prims.

{class}`DynamicState <isaacsim.core.utils.DynamicState>` and {class}`DynamicsViewState <isaacsim.core.utils.DynamicsViewState>` handle dynamic rigid body states including velocities.

{class}`ArticulationAction <isaacsim.core.utils.ArticulationAction>` and {class}`ArticulationActions <isaacsim.core.utils.ArticulationActions>` define joint control commands for robotic systems.

### Semantic Labeling System
Semantic utilities provide comprehensive labeling support using UsdSemantics.LabelsAPI for scene understanding and automated annotation. The system supports label validation, missing label detection, and automatic migration from deprecated semantic APIs.

### Physics Utilities
Tetrahedral mesh generation and physics simulation utilities support soft body dynamics and advanced material modeling. The extension includes tools for creating tetrahedral meshes from voxel grids and geometric primitives.

### Camera and Viewport Systems
Dynamic camera utilities implement physics-based camera movement using spring-damper systems for smooth, natural motion. The system supports automatic focus adjustment and configurable motion characteristics for cinematic camera control.

## Relationships

The extension integrates with **omni.physics** and **omni.physx** for physics simulation capabilities, **omni.warp.core** for high-performance array operations, and **omni.usd.schema.semantics** for semantic labeling functionality. These integrations enable seamless data flow between USD authoring, physics simulation, and computational frameworks commonly used in robotics and AI applications.
