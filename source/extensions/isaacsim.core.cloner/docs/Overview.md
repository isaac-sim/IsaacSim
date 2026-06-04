# Overview

The isaacsim.core.cloner extension provides APIs for efficiently duplicating USD prims and environments within Isaac Sim. This extension enables batch creation of identical objects at specified positions, making it particularly useful for creating multiple environments for parallel simulation workflows.

## Key Components

### {class}`Cloner <isaacsim.core.cloner.Cloner>`

The {class}`Cloner <isaacsim.core.cloner.Cloner>` class provides the core functionality for duplicating USD prims with precise control over positioning and physics replication.

**Basic cloning workflow:**
```python
from isaacsim.core.cloner import Cloner

cloner = Cloner()
cloner.define_base_env("/World/envs")
prim_paths = cloner.generate_paths("/World/envs/env", 4)
cloner.clone(
    source_prim_path="/World/envs/env_0",
    prim_paths=prim_paths,
)
```

**Performance optimization methods** help manage computational overhead during large-scale cloning operations:
- `disable_change_listener()` and `enable_change_listener()` temporarily disable USD notice handlers
- `replicate_physics()` uses **omni.physics** replication interface to avoid physics parsing bottlenecks

**Collision filtering** prevents clones from colliding with each other while maintaining interactions with specified global objects:
```python
cloner.filter_collisions(
    physicsscene_path="/World/PhysicsScene",
    collision_root_path="/World/collisions", 
    prim_paths=prim_paths,
    global_paths=["/World/ground_plane"],
)
```

### {class}`GridCloner <isaacsim.core.cloner.GridCloner>`

The {class}`GridCloner <isaacsim.core.cloner.GridCloner>` class extends {class}`Cloner <isaacsim.core.cloner.Cloner>` to automatically arrange clones in a regular grid pattern, eliminating the need for manual position calculations.

```python
from isaacsim.core.cloner import GridCloner

cloner = GridCloner(spacing=2.0, num_per_row=3)
positions = cloner.clone(
    source_prim_path="/World/envs/env_0",
    prim_paths=prim_paths,
)
```

The grid positioning is computed automatically based on the specified spacing and row configuration, with support for additional position and orientation offsets.

## Functionality

### Physics Replication

The extension integrates with **omni.physics** to replicate physics properties directly rather than parsing them for each clone. This approach significantly improves performance when creating physics-enabled environments by using the PhysX replicator interface.

### Environment ID Support

When `enable_env_ids=True`, clones can be co-located in physics space with automatic collision filtering between environments, enabling more efficient parallel simulations.

### Fabric Integration

The extension supports cloning operations in Fabric (`clone_in_fabric=True`) for improved performance in scenarios requiring high-throughput environment creation.
