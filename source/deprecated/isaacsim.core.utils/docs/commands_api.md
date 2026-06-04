# Commands
Public command API for module **isaacsim.core.utils**:

- [IsaacSimDestroyPrim](#isaacsimdestroyprim)
- [IsaacSimScalePrim](#isaacsimscaleprim)
- [IsaacSimSpawnPrim](#isaacsimspawnprim)
- [IsaacSimTeleportPrim](#isaacsimteleportprim)


## IsaacSimDestroyPrim
Command to delete a prim. This variant has less overhead than other commands as it doesn't store an undo operation.

Typical usage example:

.. code-block:: python

omni.kit.commands.execute(
"IsaacSimDestroyPrim",
prim_path="/World/Prim",
)

### Arguments
- prim_path: Path to the prim to delete.

### Usage

```python
import omni.kit.commands

# Delete a prim at the specified path
omni.kit.commands.execute("IsaacSimDestroyPrim", prim_path="/World/Prim")
```

## IsaacSimScalePrim
Command to set a scale of a prim

Typical usage example:

.. code-block:: python

omni.kit.commands.execute(
"IsaacSimScalePrim",
prim_path="/World/Prim",
scale=(1.5, 1.5, 1.5),
)

### Arguments
- prim_path: Path to the prim to scale.
- scale: Scale values for x, y, and z axes.

### Usage

```python
import omni.kit.commands

# Scale a prim at path "/World/Cube" to 1.5 times its original size on all axes
omni.kit.commands.execute(
    "IsaacSimScalePrim",
    prim_path="/World/Cube",
    scale=(1.5, 1.5, 1.5)
)
```

## IsaacSimSpawnPrim
Command to spawn a new prim in the stage and set its transform. This uses dynamic_control to properly handle physics objects and articulation.

Typical usage example:

.. code-block:: python

omni.kit.commands.execute(
"IsaacSimSpawnPrim",
usd_path="/path/to/file.usd",
prim_path="/World/Prim",
translation=(0, 0, 0),
rotation=(0, 0, 0, 1),
)

### Arguments
- usd_path: Path to the USD file to reference.
- prim_path: Path where the prim will be created in the stage.
- translation: Translation vector for the prim's position.
- rotation: Rotation quaternion for the prim's orientation.

### Usage

```python
import omni.kit.commands
import carb

# Spawn a new prim by referencing a USD file
omni.kit.commands.execute(
    "IsaacSimSpawnPrim",
    usd_path="/path/to/asset.usd",
    prim_path="/World/MySpawnedPrim",
    translation=(1.0, 2.0, 0.5),
    rotation=(0.0, 0.0, 0.707, 0.707)  # 90 degree rotation around Z-axis
)
```

## IsaacSimTeleportPrim
Command to set a transform of a prim. This uses dynamic_control to properly handle physics objects and articulation

Typical usage example:

.. code-block:: python

omni.kit.commands.execute(
"IsaacSimTeleportPrim",
prim_path="/World/Prim",
translation=(0, 0, 0),
rotation=(0, 0, 0, 1),
)

### Arguments
- prim_path: Path to the prim to teleport.
- translation: Translation vector as (x, y, z).
- rotation: Rotation quaternion as (x, y, z, w).

### Usage

```python
import omni.kit.commands

# Teleport a prim to a new position and rotation
omni.kit.commands.execute(
    "IsaacSimTeleportPrim",
    prim_path="/World/Cube",
    translation=(5.0, 0.0, 2.0),
    rotation=(0.0, 0.0, 0.707, 0.707)
)
```

