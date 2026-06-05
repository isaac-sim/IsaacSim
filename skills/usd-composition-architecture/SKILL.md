---
name: usd-composition-architecture
description: >
  NVIDIA's USD asset composition pattern for Isaac Sim — how robot assets are split into
  a binary geometry crate plus a set of human-editable USDA layers (base, instances,
  materials, physics, physx, mujoco, robot, interface) connected via composition arcs
  (payload, reference, sublayer, variant). Covers the "binary for meshes, USDA for
  everything else" rule, headless RL optimization by skipping appearance, and the
  Asset Transformer pipeline that produces this layout.
  Use when: building new robot or environment assets for Isaac Sim, restructuring an
  existing asset with the Asset Transformer, optimizing RL training startup time and
  VRAM by skipping appearance payloads, debugging why physics edits in a USDA do not
  apply, debugging joint limits or mass properties, creating asset variants with
  different configs or materials.
  Triggers on: USD payload composition, layered USD, physics payload, appearance payload,
  USDA vs USD, USDA vs USDC, geometries.usdc, RL optimization payload, composition arcs,
  Asset Transformer, Isaac Sim Asset Structure, USD asset structure.
---

# USD Composition Architecture for Isaac Sim

## When to use

- Build new robot or environment assets.
- Restructure an existing asset with the Asset Transformer.
- Optimize RL training startup time and VRAM.
- Diagnose physics edits in a USDA that aren't taking effect.
- Debug joint limits, mass, or solver parameters.
- Create variants of an existing asset (configs, materials).

## Core Concept: One Binary Crate, Many USDA Layers

Isaac Sim's recommended asset structure splits a robot into one binary geometry crate plus a set of ASCII layers, composed by an `interface.usda`:

```
{robot}/
    interface.usda                         <- Final composed asset (entry point)
    payloads/
        base.usda                          <- Simulation-ready hierarchy + xforms
        geometries.usdc                    <- Mesh data ONLY (binary crate)
        instances.usda                     <- Mesh + material + collider assembly
        materials.usda                     <- Material defs (MDL bindings)
        Textures/                          <- Texture assets
        robot.usda                         <- Isaac robot schema + metadata
        Physics/
            physics.usda                   <- Neutral USD/Newton physics
            physx.usda                     <- PhysX-only tuning (sublayers physics.usda)
            mujoco.usda                    <- MuJoCo-only tuning (sublayers physics.usda)
```

USD `payload` arcs enable **lazy loading** — a payload is only loaded when explicitly requested. This is the key to headless RL optimization.

## File Format Decision Guide

The rule is simple: **binary crate (`.usdc`) for raw mesh data, USDA for everything else.** The Asset Transformer's `GeometriesRoutingRule` enforces this split automatically.

| Layer | Format | Why |
|---|---|---|
| `geometries.usdc` | `.usdc` (binary crate) | Mesh topology, points, indices — high-volume numeric data, never edited by hand |
| `base.usda` | `.usda` | Hierarchy and transforms — diffable, hand-editable |
| `instances.usda` | `.usda` | References meshes + applies materials + collision approximation choice |
| `materials.usda` | `.usda` | Material prims, MDL shader bindings — readable look-dev |
| `physics.usda` / `physx.usda` / `mujoco.usda` | `.usda` | Joint limits, masses, solver params — frequent tuning |
| `robot.usda` | `.usda` | Isaac robot schema metadata and relationships |
| `interface.usda` | `.usda` | Composition arcs (references, payloads, variants) — the entry point |
| Texture assets | original (PNG, JPG, EXR) + `.mdl` | Stored under `Textures/` |
| Archive/portable | `.usdz` | Single-file distribution (iOS AR) |

**Rationale:** Mesh arrays are large and never hand-edited, so binary crate wins on size and load time. Everything else is small, frequently inspected, and benefits from being diffable in version control and editable by both humans and agents.

## Producing This Structure: Asset Transformer

Use the Asset Transformer (Isaac Sim Structure profile) to convert an imported URDF/MJCF asset into the layout above. The relevant rules:

- `GeometriesRoutingRule` — extracts mesh prims to `geometries.usdc` (binary), creates instanceable references in `instances.usda`. Set `save_base_as_usda: true` to keep `base` ASCII.
- `MaterialsRoutingRule` — deduplicates materials into `materials.usda`, copies textures to `Textures/`.
- `SchemaRoutingRule` — splits physics, physx, mujoco, and robot schemas into their respective USDA layers.
- `InterfaceConnectionRule` — generates `interface.usda` with the composition arcs.

Refer to the Asset Transformer Rules Reference for the full pipeline.

## Physics USDA Schema

### physics.usda — Joint and Mass Definitions

```usda
#usda 1.0

def PhysicsRevoluteJoint "FL_hip_joint" {
    uniform token physics:axis = "X"
    float physics:lowerLimit = -46.0
    float physics:upperLimit = 46.0
    rel physics:body0 = </Robot/trunk>
    rel physics:body1 = </Robot/FL_hip>
}

def RigidBodyAPI "trunk" {
    float physics:mass = 4.713
    point3f physics:centerOfMass = (0.012, 0.002, -0.002)
    float3 physics:diagonalInertia = (0.0120, 0.0220, 0.0270)
}
```

### physx.usda — PhysX-Only Tuning

```usda
#usda 1.0

def PhysxJointAPI "FL_hip_joint" {
    float physxJoint:maxJointVelocity = 20.0
    float physxJoint:jointFriction = 0.05
}

def PhysxRigidBodyAPI "trunk" {
    bool physxRigidBody:enableGyroscopicForces = true
    float physxRigidBody:maxDepenetrationVelocity = 10.0
    int physxRigidBody:solverPositionIterationCount = 32
    int physxRigidBody:solverVelocityIterationCount = 1
}
```

`physx.usda` typically sublayers `physics.usda` so PhysX-only opinions stack on top of the neutral physics definition.

## Headless RL Optimization: Skip Appearance

The biggest RL training startup optimization: **don't load appearance payloads**.

When Isaac Lab loads a robot for RL:
1. Loads: `interface.usda` + base + physics layers (joints, masses, collision shapes).
2. Skips: `materials.usda` and `Textures/` (irrelevant for physics sim).

```python
ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"{ASSETS_ROOT}/Robots/MyRobot/interface.usda",
        activate_contact_sensors=True,
        # Do NOT load appearance payloads for RL
        visual_material=None,
    )
)
```

## Composition Arc Precedence

USD applies opinions in this order (last wins for most properties):

```
Sublayers < Reference < Payload < VariantSet < Direct opinions
```

If your physics USDA changes "don't take effect", check that the override opinion is in a higher-precedence layer.

## Common Debugging

```python
from pxr import Usd
# Find which layer is setting a specific property
attr = prim.GetAttribute("physics:mass")
for spec in attr.GetPropertyStack(Usd.TimeCode.Default()):
    print(f"  Layer: {spec.layer.GetDisplayName()} = {spec.default}")
```
