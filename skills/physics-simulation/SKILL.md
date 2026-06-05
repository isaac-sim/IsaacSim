---
name: physics-simulation
description: >
  Single source of truth for physics in Isaac Sim 6.0+ / Kit 110. Covers PhysicsScene
  config (gravity, Hz, CCD, stabilization, solver type), per-prim setup (RigidBodyAPI,
  MassAPI, CollisionAPI, kinematic bodies, joint drives), contact materials with
  friction/restitution reference tables, Newton solver selection (Featherstone, MuJoCo,
  XPBD, SemiImplicit, VBD) vs PhysX, physics sensors (contact, IMU, raycast),
  physics-to-USD readback (RigidPrim vs XformCache vs DC), and 5 worked-example
  mechanism templates. Use when configuring a PhysicsScene; applying physics to USD
  prims; choosing solver iterations or Newton backend; setting up contact materials,
  joint drives, or sensors; debugging objects falling through ground; or building
  impact/crash, vibratory feeder, gyro, cradle, or escapement mechanisms.
---

# Physics Simulation in Isaac Sim

Targets Isaac Sim 6.0+ / Kit 110. Both backends share `UsdPhysics.*`; backend-specific behavior is called out per section.

## Backend selection (Kit 110)

`isaacsim.core.simulation_manager` registers physics engines and picks the active one. `default_engine` in its `extension.toml` is `"physx"`, **but** the `isaacsim.physics.newton` extension defaults `auto_switch_on_startup = true`, so any app that enables `isaacsim.physics.newton` (the standard `isaacsim.exp.full.kit` does) ends up with **Newton active** at startup.

```python
from isaacsim.core.simulation_manager import SimulationManager

SimulationManager.switch_physics_engine("newton")   # or "physx"
print(SimulationManager.get_active_physics_engine())

# Inspect what is available
from isaacsim.physics.newton import get_available_physics_engines, get_active_physics_engine
print(get_available_physics_engines())
```

Force the engine explicitly when launching:

```bash
--/exts/isaacsim.core.simulation_manager/default_engine=newton     # or =physx
--/exts/isaacsim.physics.newton/auto_switch_on_startup=false        # opt out of auto-switch
```

Newton config classes (extension Python API; not surfaced in the user-guide RST yet — see the extension's API docs page or `docs/isaacsim/physics/newton_physics.rst`):

| Class | Role |
|---|---|
| `isaacsim.physics.newton.NewtonConfig` | per-sim settings (CUDA graph capture, fabric sync, contact/joint defaults) |
| `XPBDSolverConfig` | XPBD solver (rigid + soft) |
| `MuJoCoSolverConfig` | MuJoCo Warp solver |
| `isaacsim.physics.newton.tensors` | NumPy / PyTorch / Warp frontends |

Both Newton and PhysX consume the standard `UsdPhysics.Scene` + `PhysxSchema.PhysxSceneAPI`; many `PhysxSchema.*` attributes are still honored under Newton, plus Newton reads its solver config via `omni.usd.schema.newton`.

## Stack & reading order

1. This skill: scene config, per-prim setup, contact materials, drives, sensors, readback, backend selection.
2. `usd-articulation`: multi-link articulations + Robot Schema overlay.
3. `urdf-mjcf-to-usd-conversion`: importer config (RL vs teleop drives).
4. `isaac-sim-troubleshooting`: when physics misbehaves on Kit 110.

Mechanism recipes (impact, feeders, dominoes, tops, cradles, pendulum waves, escapements) live in [Worked Examples](#worked-examples).

---

## Part 1 — Scene-Level Configuration

### PhysicsScene Setup

```python
from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, Gf

ps = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
ps.CreateGravityDirectionAttr().Set(Gf.Vec3f(0, 0, -1))
ps.CreateGravityMagnitudeAttr().Set(9.81)

px = PhysxSchema.PhysxSceneAPI.Apply(ps.GetPrim())
px.CreateTimeStepsPerSecondAttr().Set(240)   # see Hz table below
px.CreateEnableCCDAttr().Set(True)
px.CreateEnableStabilizationAttr().Set(True)
px.CreateSolverTypeAttr().Set("TGS")          # TGS or PGS; TGS preferred for articulations
```

### Physics Hz Selection

| Scenario | Hz | Notes |
|---|---|---|
| Standard rigid-body scenes | 60–120 | Default for warehouse, general sim |
| Stacking / contact-rich | 240 | Tight contact resolution |
| High-velocity impacts | 120 with 2–4 substeps | Pair with CCD |
| Small-part vibration (feeders) | ≥ 4× vibration freq, typically 480 | Resolve oscillation correctly |
| Spinning bodies / gyros | 480 | Numerical precision for angular momentum |
| Stiff contact chains (cradles, escapements) | 480 | Solver needs many sub-iterations |

**Rule of thumb:** physics timestep must be > 4× the highest frequency in the system (vibration, spin, contact-stiffness mode).

### Solver Iteration Counts (per-body)

Set on `PhysxRigidBodyAPI` per body that needs it. Higher = more accurate, slower.

| Scenario | Position iters | Velocity iters |
|---|---|---|
| Simple rigid bodies, tumbling | 16 | 4 |
| Stacking | 32 | 8 |
| Complex joints / articulations | 64 | 16 |
| Stiff contact chains (cradle, escapement) | 64 | 32 |

```python
pxrb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
pxrb.CreateSolverPositionIterationCountAttr().Set(32)
pxrb.CreateSolverVelocityIterationCountAttr().Set(8)
pxrb.CreateEnableCCDAttr().Set(True)
```

### When to Disable Stabilization

`EnableStabilizationAttr` is on by default and helps stacks settle. It **destroys angular momentum** on free-spinning bodies. Disable it for:
- Spinning tops, gyros, flywheels
- Pendulum mechanisms (clock escapements, pendulum waves)
- Anything whose correctness depends on conserved angular velocity

```python
px.CreateEnableStabilizationAttr().Set(False)
```

---

## Part 2 — Per-Prim Physics Setup

### RigidBody / Collision / Static / Kinematic

```python
from pxr import UsdPhysics, Gf

def setup_dynamic_body(stage, prim_path, mass_kg=1.0, com_offset=None):
    """Movable, simulated body. RigidBodyAPI + CollisionAPI on the same prim."""
    prim = stage.GetPrimAtPath(prim_path)
    UsdPhysics.RigidBodyAPI.Apply(prim)
    mass_api = UsdPhysics.MassAPI.Apply(prim)
    mass_api.CreateMassAttr().Set(mass_kg)
    if com_offset:
        mass_api.CreateCenterOfMassAttr().Set(Gf.Vec3f(*com_offset))
    UsdPhysics.CollisionAPI.Apply(prim)
    return prim

def setup_static_collider(stage, prim_path):
    """Immovable terrain, walls, fixed obstacles."""
    prim = stage.GetPrimAtPath(prim_path)
    UsdPhysics.CollisionAPI.Apply(prim)
    return prim

def setup_kinematic_body(stage, prim_path):
    """Scripted motion: conveyors, elevators, vibrating bowls, escape wheels."""
    prim = stage.GetPrimAtPath(prim_path)
    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.RigidBodyAPI(prim).CreateKinematicEnabledAttr().Set(True)
    UsdPhysics.CollisionAPI.Apply(prim)
    return prim
```

**Rule:** `RigidBodyAPI` + `CollisionAPI` on the **same prim**. Splitting them across parent/child causes intermittent collision failures.

### Static Colliders with Scale — Translate-First Pattern

Scaling a Cube prim with `CollisionAPI` applied directly causes PhysX to use the wrong collision bounds (objects fall through ground). Use a parent xform for position, a child mesh for scale:

```python
# CORRECT
xf = UsdGeom.Xform.Define(stage, "/World/Ground")
UsdGeom.Xformable(xf.GetPrim()).AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.05))
mesh = UsdGeom.Cube.Define(stage, "/World/Ground/Mesh")
mesh.CreateSizeAttr().Set(1.0)
UsdGeom.Xformable(mesh.GetPrim()).AddScaleOp().Set(Gf.Vec3f(50.0, 50.0, 0.1))
UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
```

**`Cube.size=1.0`** means the cube has half-extents of 0.5, not 1.0. Use `size=2.0` when you want "the scale op equals the half-extent."

### Mass & Inertia

```python
mass_api = UsdPhysics.MassAPI.Apply(prim)
mass_api.CreateMassAttr().Set(0.25)                                    # kg
mass_api.CreateCenterOfMassAttr().Set(Gf.Vec3f(0, 0, 0.05))            # local
mass_api.CreateDiagonalInertiaAttr().Set(Gf.Vec3f(1e-4, 1e-4, 2e-4))   # kg·m²
```

For URDF-imported robots, prefer `import_inertia_tensor: true` in `config.yaml` over auto-computed geometric inertia (see `urdf-mjcf-to-usd-conversion`).

---

## Part 3 — Contact Materials

```python
def create_contact_material(stage, mat_path, static_friction=0.5,
                            dynamic_friction=0.4, restitution=0.1):
    prim = stage.DefinePrim(mat_path)
    mat = UsdPhysics.MaterialAPI.Apply(prim)
    mat.CreateStaticFrictionAttr().Set(static_friction)
    mat.CreateDynamicFrictionAttr().Set(dynamic_friction)
    mat.CreateRestitutionAttr().Set(restitution)
    return mat
```

### Reference Values

| Material pairing | Static μ | Dynamic μ | Restitution |
|---|---|---|---|
| Concrete on concrete | 0.6 | 0.5 | 0.05 |
| Steel on steel | 0.74 | 0.57 | 0.6 |
| Rubber on rubber | 0.8 | 0.7 | 0.5 |
| Rubber on concrete | 1.0 | 0.8 | 0.3 |
| Wood on wood | 0.5 | 0.3 | 0.2 |
| Metal generic | 0.4 | 0.3 | 0.2 |
| Plastic (dice) | 0.4 | 0.3 | 0.3 |
| Felt (casino) | 0.5 | 0.4 | 0.2 |
| Cardboard on steel | 0.4 | 0.3 | 0.1 |

For chains of stiff contacts (Newton's cradle, escapements), set `restitutionCombineMode=max` on `PhysxMaterialAPI` so the highest restitution wins at each contact.

---

## Part 4 — Joint Drives

```python
joint = stage.GetPrimAtPath("/World/Robot/joint_arm")
drive = UsdPhysics.DriveAPI.Apply(joint, "angular")    # "angular" | "linear"
drive.CreateTypeAttr().Set("force")                    # "force" | "acceleration"
drive.CreateStiffnessAttr().Set(1000.0)                # Kp (Nm/rad for angular)
drive.CreateDampingAttr().Set(100.0)                   # Kd (Nm·s/rad)
drive.CreateMaxForceAttr().Set(500.0)                  # torque/force limit
drive.CreateTargetPositionAttr().Set(0.0)              # target (deg or m)
```

**For RL training**, the agent commands torques directly. Set drive_type to `none` and stiffness/damping to 0 in `config.yaml` (see `urdf-mjcf-to-usd-conversion`). Active PD drives fight the RL agent.

**For revolute pendulum joints** (clock escapements, pendulum waves), set joint friction to 0:
```python
joint_api = PhysxSchema.PhysxJointAPI.Apply(joint)
joint_api.CreateJointFrictionAttr().Set(0.0)
```

---

## Part 5 — Backend Selection (Newton vs PhysX)

### Quick Choice

| You want | Use |
|---|---|
| RL training with thousands of envs | **Newton** (Featherstone or MuJoCo) |
| Differentiable simulation | **Newton** |
| Legacy PhysX scene from Isaac Sim 5.x | **PhysX** |
| Soft bodies, cloth, deformables | **Newton** (VBD or XPBD) |
| Validated against MuJoCo baselines | **Newton SolverMuJoCo** |

### Newton Solvers

| Solver | Coordinates | Differentiable | Best For |
|---|---|---|---|
| **SolverFeatherstone** | Generalized | Yes (Warp) | Articulated robots (default for manipulators, legged) |
| **SolverMuJoCo** | Generalized | Yes (mujoco-warp) | Validated locomotion, MuJoCo policy ports |
| **SolverXPBD** | Maximal | Partial | Soft constraints, cables, ropes |
| **SolverSemiImplicit** | Maximal | Yes (Warp) | Fast prototyping, simple rigid bodies |
| **SolverVBD** | (deformable) | Yes | Soft bodies, deformables |

### Newton vs PhysX Differences

| Aspect | PhysX | Newton |
|---|---|---|
| Backend | Closed C++/CUDA | Warp/CUDA (open, JIT) |
| Coordinates | Maximal (6DoF per body) | Generalized (Featherstone) or maximal |
| Differentiable | No | Yes (native Warp autodiff) |
| Multi-GPU | Limited | Yes (Warp device abstraction) |
| USD integration | Schema extensions | Native USD loader |
| Performance ceiling | Good < 4096 envs | Designed for 10K+ envs |

### Newton + Torch — Critical Init Order

**Never `import torch` before Newton physics settles** — CUDA context conflict hangs Kit. Defer torch imports until after `timeline.play()` + settle loop. Use `map_location="cpu"` for policy inference if VRAM is tight.

### Newton-Specific Configuration (Isaac Lab)

```yaml
# config.yaml for URDF→USD conversion (Isaac Lab)
make_instanceable: true    # CRITICAL for RL parallel envs
fix_base: false            # true for fixed-base arm; false for mobile/legged
```

See `urdf-mjcf-to-usd-conversion` for the full schema.

---

## Part 6 — Physics Sensors

The current namespace is `isaacsim.sensors.experimental.physics` (authoring + runtime classes paired). The legacy `isaacsim.sensors.physics` import path still works but is deprecated for new code.

> **Migration:** see [Migrating from `isaacsim.sensors.physics` to `isaacsim.sensors.experimental.physics`](https://docs.isaacsim.omniverse.nvidia.com/latest/migration_guides/isaac_sim_6_0/sensors_physics_to_experimental_physics.html#isaacsim-sensors-physics-migration) for the concept mapping and updated scripts.

### Contact

```python
from isaacsim.sensors.experimental.physics import Contact, ContactSensor
import isaacsim.core.experimental.utils.app as app_utils

contact = Contact.create(
    path="/World/Robot/foot/contact",
    min_threshold=0.0, max_threshold=1e6, radius=-1,   # -1 = use collision shape
)
sensor = ContactSensor(contact)
app_utils.play(commit=True)            # required before get_data()
reading = sensor.get_data()            # ContactSensorReading
```

### IMU

```python
from isaacsim.sensors.experimental.physics import IMU, IMUSensor

imu = IMU.create(path="/World/Robot/imu", tick_rate=200.0)
sensor = IMUSensor(imu, annotators=["linear_acceleration", "angular_velocity", "orientation"])
app_utils.play(commit=True)
frame = sensor.get_data()                # returns IMUSensorReading
```

### Effort / joint state

Runtime-only classes from the same module; no separate authoring type — they wrap an existing joint by path.

```python
from isaacsim.sensors.experimental.physics import EffortSensor, JointStateSensor

effort = EffortSensor("/World/Robot/joint_arm_1")
joint  = JointStateSensor("/World/Robot/joint_arm_1")
app_utils.play(commit=True)
reading = effort.get_data()              # EffortSensorReading
state   = joint.get_data()               # JointStateSensorReading
```

### Raycast (scene query)

```python
import omni.physics.tensors as physics_tensors

sim_view = physics_tensors.create_simulation_view("cuda:0")
hit = sim_view.raycast_closest(origin, direction, max_dist)
```

For higher-fidelity sensor simulation (LiDAR scan patterns, multi-ray, vendor sensor models, depth/radar/acoustic), see `isaac-sim-sensor` and `isaac-camera`. They use the modern `isaacsim.sensors.experimental.rtx` and `.physics` namespaces.

---

## Part 7 — Physics-to-USD Readback (CRITICAL)

The most common silent bug: reading authored USD transforms instead of simulated state.

| Source | Returns | When to use |
|---|---|---|
| `UsdGeom.XformCache.GetLocalToWorldTransform()` | **Authored** USD transform (initial pose) | Editor-time queries, before play |
| `RigidPrim.get_world_pose()` | **Simulated** state | Always during simulation |
| `Articulation.get_world_poses()` (Kit 110) | **Simulated** state for articulated bodies | Articulated robots |
| Dynamic Control (DC) `dc.get_rigid_body_pose()` | **Simulated** state | Legacy / quick scripts |

### Why XformCache Is Wrong During Sim

`updateToUsd=True` writes physics state to **Fabric**, not the USD stage layer. `XformCache` reads the USD layer. Result: it always returns initial poses.

### RigidPrim / GeomPrim pattern (Kit 110)

```python
from isaacsim.core.experimental.prims import RigidPrim
import isaacsim.core.experimental.utils.app as app_utils

rp = RigidPrim(paths="/World/Dice/Die_*")
app_utils.play(commit=True)
pos_wp, quat_wp = rp.get_world_poses()   # warp arrays
positions  = pos_wp.numpy()              # (N, 3)
quaternions = quat_wp.numpy()            # (N, 4) [w, x, y, z]
```

### Articulation pattern (Kit 110)

```python
from isaacsim.core.experimental.prims import Articulation
import omni.timeline

robot = Articulation("/World/Robot")
omni.timeline.get_timeline_interface().play()        # required for tensor data
pos_wp, quat_wp = robot.get_world_poses()
dof_positions  = robot.get_dof_positions().numpy()    # (N, num_dofs)
dof_velocities = robot.get_dof_velocities().numpy()
J = robot.get_jacobian_matrices().numpy()             # for IK
```

Legacy `isaacsim.core.api.articulations.Articulation` / `isaacsim.core.api.prims.RigidPrim` still load but are superseded by the `isaacsim.core.experimental.*` stack.

> **Migration:** for the broader `omni.isaac.*` → `isaacsim.*` renaming map, see [Renaming Extensions](https://docs.isaacsim.omniverse.nvidia.com/latest/migration_guides/isaac_sim_4_5/extensions_renaming.html). For the experimental Articulation / RigidPrim APIs see the [Python scripting index](https://docs.isaacsim.omniverse.nvidia.com/latest/python_scripting/index.html).

### Quaternion Convention

USD/Isaac uses `[w, x, y, z]`; scipy uses `[x, y, z, w]`. Convert:

```python
from scipy.spatial.transform import Rotation
r = Rotation.from_quat([quat[1], quat[2], quat[3], quat[0]])
euler = r.as_euler('xyz', degrees=True)
```

---

## Part 8 — Common Gotchas

1. **`CollisionAPI` alone = static collider**; `RigidBodyAPI` + `CollisionAPI` = dynamic.
2. **Same-prim requirement**: both APIs must be on the **same prim**.
3. **Kinematic bodies**: use `CreateKinematicEnabledAttr().Set(True)`, not enable/disable on RigidBodyAPI.
4. **`Cube.size=1.0`** = half-extent 0.5. Use `size=2.0` if you want scale ops to equal half-extents.
5. **`physics:velocity` USD attributes are ignored** by PhysX at runtime. Use `RigidPrim.set_linear_velocities()` **after `timeline.play()`**.
6. **`physics:angularVelocity` is in DEGREES/second**, not rad/s. Convert with `math.degrees()`.
7. **`SimulationContext.step(render=True)`** is the only reliable physics-with-render advance. `app.update()` does not sync physics.
8. **Experimental sensors need `app_utils.play(commit=True)`** (or `timeline.play()`) before `get_data()`; do not call `initialize()` from the legacy `World` flow.
9. **`get_rigid_body_state()` does not exist in Isaac Sim 5.1+**; use `RigidPrim.get_world_poses()` from `isaacsim.core.experimental.prims`.
10. **PhysX cannot resolve sequential momentum transfer** in same-island contact chains (see [Worked Example 4: Newton's Cradle](#4-newtons-cradle--physx-same-island-contact-limitation)).
11. **Tunneling at high spin rates**: compound colliders fail above ~50 rad/s after 5–6s. Use simpler convex hulls or higher physics Hz.

---

## Worked Examples (impact, vibratory feeder, gyro, cradle, escapement)

See [`examples.md`](examples.md) for details.
