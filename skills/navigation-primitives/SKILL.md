---
name: navigation-primitives
description: >
  Shared substrate for mobile robot navigation in Isaac Sim. Covers the primitives both
  runtime navigation (isaac-sim-robot-navigation) and synthetic data generation (mobility-gen)
  build on top of: OccupancyMap from USD or ROS YAML, A* path planning + smoothing,
  collision-derived robot footprint / Z-offset / inscribed-and-circumscribed radii,
  oriented-footprint PhysX overlap_box validation, differential and holonomic wheel
  kinematics, look-at chase camera math, and standard navigation gotchas.
  Use when implementing any mobile-robot navigation in Isaac Sim, computing an occupancy
  map from a USD stage, planning paths over a grid, sizing buffers for robot footprints,
  setting up chase cameras, or as the foundation BEFORE choosing between runtime navigation
  and SDG pipelines.
  Triggers on: occupancy map, A* path planning, OccupancyMap, generate_paths, differential
  drive, holonomic, mecanum, robot footprint, chase camera, navigation buffer,
  overlap_box, oriented footprint, inscribed radius.
---

# Navigation Primitives — Shared Substrate

Foundation layer for mobile robot navigation. Consumed by:

- `isaac-sim-robot-navigation`: runtime navigation in custom scripts (RL policy, physics-vs-baked, GPU OOM).
- `mobility-gen`: two-phase MobilityGen SDG (record -> replay+render).
- `occupancy-map`: produces the `map.yaml` consumed here.

Read this first for any mobile-robot work, then jump to the specialization.

## Shared NVIDIA APIs

| Capability | Module |
|---|---|
| Occupancy maps | `isaacsim.replicator.mobility_gen.impl.occupancy_map.OccupancyMap` |
| A* path planner | `isaacsim.replicator.mobility_gen.impl.path_planner.generate_paths` |
| Runtime omap from stage | `isaacsim.asset.gen.omap.bindings._omap.Generator` |
| Robot articulation | `isaacsim.core.experimental.prims.Articulation` |
| Differential controller | `isaacsim.robot.experimental.wheeled_robots.controllers.DifferentialController` |
| Holonomic controller | `isaacsim.robot.wheeled_robots.controllers.holonomic_controller.HolonomicController` |
| Physics lifecycle | `isaacsim.core.simulation_manager.SimulationManager` |

`OccupancyMap.from_ros_yaml(path)` loads a YAML+PNG pair (produced by `occupancy-map`). Both isaac-sim-robot-navigation and mobility-gen consume this same format.

## Robot Footprints & Z-Offsets — Derived at Runtime

Do not hardcode footprints. Walk the articulation's collider prims and union their world-space AABBs. This handles every robot (Spot, Carter, VSVXL, Jetbot, Kaya, H1, custom) and stays correct when assets change.

```python
import isaacsim.core.experimental.utils.bounds as bounds_utils
from pxr import Usd, UsdPhysics
import numpy as np

def compute_robot_footprint(stage: Usd.Stage, robot_root: str) -> dict:
    """Return footprint dims + Z-offset + inscribed/circumscribed radii.

    Uses prims tagged with UsdPhysics.CollisionAPI under `robot_root`. Falls back
    to UsdGeom.Imageable if no colliders are authored.
    """
    collider_paths = []
    for prim in Usd.PrimRange(stage.GetPrimAtPath(robot_root)):
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            collider_paths.append(prim.GetPath())
    if not collider_paths:
        collider_paths = [stage.GetPrimAtPath(robot_root).GetPath()]

    aabb = bounds_utils.compute_combined_aabb(collider_paths)  # [xmin,ymin,zmin, xmax,ymax,zmax]
    mn, mx = aabb[:3], aabb[3:]
    size = mx - mn

    origin_z = stage.GetPrimAtPath(robot_root).GetAttribute("xformOp:translate").Get()[2]
    z_offset = max(0.0, origin_z - mn[2])  # how far origin sits above lowest collider

    half_w, half_d = size[0] / 2.0, size[1] / 2.0
    return {
        "size": tuple(size),                       # full footprint extents (m)
        "z_offset": float(z_offset),               # origin → lowest collider (m)
        "inscribed_radius":  float(min(half_w, half_d)),  # safe for ANY yaw
        "circumscribed_radius": float(np.hypot(half_w, half_d)),  # worst-case yaw
        "aabb_min": tuple(mn),
        "aabb_max": tuple(mx),
    }
```

Use `inscribed_radius` when the robot can rotate freely in place (over-conservative, zero clip). Use `circumscribed_radius` only when you require zero false negatives. For non-circular robots (Spot, VSVXL), prefer the oriented-footprint check below over a single radius.

**Always spawn the robot at `z = ground + z_offset`**. Missing the Z-offset is the #1 cause of "robot falls through the floor" or "feet pop above ground" bugs.

### Reference Values (sanity check only)

If your `compute_robot_footprint` output is far from these, your collider authoring or scene units are wrong:

| Robot | Expected size (m) | Expected z_offset | Inscribed r |
|---|---|---|---|
| Spot | ~1.08 × 0.44 × 0.55 | ~0.69 | ~0.22 |
| Spot + arm | ~1.10 × 0.40 × 1.20 | ~0.69 | ~0.20 |
| Nova Carter | track_w=0.499, wheel_r=0.14 | ~0.0 | ~0.25 |
| VSVXL | ~2.52 × 1.72, 6-wheel diff | ~0.0 | ~0.86 |
| Jetbot | wheel_base=0.1125, wheel_r=0.03 | ~0.02 | ~0.06 |
| Kaya (holonomic) | wheel_base=0.10, wheel_r=0.04 | ~0.02 | ~0.10 |
| H1 (humanoid) | — | ~1.05 | ~0.20 |

## Occupancy Map from USD (Direct Projection)

Use when you need a runtime omap and don't already have a `map.yaml`. For the canonical `map.yaml` workflow consumed by MobilityGen, use `occupancy-map` instead.

```python
from pxr import Usd, UsdGeom, Gf
import numpy as np

stage = Usd.Stage.Open(stage_path)
bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])

RESOLUTION = 0.25  # meters per cell
X_RANGE = (-15, 175)
Y_RANGE = (-25, 155)
Z_CUTOFF = 4.5  # ignore obstacles above this height
# Robot dims come from compute_robot_footprint(stage, "/World/Robot"); see above.

grid_w = int((X_RANGE[1] - X_RANGE[0]) / RESOLUTION)
grid_h = int((Y_RANGE[1] - Y_RANGE[0]) / RESOLUTION)
grid = np.zeros((grid_h, grid_w), dtype=np.uint8)  # 0=free, 255=occupied

for prim in Usd.PrimRange(stage.GetPrimAtPath("/World")):
    bb = bbox_cache.ComputeWorldBound(prim)
    r = bb.ComputeAlignedRange()
    if r.IsEmpty(): continue
    mn, mx = r.GetMin(), r.GetMax()
    if mn[2] > Z_CUTOFF: continue
    x0 = max(0, int((mn[0] - X_RANGE[0]) / RESOLUTION))
    x1 = min(grid_w, int((mx[0] - X_RANGE[0]) / RESOLUTION))
    y0 = max(0, int((mn[1] - Y_RANGE[0]) / RESOLUTION))
    y1 = min(grid_h, int((mx[1] - Y_RANGE[0]) / RESOLUTION))
    grid[y0:y1, x0:x1] = 255
```

### Obstacle Filtering (CRITICAL — learned 2026-03-13)

Two strategies, in order of preference:

**A. Collider-driven (preferred when assets have authored colliders).** Iterate only prims with `UsdPhysics.CollisionAPI`. This already excludes visual-only geometry (signage, decals, light cones, debug arrows) without a filter list.

```python
from pxr import UsdPhysics
for prim in Usd.PrimRange(stage.GetPrimAtPath("/World")):
    if not prim.HasAPI(UsdPhysics.CollisionAPI):
        continue
    enabled = prim.GetAttribute("physics:collisionEnabled")
    if enabled and enabled.Get() is False:
        continue
    # rasterize this prim's AABB into the grid
```

**B. Visual-bbox + filter list (fallback for scenes without colliders).** Naive bbox projection fills the grid with shell, zones, signage. Filter aggressively:

```python
SKIP_SCOPES = {"GroundPlane", "Looks", "Lighting", "Render", "PushGraph",
               "DomeLight", "DemoCamera", "Spot_01", "Spot_02", "Obstacles_Anim"}
SKIP_PREFIXES = ("Floor", "FL_", "FR_", "Exit_", "Hum", "SM_Deluxe")
SKIP_CHILDREN = ("sm_warehouse_mega", "Zones", "signage")
```

Geometric filters (apply after either strategy):
- Skip area > 3000 m² (shell, zone assemblies)
- Skip height < 0.1 m (floor markings, safety tape)
- Skip Z_min > 3.6 m (ceiling-only objects, ≈ 3× robot height)
- Skip Z_max < `fp["z_offset"]` + 0.05 m (anything the robot can drive over)

**Result on V4 KION**: 244 real obstacles (vs 5000+ before filtering), 70% free space.

### Buffer Sizing — Derived from Footprint

The buffer is `circumscribed_radius + safety_margin`, not a magic constant. Empirical `safety_margin` defaults (validated 2026-03-15):

| Context | Safety margin |
|---|---|
| Open corridor, smooth control | 0.10 m |
| Aisle navigation, cluttered | 0.30 m |
| Cluttered + non-zero yaw error | 0.50 m |

```python
fp = compute_robot_footprint(stage, "/World/Robot")
buffer_m = fp["circumscribed_radius"] + 0.30  # aisle default
buffer_cells = int(round(buffer_m / RESOLUTION))
```

For Spot (`circumscribed_radius ≈ 0.58 m`) this yields ~0.88–1.08 m, not the legacy 1.5 m blanket value. The legacy value was conservatively tuned against a circular proxy; with the oriented-footprint check (below) you recover the extra ~0.5 m of navigable space.

## A* Path Planning

Erode by `inscribed_radius` (fast, conservative). Then validate the smoothed path with an oriented-footprint collision check, which recovers the navigable space the inscribed-radius erosion threw away.

```python
from scipy.ndimage import binary_erosion
import numpy as np

fp = compute_robot_footprint(stage, "/World/Robot")
kernel_r = int(fp["inscribed_radius"] / RESOLUTION)
kernel = np.zeros((2*kernel_r+1, 2*kernel_r+1), dtype=bool)
for dy in range(-kernel_r, kernel_r+1):
    for dx in range(-kernel_r, kernel_r+1):
        if dx*dx + dy*dy <= kernel_r*kernel_r:
            kernel[dy+kernel_r, dx+kernel_r] = True

navigable = (grid == 0)
eroded = binary_erosion(navigable, structure=kernel)
# Standard A* over `eroded` (heapq-based)
```

### Oriented-Footprint Collision Check (recommended for non-circular robots)

Drives the same PhysX query the simulator uses. Works in two modes:

**1. Against PhysX scene (after `SimulationManager.initialize()`):** use `get_physx_scene_query_interface().overlap_box`. Returns hit count; >0 means clip.

```python
import carb
from omni.physx import get_physx_scene_query_interface
from pxr import Gf

def footprint_clips(x: float, y: float, yaw: float, fp: dict, z_query: float = 0.2) -> bool:
    # half-extents of the robot footprint
    half = carb.Float3(fp["size"][0] / 2, fp["size"][1] / 2, fp["size"][2] / 2)
    origin = carb.Float3(x, y, z_query + fp["size"][2] / 2)
    # quaternion (x, y, z, w) for yaw about Z
    rot = Gf.Rotation(Gf.Vec3d(0, 0, 1), np.degrees(yaw)).GetQuat()
    quat = carb.Float4(*rot.GetImaginary(), rot.GetReal())
    hits = get_physx_scene_query_interface().overlap_box(
        half, origin, quat, lambda h: True, anyHit=True,  # early-exit on first hit
    )
    return hits > 0
```

**2. Against the rasterized omap (no PhysX needed):** stamp the rotated rectangle onto the obstacle grid and AND with the occupied mask.

```python
import cv2

def footprint_clips_grid(px: int, py: int, yaw: float, fp: dict, grid: np.ndarray) -> bool:
    w_cells = fp["size"][0] / RESOLUTION
    h_cells = fp["size"][1] / RESOLUTION
    rect = ((px, py), (w_cells, h_cells), np.degrees(yaw))
    pts = cv2.boxPoints(rect).astype(np.int32)
    mask = np.zeros_like(grid, dtype=np.uint8)
    cv2.fillPoly(mask, [pts], 1)
    return bool(np.any((grid > 0) & (mask > 0)))
```

### Validation Pipeline (MANDATORY)

1. `compute_robot_footprint(stage, robot_root)` — get size, z_offset, radii.
2. Rasterize obstacles onto grid (0.25 m resolution typical).
3. Binary erode with circular kernel of radius = `inscribed_radius / resolution`.
4. A* pathfind on eroded grid.
5. Catmull-Rom smooth the raw path; assign yaw = `atan2(dy, dx)` along the curve.
6. **For every smoothed waypoint, run `footprint_clips_grid(px, py, yaw, fp, grid)`** (or `footprint_clips(...)` against PhysX). Reject the path on any hit.
7. If a single waypoint fails, snap to the nearest navigable cell and re-validate. If multiple fail, the inscribed-radius A* path is fundamentally bad — re-plan with a larger erosion kernel (`circumscribed_radius`).

Skipping steps 6–7 produces paths that look fine on the omap but clip walls in render — especially on rectangular robots (Spot, VSVXL) cornering through aisles.

## Differential Drive Kinematics

```python
# Wheel velocities from body twist
vL = (vx - omega * track_w / 2) / wheel_r
vR = (vx + omega * track_w / 2) / wheel_r
```

PD steering (validated on Nova Carter / VSVXL):
- KP=2.5, KD=1.2, MAX_W=1.5 rad/s
- Waypoint tolerance: 4.0m
- Speed reduction near waypoints and during large heading errors
- Out-of-bounds: |Z| > 50 or |X|/|Y| > 500 → mark dead

VSVXL validated parameters (LLM Advisor Grok-4, 2026-03-15):
- ω = v / r (0.15m wheels → 10 rad/s for 1.5 m/s)
- Physics: dt=1/120s, substeps=4 (effective 480Hz)
- PID heading: Kp=2.0, Ki=0.1, Kd=0.5, max angular 1.0 rad/s
- Aisle speed: 0.8 m/s; corridor speed: 1.5 m/s
- Corridor buffer: 0.5m; aisle buffer: 0.2m + reduced speed

## Holonomic / Mecanum (Kaya, AMR)

Use `HolonomicController` with `HolonomicRobotUsdSetup` to extract wheel positions, orientations, mecanum angles from the robot USD. Apply via `WheeledRobot.apply_wheel_actions`.

2D MobilityGen action `[lin, ang]` → 3D holonomic command `[forward, lateral=0, yaw]`. See `mobility-gen` for the `WheeledMobilityGenRobot.build()` override pattern.

## DifferentialController + Articulation (Kit 110)

```python
from isaacsim.robot.experimental.wheeled_robots.controllers import DifferentialController
from isaacsim.core.experimental.prims import Articulation
import numpy as np

robot = Articulation("/World/Robot")
robot.initialize_cpp_data_view()

dc = DifferentialController(wheel_radius=0.15, wheel_base=1.52)
wheel_vels = dc.forward(np.array([linear_speed, angular_speed]))

vel_targets = np.zeros((1, num_dofs))
for i in left_wheel_indices:  vel_targets[0, i] = wheel_vels[0]
for i in right_wheel_indices: vel_targets[0, i] = wheel_vels[1]
robot.set_dof_velocity_targets(vel_targets)
```

## World Transform Extraction

`BBoxCache.ComputeWorldBound()` returns LOCAL bounds for Cube prims — **wrong** for world position. For actual world position use:

```python
xf_cache = UsdGeom.XformCache(Usd.TimeCode.Default())
world_mat = xf_cache.GetLocalToWorldTransform(prim)
position = world_mat.ExtractTranslation()
```

For **simulated** position (Articulation runtime, not authored), use:
```python
pos_wp, quat_wp = robot.get_world_poses()
pos = pos_wp.numpy()[0]    # shape (3,)
quat = quat_wp.numpy()[0]  # shape (4,) [w,x,y,z]
```

`XformCache` reads authored USD. `Articulation.get_world_poses()` reads simulated state.

## Look-At Chase Camera (MANDATORY pattern)

Never use manual rotation matrices for chase cameras. Always use look-at:

```python
from pxr import Gf

def look_at(eye, target, up=Gf.Vec3d(0, 0, 1)):
    fwd = (target - eye).GetNormalized()
    right = Gf.Vec3d.GetCross(fwd, up).GetNormalized()
    cam_up = Gf.Vec3d.GetCross(right, fwd)
    # USD camera: -Z is forward
    return Gf.Matrix4d(
        right[0],  right[1],  right[2],  0,
        cam_up[0], cam_up[1], cam_up[2], 0,
        -fwd[0],  -fwd[1],   -fwd[2],   0,
        eye[0],    eye[1],    eye[2],   1)
```

Standard camera offsets:
- **Chase**: 4m behind robot, 2.5m up, looking at robot center
- **Overhead**: 10m up, looking straight down
- **POV**: at robot front (1.26m forward), 0.8m height

### Degenerate Up-Vector

When camera looks straight down (fwd ≈ 0,0,−1), `cross(fwd, up=(0,0,1))` is zero → broken matrix → blank render. Always fallback:

```python
up = Gf.Vec3d(0, 0, 1)
if abs(fwd * up) > 0.99:
    up = Gf.Vec3d(0, 1, 0)
```

## Common Gotchas (shared across all navigation skills)

1. **Feet/wheels below origin**: many robots (Spot ~0.69 m, H1 ~1.05 m) have their articulation origin above the ground contact. Always call `compute_robot_footprint(stage, root)` and spawn at `z = ground + fp["z_offset"]`.
2. **Instancing invisible in headless Hydra**: `instanceable=true` prims don't render in arm64 headless. Flatten first.
3. **OmniGraph crashes on out-of-range frames**: jumping past `endTimeCode` crashes PushGraph camera animation nodes.
4. **`next_update_async`**: doesn't exist in Isaac Sim 6 — use `omni.kit.app.get_app().next_update_async()`.
5. **Per-frame yaw smoothing**: `current_yaw += yaw_diff * 0.15` for smooth turning instead of instant snapping.
6. **Frame numbering for ffmpeg**: sequential `frame_0000.png`, `frame_0001.png`... NOT sparse — ffmpeg skips gaps.
7. **Routes need lit zones**: A* paths through dark corridors render as black frames. Define route waypoints in populated/lit areas, or add SphereLights at Z=3m along the route (intensity 800, radius 0.3).

## Integration Points

- **RECEIVES from:** `occupancy-map` — `map.yaml` + `map.png` pair
- **RECEIVES from:** any USD scene with traversable geometry
- **PRODUCES for:** `isaac-sim-robot-navigation` — primitives for runtime navigation
- **PRODUCES for:** `mobility-gen` — primitives for SDG record/replay

## Specialization Skills (read after this one)

| Goal | Skill |
|---|---|
| Drive a robot through a scene in real time (RL policy, physics, baked) | `isaac-sim-robot-navigation` |
| Record a trajectory and re-render it with sensors for SDG | `mobility-gen` |
| Generate the `map.yaml` from USD | `occupancy-map` |
| Publish/subscribe nav topics to ROS 2 / Nav2 | `isaac-sim-ros2-bridge` |
