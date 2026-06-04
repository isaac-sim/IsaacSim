---
name: occupancy-map
description: Generate ROS-compatible occupancy maps from USD scenes for robot navigation and perception training. Covers obstacle extraction, 2D grid projection, height filtering, dilation buffers, and Nav2/MobilityGen/A* path planning integration.
---

# Warehouse Occupancy Map Generation

Generate ROS-compatible occupancy maps from USD warehouse scenes for navigation and perception training. Two paths: the documented `isaacsim.asset.gen.omap` extension (PhysX-collider-based, GUI + Python), and a direct USD-projection fallback for prototypes or non-collider scenes.

## When to use

- Nav2 / MobilityGen / A* path planning setup.
- Perception training data generation.
- AMR fleet path planning validation.
- Collision-avoidance buffer-zone calculation.

## Path 1 (recommended): `isaacsim.asset.gen.omap` extension

Documented in `docs/isaacsim/digital_twin/ext_isaacsim_asset_generator_occupancy_map.rst` ([Mapping](https://docs.isaacsim.omniverse.nvidia.com/latest/digital_twin/ext_isaacsim_asset_generator_occupancy_map.html)). The extension uses physics collision geometry, so every prim you want captured must have **Collisions Enabled**; the **Start** location cannot be occupied.

### GUI workflow

1. Open the stage.
2. **Tools > Robotics > Occupancy Map**.
3. Set **Origin** (free point inside the area), **Lower/Upper Bound** (clamp the mapped extent), **Cell Size**, optionally toggle **Use PhysX Collision Geometry**.
4. **CALCULATE**, then **VISUALIZE IMAGE** to preview.
5. From the visualization window: **Save Image** (PNG) and **Save YAML** (ROS occupancy-map parameters file).

### Python (programmatic, simulation playing)

```python
from isaacsim.asset.gen.omap.bindings import _omap
import omni.physx
import omni.usd

physx   = omni.physx.acquire_physx_interface()
stage_id = omni.usd.get_context().get_stage_id()

generator = _omap.Generator(physx, stage_id)
generator.update_settings(
    cell_size=0.1,         # meters per pixel
    z_min=0.1,             # height to map at (m above origin)
    z_max=0.0,             # 0 = use cell_size
    occupancy_threshold=0.5,
)
generator.set_transform(origin=(0, 0, 0), lo_offset=(-10, -10, 0), hi_offset=(10, 10, 0))
generator.generate2d()
buffer = generator.get_buffer()    # raw occupancy data
```

Requires the timeline to be **playing** for PhysX raycasts. For headless runs, pair with `app_utils.play(commit=True)`.

## Path 2 (fallback): direct USD projection

Use when the scene lacks colliders, you need a deterministic projection from authored geometry, or you are prototyping with placeholder cubes. Faster and reproducible for those cases but does not respect physics collision approximations.

### 1. Extract Obstacles from USD
Read all prims, project XY footprint onto 2D grid. Filter by height to separate navigable floor markings from solid obstacles.

```python
from pxr import Usd, UsdGeom, Gf
import numpy as np

stage = Usd.Stage.Open(usd_path)
mpu = UsdGeom.GetStageMetersPerUnit(stage)
bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])

RESOLUTION = 0.1  # meters per pixel
ROBOT_HEIGHT_MIN = 0.05  # ignore floor markings below this
ROBOT_HEIGHT_MAX = 2.0   # robot clearance height

grid_w = int(facility_width / RESOLUTION)
grid_h = int(facility_depth / RESOLUTION)
grid = np.ones((grid_h, grid_w), dtype=np.uint8)  # 1=free

# Skip navigable prims (routes, floor, humans, exits, List below as a reference)
SKIP_PREFIXES = ("Floor", "FL", "AR", "HR", "FR", "AMR", "Exit", "Hum", "Divider", "Stair")

for prim in stage.TraverseAll():
    if not prim.IsA(UsdGeom.Gprim):
        continue
    name = prim.GetName()
    if any(name.startswith(p) for p in SKIP_PREFIXES):
        continue

    bbox = bbox_cache.ComputeWorldBound(prim)
    rng = bbox.ComputeAlignedRange()
    lo, hi = rng.GetMin(), rng.GetMax()

    if hi[2] * mpu < ROBOT_HEIGHT_MIN or lo[2] * mpu > ROBOT_HEIGHT_MAX:
        continue

    x_min = max(0, int(lo[0] * mpu / RESOLUTION))
    x_max = min(grid_w, int(hi[0] * mpu / RESOLUTION) + 1)
    y_min = max(0, int(lo[1] * mpu / RESOLUTION))
    y_max = min(grid_h, int(hi[1] * mpu / RESOLUTION) + 1)

    r_min = grid_h - y_max  # flip Y for image coords
    r_max = grid_h - y_min
    grid[r_min:r_max, x_min:x_max] = 2  # occupied
```

### 2. Apply Robot Buffer
```python
from scipy.ndimage import binary_dilation

ROBOT_RADIUS = 0.5  # meters
buffer_px = int(ROBOT_RADIUS / RESOLUTION)
kernel_size = 2 * buffer_px + 1
kernel = np.zeros((kernel_size, kernel_size), dtype=bool)
for r in range(kernel_size):
    for c in range(kernel_size):
        if (r - buffer_px)**2 + (c - buffer_px)**2 <= buffer_px**2:
            kernel[r, c] = True
buffered = binary_dilation((grid == 2), structure=kernel)
```

### 3. Export ROS Format
```python
from PIL import Image
import yaml

# Grayscale map (ROS standard)
img_data = np.full_like(grid, 205)  # grey=unknown
img_data[grid == 1] = 254  # white=free
img_data[grid == 2] = 0    # black=occupied
Image.fromarray(img_data, 'L').save("map.png")

# YAML config
yaml_data = {
    "image": "map.png",
    "resolution": RESOLUTION,
    "origin": [0.0, 0.0, 0.0],  # [x, y, yaw] of bottom-left pixel
    "negate": 0,
    "occupied_thresh": 0.65,
    "free_thresh": 0.196,
}
with open("map.yaml", "w") as f:
    yaml.dump(yaml_data, f, default_flow_style=False)
```

### 4. Generate Colored Visualization
```python
color = np.zeros((grid_h, grid_w, 3), dtype=np.uint8)
color[grid == 1] = [255, 255, 255]  # white=free
color[grid == 2] = [0, 0, 0]        # black=occupied
color[buffered & (grid != 2)] = [255, 200, 200]  # pink=buffer
Image.fromarray(color, 'RGB').save("map_colored.png")
```

## Key Design Decisions

### What to Mark as Obstacles
- **INCLUDE**: Racks, GSRC modules, docks, tables, pack stations, conveyors (they're elevated but have supports), VLMs, sort equipment, walls, columns
- **EXCLUDE**: Floor plane, fire lane markings, AMR route overlays, human walkway markings, forklift lane markings, humans (they move), AMR robots (they move), exit signs, stairs (navigable)

### Height Filtering
- `z_max < 0.05m` → floor marking, skip (route overlays are at z=0.02-0.04)
- `z_min > 2.0m` → above robot, skip (overhead conveyors at z=6+, HVAC at z=13+)
- Everything else in the 0.05-2.0m band → obstacle

### Resolution Selection
| Use Case | Resolution | Grid Size (220×180m) |
|----------|-----------|---------------------|
| Coarse planning | 0.5m/px | 440×360 |
| Standard nav | 0.1m/px | 2200×1800 |
| Fine perception | 0.05m/px | 4400×3600 |

### Robot Buffer Sizing
| Robot Type | Radius | Buffer |
|-----------|--------|--------|
| Small AMR (e.g. MiR100) | 0.3m | 0.4m |
| Standard AMR (e.g. MiR250) | 0.5m | 0.6m |
| Forklift | 1.0m | 1.2m |
| Human (for walkway planning) | 0.3m | 0.5m |

## Isaac Sim OccupancyMap Class
For integration with MobilityGen path planning:
```python
from isaacsim.replicator.mobility_gen.impl.occupancy_map import OccupancyMap
omap = OccupancyMap.from_ros_yaml("map.yaml")
omap_buffered = omap.buffered_meters(0.5)
# Use with A* planner, spawn placement, etc.
```

## Coordinate Conventions
- **USD world**: X=east, Y=north, Z=up (meters)
- **Image**: row 0 = top = max Y (north), col 0 = left = min X (west)
- **ROS origin**: [x, y, yaw] of bottom-left pixel in world coords
- `world_to_pixel`: world_x / resolution = col, (max_y - world_y) / resolution = row
