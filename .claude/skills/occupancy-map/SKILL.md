---
name: occupancy-map
description: "Export ROS occupancy grids (map.yaml/png) from USD for Nav2. Not for footprints, A*, or kinematics (use navigation-primitives)."
license: Apache-2.0
metadata:
  author: Renato Gasoto <info@nvidia.com>
---

# Warehouse Occupancy Map Generation

## Purpose

Export ROS-compatible occupancy grids from USD scenes via the omap extension or a USD-projection fallback for Nav2, MobilityGen, and A* planners.

## Prerequisites

- Built Isaac Sim exposing `isaacsim.asset.gen.omap` (or a USD stage for the projection fallback).
- `$ISAAC_SIM_DIR` pointing at `_build/linux-x86_64/release`; `numpy`, `scipy`, `Pillow` for the projection/export path.
- For Path 1 (collider-based), prims must have Collisions Enabled and the timeline must be playing.

## Limitations

- Path 1 requires authored PhysX colliders; prototype scenes without `CollisionAPI` fall through to USD projection.
- Output is a 2D top-down grid at a single height band; multi-floor or overhanging geometry is not represented.
- Path 2 (projection) ignores physics collision approximations and is only a prototype substitute.

## Troubleshooting

| Error / symptom | Cause | Solution |
|---|---|---|
| `_omap` import fails | `isaacsim.asset.gen.omap` not built or offline | Use `generate_occupancy_map.py` — it falls back to USD projection |
| Zero occupied cells | Prims lack colliders or timeline not playing | Enable Collisions and `play(commit=True)`, or use Path 2 |
| Map extent clipped / empty | Wrong origin or lower/upper bounds | Set bounds to cover the facility; origin must be a free cell |

Generate ROS-compatible occupancy maps from USD warehouse scenes for navigation and perception training. Two paths: the documented `isaacsim.asset.gen.omap` extension (PhysX-collider-based, GUI + Python), and a direct USD-projection fallback for prototypes or non-collider scenes.

## Available Scripts

| Script | Purpose | Arguments |
|---|---|---|
| `scripts/generate_occupancy_map.py` | Unified entry point — tries colliders, falls back to USD projection | see script --help |
| `scripts/usd_projection_pipeline.py` | Direct USD-projection occupancy map pipeline (Path 2 only) | see script --help |

## Running scripts

From agent runtimes that expose skill execution helpers, invoke helpers with `run_script()`:

```python
run_script("scripts/usd_projection_pipeline.py", args=["--help"])
```

From a built Isaac Sim tree, run the same file with `./python.sh` (Linux) or `python.bat` (Windows) from `_build/*/release`, or execute shell helpers directly when they do not require the simulator.

## When to use

- Nav2 / MobilityGen / A* path planning setup.
- Perception training data generation.
- AMR fleet path planning validation.
- Collision-avoidance buffer-zone calculation.

## Automatic Fallback (recommended entry point)

Use `scripts/generate_occupancy_map.py` for all occupancy-map generation. It
attempts the collider-based Path 1 first and automatically falls back to
USD projection (Path 2) when:

- The `isaacsim.asset.gen.omap` extension is not importable (offline / no runtime).
- The PhysX overlap query returns zero occupied cells (prototype assets without `CollisionAPI`).
- The generator raises any exception (missing stage, timeline not playing, etc.).

```python
from generate_occupancy_map import generate_occupancy_map

grid = generate_occupancy_map("warehouse.usd", output_dir="./maps", resolution=0.1)
```

Or from the command line:
```bash
python generate_occupancy_map.py warehouse.usd --output_dir ./maps --resolution 0.1
```

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

`extract_obstacles_from_usd(usd_path, resolution, facility_width, facility_depth, robot_height_min, robot_height_max, skip_prefixes)` — returns a uint8 grid (1=free, 2=occupied).

See [`scripts/usd_projection_pipeline.py`](scripts/usd_projection_pipeline.py).

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
`export_ros_map(grid, output_dir, resolution)` — writes `map.png` (grayscale, ROS standard) and `map.yaml` to `output_dir`.

See [`scripts/usd_projection_pipeline.py`](scripts/usd_projection_pipeline.py).

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

