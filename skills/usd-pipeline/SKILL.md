---
name: usd-pipeline
description: "USD asset discovery, measurement, placement, and shader compatibility. Use for scene assembly and headless render prep."
license: Apache-2.0
metadata:
  author: Renato Gasoto <info@nvidia.com>
---

# USD Asset Pipeline

## Purpose

Discover assets, measure bounds and shaders, swap placeholders, correct offsets, and validate headless shader compatibility before rendering.

## Prerequisites

- Built Isaac Sim (`$ISAAC_SIM_DIR` or `_build/linux-x86_64/release`).
- NVIDIA GPU with a current driver (`nvidia-smi`).
- Shell env contract from `isaac-sim-orchestrator`: `$ISAAC_SIM_DIR`, `$ISAAC_LAB_DIR`, `$WORKSPACE_DIR`.

## Limitations

- Targets Isaac Sim 6 / Kit 110 unless a section states otherwise.
- Does not replace official NVIDIA documentation for unsupported edge cases.

## Troubleshooting

| Error / symptom | Cause | Solution |
|---|---|---|
| Extension or import not found | Wrong `$ISAAC_SIM_DIR` or stale build | Point env vars at `_build/linux-x86_64/release` or rebuild |
| Black or empty frames | Missing lights or non-RTX render mode | Add dome/key light; confirm RTX / PathTracing settings |
| Hang on stage load or first render | MDL compile or oversized stage | Follow isolation steps in `isaac-sim-troubleshooting` |

## Available Scripts

| Script | Purpose | Arguments |
|---|---|---|
| `scripts/articulation_builder.py` | Helpers for building USD articulation trees with physics joints | see script --help |
| `scripts/measure_asset.py` | Measure a USD asset: bounding box, shader type, and prim count | see script --help |
| `scripts/place_assets.py` | Place USD assets at block positions with bbox-center offset correction | see script --help |

## Running scripts

From agent runtimes that expose skill execution helpers, invoke helpers with `run_script()`:

```python
run_script("scripts/articulation_builder.py", args=["--help"])
```

From a built Isaac Sim tree, run the same file with `./python.sh` (Linux) or `python.bat` (Windows) from `_build/*/release`, or execute shell helpers directly when they do not require the simulator.

## When to use

- Catalog USD assets from a folder tree (sizes, shaders, prim counts).
- Replace placeholder geometry (cubes/spheres/bboxes) with real assets.
- Build set-dressed scenes from modular libraries.
- Validate headless rendering compatibility (MDL vs UsdPreviewSurface).
- Cube-prototype to real-asset swap workflows.

## Core Concepts

### The Placeholder-to-Asset Pipeline

Real-world USD scene building follows this pattern:

1. **Prototype with cubes** — Layout spatial zones using colored UsdGeom.Cube meshes
2. **Catalog assets** — Measure every candidate USD asset (bbox, shaders, prim count)
3. **Map blocks to assets** — Match placeholder types to appropriately-sized real assets
4. **Place with offset correction** — Reference assets at block positions, correcting for asset bbox center offset
5. **Validate renders** — Vision model + domain expert scoring
6. **Iterate** — Fix overshoot, corridor intrusion, scale mismatches

### Why Bbox Offset Correction Matters

Most USD assets are NOT centered at origin. A rack asset might have its bbox center at `(46.7, 104.6, 4.5)` — if you place it at the target position `(112.0, 20.0, 0.0)` without correction, it lands 46.7m east and 104.6m north of where you want it.

**Formula:**
```
translate_x = target_x - asset_bbox_center_x
translate_y = target_y - asset_bbox_center_y
translate_z = -asset_bbox_min_z  (puts asset base on ground plane)
```

## Phase 1: Asset Discovery & Measurement

### Script Pattern (Kit Python — No Renderer Needed)

`measure_asset(path)` — open a USD file, compute bbox, detect UsdPreviewSurface shader, count prims.  Returns dict with width/depth/height/center/mpu/prims/dual_shader/shader_tag.  `catalog_assets(root_dir, extensions)` — recursively find and measure all USD assets.

See [`scripts/measure_asset.py`](scripts/measure_asset.py).

### Key Learnings

- **Use `Usd.Stage.Open()` not `Sdf.Layer.FindOrOpen()`** — Sdf fails silently on binary .usd crate files, returns default mpu=1.0
- **Always check mpu** — Some assets use cm (mpu=0.01), some use meters (mpu=1.0). Scale measurements accordingly.
- **Invalid bbox (min > 1e30)** means the asset didn't compose — usually missing references or payloads
- **Kit Python** (`kit/python/bin/python3`) is fastest for measurement — no renderer startup needed
- **For bbox in Kit runtime** (SimulationApp), define a temp prim with reference, update a few frames, then compute bbox — more reliable for complex compositions

## Phase 2: Shader Compatibility Check

### The MDL Problem on Headless arm64

| Shader Type | Headless SimulationApp | Isaac Sim GUI | OVRTX |
|---|---|---|---|
| UsdPreviewSurface only | renders | renders | renders |
| MDL + UsdPreviewSurface (dual) | falls back to Preview | uses MDL | uses MDL |
| MDL only (sourceAsset) | **black** | renders | renders |
| No materials | grey/invisible | grey | grey |

**Rule:** For headless rendering pipelines, ONLY use assets with UsdPreviewSurface fallback (dual-shader) or native UsdPreviewSurface.

### Xvfb Discovery

Running Isaac Sim under `DISPLAY=:99` (Xvfb virtual framebuffer) instead of the locked real display `:0` produces non-black renders for some MDL assets. Not fully reliable but worth trying:

```bash
# Start Xvfb
Xvfb :99 -screen 0 1920x1080x24 &>/dev/null &

# Run with virtual display
DISPLAY=:99 isaac-sim.sh --exec script.py
```

### Identifying Dual-Shader Assets

Look for these patterns in USD:
- `info:id = "UsdPreviewSurface"` on any Shader prim → headless-safe
- `info:mdl:sourceAsset` without UsdPreviewSurface sibling → MDL-only, headless-unsafe
- Lightspeed-processed assets typically = MDL-only
- "Collected" Dematic assets often = dual-shader

## Phase 3: Placeholder-to-Asset Mapping

### Strategy

1. Group placeholder cubes by name prefix (e.g., `CvL001`→`CvL`, `BRk045`→`BRk`)
2. For each prefix, find the best-fit asset by:
   - Similar function (racks→rack assets, conveyors→conveyor assets)
   - Compatible size (asset shouldn't massively overshoot the placeholder zone)
   - Dual-shader compatibility (headless rendering requirement)
3. Document the mapping table before building

### Mapping Table Format

```
| Block Prefix | Count | Placeholder Size | Asset | Asset Size | Shader | Notes |
|---|---|---|---|---|---|---|
| BRk | 352 | 3.5×1.2×5.0m | ASRS_Racks_Center | 2.36×2.82×8.29m | dual | Per-block, no scaling |
| CvL | 190 | 2.0×4.0×1.5m | Conveyor_09 | 0.94×6.72×1.66m | dual | Roller conveyor |
```

### Size Philosophy

**Use natural asset sizes, NOT scaled-to-cube.** Scaling assets to match cube dimensions destroys visual density and realism. Place at the block's XY position with the asset's natural dimensions.

Exception: If an asset is dramatically larger than its zone (e.g., 83m assembly in a 20m zone), use smaller modular pieces instead.

## Phase 4: Placement Script Pattern

### SimulationApp Runtime Placement

`get_asset_bbox(stage, asset_path, app)` — reference asset temporarily to get accurate bbox center. `collect_blocks(stage)` — group visible Cube prims by name prefix with world positions. `place_assets(stage, blocks, asset_map, asset_bboxes, module_name)` — place assets at block positions with bbox-center offset correction; hides original cubes.

See [`scripts/place_assets.py`](scripts/place_assets.py).

### Hierarchy Convention

```
/World/
  Module1/          # Racks
    VNA/
      VNA001        # Individual asset reference
      VNA002
    BRk/
      BRk001
  Module2/          # Conveyors, sorters
    CvL/
      CvL001
    Sort/
      Sort001
  Module3/          # Safety, humans
```

## Phase 5: Animation & Articulation

### Animation Baking

Use `bake_waypoints()` from `spatial-reasoning` skill to animate robots, humans, or objects along paths.

`bake_waypoints(xform_op, waypoints, speed_mps, fps, mpu)` — keyframe an XformOp along a list of (x,y,z) waypoints. Defined in the `spatial-reasoning` skill at [`../spatial-reasoning/scripts/spatial.py`](../spatial-reasoning/scripts/spatial.py).

### Articulation Builder

For robot USD:

`build_forklift_articulation(output_path)` — create chassis + fixed-joint mast + prismatic lift joint; includes `_create_box`, `_create_fixed_joint`, `_create_prismatic_joint` helpers.

See [`scripts/articulation_builder.py`](scripts/articulation_builder.py).

### UV Mapping & Materials

For textures:
- Use `sphereUV` mapping for global assets
- Use `linear` for flat planes (floors, walls)
- Always use `UsdPreviewSurface` as fallback
- Never use MDL-only materials for headless

## Phase 6: Validation

### Color Key for Placeholder Flow
| Color | Value | Represents |
|-------|-------|------------|
| Red | (1.0, 0.2, 0.2) | Failed validation
| Yellow | (1.0, 1.0, 0.2) | Warning (near overlap)
| Green | (0.2, 1.0, 0.2) | Valid, ready to replace with asset

### Format Validation Checklist
- [ ] All assets have `.usd`, `.usda`, or `.usdc` extension
- [ ] All paths use forward slashes `/`
- [ ] No `..` relative paths
- [ ] `mpu` = 1.0 for all assets (script validates scale)
- [ ] No `./` or `../` syntax in references
- [ ] Topology: artifacts must not be nested under `Xform` if empty

### Asset Recommendation (Based on Scan)

First, scan all assets with `catalog_assets()`, then:

- Filter: `dual_shader` is True
- Sort: `prims < 5000`
- Assign: Match bbox dimensions within 20% tolerance of placeholder cube
- Reject: All `blockpallet_a*`, `palletstack_a*` — these are not real assets

## Hard-Won Lessons

1. **Never scale assets to match cube dimensions** — destroys visual density. Use natural sizes.
2. **Large assemblies (>20m) rarely fit block clusters** — use smaller modular pieces instead.
3. **Always correct for bbox center offset** — most assets aren't origin-centered.
4. **Lightspeed-processed assets = MDL-only = BLACK on headless arm64.** Only use "Collected" dual-shader variants.
5. **Tote_01, Pallet_Pile, AMR_Table = MDL-only** — they'll place but render black. Use Scissor_Lift or Electrical_Panel as functional placeholders.
6. **Kill ALL kit processes before new Isaac Sim launch** — zombie processes cause 90-170s cold starts.
7. **Clean `/dev/shm/carb-*`** between restarts to prevent SIGKILL.
8. **SimulationApp headless requires explicit DomeLight + DistantLight** — GUI adds viewport lights automatically, headless does NOT.
9. **Xvfb (DISPLAY=:99)** can improve MDL rendering over locked desktop (:0), but not a universal fix.
