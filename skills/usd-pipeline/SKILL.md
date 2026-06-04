---
name: usd-pipeline
description: >
  USD asset discovery, measurement, placement, and validation in Omniverse.
  Asset cataloging (bbox, shaders, prim count), placeholder-to-asset swap,
  offset correction, shader compatibility for headless renders, animation
  baking, articulation, and USD composition architecture.
---

# USD Asset Pipeline

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

```python
from pxr import Usd, UsdGeom, UsdShade
import os

def measure_asset(path):
    """Measure a USD asset: bbox, shader type, prim count."""
    if not os.path.exists(path):
        return None

    stage = Usd.Stage.Open(path)
    dp = stage.GetDefaultPrim()
    if not dp:
        children = list(stage.GetPseudoRoot().GetChildren())
        dp = children[0] if children else None
    if not dp:
        return {"error": "no default prim"}

    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    r = bc.ComputeWorldBound(dp).ComputeAlignedRange()
    mn, mx = r.GetMin(), r.GetMax()

    if mn[0] > 1e30:
        return {"error": "invalid bbox"}

    mpu = UsdGeom.GetStageMetersPerUnit(stage)
    w, d, h = (mx[0]-mn[0])*mpu, (mx[1]-mn[1])*mpu, (mx[2]-mn[2])*mpu
    cx = (mn[0]+mx[0])/2
    cy = (mn[1]+mx[1])/2
    cz = mn[2]  # base of asset

    # Check for UsdPreviewSurface (headless-compatible)
    has_preview_surface = False
    for p in stage.Traverse():
        if p.IsA(UsdShade.Shader):
            sid = p.GetAttribute("info:id")
            if sid and sid.Get() and "Preview" in str(sid.Get()):
                has_preview_surface = True
                break

    prims = sum(1 for _ in stage.Traverse())

    return {
        "width": w, "depth": d, "height": h,
        "center": (cx, cy, cz),
        "mpu": mpu,
        "prims": prims,
        "dual_shader": has_preview_surface,  # True = renders everywhere
        "shader_tag": "dual" if has_preview_surface else "MDL-only"
    }
```

### Batch Discovery Pattern

```python
import glob

def catalog_assets(root_dir, extensions=(".usd", ".usda", ".usdc")):
    """Recursively find and measure all USD assets in a directory tree."""
    results = {}
    for ext in extensions:
        for path in glob.glob(f"{root_dir}/**/*{ext}", recursive=True):
            info = measure_asset(path)
            if info and "error" not in info:
                name = os.path.basename(path).replace(".usd", "").replace(".usda", "")
                results[name] = {**info, "path": path}
    return results
```

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

```python
from isaacsim import SimulationApp
app = SimulationApp({"headless": True, "width": 1920, "height": 1080,
                      "renderer": "RayTracedLighting"})

import omni.usd
from pxr import Gf, UsdGeom, Usd
from collections import defaultdict

stage = omni.usd.get_context().get_stage()

# 1. Compute asset bbox in Kit runtime (more reliable than offline)
def get_asset_bbox(stage, asset_path, app):
    """Reference asset temporarily to get accurate bbox."""
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    name = os.path.basename(asset_path).replace(".", "_")
    test = stage.DefinePrim(f"/BBoxTest_{name}", "Xform")
    test.GetReferences().AddReference(asset_path)
    for _ in range(5): app.update()
    r = bc.ComputeWorldBound(test).ComputeAlignedRange()
    mn, mx = r.GetMin(), r.GetMax()
    stage.RemovePrim(f"/BBoxTest_{name}")
    if mn[0] > 1e30:
        return None
    return {"cx": (mn[0]+mx[0])/2, "cy": (mn[1]+mx[1])/2, "cz": mn[2]}

# 2. Collect visible cube blocks by prefix
def collect_blocks(stage):
    xf_cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    blocks = defaultdict(list)
    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Cube): continue
        img = UsdGeom.Imageable(prim)
        if img.ComputeVisibility(Usd.TimeCode.Default()) == "invisible": continue
        name = prim.GetName()
        prefix = ""
        for c in name:
            if c.isdigit(): break
            prefix += c
        mtx = xf_cache.GetLocalToWorldTransform(prim)
        pos = mtx.ExtractTranslation()
        blocks[prefix].append({
            "name": name, "path": str(prim.GetPath()),
            "x": pos[0], "y": pos[1], "z": pos[2]
        })
    return blocks

# 3. Place assets with offset correction
def place_assets(stage, blocks, asset_map, asset_bboxes, module_name):
    root = stage.DefinePrim(f"/World/{module_name}", "Xform")
    placed = 0

    for prefix, asset_path in asset_map.items():
        block_list = blocks.get(prefix, [])
        if not block_list: continue
        bb = asset_bboxes.get(asset_path)
        if not bb: continue

        group = stage.DefinePrim(f"/World/{module_name}/{prefix}", "Xform")

        for b in block_list:
            prim = stage.DefinePrim(
                f"/World/{module_name}/{prefix}/{b['name']}", "Xform")
            xf = UsdGeom.Xformable(prim)
            # OFFSET CORRECTION — the key insight
            tx = b["x"] - bb["cx"]
            ty = b["y"] - bb["cy"]
            tz = -bb["cz"]  # ground the asset
            xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(
                Gf.Vec3d(tx, ty, tz))
            prim.GetReferences().AddReference(asset_path)
            placed += 1

        # Hide original cubes
        for b in block_list:
            orig = stage.GetPrimAtPath(b["path"])
            if orig:
                UsdGeom.Imageable(orig).MakeInvisible()

    return placed
```

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

```python
# In spatial-reasoning skill:
def bake_waypoints(xform_op, waypoints, speed_mps, fps=30, mpu=1.0):
    """Create keyframed animation for XformOp along a list of (x,y,z) points."""
    time_count = len(waypoints)
    time_samples = [i/fps for i in range(time_count)]

    for i, wp in enumerate(waypoints):
        t = time_samples[i]
        xform_op.Set(Gf.Vec3d(wp[0]*mpu, wp[1]*mpu, wp[2]*mpu), t)
```

### Articulation Builder

For robot USD:

```python
from pxr import Usd, UsdGeom, UsdPhysics, Gf, Sdf

# Creates a nested articulation tree with joints
stage = Usd.Stage.CreateNew("robot.usd")

root = UsdGeom.Xform.Define(stage, "/Robot")
UsdPhysics.ArticulationRootAPI.Apply(root.GetPrim())

# Base link
base = _create_box(stage, "/Robot/base", size=(2.5, 1.2, 0.4), mass=2000.0)
UsdPhysics.RigidBodyAPI.Apply(base)

# Mast base (fixed to chassis front)
mast_base = _create_box(stage, "/Robot/base/MastBase", size=(0.1, 1.0, 2.0), mass=200.0)
UsdPhysics.RigidBodyAPI.Apply(mast_base)
_create_fixed_joint(stage, "/Robot/base/MastBase/FixedJoint",
        body0="/Robot/base", body1="/Robot/base/MastBase",
        local_pos=Gf.Vec3f(1.3, 0, 0.3))

# Inner Mast (prismatic lift)
inner_mast = _create_box(stage, "/Robot/base/MastBase/InnerMast", size=(0.08, 0.9, 1.8), mass=100.0)
UsdPhysics.RigidBodyAPI.Apply(inner_mast)
_create_prismatic_joint(
    stage, "/Robot/base/MastBase/InnerMast/LiftJoint",
    body0="/Robot/base/MastBase", body1="/Robot/base/MastBase/InnerMast",
    axis="Z", lower_limit=0.0, upper_limit=3.0,
    drive_stiffness=1e6, drive_damping=1e4)
```

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
