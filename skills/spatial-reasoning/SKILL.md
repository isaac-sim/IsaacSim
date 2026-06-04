---
name: spatial-reasoning
description: >
  3D spatial reasoning for USD scene composition. Coordinate math,
  metersPerUnit conversion, bounding-box analysis, placement transforms
  (T*R*S ordering), look-at, collision-free grid layouts, zone boundaries,
  quaternion rotation, spatial indexing (R-tree/grid), SAT/GJK,
  A* + smoothing, Dubins, 2D bin packing, frustum culling, coordinate
  system conversions (USD/Unity/Unreal), warehouse standards (OSHA/NFPA/RMI),
  ABC analysis, numerical stability.
---

# 3D Spatial Reasoning

Core spatial math for placing objects correctly in OpenUSD scenes.

## Coordinate System

- Stage: Z-up, metersPerUnit=1.0 (meters)
- All placement coordinates are in meters
- Rotations: always `AddRotateXYZOp()` — never individual axis ops

## metersPerUnit Conversion (CRITICAL)

Assets have their own metersPerUnit. Common values:
- `1.0` = meters (robots, environments)
- `0.01` = centimeters (most KION lightspeed assets, SimModelAnimation, compositions)

**Rule:** Before referencing any asset, read its metersPerUnit:
```python
asset_stage = Usd.Stage.Open(asset_path)  # Must use Usd.Stage, NOT Sdf.Layer
mpu = UsdGeom.GetStageMetersPerUnit(asset_stage)
```
`Sdf.Layer.FindOrOpen()` FAILS SILENTLY on binary .usd crate files — always returns None. Never use it for mpu detection.

**Getting real-world size of an asset:**
```python
bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
raw_range = bbox_cache.ComputeWorldBound(default_prim).ComputeAlignedRange()
raw_size = raw_range.GetMax() - raw_range.GetMin()
real_size_meters = [raw_size[i] * mpu for i in range(3)]
```

## Transform Matrix Order (CRITICAL)

When placing an asset with scale + rotation + translation:
```python
# CORRECT: Translate * Rotate * Scale
# Scale shrinks asset to meters, Rotate orients it, Translate positions it
scale_mat = Gf.Matrix4d().SetScale(Gf.Vec3d(mpu, mpu, mpu))
rot_mat = Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(0,0,1), heading_deg))
trans_mat = Gf.Matrix4d().SetTranslate(Gf.Vec3d(x, y, z))
xf.MakeMatrixXform().Set(trans_mat * rot_mat * scale_mat)
```

**WRONG:** `scale * translate` scales the translation vector too — a 3m offset becomes 0.03m for mpu=0.01.

## Placement Helper

```python
def place(stage, prim_path, asset_path, x, y, z=0, rot_z=0, mpu=0.01):
    """Place an asset with correct metersPerUnit scaling."""
    prim = stage.DefinePrim(prim_path, "Xform")
    prim.GetReferences().AddReference(asset_path)
    xf = UsdGeom.Xformable(prim)
    S = Gf.Matrix4d().SetScale(Gf.Vec3d(mpu, mpu, mpu))
    R = Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(0,0,1), rot_z))
    T = Gf.Matrix4d().SetTranslate(Gf.Vec3d(x, y, z))
    xf.MakeMatrixXform().Set(T * R * S)
    return prim
```

## Look-At Camera Math

```python
def look_at_rotation(cam_pos, target_pos):
    """Compute XYZ Euler rotation for camera to look at target. Z-up stage."""
    dx = target_pos[0] - cam_pos[0]
    dy = target_pos[1] - cam_pos[1]
    dz = target_pos[2] - cam_pos[2]
    horiz = math.sqrt(dx*dx + dy*dy)
    pitch = math.degrees(math.atan2(horiz, -dz))
    yaw = math.degrees(math.atan2(dx, -dy))
    return Gf.Vec3f(pitch, 0, yaw)
```

## Grid Layout

```python
def grid_positions(n_items, spacing, origin=(0,0)):
    """Generate grid positions for n items with given spacing."""
    cols = min(8, math.ceil(math.sqrt(n_items * 1.5)))
    positions = []
    for i in range(n_items):
        row, col = divmod(i, cols)
        x = origin[0] + col * spacing
        y = origin[1] + row * spacing
        positions.append((x, y))
    return positions, cols
```

## Baked Waypoint Animation

_See `bake_waypoints()` in [`scripts/spatial.py`](scripts/spatial.py) (28 lines)._


## Zone Boundary Check

```python
def in_zone(x, y, zone):
    """Check if (x,y) is within a zone dict with 'x':[min,max], 'y':[min,max]."""
    return zone['x'][0] <= x <= zone['x'][1] and zone['y'][0] <= y <= zone['y'][1]
```

## Shell vs Interior Coordinate Alignment (CRITICAL)

The KION mega warehouse shell at mpu=0.01 is ~170×100m in stage meters. When placing interior cubes/assets at specific zone coordinates (e.g., racks at X=30-60), the camera must be positioned WITHIN the layout zone, not at the building origin.

**Hero camera rule:** Place camera at `(zone_start_x + 3, zone_center_y, 2.5)` looking INTO the zone — never at `(0, y, z)` which will be inside the shell wall geometry.

**Top-down camera rule:** Place at `(layout_center_x, layout_center_y, max(W,D)*0.9)` with focal 12mm. This is the money shot for layout validation — always render this first.

**Camera-to-layout match:** Before rendering, verify camera pos is inside the bounding box of your placed content, not just inside the shell.

## Render File Size Validation

| Size | Meaning |
|------|---------|
| ~82KB | Blank grey — nothing rendered |
| ~275KB | Viewport grid only — stage loaded but camera sees nothing |
| 1-2MB | Partial scene — some geometry visible |
| 2-5MB | Full scene with detail |
| Same size across all views | Camera switching FAILED — all captures from same view |

**If all renders within a stage have identical byte counts, the camera path switch didn't take effect.** Fix: create a fresh camera prim for each shot (remove + redefine), not just change the path.

## KION Asset Scale Map (CRITICAL — 2026-03-14)

The KION asset tree has MIXED units. ALWAYS validate with `Usd.Stage.Open()` + `UsdGeom.BBoxCache`:

| Asset Category | Example | Raw BBox | True Units | Scale Factor |
|---|---|---|---|---|
| Shell (sm_warehouse_mega) | 18586×14200 raw | Centimeters | **0.01** |
| GSRC module | 58.3×82.8×8.4 raw | Meters | **1.0** |
| Conveyors | 0.9×6.7×1.7 raw | Meters | **1.0** |
| PackStation | 2.9×2.6×2.9 raw | Meters | **1.0** |
| Depalletizer | 2.8×1.2×3.3 raw | Meters | **1.0** |
| PutAway Rack | 0.8×5.3×2.0 raw | Meters | **1.0** |
| Trolly | 0.7×1.6×1.3 raw | Meters | **1.0** |
| SimReady racks | 2.5×1.3×3.7 | Meters | **1.0** |
| SimReady barriers | 6.1×0.5×1.7 | Meters | **1.0** |
| claw_lighting.usd | ~17000 raw | Centimeters | **0.01** |

**Rule: ONLY Shell + Lighting need 0.01. ALL other KION assembly assets are in meters.**

Placing meter-scale assets at mpu=0.01 makes them 100× too small (invisible). This bug silently breaks scenes — assets "exist" in USD but render as sub-centimeter specs.

**Validation command:**
```python
stage = Usd.Stage.Open(asset_path)
bbox = UsdGeom.BBoxCache(0, [UsdGeom.Tokens.default_]).ComputeWorldBound(stage.GetPseudoRoot()).ComputeAlignedRange()
size = bbox.GetMax() - bbox.GetMin()
# If any dimension > 1000 → centimeters (use 0.01)
# If all dimensions < 100 → meters (use 1.0)
```

## Common Gotchas

0a. **Shell interior bounds ≠ shell bbox** — `sm_warehouse_mega` bbox is (-13.2,-5.2)→(172.6,136.8) but Corner module starts at X=0.6 and Center (floor) at X=11.6. Actual usable interior: X=3-170, Y=0-134. Always check module children bboxes, not just root. 0b. **UsdGeom.Cube extent is [-1,1]³ (size 2)** — scale by `w/2, d/2, h/2` NOT `w, d, h`. Using full dimensions makes every block 2× its intended size. This caused 14,570 phantom clashes in uber warehouse v2. 0b. **`SetTranslateOnly()` WIPES scale from Gf.Matrix4d** — if you set `mat[0][0]=0.01` then call `mat.SetTranslateOnly(...)`, the scale reverts to identity. Build the matrix with explicit 16 floats: `Gf.Matrix4d(sx,0,0,0, 0,sy,0,0, 0,0,sz,0, tx,ty,tz,1)`. This caused a 100× shell scaling bug.
1. `Sdf.Layer.FindOrOpen` fails on binary `.usd` crate files — use `Usd.Stage.Open`
2. Matrix order: `T * R * S` not `S * R * T`
3. Cameras default look along -Z in camera space = straight down in Z-up stage (no rotation needed for top-down)
4. `xformOp:translate already exists in xformOpOrder` → use `MakeMatrixXform()` instead of `AddTranslateOp()`
5. DHGen humans already have xformOps defined — must use `MakeMatrixXform()` which clears and replaces
6. Shell is ~170×100m — layout area is usually a subset. Camera must be inside the layout area, not at origin.
7. Isaac Sim `vp.camera_path` sometimes silently fails between rapid renders — always create fresh camera prim with `stage.RemovePrim()` + `UsdGeom.Camera.Define()` for each shot
8. RT2 needs 200+ settle frames for convergence. Less = noisy/incomplete renders.

## Lessons Learned — GSRC Integration (2026-03-12)

### Scaling Awareness
- When placing large equipment (50m+ footprint), the facility MUST grow proportionally
- A 58×83m module in a 120×80m warehouse = impossible overlap
- Always calculate: `equipment_area / facility_area` — if > 30%, facility needs to grow

### Collision Prevention
- Before placing a new zone, check all existing zone bounding boxes
- Zone overlaps are the #1 failure mode when scaling up quickly (V7/V8 had overlaps)
- Use coordinate-based collision detection: `if new_x_range overlaps existing_x_range AND new_y_range overlaps existing_y_range → collision`

### Variation Workflow
- Build ONE variation at a time — render — show — get feedback — iterate
- Don't batch multiple variations without showing intermediate results
- Each variation should have a distinct spatial philosophy, not just moved zones

### Occupancy Map as Validation
- Generate occupancy map after each variation as a spatial sanity check
- If free_space% < 40%, the layout is too dense for robot navigation
- If free_space% > 80%, the layout has too much wasted space
- Sweet spot for warehouses: 55-70% free space


---

## Advanced Topics

See [`advanced.md`](advanced.md) for details.
