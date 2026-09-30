---
name: isaac-sim-rendering
description: "Headless RT2/PathTracing production rendering with ACES tuning. Use when capturing frames or validating render quality."
license: Apache-2.0
metadata:
  author: Renato Gasoto <info@nvidia.com>
---

# Isaac Sim Headless Rendering (Kit 110 / Isaac Sim 6.0+)

## Purpose

Capture production-quality headless frames with RT2 or PathTracing, ACES tone mapping, warehouse lighting patterns, and quantitative validation thresholds.

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

Capture pipeline, lighting recipes, ACES calibration, camera math, validation. Host-agnostic; adapt paths to your environment.

## Available Scripts

| Script | Purpose | Arguments |
|---|---|---|
| `scripts/capture_pipeline.py` | Standard Kit 110 / Isaac Sim 6.0+ headless capture pipeline | see script --help |
| `scripts/look_at_camera.py` | Look-at camera math for USD cameras (Z-up, USD -Z forward convention) | see script --help |
| `scripts/warehouse_lighting.py` | Multi-layer warehouse lighting recipes for headless Isaac Sim rendering | see script --help |

## Running scripts

From agent runtimes that expose skill execution helpers, invoke helpers with `run_script()`:

```python
run_script("scripts/capture_pipeline.py", args=["--help"])
```

From a built Isaac Sim tree, run the same file with `./python.sh` (Linux) or `python.bat` (Windows) from `_build/*/release`, or execute shell helpers directly when they do not require the simulator.

## Read first

- `navigation-primitives`: look-at chase camera math (cross-referenced).

## Capture: SimulationApp + Replicator RGB Annotator

Standard Kit 110 / Isaac Sim 6.0+ capture pipeline. Works headless including on ARM64 / GB10 Spark (the older 5.1.0 black-frame bug is resolved).

`setup_capture_pipeline(stage_path, width, height, renderer, settle_frames)` — open stage, define camera, attach RGB annotator, settle, return `(app, rgb_annot, render_product)`. `capture_frame(rgb_annot)` — step replicator and return `(H, W, 3)` uint8 RGB array.

See [`scripts/capture_pipeline.py`](scripts/capture_pipeline.py).

For live controller demos where simulation remains the time authority, disable Replicator capture-on-play and capture snapshots without pausing the timeline. See [`scripts/capture_pipeline.py`](scripts/capture_pipeline.py) for the executable pattern.

Inside a Python-server file that is otherwise synchronous, avoid the sync
`rep.orchestrator.step()` call in live Kit. Schedule `step_async(...)` and pump
`app_utils.update_app()` until the task completes; this prevents event-loop
reentrancy failures while still advancing the render product.

**Capture method choice**:
- `omni.replicator.core` RGB annotator -> reliable, supports any resolution.
- `RtxCamera` + `CameraSensor` (from `isaacsim.sensors.experimental.rtx`) for tick-rate control, OpenCV / fisheye lens distortion, ISP, tiled multi-view, or stereo depth (see `isaac-camera`).
- Swapchain capture -> also works on Kit 110 if you explicitly set window size matching the render resolution.
- Replicator render products may return empty arrays for Gaussian splat scenes; fall back to swapchain capture in that case.

## RT2 vs PathTracing

```python
settings.set("/rtx/rendermode", "RayTracedLighting")  # RT2 — real-time
# settings.set("/rtx/rendermode", "PathTracing")      # offline only
```

| Mode | Convergence | Per-frame time | Use for |
|---|---|---|---|
| **RayTracedLighting (RT2)** | ~200 settle frames (~10-15s) | 10-15s | All iterative work, warehouse scenes, training data |
| **PathTracing** | converges over many subframes | 5-30 min | Final hero shots only, when explicitly requested |

**Default to RT2.** Switch to PathTracing only after RT2 has been calibrated and the user asks for hero quality.

## Headless Lighting — Add Explicit Lights

Headless Isaac Sim has **NO default lighting**. Without explicit lights, frames are black (RGB=0). Always inject at least a `DomeLight` + `DistantLight` baseline.

```python
from pxr import UsdLux, UsdGeom, Gf

dome = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
dome.GetIntensityAttr().Set(400.0)

sun = UsdLux.DistantLight.Define(stage, "/World/Sun")
sun.GetIntensityAttr().Set(1500.0)
UsdGeom.Xformable(sun.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(-50, 20, 0))
```

### Baseline Intensity Guide

| Scene Type | DomeLight | DistantLight | Notes |
|---|---|---|---|
| Warehouse (default) | 400 | 1500 | Good general balance |
| Close-up robot | 300 | 1200 | Slightly softer |
| Outdoor | 500 | 2000 | Brighter sun |
| Dark/moody | 100 | 800 | Dramatic shadows |

## ACES Tone Mapping — The Single Biggest Quality Lever

Without ACES, no amount of intensity tuning produces balanced indoor renders. This is the single most impactful render setting after lighting.

```python
import carb
s = carb.settings.get_settings()

s.set("/rtx/post/tonemap/op", 4)            # ACES
s.set("/rtx/post/tonemap/filmIso", 600.0)   # key parameter (see table)
s.set("/rtx/post/tonemap/whitepoint", 6500.0)
s.set("/rtx/post/tonemap/enabled", True)
s.set("/rtx/post/aa/op", 3)                 # TAA for RT2
```

### filmIso Calibration (validated on warehouse interiors)

| Scene | filmIso | Notes |
|---|---|---|
| General warehouse RT2 | 200 | Photorealistic starting point |
| Deep-aisle indoor (hero camera) | 600 | Best balance across hero/overview/aisle/topdown |
| Aerial/overview-heavy | 400 | Avoid overexposure on open views |

### Anti-Recipes (don't waste time on these)
- Wide rect lights (width=5+) → flat, no light pools
- High dome intensity (400+) with ACES filmIso 600 → washes out shadows
- Reinhard tonemapping → muddy, low contrast
- PathTracing for iterative work → 5-30 min per frame, kills velocity

## Warehouse Lighting Recipe (proven 7/10 → 9/10)

The biggest single quality improvement came from this lighting + fog recipe.

`add_warehouse_lighting(stage, n_lights, settings)` — low-ambient dome + focused rect lights + optional fog. Pass `settings=carb.settings.get_settings()` to enable fog.

See [`scripts/warehouse_lighting.py`](scripts/warehouse_lighting.py).

For 40m warehouse: fog density 0.003 adds depth without murk.

## Deep-Aisle Indoor Lighting

### Problem
Ground-level camera in narrow aisle = black frame (82KB / mean_RGB < 5). Ceiling rect lights at Z=10m can't illuminate a 3.5m-wide × 8m-tall aisle to ground level — RT2 struggles with deep occlusion.

### Solution: Multi-Layer Lighting
```python
# Layer 1: dense ceiling grid (6×12 across facility)
# Rect lights at ceil_z-0.3, pointing down
# intensity=200000, width=4.0, height=3.0  (wide coverage)

# Layer 2: low sphere lights IN each aisle at Z=3.5m (head-height)
# Directly in camera FOV between tier 1 and ground
for aisle_y, lx in aisle_light_positions:
    lt = UsdLux.SphereLight.Define(stage, lp)
    lt.GetRadiusAttr().Set(0.15)
    lt.GetIntensityAttr().Set(100000.0)
```

- **500 settle frames** for indoor aisle scenes (not 200-300)
- Dome at 300 intensity is optional ambient fill — don't go higher or open views wash out

### Dome vs Deep-Aisle Tension (fundamental conflict in enclosed scenes)
- High dome → overview/topdown overexpose (mean > 220)
- Low/no dome → deep aisle underexpose (mean < 10)
- **Best balance**: no dome + sphere lights in aisles + 500K rect grids + 500 settle frames
  - Hero aisle: mean ~60
  - Overview (elevated 3/4): mean ~140-175
  - Cross-aisle: mean ~230

### Validated ACES filmIso=600 Light Intensities
- Ceiling rect lights: **70,000** intensity, 2.5×1.5m, warm white (1.0, 0.97, 0.92)
- Aisle sphere lights: **15,000** intensity, radius=0.1, at Z=3.5m
- Grid: 8×14 ceiling panels
- **No dome light** — ACES handles exposure
- Result: mean 60–155 across all view types

**Camera tip**: place "hero" camera at cross-aisle intersections, not deep in narrow aisles. The junction has more open space for light to reach.

## Frame Quality Validation

Always validate captured frames before delivery. Don't ship black/overexposed frames.

| Indicator | Meaning | Action |
|---|---|---|
| File ~82KB | Black frame (RGBA padding only) | Add explicit lights |
| File 200–500KB | Partial render / very simple scene | Check settle frames |
| File 1–2MB | Full rendered frame | OK |
| `rgb.max() == 0` | No lighting reaching camera | Add `DomeLight + DistantLight` |
| `rgb.max() > 200`, mean 60–180 | Good render | OK |
| `rgb.mean() > 220` | Overexposed | Reduce light intensity or filmIso |
| `rgb.mean() < 10` | Underexposed | Add aisle-level lights or raise filmIso |

```python
import numpy as np
def validate_frame(rgb_array):
    """Returns (ok: bool, reason: str)."""
    if rgb_array.max() == 0:
        return False, "no light reaches camera — add DomeLight + DistantLight"
    if rgb_array.mean() > 220:
        return False, f"overexposed (mean={rgb_array.mean():.0f}) — reduce intensity"
    if rgb_array.mean() < 10:
        return False, f"underexposed (mean={rgb_array.mean():.0f}) — add aisle lights"
    return True, f"ok (mean={rgb_array.mean():.0f}, max={rgb_array.max()})"
```

## Look-At Camera Math

For chase/POV/overview cameras pointing at a target, always use a look-at matrix. Don't hand-tune Euler angles — they're brittle and you'll waste hours on sign flips.

`look_at_matrix(eye, target, up)` — returns `Gf.Matrix4d` for a USD camera at `eye` looking at `target`. Handles degenerate up-vector (straight down/up).

See [`scripts/look_at_camera.py`](scripts/look_at_camera.py).

### Third-Person Camera Offsets (Z-up, robot facing +X at yaw=0)

| Direction | Vector |
|---|---|
| Behind robot | `-X` |
| Right of robot | `-Y` |
| Left of robot | `+Y` |
| Above robot | `+Z` |

```python
import math

behind_dir_x = -math.cos(yaw)
behind_dir_y = -math.sin(yaw)
right_dir_x  = -math.sin(yaw)
right_dir_y  =  math.cos(yaw)

cam_x = robot_x + behind_dist * behind_dir_x + side_offset * right_dir_x
cam_y = robot_y + behind_dist * behind_dir_y + side_offset * right_dir_y
cam_z = height
```

- `side_offset = -2.5` → camera on robot's **right**
- `side_offset = +2.5` → camera on robot's **left**
- Flip the *offset value* to change sides, NOT the trig signs.

## Dynamic Camera Height (Obstacle Avoidance)

When tracking through cluttered environments, the chase camera will clip into tall geometry. Pre-compute obstacle bboxes, then raise the camera each frame as needed.

```python
# Build obstacle lookup from USD geometry once at startup
obstacles = []
for prim in stage.Traverse():
    if prim.IsA(UsdGeom.Cube):
        # ... extract (xmin, xmax, ymin, ymax, height) ...
        obstacles.append((xmin, xmax, ymin, ymax, height))

def cam_max_height_at(cx, cy, margin=0.5):
    """Highest obstacle near (cx, cy). Camera must clear this."""
    return max((h for xmn, xmx, ymn, ymx, h in obstacles
                if xmn-margin <= cx <= xmx+margin and ymn-margin <= cy <= ymx+margin),
               default=0.0)

# Per-frame:
target_h = max(base_height, cam_max_height_at(cam_x, cam_y) + 1.0)
smooth_h = smooth_h * 0.95 + target_h * 0.05   # smooth transitions
```

## Robot XformOp Discipline

URDF-imported robots (Spot, Carter, etc.) already have authored `translate + orient + scale` xformOps on the root prim.

- Use `xf.ClearXformOpOrder(); xf.MakeMatrixXform()` on the **root prim only** for initial placement.
- **Never** add ops to child body/link prims — physics drives those.

## Video Assembly

```bash
ffmpeg -y -framerate 30 -i frames/frame_%05d.png \
  -c:v libx264 -pix_fmt yuv420p -crf 18 output.mp4
```

Frame numbering must be **sequential** (`frame_0000.png`, `frame_0001.png`, …) — ffmpeg skips gaps.

## Session Management

For batch/iterative rendering, keep the Kit app running and switch stages in-place rather than restarting:

- Cold start = 5-7 min wasted
- Persistent session = 10-15s per render
- Use `stage_utils.open_stage(path)` (`isaacsim.core.experimental.utils.stage`) to switch scenes
- Only restart Kit if it crashes or hits OOM

Implementation is up to you (REPL, command file, IPC, etc.) — the principle is "don't pay the cold-start cost more than once."

## Checklist Before Delivering Renders

1. RT2 enabled (`/rtx/rendermode = RayTracedLighting`)
2. ACES tone mapping enabled (`/rtx/post/tonemap/op = 4`)
3. filmIso calibrated for scene type (200 general / 400 aerial-heavy / 600 deep-aisle)
4. Explicit `DomeLight + DistantLight` (or scene-specific multi-layer setup)
5. Settle frames sufficient (200 standard / 500 deep-aisle)
6. Frame validation passed (`rgb.mean()` in 30-200 range, file size > 200KB)
7. Frame sequence is gapless for ffmpeg

## Integration Points

- **RECEIVES from:** `urdf-mjcf-to-usd-conversion`, `usd-articulation`, `mobility-gen`, `isaac-sim-robot-navigation` — populated stages to render
- **PRODUCES for:** `data-collection-sim` — validated frame sequences for SDG
- **PRODUCES for:** `isaac-sim-validator` — outputs for final QA gate
