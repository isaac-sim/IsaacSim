---
name: calibrate-metropolis-camera
description: "Extract camera intrinsics, extrinsics, and FOV polygons to calibration.json. Use after cameras are placed; to place them first use place-camera-max-coverage."
license: Apache-2.0
metadata:
  author: NVIDIA Isaac Sim <isaac-sim@nvidia.com>
---

# Calibrate Placed Cameras

## Purpose

Produce a Metropolis-format `calibration.json` — per-camera intrinsics, extrinsics, homography, and field-of-view polygons — plus a top-view reference image, using `CameraCalibrationManager` from `isaacsim.sensors.rtx.calibration`.

## Prerequisites

- Isaac Sim 6 / Kit 110 running with the Python server (`isaacsim.code_editor.python_server`, port 8226) — see `isaac-sim-remote`.
- `isaacsim.sensors.rtx.calibration` resolvable. It is pinned by `isaacsim.exp.action_and_event_data_generation.base.kit`; launch that app, or `--enable isaacsim.sensors.rtx.calibration`.
- **Cameras already placed** under the camera parent prim (default `/World/Cameras`) — use `place-camera-max-coverage` or `place-camera-aim-at` first.
- A local output folder. Nucleus / cloud URIs are rejected.
- Shell env contract from `isaac-sim-orchestrator`: `$ISAAC_SIM_DIR`, `$ISAAC_LAB_DIR`, `$WORKSPACE_DIR`.

## Limitations

- **This repo does not build the extension.** `isaacsim.sensors.rtx.calibration` is authored in the `metrosim` repo and consumed here as an exact registry pin declared in `source/apps/isaacsim.exp.action_and_event_data_generation.base.kit` (read the version there rather than from this skill). This skill drives it; API changes belong upstream.
- Needs a live Kit session — the top view is a real orthographic render, so there is no offline path.
- Every camera must fall inside the top-view frustum. One camera outside it aborts the whole calibration.
- The top-view camera path is stored in a **non-persistent** setting, so the top-view stage must be re-run each session before calibration.
- Calibration describes cameras as they are *now*. Moving or re-placing a camera invalidates the file; re-run.

## Troubleshooting

| Error / symptom | Cause | Solution |
|---|---|---|
| `CALIBRATE_PREFLIGHT: ext=unavailable` | App does not resolve the extension | Launch `isaacsim.exp.action_and_event_data_generation.base.sh`, or add `--enable isaacsim.sensors.rtx.calibration` |
| `cameras=0` in preflight | Nothing under the camera parent prim | Place cameras first, or pass `camera_parent_prim_path` |
| `place_info=invalid` | No usable `key=value` pair | Pass e.g. `place_info=room=warehouse`; pairs are joined with `/` |
| `scene_root=missing` | `scene_root` does not resolve | Point it at a prim that bounds the area, usually `/World` |
| "camera … is not covered by top view!" | A camera sits outside the top-view frustum | Widen `scene_root` to a prim whose bbox contains every camera, or remove the stray camera |
| `calibration.json` not written | Pipeline aborted mid-run | Read the Isaac Sim console; the abort reason is logged via `carb.log_error` |
| Calibration silently does nothing | Called the sync `generate_calibration()` | It is fire-and-forget; `await generate_calibration_async()` instead |
| `if is_ready_to_calibrate():` never true | Returns `None` on success — see below | Compare `is not False`, never truthiness |

## When to use this skill

| You want | Skill |
|---|---|
| `calibration.json` for cameras that already exist | **this skill** |
| Decide *where* cameras go to cover an area | `place-camera-max-coverage` |
| Decide *where* cameras go to watch one object | `place-camera-aim-at` |
| Author one camera's intrinsics, AOVs, distortion by hand | `isaac-camera` |

## The pipeline

Four stages, strictly ordered — each one depends on the previous having finished:

1. **Top-view camera.** An orthographic camera framed on `scene_root`. Sets `top_view_camera_path`, which every later stage validates.
2. **Calibration dots.** `dot_count` reference points per camera under `/World/Calibration_Dots`, used to solve the world↔image mapping.
3. **Calibration file.** Per-camera intrinsics, extrinsics, homography, and FOV polygons → `calibration.json`.
4. **Top-view image.** Orthographic render → `Top.png` plus `imageMetadata.json`.

Parameter semantics and settings are in
[`references/calibration-api.md`](references/calibration-api.md); the output
schema and async/ordering rules are in
[`references/calibration-runtime-notes.md`](references/calibration-runtime-notes.md).

## Available Scripts

| Script | Purpose | Arguments |
|---|---|---|
| `scripts/calibrate_camera.py` | Extract intrinsics, extrinsics, and FOV polygons for placed cameras | injected `key=value` args (see script docstring) |

## Running scripts

The script is a python_server payload, not a standalone program. From agent
runtimes that expose skill execution helpers, invoke it with `run_script()`:

```python
run_script("scripts/calibrate_camera.py", args=["action=preflight"])
```

From a built tree, send it through the `isaac-sim-remote` client:

```bash
python skills/isaac-sim-remote/scripts/isaacsim_send.py \
  --file skills/calibrate-metropolis-camera/scripts/calibrate_camera.py \
  --arg action=calibrate --arg output_dir=/tmp/calib
```

## Workflow

1. **Preflight.** `action=preflight` (the default) reports extension, stage,
   camera count, `place_info`, `scene_root`, and output folder without touching
   the stage. It never raises.

   ```python
   run_script("scripts/calibrate_camera.py", args=["action=preflight"])
   ```

   Expect `CALIBRATE_PREFLIGHT: ext=enabled stage=ok cameras=5 place_info=ok scene_root=ok output_dir=...`.
   A `cameras=0` here is the usual blocker — place cameras first.

2. **Calibrate.** Runs all four stages and verifies the files landed.

   ```python
   run_script("scripts/calibrate_camera.py", args=["action=calibrate", "output_dir=/tmp/calib"])
   ```

   Expect `CALIBRATE_RESULT: action=calibrate cameras=5 top_camera=... output=/tmp/calib`
   followed by a `wrote <file> (<n> bytes)` line per artifact.

3. **Verify.** Confirm `calibration.json` has one `sensors[]` entry per camera and
   that `Top.png` is not black — a black top view means the orthographic camera
   framed empty space, and the homographies derived from it are meaningless.

4. **Re-run after any camera moves.** The file is a snapshot of the current poses.

Use `action=topview` to refresh only the top-view camera and image without
redoing the dots and calibration.

## Calling the API directly

```python
from isaacsim.sensors.rtx.calibration import CameraCalibrationManager, CameraCalibrationSettings

CameraCalibrationSettings.camera_calibration_output_folder_path = "/tmp/calib"
CameraCalibrationSettings.place_info = "room=warehouse"
CameraCalibrationSettings.scene_bounding_box_path = "/World"

manager = CameraCalibrationManager.get_instance()
manager.create_top_view_camera(root_prim_path="/World")
await manager.generate_calibration_dot_prim_async()

# NOT `if manager.is_ready_to_calibrate():` -- see below.
if manager.is_ready_to_calibrate() is False:
    raise RuntimeError("stage is not ready; see the carb log")

await manager.generate_calibration_async()
await manager.capture_and_process_camera_view_image()
```

Three traps worth stating up front:

- **`is_ready_to_calibrate()` returns `None` on success.** It has three
  `return False` branches and no `return True`, so it falls off the end. Testing
  it for truthiness means the guard never passes. Compare `is not False`.
- **The sync entry points are fire-and-forget.** `generate_calibration()` wraps
  `asyncio.ensure_future(...)` and returns immediately, so the next stage races
  it. Await the `_async` variants; python_server payloads support top-level
  `await`.
- **`top_view_camera_path` is non-persistent.** It is only set by
  `create_top_view_camera()`, so stage 1 must run every session even if the
  top-view camera prim still exists on the stage.

## Related skills

- `place-camera-max-coverage` / `place-camera-aim-at` — produce the cameras this skill calibrates.
- `isaac-sim-remote` — the transport this skill's script rides on.
- `isaac-camera` — per-camera intrinsics, render products, AOVs, distortion.
- `isaac-sim-sensor` — attaching annotators and writers once cameras exist.
