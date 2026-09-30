---
name: place-camera-max-coverage
description: "Greedy CameraPlacementManager that fills a navmesh or XY scope until a patch-coverage ratio is met. Use for facility surveillance layouts that minimize camera count."
license: Apache-2.0
metadata:
  author: NVIDIA Isaac Sim <isaac-sim@nvidia.com>
---

# Place Cameras for Maximum Coverage

## Purpose

Lay out infrastructure cameras across a floor area so a target fraction of it is observed, using the `CameraPlacementManager` API of `isaacsim.sensors.rtx.placement`.

## Prerequisites

- Running Isaac Sim 6 / Kit 110 with python_server (port 8226); see `isaac-sim-remote`.
- Extension `isaacsim.sensors.rtx.placement` enabled (action_and_event_data_generation app, or `--enable`).
- Stage has the **facility/environment** loaded plus a baked navmesh **or** explicit `scope=xmin,xmax,ymin,ymax`.
- Shell env: `$ISAAC_SIM_DIR`, `$ISAAC_LAB_DIR`, `$WORKSPACE_DIR` (`isaac-sim-orchestrator`).

## Limitations

- Does not build `isaacsim.sensors.rtx.placement` — consume the pin from
  `source/apps/isaacsim.exp.action_and_event_data_generation.base.kit`.
- Live Kit only; coverage uses a single focus plane at `focus_height` (no multi-level model).
- Greedy solver stops at ratio / budget / `min_coverage_increase` — not a proven minimum set.
- Cost scales with area / `patch_size`² (tens of seconds on large stages).
- For single-object unoccluded views (not floor coverage), use `place-camera-aim-at`.

## Troubleshooting

| Error / symptom | Cause | Solution |
|---|---|---|
| `MAX_COVERAGE_PREFLIGHT: ext=unavailable` | App does not resolve the extension | Launch `isaacsim.exp.action_and_event_data_generation.base.sh`, or add `--enable isaacsim.sensors.rtx.placement` |
| `scope=none` in preflight | No navmesh baked and no explicit scope | Bake a navmesh, or pass `scope=xmin,xmax,ymin,ymax` |
| Script raises "No cameras were authored" | Solver produced nothing | Check the console; usually a scope that misses the walkable floor, or `camera_on_navmesh` with no navmesh |
| "input section distance … is too large" warning | `patch_size` exceeds `shorter_span / 10` | Harmless — the value is auto-shrunk; pass a smaller `patch_size` to control it |
| Far fewer cameras than expected | Coverage target met early, or `min_coverage_increase` stopped the loop | Raise `target_coverage_ratio`, or raise `required_camera_per_patch` for redundancy |
| Placement never finishes on a large stage | `patch_size` too small for the area | Raise `patch_size`; cost scales with area / `patch_size`² |
| `camera_info_payload.json` not written | No output folder configured | Pass `output_dir`, which sets `camera_placement_output_folder_path` |

## When to use this skill

| You want | Skill |
|---|---|
| Cover *this whole floor area* to a coverage ratio | **this skill** |
| N unobstructed views of *this pallet / shelf / robot* | `place-camera-aim-at` |
| Author a single camera's intrinsics, AOVs, distortion | `isaac-camera` |

## How the algorithm works

1. Resolve the target scope — either explicit `[(x_min, x_max), (y_min, y_max)]`
   bounds or the navmesh extents.
2. Split the scope into `patch_size` cells on a focus plane at `focus_height`,
   and mark which cells are accessible.
3. For each of four cardinal directions, estimate the ground-plane footprint of
   a candidate camera's frustum and greedily pick the pose that adds the most
   uncovered patches.
4. Stop when `target_coverage_ratio` is reached, the `num_cameras` budget is
   spent, or a new camera would add less than `min_coverage_increase` patches.

Full parameter semantics and the carb-settings coupling live in
[`references/coverage-api.md`](references/coverage-api.md); scope resolution,
coverage visualisation, and the JSON payload are in
[`references/coverage-runtime-notes.md`](references/coverage-runtime-notes.md).

## Available Scripts

| Script | Purpose | Arguments |
|---|---|---|
| `scripts/place_camera_max_coverage.py` | Cover a floor area with cameras using isaacsim.sensors.rtx.placement | injected `key=value` args (see script docstring) |

## Running scripts

The script is a python_server payload, not a standalone program. From agent
runtimes that expose skill execution helpers, invoke it with `run_script()`:

```python
run_script("scripts/place_camera_max_coverage.py", args=["action=preflight"])
```

From a built tree, send it through the `isaac-sim-remote` client:

```bash
python skills/isaac-sim-remote/scripts/isaacsim_send.py \
  --file skills/place-camera-max-coverage/scripts/place_camera_max_coverage.py \
  --arg action=place --arg num_cameras=5 --arg output_dir=/tmp/isp
```

## Workflow

1. **Preflight.** `action=preflight` (the default) reports extension, stage, and
   scope state without touching the stage. It never raises, so it is safe first.

   ```python
   run_script("scripts/place_camera_max_coverage.py", args=["action=preflight"])
   ```

   Expect `MAX_COVERAGE_PREFLIGHT: ext=enabled stage=ok scope=navmesh` plus the
   resolved bounds. Resolve anything that is not `ok` before continuing.

2. **Confirm the scope.** `action=scope` prints the navmesh-derived bounds
   without placing anything. Compare them against the facility you expect —
   a navmesh that leaked outside the building silently wastes cameras.

   ```python
   run_script("scripts/place_camera_max_coverage.py", args=["action=scope"])
   ```

3. **Place.** Either give a camera budget or let coverage drive the count.

   ```python
   run_script(
       "scripts/place_camera_max_coverage.py",
       args=["action=place", "num_cameras=5", "height_range=5,7.5", "output_dir=/tmp/isp"],
   )
   ```

   `num_cameras=-1` (the default) places until `target_coverage_ratio` is met.
   Expect `MAX_COVERAGE_RESULT: placed=5 total=5 scope=... ratio=0.9 ...` plus one
   line per new camera. The script counts prims on the stage rather than trusting
   the API return value.

4. **Verify coverage.** `action=coverage` debug-draws the coverage frequency of
   the cameras currently under the parent prim and logs the achieved coverage
   and full-coverage ratios to the console.

   ```python
   run_script("scripts/place_camera_max_coverage.py", args=["action=coverage"])
   ```

5. **Iterate.** Vary `random_seed` for a different layout under the same
   constraints; raise `required_camera_per_patch` when downstream perception
   needs overlapping views.

## Calling the API directly

```python
from isaacsim.sensors.rtx.placement import CameraPlacementManager

manager = CameraPlacementManager.get_instance()
cameras = manager.place_camera_in_target_scope_explicit(
    target_scope=[(-9.5, 9.5), (-19.0, 0.0)],
    camera_num=5,
    target_coverage_ratio=0.9,
    camera_height_range=(5.0, 7.5),
    camera_distance_range=(8.0, 16.0),
    camera_look_down_angle_range=(20.0, 60.0),
    spawn_camera=True,
    output_camera_data=True,
)
for camera in cameras or []:
    print(camera["translate"], camera["rotation"])
```

Two traps worth stating up front:

- **`output_camera_data` defaults to `False`, and the call returns `None` in that
  case even on success.** Pass `True` when you want the poses back.
- **Exceptions are swallowed into `carb.log_error`** and the call then returns a
  partially filled list. Verify against the stage, not the return value — which
  is what `scripts/place_camera_max_coverage.py` does.

Use `place_camera_in_target_scope_explicit`, not
`place_camera_in_target_scope`. The latter is the GUI path and reads every
parameter from persistent carb settings.

## Related skills

- `isaac-sim-remote` — the transport this skill's script rides on.
- `place-camera-aim-at` — the single-target sibling.
- `occupancy-map` — an alternative source of floor bounds when no navmesh exists.
- `isaac-camera` — per-camera intrinsics, render products, AOVs, distortion.
- `isaac-sim-sensor` — attaching annotators and writers once cameras exist.
