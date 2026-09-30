---
name: place-camera-aim-at
description: "Occlusion-aware CircularCameraPlacement around one target prim or world point. Use for multi-angle product or robot inspection shots that must keep the subject visible."
license: Apache-2.0
metadata:
  author: NVIDIA Isaac Sim <isaac-sim@nvidia.com>
---

# Place Cameras Aimed At a Target

## Purpose

Ring a single object with N cameras that each have a clear, unoccluded view of it, using the `CircularCameraPlacement` API of `isaacsim.sensors.rtx.placement`.

## Prerequisites

- Running Isaac Sim 6 / Kit 110 with python_server (port 8226); see `isaac-sim-remote`.
- Extension `isaacsim.sensors.rtx.placement` enabled (action_and_event_data_generation app, or `--enable`).
- Stage contains the **target object** and a baked navmesh (or pass `scope` with `on_navmesh=false`).
- Shell env: `$ISAAC_SIM_DIR`, `$ISAAC_LAB_DIR`, `$WORKSPACE_DIR` (`isaac-sim-orchestrator`).

## Limitations

- Does not build `isaacsim.sensors.rtx.placement` — consume the pin from
  `source/apps/isaacsim.exp.action_and_event_data_generation.base.kit`.
- Live Kit only (raycast + navmesh); no offline substitute.
- Occlusion uses target bounding boxes, not per-triangle visibility.
- Sampling is stochastic; `random_seed` / density / geometry change results.
- For floor-area coverage ratios (not single-object aim), use `place-camera-max-coverage`.

## Troubleshooting

| Error / symptom | Cause | Solution |
|---|---|---|
| `AIM_AT_PREFLIGHT: ext=unavailable` | App does not resolve the extension | Launch `isaacsim.exp.action_and_event_data_generation.base.sh`, or add `--enable isaacsim.sensors.rtx.placement` |
| `ValueError: not enough values to unpack (expected 2, got 1)` | **Upstream defect** — zero surviving directions crashes inside `select_sparse_vectors` instead of returning `None` | The script translates this into remedies: widen `distance_range`/`height_range`/`yaw_ranges`, raise `occlusion_threshold` (`-1` disables), or set `on_navmesh=false` |
| Script raises "returned no poses" | Validation failed and logged via `carb.log_error` | Read the Isaac Sim console; usually `height_range` below the target, an invisible target, or inverted min/max ranges |
| "Fail to place N cameras from different directions" | Surviving directions non-zero but fewer than `num_cameras` | Raise `occlusion_threshold`, widen `distance_range`/`yaw_ranges`, or lower `num_cameras` |
| "camera height should not be lower than object" | Target sits above `height_range[1]` | Raise the height range above the target's Z |
| "target object is invisible" | Target prim already hidden | Make it visible; the API hides it itself during the sweep |
| Cameras appear but views are blocked | Occlusion filtering was silently disabled | Confirm `target_prim` has `Mesh` descendants and `occlusion_threshold != -1` |

## When to use this skill

| You want | Skill |
|---|---|
| N views of *this pallet / shelf / robot*, each unobstructed | **this skill** |
| Cover *this whole floor area* to a coverage ratio | `place-camera-max-coverage` |
| Author a single camera's intrinsics, AOVs, distortion | `isaac-camera` |

## How the algorithm works

1. Hide the target prim, then raycast a sweep of candidate observation directions around it, constrained by `height_range`, `distance_range`, `look_down_angle_range`, and `yaw_ranges`.
2. Restore the target's visibility, then walk each direction outward in `camera_distance_step_size` increments, keeping positions that are on the navmesh (and inside `scope`, when given).
3. Reject any pose whose occlusion ratio for the target is at or above `occlusion_threshold`.
4. From the surviving directions, pick the `num_cameras` most widely separated (`math_util.select_sparse_vectors`) so the views are not redundant, and author a `Camera` prim per pick.

Full parameter semantics and the non-obvious validation behavior live in
[`references/aim-at-api.md`](references/aim-at-api.md); occlusion internals and
the destructive `refresh` flag are in
[`references/aim-at-runtime-notes.md`](references/aim-at-runtime-notes.md).

## Available Scripts

| Script | Purpose | Arguments |
|---|---|---|
| `scripts/place_camera_aim_at.py` | Place cameras aimed at a target object using isaacsim.sensors.rtx.placement | injected `key=value` args (see script docstring) |

## Running scripts

The script is a python_server payload, not a standalone program. From agent
runtimes that expose skill execution helpers, invoke it with `run_script()`:

```python
run_script("scripts/place_camera_aim_at.py", args=["action=preflight"])
```

From a built tree, send it through the `isaac-sim-remote` client:

```bash
python skills/isaac-sim-remote/scripts/isaacsim_send.py \
  --file skills/place-camera-aim-at/scripts/place_camera_aim_at.py \
  --arg action=place --arg target_prim=/World/Pallet --arg num_cameras=3
```

## Workflow

1. **Preflight.** `action=preflight` (the default) reports extension, stage,
   target, mesh count, and navmesh state without touching the stage. It never
   raises, so it is safe to run first.

   ```python
   run_script("scripts/place_camera_aim_at.py", args=["action=preflight", "target_prim=/World/Pallet"])
   ```

   Expect: `AIM_AT_PREFLIGHT: ext=enabled stage=ok target=ok meshes=12 navmesh=ok`.
   Resolve anything that is not `ok` before continuing — the remedy lines say how.

2. **Place.** Give either `target_prim` (preferred — enables occlusion filtering)
   or `target_point`.

   ```python
   run_script(
       "scripts/place_camera_aim_at.py",
       args=["action=place", "target_prim=/World/Pallet", "num_cameras=3", "height_range=2,4"],
   )
   ```

   Expect `AIM_AT_RESULT: placed=3 ...` plus one `camera[i] position=... focus=...`
   line each. Fewer cameras than requested is a hard error, not a warning.

3. **Verify.** Cameras land under the parent prim path (default `/World/Cameras`).
   Confirm the count and eyeball one render before trusting the placement — the
   occlusion test is bbox-based.

4. **Iterate.** Vary `random_seed` for a different arrangement with identical
   constraints; loosen `occlusion_threshold` or widen `distance_range` when
   placement fails for lack of surviving directions.

## Calling the API directly

When you need the poses without authoring prims (`spawn_camera` defaults to
`False`):

```python
from isaacsim.sensors.rtx.placement import CircularCameraPlacement

poses = CircularCameraPlacement.circular_camera_placement(
    target_object_path="/World/Pallet",
    number_of_camera_to_select=3,
    camera_height_range=(2.0, 4.0),
    camera_distance_range=(6.0, 18.0),
    occlusion_threshold=0.4,
    spawn_camera=False,
)
if not poses:
    raise RuntimeError("placement failed; see the carb log for the reason")
for pose in poses:
    print(pose.camera_position, pose.focus_point)
```

`poses` is a list of `CameraPose` dataclasses with `camera_position`,
`camera_direction`, and `focus_point` as `np.ndarray`. **The API returns `None`
on every failure path** rather than raising — always null-check.

## Related skills

- `isaac-sim-remote` — the transport this skill's script rides on.
- `place-camera-max-coverage` — the area-coverage sibling.
- `isaac-camera` — per-camera intrinsics, render products, AOVs, distortion.
- `isaac-sim-sensor` — attaching annotators and writers once cameras exist.
