---
name: data-collection-sim
description: >
  Build headless static-scene data collection pipelines using Isaac Sim 6 /
  Kit 110 Replicator. Produces annotated RGB, depth, segmentation, bounding
  boxes, and pose data via `BasicWriter`, `KittiWriter`, `CocoWriter`,
  `CosmosWriter`, `FPSWriter` (from `omni.replicator.core`) and `PoseWriter`,
  `DataVisualizationWriter` (from `isaacsim.replicator.writers`). Uses
  `isaacsim.core.experimental.utils.stage` to author the scene and either
  `rep.functional.modify.semantics` or
  `isaacsim.core.experimental.utils.semantics.add_labels` to label prims for
  annotators. Use when generating synthetic data, creating training datasets,
  or building SDG pipelines. For mobile-robot trajectory-driven SDG see
  `mobility-gen`; for grasp / teleop / episode-record workflows see
  `isaacsim.replicator.*` pointers below.
---

# Data Collection Simulation

Build headless SDG pipelines using Isaac Sim 6.0 Replicator. Outputs annotated frames (RGB, depth, segmentation, bbox, pose) to disk via writers.

## Related Skills

- `mobility-gen` — mobile-robot trajectory recording then replay+render (two-phase, robot-mounted sensors)
- `isaac-sim-sensor` — sensor primitives (camera, LiDAR, IMU, contact)
- `occupancy-map` — produces occupancy maps for spawn placement

## Architecture

```
Config (YAML/JSON) → SimulationApp (headless) → Scene Setup → Randomizers → Capture Loop → Writer → Disk
```

## Required Imports

```python
from isaacsim import SimulationApp
simulation_app = SimulationApp({"renderer": "RealTimePathTracing", "headless": True})

import carb.settings
import omni.replicator.core as rep
import omni.usd
import isaacsim.core.experimental.utils.stage as stage_utils
from isaacsim.core.experimental.utils.semantics import add_labels, remove_all_labels
from isaacsim.storage.native import get_assets_root_path
```

`stage_utils.open_stage()` / `stage_utils.add_reference_to_stage()` / `stage_utils.define_prim()` / `stage_utils.get_current_stage()` are the Kit 110 replacements for the legacy `isaacsim.core.utils.stage.*` and `isaacsim.core.utils.prims.create_prim` flow. Keep `omni.replicator.core` for SDG primitives.

> **Migration:** for the full `omni.isaac.*` → `isaacsim.*` mapping, see [Renaming Extensions](https://docs.isaacsim.omniverse.nvidia.com/latest/migration_guides/isaac_sim_4_5/extensions_renaming.html).

## Writers

Resolve writers via `rep.WriterRegistry.get(name)` (or `rep.writers.get(name)`, same registry).

`omni.replicator.core` built-in writers (`omni.replicator.core/scripts/writers_default/`):

| Writer | Use case |
|---|---|
| `BasicWriter` | RGB, bbox_2d (tight/loose), bbox_3d, semantic/instance/instance_id segmentation, depth (image-plane / camera), normals, occlusion, motion vectors, camera params, pointcloud, skeleton |
| `KittiWriter` | KITTI-format datasets |
| `CocoWriter` | COCO-format datasets (instance/bbox) |
| `CosmosWriter` | Cosmos warehouse video clips (PNG sequences + MP4 per modality: rgb, shaded_seg, segmentation, depth, edges). Requires `/app/omni.graph.scriptnode/opt_in = True` |
| `FPSWriter` | Frame-rate / capture-time telemetry |
| Custom `Writer` subclass | direct access to annotator tensors (subclass `omni.replicator.core.Writer`, register with `rep.writers.register_writer`) |

`isaacsim.replicator.writers` adds Isaac-specific writers:

| Writer | Use case |
|---|---|
| `PoseWriter` | 6-DoF object pose estimation (optional `write_debug_images=True`) |
| `DataVisualizationWriter` | debug / overlay visualization |

Deprecated and not for new work: `DOPEWriter`, `YCBVideoWriter`, `PytorchWriter`, `PytorchListener` (also `OgnPose` node). They will be removed in a future major release.

### Writer initialization patterns

Two equivalent patterns are supported. Recent (Isaac Sim 6.0+) examples favor the explicit-backend form:

```python
backend = rep.backends.get("DiskBackend")
backend.initialize(output_dir="/tmp/sdg_output")
writer = rep.writers.get("BasicWriter")
writer.initialize(backend=backend, rgb=True, bounding_box_2d_tight=True)
writer.attach(rp)
```

Legacy short form (still works; backend created implicitly from `output_dir`):

```python
writer = rep.WriterRegistry.get("BasicWriter")
writer.initialize(output_dir="/tmp/sdg_output", rgb=True, bounding_box_2d_tight=True)
writer.attach(rp)
```

## Annotators

Core annotators available via `rep.annotators.get(name)`:

- `rgb` — RGBA uint8
- `distance_to_image_plane` — depth float32
- `semantic_segmentation` — per-pixel class labels
- `instance_segmentation` — per-pixel instance IDs
- `bounding_box_2d_tight` — tight 2D bboxes
- `bounding_box_2d_loose` — loose 2D bboxes
- `bounding_box_3d` — 3D bboxes in world coords
- `camera_params` — intrinsics, extrinsics, resolution
- `occlusion` — visibility ratio per instance
- `normals` — surface normals
- `pointcloud` — 3D point cloud from depth

## Minimal Pipeline

```python
# 1. Scene
rep.orchestrator.set_capture_on_play(False)
carb.settings.get_settings().set("rtx/post/dlss/execMode", 2)  # DLSS Quality
assets_root = get_assets_root_path()
stage_utils.open_stage(assets_root + "/Isaac/Environments/Simple_Warehouse/full_warehouse.usd")

# 2. Semantic labels — add a referenced prop and tag it for annotators.
stage_utils.add_reference_to_stage(
    usd_path=assets_root + "/Isaac/Props/YCB/Axis_Aligned/003_cracker_box.usd",
    path="/World/MyObj",
)
prim = stage_utils.get_current_stage().GetPrimAtPath("/World/MyObj")
# Either API writes the UsdSemantics LabelsAPI schema on the prim:
add_labels(prim, labels=["cracker_box"], taxonomy="class")
# or, equivalently:
# rep.functional.modify.semantics(prim, {"class": "cracker_box"}, mode="add")

# 3. Camera + render product
cam = rep.functional.create.camera(position=(3, 3, 2), look_at=(0, 0, 0), name="DataCam")
rp = rep.create.render_product(cam, (1280, 720), name="main_view")

# 4. Writer (explicit backend form)
backend = rep.backends.get("DiskBackend")
backend.initialize(output_dir="/tmp/sdg_output")
writer = rep.writers.get("BasicWriter")
writer.initialize(
    backend=backend,
    rgb=True,
    bounding_box_2d_tight=True,
    semantic_segmentation=True,
    distance_to_image_plane=True,
)
writer.attach(rp)

# 5. Capture loop with randomization (static scene: delta_time=0.0 freezes timeline)
for i in range(NUM_FRAMES):
    # Randomize object pose, camera, lights here
    rep.orchestrator.step(delta_time=0.0, rt_subframes=32)

# 6. Cleanup
rep.orchestrator.wait_until_complete()
writer.detach()
rp.destroy()
simulation_app.close()
```

## Domain Randomization

Use `rep.functional` API for randomization each frame:

```python
# Scatter objects on surface
rep.functional.randomizer.scatter_2d(prims=objects, surface_prims=plane, check_for_collisions=True, rng=rng)

# Randomize camera pose
rep.functional.modify.pose(cam, position_value=rng.uniform(pos_min, pos_max),
                           look_at_value=target_prim, look_at_up_axis=(0, 0, 1))

# Randomize lights via OmniGraph events
rep.utils.send_og_event(event_name="randomize_lights")
```

## Headless Execution

The canonical Isaac Sim Python launcher is `python.sh` (use `python.bat` on Windows). `isaac-sim.sh` launches the full editor app and is not the right entry point for standalone SDG scripts.

```bash
# $ISAAC_SIM_DIR is either the install root or <repo>/_build/linux-x86_64/release
"$ISAAC_SIM_DIR/python.sh" "$WORKSPACE_DIR/data_collection.py" --config config.yaml
```

Headlessness is controlled by `SimulationApp({"headless": True})` inside the script, not by a launcher flag.

Or via the Isaac Lab runner ($ISAAC_LAB_DIR is your Isaac Lab checkout):

```bash
"$ISAAC_LAB_DIR/isaaclab.sh" -p data_collection.py --config config.yaml
```

## Configuration Pattern

Use YAML config to parameterize everything:

```yaml
resolution: [1280, 720]
rt_subframes: 32
num_frames: 100
headless: true
env_url: "/Isaac/Environments/Simple_Warehouse/full_warehouse.usd"
writer: BasicWriter
output_dir: /tmp/sdg_output
annotations:
  rgb: true
  bounding_box_2d_tight: true
  semantic_segmentation: true
  distance_to_image_plane: true
  bounding_box_3d: true
objects:
  - url: "/Isaac/Props/YCB/Axis_Aligned/003_cracker_box.usd"
    label: cracker_box
    count: 5
  - url: "/Isaac/Props/YCB/Axis_Aligned/008_pudding_box.usd"
    label: pudding_box
    count: 3
```

## Validation Checklist

1. Output directory contains expected number of frames
2. RGB images are non-black (mean RGB > 30)
3. Annotation files match frame count
4. Semantic labels appear in segmentation maps
5. Bounding boxes have non-zero area
6. No NaN in depth maps

## Key Rules

- Set `rep.orchestrator.set_capture_on_play(False)` for manual step control.
- `rt_subframes`: render the same frame multiple times to reduce ghosting from large pose deltas and to let materials/textures converge. Tune for your renderer: small (4-8) is often enough for RTX Real-Time + DLSS Quality; 16-32 is typical for path tracing or scenes with heavy material streaming.
- DLSS Quality: `carb.settings.get_settings().set("rtx/post/dlss/execMode", 2)`. Recommended for SDG; default Performance mode can produce edge artifacts below ~600x600.
- Tag every prim you want annotated via `add_labels(...)` (taxonomy-aware) or `rep.functional.modify.semantics(...)`. Both write the `UsdSemantics.LabelsAPI` schema.
- For static-scene SDG, pass `delta_time=0.0` to `rep.orchestrator.step` so the timeline does not advance between captures.
- Call `rep.orchestrator.wait_until_complete()` before cleanup so the background backend has flushed everything to disk.
- Use `rng = np.random.default_rng(seed)` and `rep.set_global_seed(seed)` for reproducible randomization.
- Performance knobs for high-throughput captures (see `sdg_getting_started_05.py`):
  - `rep.orchestrator.step(wait_for_render=False)` decouples capture from render completion. Data may correspond to a previous frame; only use when strict frame-to-data correspondence is not required.
  - `carb.settings.get_settings().set("/exts/omni.replicator.core/enableWriteToFabric", True)` writes randomization deltas directly to Fabric instead of going through USD first. Faster, but transient — changes are not persisted in the USD stage.

## Sibling SDG workflows

| Workflow | Module / example |
|---|---|
| Mobile-robot trajectory record + replay | `mobility-gen` skill + `isaacsim.replicator.mobility_gen` |
| Grasp dataset generation | `isaacsim.replicator.grasping` (`GraspingManager`, `GraspPhase`) — `source/standalone_examples/api/isaacsim.replicator.grasping/grasping_workflow_sdg.py` |
| Episode record + replay | `isaacsim.replicator.episode_recorder` |
| Teleop record + replay | `isaacsim.replicator.teleop` |
| Reusable randomization behavior scripts (attached to prims, USD-persistent) | `isaacsim.replicator.behavior` |
| Sensor primitives (LiDAR, IMU, camera) | `isaac-sim-sensor`, `isaac-camera` |
| Spawn placement from a map | `occupancy-map` |

## Reference examples

Paths relative to `$ISAAC_SIM_DIR` (install root or `<this-repo>` for a source build) and `$ISAAC_LAB_DIR`:

- API examples: `$ISAAC_SIM_DIR/source/standalone_examples/api/isaacsim.replicator.examples/sdg_getting_started_0[1-5].py`, `sdg_workflow_0[12].py`, `multi_camera.py`, `motion_blur.py`, `cosmos_writer_simple.py`, `simready_assets_sdg.py`, `sdg_deformables.py`, `sdg_geomsubset.py`, `subscribers_and_events.py`, `custom_event_and_write.py`, `custom_fps_writer_annotator.py`.
- Scene-based SDG: `$ISAAC_SIM_DIR/source/standalone_examples/replicator/scene_based_sdg/`
- Object-based SDG: `$ISAAC_SIM_DIR/source/standalone_examples/replicator/object_based_sdg/`
- Augmentation pipelines: `$ISAAC_SIM_DIR/source/standalone_examples/replicator/augmentation/`
- Cosmos warehouse writer: `$ISAAC_SIM_DIR/source/standalone_examples/replicator/cosmos_writer_warehouse.py`
- Infinigen SDG: `$ISAAC_SIM_DIR/source/standalone_examples/replicator/infinigen/`
- Grasping SDG: `$ISAAC_SIM_DIR/source/standalone_examples/api/isaacsim.replicator.grasping/grasping_workflow_sdg.py`
- Isaac Lab imitation learning: `$ISAAC_LAB_DIR/scripts/imitation_learning/`
